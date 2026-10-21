# Independent Angular References

The servo's encoder limits, its angular zero and its degrees-per-tick scale are
different quantities. Mechanical range endpoints must not automatically become
-180/+180 degrees, nor must their midpoint automatically become physical zero.

For calibrated angular references, cf-robot supports an optional persistent
sidecar next to the arm's LeRobot JSON:

```text
so101_leader.json            # SDK calibration; MIN/MAX mirror hardware.
so101_leader.cf-robot.json   # Optional cf-robot angular references.
```

Hardware MIN/MAX are the authoritative travel limits. The sidecar stores
independent zero/scale references, not another travel range.
Servo limits are not physical interlocks or a certified force limiter; an
unpowered joint can still move by hand. Keep physical support and clearances.
The sidecar is local to that arm ID and role. It is not a servo EEPROM setting
and is not copied to the other arm. It survives application restarts and machine
reboots. Back it up with the original LeRobot calibration when moving computers
or changing the Hugging Face data directory.

## Leader J5 Example

For the verified `so101_leader` wrist-roll measurements:

| Reference | Ticks |
| --- | --- |
| Usable minimum | 130 |
| Known -90 degrees | 1015 |
| Practical zero | 2038 |
| Known +90 degrees, clockwise | 3062 |
| Usable maximum | 3936 |

The known-angle span gives `180 / 2047` degrees per tick:

```text
degrees = (ticks - 2038) * 180 / 2047
ticks = nearest_integer(2038 + degrees * 2047 / 180)
```

The exact two-reference midpoint is 2038.5; rounding the practical zero to 2038
shifts angles by only about 0.044 degrees. Thus the references read approximately
-89.956 and +90.044 degrees, with working limits approximately -167.78 to
+166.90 degrees. The usable travel is about 334.68 degrees, not 360.

This profile is bound to the leader's existing homing offset **-1786**. That
offset was verified in both the saved SDK calibration and live motor report.
The profile does not write that offset or alter the encoder coordinate system.
Do not copy this homing offset or profile to a different arm.

Example sidecar after the confirmed hardware-limit update:

```json
{
  "version": 1,
  "side": "leader",
  "arm_id": "so101_leader",
  "joints": {
    "wrist_roll": {
      "id": 5,
      "model": "sts3215",
      "homing_offset": -1786,
      "zero_ticks": 2038,
      "degrees_per_tick": 0.08793356130923302,
      "positive_direction": "clockwise"
    }
  }
}
```

## Shared Conversion And Safety

The hardware factories load the sidecar for the arm's actual calibration path.
Only the specified J1-J5 conversions are overlaid on that bus instance. Gripper
overrides are rejected so an angular profile cannot inadvertently reinterpret
J6's calibrated percentage range. Motors without an entry use the original
SDK conversion unchanged.

The same conversion hooks serve SDK normalized reads/writes, direct monitor
sampling, normal CLI teleoperation and browser teleoperation. Degree-to-tick
inversion uses the same zero/scale. Raw and normalized goal writes are checked
against the verified hardware range before issuing commands.

On connection, the app verifies motor identity, degree mode, saved homing
offset, live homing offset and agreement between SDK and live servo limits.
A mismatch blocks the affected angular conversion and teleoperation, while
raw monitor readings remain available for diagnosis. Calibration-register
writes invalidate the active mapping until a new connection verifies it again.
Stop monitoring/teleoperation before using any external calibration tool.

The inspector reports the sidecar path/hash, bound offset, zero, scale and
verification result. The normal MIN/MAX field displays actual hardware limits.
The first software-only profiles remain readable for migration and may show
`work` / `HW` until the confirmed hardware update. That update removes their
range fields; only the verified hardware range remains authoritative.
The legacy `working_range` wire field remains an alias for compatibility with
older operator assets. After migration it equals hardware MIN/MAX and is not
displayed as a separate range by current clients.

Angular profile changes participate in the physical-validation fingerprint, so
captures made under a different mapping are not silently reused.

## Program Leader J5 Limits

Install/update the Mac package and restart ARMS MONITOR or the local browser
bridge; an already-running process retains its previous conversion. Close ARMS
MONITOR, the local leader bridge and teleoperation before this local operation.
Support the leader and ensure J5 torque is already OFF. Position J5 well inside
130..3936, at least 20 ticks from either end. The command never releases torque
automatically or sends a goal.

```sh
python -m cf_robot.leader_limits \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader --joint 5 --min 130 --max 3936
```

The read-only preview lists the proposed EEPROM writes. After reviewing, type
`APPLY J5 LIMITS` at its prompt to confirm. It changes only J5
`Min_Position_Limit=130` and `Max_Position_Limit=3936`, temporarily unlocking
EEPROM if needed. It verifies readback and restores the original lock state.
The homing offset remains -1786, torque stays OFF and other joints, including
J6, are unchanged. Existing correct limits are not rewritten unnecessarily.

Original SDK/profile bytes and the selected live calibration are backed up.
The SDK file is updated to match the verified hardware; the angular file keeps
zero 2038 and scale 180/2047, with no `range_min`/`range_max` fields. The new
EEPROM limits persist through servo power cycles; normal restarts/reboots retain
the matching files. No additional save command is required.

With torque OFF, manually check leader J5 at the known zero and +/-90 degree
poses. Readings should be approximately 0, -90 and +90, respectively. Do not
force endpoints. Inspect J6 to confirm its existing percentage calibration is
unchanged. MIN/MAX should now report 130..3936 directly, without a separate
working range. This does not validate or repair follower J5.

The sidecar is specific to cf-robot; stock LeRobot commands run outside this
application do not load it; their default midpoint-based angles are not the
independently fitted zero/scale. The hardware limits apply regardless of client.
Updating the code or saving the original software-only profile does not itself
program hardware: only the explicit confirmation does.

## Follower J5 And Isolated Testing

The follower has its own references: usable ticks 130..3946, -90 degrees at
1005 and +90 degrees at 3044. These imply:

```text
zero_ticks = (1005 + 3044) / 2 = 2024.5
degrees_per_tick = 180 / (3044 - 1005) = 180 / 2039
degrees = (ticks - 2024.5) * 180 / 2039
working angular range: approximately -167.24 .. +169.63 degrees
```

The 2038-tick midpoint of the working endpoints is not angular zero. The leader
and follower must retain their separate reference points and homing offsets.

Update both packages and restart the Pi follower service. Stop teleoperation,
secure/support the wrist and gripper, and start follower-only ARMS MONITOR:

```sh
cf-robot-pose-compare --side follower
```

With J5 physically supported, explicitly disable only its torque, then request
the read-only preview of hardware MIN/MAX and matching files:

```text
J5 TD
J5 ANGULAR PREVIEW 130 3946 1005 3044
```

Review the working bounds, zero 2024.5, scale 180/2039 and current live homing
offset. The tick measurements must have been taken with that live offset.
The earlier follower report had live -1164 and saved -1214; this procedure does
not change the live offset or shift encoder readings. It aligns only J5's SDK
file record to its live offset, programs only MIN/MAX to 130..3946 and stores
the independent zero/scale in `so101_follower.cf-robot.json`, without a second
range. Ensure the current supported wrist is at least 20 ticks inside those
limits before previewing.

Only after reviewing the preview:

```text
J5 ANGULAR CONFIRM
```

The one-use preview expires after 60 seconds. Changes to files, motor settings,
torque or joint position require a new preview. Original SDK/profile bytes are
backed up before replacement. Other SDK joints, existing other angular entries
and J6 calibration are preserved. Confirmation writes selected MIN/MAX EEPROM,
restores the original EEPROM lock state and verifies hardware/file readback.
There are no homing-offset, goal or torque writes. The monitor ends after
confirmation; success details and
backup paths are retained in the CLI output.

Hardware and files are not a single transaction. An interrupted update may
leave partial EEPROM settings or mismatched files. The saved/live and angular
coordinate checks block motion until repaired. No automatic write retry or
rollback is attempted. Keep torque OFF and inspect readback/backups on failure.
Old file-only clients cannot perform this hardware update: requests require
explicit hardware-write intent and the new limit-specific confirmation.

After success, use the usual leader command with:

```sh
cf-robot-leader \
  --url "$CF_ROBOT_CONTROL_URL" \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader --J5-only
```

This uses `/control/joint/5` and requires explicit J5-only acknowledgement.
Only wrist-roll targets are sent, with shared startup sync, WAIT/Ready,
bounded following, feedback holds and watchdogs. Start with small movements,
then validate +/-90 degrees. The two arms' mechanical endpoint ranges differ;
do not force them or request positions outside the follower's working range.
On stop J5 remains powered holding its position to prevent wrist/gripper droop.
A subsequent J5 test can reconnect while the follower wrist is already holding;
it does not require releasing that supporting axis between tests. The leader's
selected joint must remain torque OFF for manual operation. File installation
still requires follower J5 torque OFF and physical support.
J6-only still releases its gripper as before.

To test both calibrated axes together, replace the single-joint flag with
`--J5-J6-only`. This uses the dedicated `/control/joints/5-6` endpoint and
requires updated Mac and Pi packages. After Enter, WAIT remains active while
both axes synchronise sequentially to a fixed leader snapshot. Ready appears
only when both have settled on fresh measured feedback. Live following then
streams J5 and J6 together; J1-J4 are never commanded or torque-changed.
On stop J6 releases and J5 holds, so keep the gripper empty. Existing calibration
files and EEPROM values need no changes for this combined mode.

`--J1-only` through `--J5-only` use the same selected-joint implementation and
hold the selected supporting axis on stop. Each requires its own valid
calibration before use. Flags are mutually exclusive. The angular preview
command supports J1-J5 only, and its current reference syntax assumes safely
accessible -90/+90 poses; never force another joint to those positions.
