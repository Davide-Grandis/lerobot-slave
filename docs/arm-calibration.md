# Inspect, Validate And Set Arm Calibration

Normal teleoperation and joint-only tests use the
[same shared teleoperation policy](teleoperation.md). Improvements made while
testing J6 apply to normal CLI and browser leader control too; only the selected
joints and appropriate stop/holding behavior differ.

Calibration has three independent layers: the saved LeRobot record, the live
servo configuration and the physical meaning of the resulting joint value.
Leader/follower comparison is optional, not a substitute for validating each
arm against a known physical reference.

## Inspect One Arm

In Pose Monitor, choose **Leader only**, **Follower only** or **Both arms** before
starting. Leader-only monitoring opens no follower connection; follower-only
monitoring never opens the leader serial port. Monitoring preserves existing
torque and sends no position goals at startup.

The calibration inspector reports:

- Arm identity, serial port, installed LeRobot version and calibration path.
- File SHA256, modification time and whether its content matches the loaded
  calibration. The loaded-record fingerprint is distinct from the file hash.
- Per-joint configured motor ID/model and the model number read by a bus ping.
- Saved and live `id`, `drive_mode`, `homing_offset`, `range_min`, `range_max`.
- Actual normalization mode, encoder resolution and operating mode.
- Reference midpoint and converted endpoints using the running SDK's conversion.
- Saved/live discrepancies, missing information and unexpected units.

Live `drive_mode` in the Feetech SDK's calibration record is metadata, not a
hardware direction-register read. The inspector reports whether the backend
applies the saved drive mode to normalization. Positive physical direction must
still be checked by a person. A responding bus ID/model does not prove that its
motor is mounted on the intended joint: verify the joint-to-ID mapping physically.

Live MIN/MAX in the original readings table are servo position-limit registers.
They are not the saved calibration range. Invalid/reversed limits remain visible
in inspection rather than being silently corrected.

J1-J5 are explicitly configured in degrees and J6 in normalized 0-100 percent.
Unexpected modes are flagged. The inspector uses LeRobot's actual conversion,
not a duplicate formula. The homing offset is already handled by the servo and
must not be added again in the display. The degree reference in the SO-arm SDK
is based on the recorded range midpoint; it is not automatically a universal
physical "straight arm" zero.

Register inspection is a snapshot at monitor startup, refreshed automatically
after targeted J6 restore commands. Stop/restart monitoring after other
calibration or configuration changes. Missing calibration can leave
normalized values unavailable while raw inspection remains usable. An unreadable
or malformed file that prevents LeRobot constructing the arm must be repaired
or replaced before monitoring can start; it can also prevent the guided
calibrator from starting. Preserve the original file before repairing it.

Follower position commands are refused while calibration consistency warnings
remain. Torque commands remain explicit and available, subject to normal session
checks. This is not a physical safety certification or a check of every possible
collision and joint limit.

## Validate Without A Second Arm

1. Secure the selected arm and keep the physical emergency stop accessible.
   Do not release torque unless the arm is physically supported. The monitor
   never changes the leader's torque; it only reads it.
2. Define the joint's physical reference and positive direction relative to its
   parent link. Use an independent measurement or fixture, not the 3D model.
3. In **Capture a checked pose**, select the arm and joint. Enter a label, known
   angle (J1-J5) or percentage (J6), and a positive tolerance in the same units.
   Confirm that the physical pose has been independently checked.
4. Capture at least three distinct safe interior positions, including the
   reference and a small positive displacement. Repeat in both directions if
   checking backlash. Never force mechanical stops.
5. Review each residual (`observed - expected`) and the fitted scale/offset.
   A negative slope suggests reversed direction; scale far from 1 suggests a
   scale/unit problem; a constant error suggests a reference problem. The fit
   is a diagnostic heuristic, not an automatic correction.
6. Check continuous motion and watch for out-of-range/clipping or possible
   encoder-wrap warnings. Large raw changes can also mean rapid physical motion;
   the warning alone does not prove a wrap.

Gripper percent is a fraction of the calibrated encoder range, not millimeters.
Check the intended meanings of 0, an intermediate opening and 100 independently.
Do not use excessive closing force to establish an endpoint.

The optional **Matched physical poses** capture records both arms and their
per-joint differences. It requires fresh samples within 300 ms and equal
normalization modes. Independent captures require only the selected arm.

Evidence is bounded to 120 captures and held in the controller process, not
persisted across controller restarts. **Export inspection + evidence** downloads
the full readings, calibration records, captures and analysis as JSON. Clearing
evidence requires separate confirmation. Export before clearing or restarting.
Fits exclude evidence from a different arm/configuration/calibration fingerprint.
Three passing poses do not validate an entire arm, its endpoints or its workspace.

## Set Calibration Independently

Stop monitoring and teleoperation. Export the current report first.

**Set calibration** runs LeRobot's existing guided calibrator for exactly one arm:
the local Mac leader, or the Pi follower through the authenticated `/calibrate`
route. Type `CALIBRATE`, confirm physical support/presence and follow the streamed
prompts. Empty reply sends Enter. At the existing-calibration prompt:

- Enter reapplies that arm's saved calibration to its motors.
- `c` measures new calibration and may overwrite the selected arm's saved file.

This procedure can release torque and change servo EEPROM settings. It does not
derive or write correction offsets from captured validation poses, and it never
copies calibration between arms. Keep someone at the selected physical arm.
The calibrator's real prompts are authoritative.

Browser heartbeat loss, cancellation, software stop or controller shutdown
cancels the procedure. Serial ownership is retained during local process
cleanup; the Pi similarly retains its maintenance lock during remote cleanup.
Cancellation is not rollback: settings may have been partially changed. Inspect
and complete/repeat calibration before resuming. No calibration automatically
resumes or automatically restarts teleoperation.

After completion, restart monitoring to reload calibration and repeat physical
validation. Whole-arm guided calibration and live monitoring cannot run concurrently.

## Restore Only Follower J6 From The Saved Calibration

This separate operation runs inside the follower monitor. It repairs a mismatch
between the saved gripper calibration and its live servo configuration without
recalibrating any joint or editing the saved file. Update both the controller
package and Pi package first; publish the new static assets for browser controls.
Stop teleoperation and competing monitor sessions.

In the CLI, start follower-only monitoring, secure the arm, keep fingers clear
and use:

```text
J6 TD
J6 RESTORE PREVIEW
```

`J6 TD` explicitly disables only the gripper's torque; it does not disable the
arm's other joints. Move the gripper gently to a safe interior position only
with torque OFF, then keep it still and request a fresh preview.

Preview is read-only. It shows the current and proposed saved offset/ranges,
current raw position, predicted raw position after the offset change and the
source file fingerprint. Verify the proposed values. For the reported repair,
these are offset `-873` and limits `2038..3488`; values always come from the
loaded saved calibration, not from arbitrary command arguments.

Feetech uses `Present_Position = Actual_Position - Homing_Offset`, so the
prediction is `current_raw + live_offset - saved_offset`. No wrap is assumed.
The predicted position must lie at least 20 ticks inside each proposed limit.
A fully open reading of 2863 with live offset -208 predicts 3528 under -873 and
is therefore refused for limits 2038..3488. An interior reading of 2100 predicts
2765 and passes the position check.

Only after physically checking the workspace/support and reviewing the preview:

```text
J6 RESTORE CONFIRM
```

The CLI attaches the one-use preview token automatically; no long token needs
to be copied. Confirmation is valid only in the same monitor session and within
60 seconds. Quit/reconnect, expired previews, more than 20 ticks of gripper
movement, changed calibration/limits or torque ON require a new preview.
Non-calibration servo commands invalidate the pending preview.

In the browser, use **Restore gripper calibration** below the servo command
form. Explicitly disable J6 torque first, select **Preview J6 restore**, inspect
the report, type `RESTORE J6`, check the support/clearance confirmation and confirm.

The Pi validates the arm/model, saved-file consistency, operating mode, torque,
preview freshness and live configuration again before writing. Only J6's
Homing_Offset, Min_Position_Limit and Max_Position_Limit are restored; J6's EEPROM
lock is temporarily unlocked if necessary and its original lock state restored
on success. No Goal_Position or Torque_Enable is written. EEPROM writes are not
automatically retried. J1-J5 settings and torque states are not changed.

Readback verifies the J6 calibration, predicted encoder position, unchanged
J1-J5 calibration and unchanged file. J6 stays torque OFF. The inspector is
refreshed automatically; warnings on J1-J5 remain until those joints are repaired
and physically validated separately. No teleoperation automatically starts.

A failed or interrupted operation is **not transactional**: partial register
changes or an unlocked EEPROM may remain. There is no automatic rollback.
The current monitor session blocks position commands and torque enable after
an uncertain restore; torque OFF and a fresh restore preview remain available.
Do not reconnect to bypass that interlock. Inspect the result and use a new
preview before an explicitly confirmed retry.

CLI preview and restore results are reprinted on exit for copying. Browser
results remain in the dedicated restore output until another restore result.
Export the inspection before repair if you need a retained record of prior
settings. After success, gently validate J6's physical endpoints and intermediate
readings with torque OFF before considering powered commands.

## CLI

```sh
# Local leader only; no follower URL or connection needed.
cf-robot-pose-compare --side leader --port /dev/tty.usbmodemYOUR_LEADER_DEVICE

# Remote follower only; uses CF_ROBOT_CONTROL_URL.
cf-robot-pose-compare --side follower

# Both (the default).
cf-robot-pose-compare --port /dev/tty.usbmodemYOUR_LEADER_DEVICE
```

The table uses spaced Leader/Follower/Delta columns, short `LT`/`FT` torque
headings and a unit note below the sample timestamps. Separators bracket the
header, readings and timestamps.

`INSPECT L` / `INSPECT F` shows the selected arm's full inspection record.
The latest explicitly inspected record for each arm is printed again after
`q` or Ctrl-C restores the terminal, so it stays available for copying even
if subsequent live redraws or commands replaced it.

To save a snapshot directly from the monitor:

```text
SAVE F ~/Desktop/follower-inspection.json
SAVE L "~/Desktop/leader inspection.json"
```

SAVE exports the selected arm's current calibration inspection as JSON and
also retains it for printing on exit. Parent directories must already exist;
existing files are never overwritten. Use another filename for a new export.
Neither INSPECT nor SAVE sends robot commands or changes calibration.

`CAPTURE L J1 0 2 measured-reference` checks the local shoulder-pan reading
against an independently measured zero, with a 2-degree tolerance; use `F` for
the follower. Issuing CAPTURE confirms the physical measurement. CLI captures
are displayed only, not retained as a multi-pose validation dataset; use the
browser for fitted analysis and JSON export. INSPECT, SAVE and CAPTURE do not
write robot settings; the separately confirmed J6 RESTORE operation does.

For terminal-based setting, use `lerobot-calibrate --teleop.type=so101_leader`
with the actual leader port/ID, or `cf-robot-follower-calibrate` for the Pi.
Stop monitoring first. The same physical-support requirements apply.

## J6-Only Live Following Test

Normal teleoperation sends all six targets. Moving only the leader gripper does
not isolate it: the other five follower joints can still move toward their
leader targets. Use the dedicated CLI mode instead, after updating both Mac and
Pi packages and restarting the Pi service:

```sh
.venv/bin/cf-robot-leader \
  --url "$CF_ROBOT_CONTROL_URL" \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader \
  --J6-only
```

Use the actual leader serial port and source `controller.env` first. No hosted
UI deployment is needed for this CLI mode. The regular browser teleoperation
button is not a gripper-only control.

1. Secure both arms in stable poses and keep the physical emergency stop
   accessible. Keep fingers/objects clear of both grippers and maintain a direct
   or live camera view of the follower. Nothing should be held by the gripper:
   stopping releases it.
2. With torque OFF, choose a safe leader starting opening, preferably around
   40-60%. Manual follower alignment is not required, but both encoder readings
   must remain inside their saved ranges. Use Arms Monitor if needed,
   then stop that monitor so the CLI can own the ports. On the follower, `J6 TD`
   explicitly disables J6 only. Do not release the other joints.
3. Start the command. It verifies J6's saved/live calibration on both arms,
   checks torque is already OFF, reports the initial percentages and waits for
   Enter. Clear the follower gripper's path before confirming: it will move.
   Hold the leader still while J6 gradually synchronizes to the leader opening
   captured after confirmation. No J1-J5 startup goals or torque changes are sent.
   After Enter the CLI shows `WAIT - syncing J6` and prefixes sync progress
   with `WAIT`. The Pi's hardware-ready status is not displayed as teleop readiness.
4. Live following begins only after fresh measured J6 readback is within
   1 percentage point of that fixed startup target. Then move the leader slowly
   through a small interior interval initially. The terminal prints leader/follower percent
   and delta at about 4 Hz. Check direction, settling and relative travel.
   The CLI displays `Ready for teleop` when live following begins, not before.
5. Stop with Ctrl-C. Follower J6 torque is disabled and its previous speed setting
   restored. Inspect before restarting; there is no automatic reconnect.

The Pi's authenticated `/control/gripper` route shares the normal control and
maintenance interlocks, but accepts only `gripper.pos`, strictly in 0-100%.
The CLI requires explicit J6-only acknowledgement; it never falls back to the
normal `/control` route on an older Pi. Both drivers use direct bus access and
avoid LeRobot's whole-arm configure/torque/disconnect methods.

Only J6 Goal_Position, Goal_Velocity and Torque_Enable may be written in this
mode. The first target seeds J6's goal from its current encoder position before
enabling its torque. Speed is capped at the monitor's reduced speed (or an
existing lower nonzero speed). J1-J5 calibration, goals and torque are untouched;
their remaining mismatches do not prevent the isolated test and are not
considered validated by it.

The 500 ms command watchdog and sequence/timestamp validation remain active.
During following, every applied goal is bounded to at most 5 percentage points
from the measured follower position. The requested leader position may be
farther ahead because the follower has a deliberately reduced speed. Such lag
does not immediately stop the test: only the latest request is used, with no
queued backlog. A leader reversal replaces the previous goal immediately.
Startup alignment checks remain unchanged.

While an applied goal is more than 1 percentage point away and the leader
continues requesting movement in that direction, the follower must make at
least 0.25 percentage points of measured progress within 1 second. Otherwise
the test stops with measured/requested/previous-goal percentages and the speed
register value. Check for obstruction or an unexpectedly low speed setting
before retrying; do not bypass the stop. Small encoder jitter does not count as
progress. Missing/stale follower readback also stops the CLI. These checks are
not a force limiter or a certified emergency stop. Stop immediately at unexpected
resistance or movement. Full-arm teleoperation must remain stopped until J1-J5
are repaired and physically validated separately.

J6 startup sync first sends the measured follower opening to preserve the Pi's
first-target alignment guard, then sends only J6 goals at most 2.5 percentage
points from measured position every 0.1 seconds. Keepalives preserve the command
watchdog while moving/settling. The leader startup target stays fixed throughout
sync, even if the leader is moved. Persistent stale feedback, cancellation, driver errors
or a 30-second timeout stop the session instead of entering live following.
For a brief startup feedback gap, after 0.5 seconds the CLI displays
`WAIT - feedback delayed; holding the last J6 sync goal` and repeats only the
last bounded goal to maintain the command watchdog. It neither issues a farther
goal nor enters live following using stale samples. Fresh feedback resumes
sync; a gap reaching 1 second stops the session. Live following retains its
0.5-second feedback stop. Repeated copies of the same hardware timestamp do not
reset these timers. Feedback errors report the ages of the last accepted sample
and last status packet, plus the last timestamp/validation result, to distinguish
missing packets from cached samples or clock-offset rejection. The Pi's command
watchdog and timestamp/sequence limits are unchanged.
`--no-startup-sync` opts out: in that case manually align J6 within 1 percentage
point before pressing Enter, or startup is rejected.
