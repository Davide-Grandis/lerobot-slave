# Inspect, Validate And Set Arm Calibration

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

Register inspection is a snapshot at monitor startup. Stop/restart monitoring
after any calibration or configuration change. Missing calibration can leave
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
validation. Calibration and live monitoring cannot run concurrently.

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
browser for fitted analysis and JSON export. All of these commands are read-only.

For terminal-based setting, use `lerobot-calibrate --teleop.type=so101_leader`
with the actual leader port/ID, or `cf-robot-follower-calibrate` for the Pi.
Stop monitoring first. The same physical-support requirements apply.
