# Read-Only Follower Servo Diagnostics

After updating the Mac package and Pi service to versions with diagnostic
support, stop teleoperation and start follower-only ARMS MONITOR:

```sh
cf-robot-pose-compare --side follower
```

Then request a snapshot for the selected joint:

```text
J2 STATUS
```

This uses the existing authenticated `/monitor` WSS session and its sole
servo-bus owner. J1-J6 are supported individually. The operation reads only a
fixed register list: current and goal positions, torque enable, goal/present
velocity, acceleration, load/current, voltage, temperature, status/moving,
torque limits, PID coefficients, dead zones, startup force, protection
settings, operating mode, homing offset and position limits.

No torque, goal, speed, PID, EEPROM or calibration writes are performed.
Existing torque is preserved when the monitor starts and exits. Support the
arm; diagnostics neither release nor re-arm it. Do not repeatedly retry a
stalled move or raise motor limits merely to obtain readings.

The response's `servo_diagnostics.registers` contains raw SDK register values,
not normalized degrees, percentages or inferred engineering units. Registers
that fail or are unavailable on a firmware version appear in `errors`, not as
zero readings. A successful snapshot response is not a claim that the servo is
healthy. Reads are sequential, not a simultaneous measurement.

The CLI displays the JSON snapshot and retains it for printing on exit. This
is a point-in-time diagnostic, not a movement test or an automatic correction.
After teleoperation stops, its hold-on-stop cleanup may already have replaced
the failed movement goal with a holding goal; do not treat that snapshot's goal
as evidence of the earlier command.

For protocol clients, send the normal monitor start request followed by:

```json
{
  "version": 1,
  "type": "command",
  "command_id": 1,
  "joint_id": 2,
  "mode": "diagnostics"
}
```

The server rejects extra fields, wildcard joints and arbitrary register names.
Monitor exclusivity and authentication are unchanged.
