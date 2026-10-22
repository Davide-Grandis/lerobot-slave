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

Each STATUS response is automatically saved to a uniquely named JSON file in
the Mac's temporary directory, for example `follower-j3-status-<unique>.json`.
The CLI displays the full path with its compact summary and repeats it on
exit. Previous captures are never overwritten. A file-write failure is shown
explicitly; the report remains available in memory.

The latest complete snapshot is also retained separately for each joint.
To export an already captured J3 snapshot to a chosen path on the Mac:

```text
SAVE J3 /tmp/follower-j3-status.json
```

Quote paths containing spaces. Export refuses to overwrite existing files and
sends no servo commands or new requests. Use `q` to stop the live display and
print all retained reports as stable, copyable JSON. New CLI display/export
behavior requires restarting the Mac monitor, not updating the Pi service.

This is a point-in-time diagnostic, not a movement test or an automatic correction.
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

## Protection Timer Readback

STATUS also reads `Protection_Time` and `Over_Current_Protection_Time`. In the
installed LeRobot STS3215 register map these are single-byte registers at
addresses 35 and 38 respectively, so their raw values must fit in 0..255.
Raw values 300 or 400 cannot be represented. For a firmware interpretation of
10 ms per unit, 255 corresponds to 2.55 seconds, not 3-4 seconds.

Do not confuse the servo's overload protection timer with a software movement
deadline or no-progress timeout. Delaying overload protection does not increase
the motor's rated torque and can increase heating and damage risk. Protection
timers are read only by diagnostics; no changes are made. Verify the exact
firmware documentation before considering any protection-setting adjustment.
