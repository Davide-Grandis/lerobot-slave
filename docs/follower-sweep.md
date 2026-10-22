# Interactive Follower Joint Sweeps

`cf-robot-follower-sweep` tests J1 through J6 in order over the authenticated
`/monitor` WSS connection. It never opens the leader serial port.

This is a powered full-range diagnostic, not calibration, collision avoidance
or a force-limited test. Someone must physically support the arm throughout:
all joints except the one being tested are unpowered and can fall. Clear the
entire selected joint's path, cables and gripper jaws. Keep a physical stop or
servo power cutoff accessible. Never force a joint or repeatedly retry a stall.

## Installation And Startup

Update both packages first. The Pi must advertise `safe-monitor-moves-v1`;
the CLI refuses older services before any torque or movement commands.
Installing only the Mac command is insufficient.

After installing the updated package, stop teleoperation, ARMS MONITOR and the
browser Pose Monitor, then run:

```sh
source "$HOME/.config/cf-robot/controller.env"
.venv/bin/cf-robot-follower-sweep
```

The default order is J1, J2, J3, J4, J5, J6. To diagnose J3 without running the
other sweeps:

```sh
.venv/bin/cf-robot-follower-sweep --joint 3
```

Even single-joint mode releases all six joints only after confirmation.
`--url wss://<hostname>/control` overrides `CF_ROBOT_CONTROL_URL`.
`--joint` accepts 1-6; omitting it selects all six joints. There is no `--yes`
option, leader port, automatic reconnection or piped-input mode.

Before reinstalling the console entry point, the same routine is available as:

```sh
.venv/bin/python -m cf_robot.sweep_client --joint 3
```

## Confirmed Sequence

For each selected joint the CLI shows its verified raw MIN/MAX and physical
zero, then:

1. Ask you to support all joints. ENTER releases all torque, including any
   joint still holding from the previous test.
2. Let you manually position the unpowered links to clear the whole sweep.
   A second ENTER enables only the selected joint, seeds a holding goal first,
   moves to MIN, then sweeps to MAX.
3. Leave that joint powered at MAX. Ask for ENTER before returning to its
   independently calibrated zero.
4. Hold at zero until the next confirmed release. After the last joint, a final
   ENTER releases all torque and ends the session.

Every confirmation offers **S then ENTER to skip** (`s` also works):

- At the torque-release prompt, skip the entire joint without releasing torque
  or running its dependent movement tasks.
- At the MIN/MAX prompt, skip that sweep and proceed to the zero prompt. The
  joint stays unpowered unless you separately confirm enabling it for zero.
- At the zero prompt, skip the return-to-zero movement and advance.
- At the final release prompt, finish without changing the existing torque
  states; any powered joint continues holding.

Skipping never sends a movement or torque command for the skipped task. Use
`q` then ENTER or Ctrl-C to cancel instead of skipping.

J1-J5 return to 0 degrees using their verified independent zero ticks, not the
range midpoint. J6 returns to calibrated 0% at MIN, not 0 degrees; ensure closing
the gripper will not trap anything.

The CLI requires consistent saved/live calibration and verified angular zeros;
provisional ranges or unverified zeros block the test. MIN/MAX are servo limits,
not proof that every configuration is collision-free. A move is reported as
reached within the existing 1-degree/1-percentage-point tolerance, not exact
contact with a mechanical stop.

## Live Load Readback

During movement, progress reports include the tested joint's `Present_Load`
register (address 60), read by the existing hardware-owner thread. Periodic
reports are sampled approximately every 0.25 seconds, not on every bus cycle.
Initial and terminal reports also include load when available.

```text
J2: 61.88 -> 0.00 degrees | load=+520 raw (+52.0% est.)
```

The SDK already decodes the direction bit; negative readings remain negative.
The percent is the signed raw value divided by ten on the STS3215 scale. It is
an estimated controller load indicator, not calibrated torque in Nm or electrical
current. Current is a separate register. Short spikes between samples may be
missed; correlate load with current, voltage, temperature and fault status.

Failed reads appear as `load=unavailable`, never as zero. Both the Pi's progress
handler and the Mac client must include load telemetry; an older Pi that sends
no load fields also displays `load=unavailable`. Monitoring adds no parameter,
torque or goal writes and does not change protection thresholds.

## Speed And Failure Behavior

Moves use the existing bounded monitor steps of at most 2.5 degrees/percentage
points and a five-second settling deadline per step. A lower existing nonzero
speed is preserved; otherwise the raw speed cap is 102. This is a raw servo
setting, not a claim of a calibrated percentage of maximum speed.

The selected servo retains that reduced speed rather than restoring a higher
or zero/unrestricted setting. A failed move holds its fresh measured position
before reporting failure. Cancellation requests the same safe monitor stop.
No EEPROM, calibration, PID or torque-limit changes are made.

When a position move explicitly fails with a confirmed in-range measured hold,
the CLI obtains fresh readback and offers ENTER to retry the same target, S to
skip, or q to cancel. Each movement permits at most two explicitly confirmed
retries. Speed, torque, limits and PID are not increased. A reported nonzero
servo fault blocks retries. Inspect for obstruction, strain or heat before
confirming; a retry is not a diagnosis or a way to force a blocked mechanism.

Skipping a failed MIN/MAX movement skips the rest of that sweep and offers the
separately confirmed zero task. Skipping a failed zero advances to the next
joint's release confirmation.

Connection timeouts, uncertain outcomes, failed/unverified holds, rejected or
partial commands, and exhausted retries still stop the routine without replaying
commands or advancing. Ctrl-C does not automatically release the powered joint,
because that could drop the arm. Previously released joints remain unpowered.
Support the arm before explicitly changing torque afterward. Lost communication
or a failed hold may prevent software stopping from being verified; use the
physical stop when necessary.
