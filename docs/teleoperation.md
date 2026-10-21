# Shared Teleoperation Policy

Normal CLI, browser leader mode and joint-only calibration tests use the same
feedback, startup-sync and bounded-following implementations. A joint-only mode
selects fewer joints; it is not a separate motion algorithm.

| Layer | Shared implementation |
| --- | --- |
| Feedback validation, sample generations and diagnostics | `teleop_runtime.Feedback` |
| Sequential startup and measured settling | `teleop_runtime.StartupSync` / `synchronize` |
| Applied-goal windows and no-progress detection | `teleop_motion.BoundedMotion` |
| Saved/live calibration and direct-bus motor ownership | `teleop_hardware.TeleopLeader` / `TeleopFollower` |

## Startup

Normal operation requires consistent saved/live calibration on **all six joints
of both arms**. Joint-only tests require it only on the selected joints.
Invalid files, offsets, motor identity, operating modes or normalization modes
block connection before motor writes. J1-J5 use degrees; J6 uses percent.
Leader torque must already be OFF; physically support it before releasing torque
through an appropriate maintenance procedure. The full follower may retain its
existing torque state during read-only connection. J6-only testing requires J6
torque OFF on both arms.

The CLI shows current poses and asks **Press Enter to continue**. Browser users
connect the leader, select **Leader arm** and explicitly confirm the movement
dialog. After confirmation:

1. Show **WAIT** and capture the leader's startup pose.
2. Seed all selected follower goals from fresh measured follower positions.
   Goals are seeded before enabling any selected motor; no old goal is reused.
3. Synchronize one joint at a time using goals at most 2.5 degrees or percentage
   points from measured position every 0.1 seconds. Other selected joints keep
   their seeded or already-synchronized goals. Unselected joints are untouched.
4. Require a newer measured sample within 1 degree/percentage point before
   advancing to the next joint. Recheck every selected joint before completion.
5. Show **Ready for teleop** only after synchronization completes.

Keep the leader still and the workspace/jaws clear. The target snapshot stays
fixed during sync. There is a 30-second deadline per joint and a finite overall
deadline of 30 seconds times the selected-joint count. Cancellation, errors and
timeouts stop the session, never automatically resume.

`--no-startup-sync` skips the ramp, but requires manual alignment within 1 unit
before the CLI starts. It does not bypass motor calibration or motion limits.
Normal browser manual mode has no leader target to synchronize to: it starts
from the measured follower pose and retains bounded explicit jog commands.

## Feedback And Following

All modes reject malformed, stale, future-dated or repeated cached hardware
samples. Error messages include accepted-sample age, status-packet age and the
last validation result. Keep clocks synchronized: timestamp age includes network
delay and clock offset.

After a 0.5-second startup feedback gap, show **WAIT - feedback delayed** and
repeat only the last bounded goal, not a distant leader request. Resume sync
only with fresh feedback. A gap reaching 1 second stops. During live following
or browser manual operation, stale feedback stops control after 0.5 seconds.
The Pi's separate 0.5-second command watchdog, sequence and timestamp checks
remain active.

During normal and joint-only following, the latest requested leader target
replaces the previous request. Each applied goal is bounded within 5 degrees
(J1-J5) or percentage points (J6) of measured position; a slow but moving servo
does not fail just because the leader gets ahead. Goals outside the follower's
saved calibrated range are rejected. Existing lower nonzero speeds are
respected; otherwise the conservative raw speed cap is 102. These limits
are shared and are not guarantees of physical velocity or force.

### Manual Impulses And Catch-Up

A sudden manual leader movement is not an alignment fault during an armed
session, even when the follower is more than 5 units behind. For example, if the
leader jumps from 20% to 80% and then stays at 80%, the follower keeps approaching
80% with bounded goals. There is no one-second deadline to finish that travel:
catch-up may take many seconds while measured motion continues. A stationary
leader still sends fresh repeated targets; it is not a stopped control stream.

Only the newest leader position is followed. If an impulse is followed by a
different position, the controller does not replay an old queue of movements.
The 5-unit bound constrains each applied goal, not the total leader/follower gap.
This applies to normal CLI, browser leader mode and isolated joint tests.

When an outstanding goal remains more than 1 unit away, movement must make at
least 0.25 units of progress toward it within 1 second. Failure stops the session
with the joint name, units, requested/measured/previous-goal values and speeds.
Reversal replaces the goal immediately. These are not collision detection,
effort limits or certified emergency-stop functions.

## Stop And Deployment

Full-arm stop/disconnect attempts to hold each selected joint at its measured
position and leaves supporting motors powered. It does not disable the arm and
drop a load. If readback/hold cleanup fails, the last bounded goal and conservative
speed remain; inspect the Pi service logs and use the physical stop as needed.
Successful hold restores prior speed registers, not calibration/EEPROM settings.
Loss of power or communication can still defeat holding: secure the arm.

The isolated J6 test deliberately releases only J6 on stop, preserving its
documented empty-gripper test behavior. Other joint-only modes must use suitable
holding semantics for gravity-loaded axes. No stop operation enables additional
motors or automatically starts another session.

Update both Mac and Pi packages and restart their processes. Publish updated
static assets for the browser confirmation and WAIT/Ready messages.
Do not attempt full-arm operation until J1-J5 are repaired and independently
physically validated; testing J6 alone does not validate the arm.
