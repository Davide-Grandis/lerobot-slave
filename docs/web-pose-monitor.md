# Browser Pose Monitor

The Field Console has a **Pose Monitor** tab alongside teleoperation. It uses
the same `/monitor` protocol and servo-command implementation as
`cf-robot-pose-compare`; there is no second motion implementation in the web app.

## Start

Update the controller checkout/package and restart `cf-robot-web`. Update the
follower as well if it does not yet support interactive `/monitor` commands.
Deploy the updated static assets from `src/cf_robot/operator` to publish the tab.

For example, in the controller's existing configured environment:

```sh
cf-robot-web \
  --url https://YOUR-FOLLOWER-HOSTNAME \
  --port /dev/YOUR-LEADER-SERIAL-PORT \
  --arm-id so101_leader \
  --operator-origin https://cf-robot.davideslab.eu
```

Use the actual leader serial port and calibration ID. Omit `--port` for
follower-only monitoring. The follower hostname is the Pi's hostname, not the
Field Console hostname.

1. Open `https://cf-robot.davideslab.eu/` and pair with the one-run code printed
   by the local controller bridge.
2. Stop teleoperation and any CLI monitor using these arms.
3. Select **Pose Monitor** and start monitoring. Startup always preserves
   existing torque; change torque only with the explicit joint commands below.
   Switching tabs does not stop the session.

The public Field Console does not require an Access login. This utility does
not depend on the Worker's Access-identity or D1 audit endpoints. The exact
allowed browser origin and pairing code still protect every hosted-browser
request to the loopback bridge. Follower origin/Access credentials remain in
the controller process environment, never in browser code. Making the console
public does not change the separate Pi hostname's authentication requirements.

## Readings And Commands

The table shows all six joints, leader action values, measured follower values,
leader-minus-follower deltas, raw encoder counts, per-arm MIN/MAX limits, and
current per-joint torque states. J1-J5 values are degrees; J6 is normalized
0-100. Encoder readings and limits are raw ticks. Sample ages identify stale
readings; torque at startup is reported without changing it.

The follower command form supports:

- J1-J6: integer raw tick targets, 0-4095, further bounded by servo limits.
- J1-J5: degree targets.
- J6: normalized 0-100 targets.
- J1-J6: enable or disable torque on the selected follower joint only.

Confirm the workspace/support checkbox for each command. The Pi validates the
target, requires torque enabled for position moves, and applies its existing
bounded movement steps, speed limits, and settling checks. Enabling torque
first sets the goal to the joint's present position. The UI shows queue,
progress, and result messages and allows only one outstanding command.

## Stopping And Safety

**Stop monitor / command** sends a stop request that interrupts an active move
and ends sampling. It does not restore previous torque states. Disabling
torque can cause an arm to fall; stopping a monitor does not disable torque.
Keep physical support and a physical emergency stop available.

The paired browser sends heartbeats while monitoring. Lost heartbeats stop the
monitor; missing command results also stop it rather than retrying a possibly
executed command. A local leader read failure closes the remote session.
Serial-port ownership is retained until the local reader has disconnected.
Web teleoperation and the web pose monitor cannot run simultaneously; the Pi
also rejects competing follower sessions.

The existing software-stop endpoint also stops the pose monitor and latches
teleoperation until explicitly re-armed. It is not a certified emergency stop.
After a connection failure, inspect the physical arm before starting again.
No session or command automatically resumes.

## Regression Tests

Run the Python suite with the project's web/test dependencies installed:

```sh
PYTHONPATH=src python -m unittest discover -s tests -v
```

The optional headless Chrome smoke test uses only a simulated bridge, never
robot hardware. Set `CHROME_BIN` if Chrome is not at its default macOS path:

```sh
CF_ROBOT_BROWSER_TESTS=1 PYTHONPATH=src \
  python -m unittest discover -s tests -p 'test_web_pose_ui.py' -v
```
