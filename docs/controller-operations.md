# Controller-Laptop Operations

This guide summarizes what the controller laptop can do with the SO-ARM101
leader and the Raspberry Pi follower. It covers the CLI and WSS tools, not the
separate browser Operator UI.

## Prerequisites

- Complete the Mac installation in the [README](../README.md#leader-side-controller-laptop).
- Add the Cloudflare Access credentials, the Pi's `CF_ROBOT_ORIGIN_KEY`, and
  `CF_ROBOT_CONTROL_URL=wss://<your-public-hostname>/control` to
  `~/.config/cf-robot/controller.env`.
- In each Terminal session, load those settings with
  `source "$HOME/.config/cf-robot/controller.env"`.
- Use a clear workspace and physical emergency stop. Before teleoperation,
  align the leader and follower within 5 degrees on all six joints.

The follower USB path is stored on the Pi in `/etc/cf-robot/follower.env`.
Mac-side WSS commands use that Pi setting and do not ask for the follower's
USB serial path.

## Calibrate the Arms

### Leader on the Mac

The leader is physically connected to the Mac. Find its serial device with
`ls /dev/tty.usb*`, then calibrate it from the repository root:

```sh
.venv/bin/lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --teleop.id=so101_leader
```

Replace the port with the Mac's actual `/dev/tty.usb...` path. LeRobot stores
the leader calibration for the current macOS user. Use the same
`so101_leader` ID when starting teleoperation.

### Follower on the Pi

Follower calibration is LeRobot's interactive hardware calibration. It is
**not exposed as a WSS command**. Run it once from a Pi terminal, normally
over SSH from the controller laptop, or repeat it after servicing/replacing
the arm or losing its calibration:

```sh
sudo systemctl stop cf-robot-follower
/opt/cf-robot/scripts/calibrate-follower.sh
sudo systemctl start cf-robot-follower
```

The helper reads the follower port from `/etc/cf-robot/follower.env`, runs
LeRobot as the `robot` service user, and saves calibration as
`so101_follower`. Do not run calibration while a control session is active.
After the one-time WSS updater setup below, routine operations do not require
SSH; recalibration still does.

## Record or Set the Follower Parking Pose

These Mac commands use the authenticated WSS `/control` route. They require
the Pi service to be updated to a version that includes the measured pose in
its WSS `ready` response. Install the Mac entry points after updating the
checkout:

```sh
.venv/bin/python -m pip install -e . 'lerobot[feetech]==0.6.1'
```

Record the follower's current measured pose without moving it:

```sh
.venv/bin/cf-robot-parking-record
```

Review the displayed pose and type `RECORD`. The parking pose is saved locally
at `~/.config/cf-robot/follower-parking.json`.

Move the follower toward the saved pose:

```sh
.venv/bin/cf-robot-parking-set
```

Clear the workspace and type `MOVE` to confirm. The client sends small
one-servo-at-a-time targets over WSS; the Pi's per-step limit and watchdog
remain active. It does not move the leader. After parking, manually align the
leader before teleoperation.

## Teleoperate

Run from the repository root, in the Terminal with `controller.env` loaded.
The USB port below is the **Mac's leader port**, and the ID must match the
leader calibration:

```sh
.venv/bin/cf-robot-leader \
  --url wss://YOUR_PUBLIC_HOSTNAME/control \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader --fps 30
```

The `--fps` value is the maximum leader sample rate (default 30, valid 1-60).
The Pi refuses a target that differs by more than 5 degrees per joint from
the preceding accepted target. If that happens, inspect the joint-by-joint
diagnostic, stop, and realign; do not repeatedly retry without checking.

## Update the Pi over WSS

One-time setup requires SSH or a local Pi terminal: update the Pi checkout,
install the root-owned updater and `systemd` path unit, then enable the unit
as described in the Raspberry Pi section of the [README](../README.md#enable-wss-updates-one-time-ssh-setup).
The existing Cloudflare Access application protects the `/update` WSS route.

For subsequent updates, make sure no control or video session is active and
run on the Mac:

```sh
.venv/bin/cf-robot-update
```

This requests a fixed fast-forward update from `origin/main`, refreshes the
project dependencies, restarts the follower, and polls update status over the
same authenticated WSS route. The client cannot send arbitrary shell commands
or choose a Git branch.

## WSS Scope

The laptop can teleoperate, record/set the follower parking pose, and request
the fixed Pi update over authenticated WSS. The follower serial port remains
configured on the Pi. This is **not** a general-purpose remote shell: OS
package management, arbitrary `systemctl` operations, hardware calibration,
and initial updater installation are not exposed over WSS.
