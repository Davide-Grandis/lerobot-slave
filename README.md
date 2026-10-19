# SO-ARM101 remote teleoperation over Cloudflare Tunnel

This project splits LeRobot teleoperation across two computers. The operator's
laptop reads an SO-ARM101 leader over USB; a Debian Raspberry Pi drives an
SO-ARM101 follower over USB and reads its USB camera. The laptop connects to
the Pi through a Cloudflare-protected public FQDN. Phase 1 is a CLI; phase 2
adds a Cloudflare-hosted operator app with D1 session records, live follower
video, measured servo readback, and both manual and leader-arm controls. A
loopback Python agent on the controller laptop retains the hardware and
credential boundary.

This is a thin integration around LeRobot hardware drivers, not a LeRobot
fork. Cloudflare Tunnel, Access, DNS, and credentials are configured manually;
this repository does not provision Cloudflare resources.

## Table of contents

- [Requirements](#requirements)
- [Architecture](#architecture)
- [Controller laptop operations](#controller-laptop-operations)
- [Before using hardware](#before-using-hardware)
- [Install and calibrate](#install-and-calibrate)
  - [Follower side: Raspberry Pi (Debian)](#follower-side-raspberry-pi-debian)
    - [Configure the Cloudflare Tunnel](#step-1-configure-the-cloudflare-tunnel)
    - [Install the code](#step-2-install-the-code)
    - [Install dependencies](#install-dependencies)
    - [Calibrate the follower arm](#calibrate-the-follower-arm)
    - [Configure and start the follower service](#configure-and-start-the-follower-service)
    - [Enable WSS updates (one-time SSH setup)](#enable-wss-updates-one-time-ssh-setup)
    - [Record and use the follower parking pose](#record-and-use-the-follower-parking-pose)
    - [Normal startup and restart](#normal-startup-and-restart)
    - [Upgrade the Pi from GitHub](#upgrade-the-pi-from-github)
  - [Cloudflare dashboard setup](#cloudflare-dashboard-setup)
    - [Map the local app to a public URL](#map-the-local-app-to-a-public-url)
    - [Protect the hostname with Access](#protect-the-hostname-with-access)
  - [Leader side: controller laptop](#leader-side-controller-laptop)
    - [Set up and calibrate the leader](#set-up-and-calibrate-the-leader)
    - [Update the controller laptop (macOS)](#update-the-controller-laptop-macos)
    - [CLI (phase 1)](#cli-phase-1)
    - [Remote follower parking commands (WSS)](#remote-follower-parking-commands-wss)
    - [Update the Pi over WSS](#update-the-pi-over-wss)
    - [Cloud-hosted operator app (phase 2)](#cloud-hosted-operator-app-phase-2)
    - [Configuration reference](#configuration-reference)
- [Operating runbook](#operating-runbook)
  - [Troubleshooting](#troubleshooting)
- [Tests and limitations](#tests-and-limitations)

## Requirements

| Area | Requirement |
| --- | --- |
| Controller laptop | This package and serial-device access; an SO-ARM101 leader attached by USB is required for leader-stream mode and optional for manual-only browser control. |
| Raspberry Pi | Headless 64-bit Debian (AArch64); SO-ARM101 follower and UVC-compatible camera attached by USB; serial and video-device access. |
| Both hosts | This package, compatible LeRobot/Feetech dependencies, separately calibrated arms, and synchronized clocks (NTP). Use the same tested LeRobot release on both hosts. |
| Cloudflare | An account with an active zone/FQDN, a named Tunnel with a `cloudflared` connector on the Pi, an Access self-hosted application and service token, a Worker protected by Access, and a D1 database for operator records. |

The package and pinned LeRobot release require Python `>=3.12,<3.14`;
Python 3.14 is not supported. The Raspberry Pi instructions use Python 3.13.

## Architecture

See [the architecture, protocol, and trust-boundary notes](docs/architecture.md).

## Controller laptop operations

See the [controller-laptop operations guide](docs/controller-operations.md)
for arm calibration, parking-pose commands, teleoperation, and Pi updates
over WSS.

## Before using hardware

- Calibrate each arm you will use with LeRobot on the machine and under the
  user account that will run it. Use the same calibration ID when starting it.
  The CLI's default startup sync can gradually move the follower to the
  leader's initial measured pose after you press Enter to confirm. Keep the
  workspace clear and hold the leader still during this move. The per-target
  5-unit limit remains in force; without startup sync, align the arms within
  **5 units on each joint** before leader streaming. Manual mode starts from
  the follower's measured pose and sends bounded jogs.
- Synchronize the laptop and Pi clocks with NTP. Commands more than 500 ms old
  (or over 250 ms in the future) are rejected. If latency or jitter approaches
  this limit, do not operate the robot through this path.
- A lost connection or 500 ms gap after the first command ends the session.
  LeRobot's `disconnect()` behavior, including whether torque is released,
  depends on the hardware/driver. Verify the arm cannot fall or strike anything
  on disconnect or Pi reboot. Do not rely on software to hold a load.
- Only one control client and one video client are allowed at a time. The
  follower never automatically resumes movement after a failure or reboot.
- The hosted operator page requires a live follower camera before connecting
  the control session; loss of fresh camera frames stops browser control.

## Install and calibrate

### Follower side: Raspberry Pi (Debian)

These instructions assume a Raspberry Pi 4 with 4 GB RAM and 64-bit
Debian (AArch64). The dependency commands below use Python 3.13; check
that it is available on your Pi before continuing.

#### Step 1: Configure the Cloudflare Tunnel

In the Cloudflare dashboard, create a named Tunnel for the Pi and follow
the dashboard-provided **Debian installation instructions** to install
`cloudflared` as a service on the headless Pi.
Confirm the connector is connected and the service starts automatically
after a Pi reboot. 

#### Step 2: Install the code

Clone the project into **`/opt/cf-robot`**, the path expected by the
`systemd` unit:

```sh
sudo git clone https://github.com/Davide-Grandis/lerobot-slave.git /opt/cf-robot
```

Note: for upgrades see the procedure covered below.

- `/opt/cf-robot`: application code.
- `/opt/cf-robot/venv`: Python venv.
- `/etc/cf-robot/follower.env`: local configuration and origin key.
- `/var/lib/cf-robot`: service user's persistent calibration data.


#### Install dependencies

Check the installed Python version and venv package:

```sh
python3 --version
dpkg -s python3-venv
```

If `python3` is missing or `dpkg` reports that `python3-venv` is not
installed, install both Debian packages:

```sh
sudo apt update
sudo apt install python3 python3-venv
```

Confirm that `python3 --version` reports Python 3.13 for these Pi
installation commands (the project supports Python `>=3.12,<3.14`).
Then create the service user and venv **on the Pi**:

```sh
sudo useradd --system --user-group --create-home --home-dir /var/lib/cf-robot \
  --groups dialout,video robot
sudo chown -R robot:robot /var/lib/cf-robot
sudo python3 -m venv /opt/cf-robot/venv
```

Install the project's pinned PyTorch and torchvision packages first.
Use `/var/tmp` for pip's temporary files because `/tmp` may be small
on the Pi:

```sh
sudo env TMPDIR=/var/tmp /opt/cf-robot/venv/bin/python -m pip install --no-cache-dir \
  --index-url https://download.pytorch.org/whl/cpu \
  'torch==2.7.1+cpu' 'torchvision==0.22.1'
```

Install this project and LeRobot's Feetech support, then check that the
installed package requirements are consistent:

```sh
sudo env TMPDIR=/var/tmp /opt/cf-robot/venv/bin/python -m pip install --no-cache-dir \
  -e /opt/cf-robot 'lerobot[feetech]==0.6.1'
/opt/cf-robot/venv/bin/python -m pip check
```

#### Calibrate the follower arm

1. **Retrieve the USB serial device ID.** Connect the follower arm's USB
   cable to the Pi. With no control session running, list the stable
   serial-device names:

   ```sh
   ls -l /dev/serial/by-id/
   ```

   An entry may look like
   `usb-FTDI_FT232R_USB_UART_A1B2-if00-port0 -> ../../ttyUSB0`.
   The part **before `->`** is the device ID.
   If the directory is missing or empty, check
   `ls -l /dev/ttyUSB* /dev/ttyACM*` and resolve the missing device
   before calibrating.

2. **Choose the follower's full port path.** Prefix its device ID with
   `/dev/serial/by-id/`. For the example above the path would be
   `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A1B2-if00-port0`.
   A camera device under `/dev/v4l/by-id/` is not the arm's serial port.

3. **Calibrate using that path.** Ensure no control session is active, then
   run the helper from your normal SSH account (do not prefix the script
   with `sudo`; it uses `sudo` internally):

   ```sh
   /opt/cf-robot/scripts/calibrate-follower.sh
   ```

   Enter the full `/dev/serial/by-id/...` path when prompted. The helper
   verifies the path and `robot` user's read/write access, then launches
   LeRobot's interactive calibration with `so101_follower` as the calibration
   ID.

When calibration finishes, LeRobot saves it automatically under
`/var/lib/cf-robot/lerobot/calibration/robots/so_follower/so101_follower.json`.
Record that file path in your setup notes if you need it for a backup; do not move the file. Verify that it exists on the Pi:

```sh
sudo -u robot test -s /var/lib/cf-robot/lerobot/calibration/robots/so_follower/so101_follower.json \
  && echo 'Follower calibration found'
```

LeRobot stores the follower calibration under `/var/lib/cf-robot`; both the
calibration command and service use this location. Calibration is a one-time
setup step performed over SSH. The service uses the saved calibration and
does not prompt for calibration at boot or when a client connects. The camera
needs its device path in the service configuration, but no arm-calibration ID.

#### Configure and start the follower service

1. **Find the camera path on the Pi.** With the camera connected, run:

   ```sh
   ls -l /dev/v4l/by-id/
   ```

   One USB camera may appear twice, with names ending in `-video-index0`
   and `-video-index1`. These are two interfaces of the same camera, not
   two cameras. Usually `-video-index0` is the image capture interface,
   while `-video-index1` is a secondary/metadata interface. Start with
   the full `-video-index0` path for `CAMERA_DEVICE`, rather than
   `/dev/video0`, whose number may change. For example, for a Sonix
   USB2.0 CAM1:

   ```text
   CAMERA_DEVICE=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0
   ```

   Use the name reported on your Pi. If capture fails, verify which
   interface supports video capture rather than blindly switching to
   `-video-index1`. The camera path is separate from the follower's
   `/dev/serial/by-id/...` path.

2. **Generate an origin key on the Pi.** Run `openssl rand -hex 32`.
   Keep the output private; you will need the same key on the laptop.

3. **Create and edit the Pi configuration.** The copy command will not
   overwrite an existing `/etc/cf-robot/follower.env`:

   ```sh
   sudo install -d -m 0750 /etc/cf-robot
   sudo cp -n /opt/cf-robot/deploy/follower.env.example /etc/cf-robot/follower.env
   sudoedit /etc/cf-robot/follower.env
   ```

   Set all four lines in the editor, then **save and exit**:

   ```text
   FOLLOWER_PORT=/dev/serial/by-id/YOUR_FOLLOWER_DEVICE
   FOLLOWER_ID=so101_follower
   CAMERA_DEVICE=/dev/v4l/by-id/YOUR_CAMERA_DEVICE
   CF_ROBOT_ORIGIN_KEY=YOUR_GENERATED_KEY
   ```

   Replace `YOUR_FOLLOWER_DEVICE` with the actual device ID used for
   calibration, `YOUR_CAMERA_DEVICE` with the camera device ID from step 1,
   and `YOUR_GENERATED_KEY` with the output from step 2. 

4. **Protect the file and start the service.** Only do this after saving
   the real paths and key:

   ```sh
   sudo chmod 0640 /etc/cf-robot/follower.env
   sudo cp /opt/cf-robot/deploy/cf-robot-follower.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now cf-robot-follower
   sudo systemctl status cf-robot-follower
   curl http://127.0.0.1:8765/health
   ```

Keep the file readable only by administrators; do not commit it. The included
unit expects `/opt/cf-robot/venv`, runs as `robot`, binds
`127.0.0.1:8765`, starts on reboot, and restarts on process failure. It
does not connect to or move the arm until a control session is opened.
`ExecStart` points directly to the executable inside the venv, whose
interpreter and dependencies are selected automatically. **No `activate`
command, shell login, or Pi-side action is needed at boot or for normal
teleoperation.**

#### Enable WSS updates (one-time SSH setup)

The follower service can accept a fixed update request at `/update` over the
existing Cloudflare-protected WSS hostname. The request accepts no shell
commands, branch names, or file paths. A root-owned systemd updater performs
a fast-forward-only update from the configured `origin/main`, refreshes the
project dependencies, and restarts the follower. It refuses updates if the
checkout has local changes or a control/video session is active.

During your one permitted SSH setup, first update `/opt/cf-robot` to the
release containing this feature using the Pi upgrade procedure below. Then
install and enable the root-owned updater and its path watcher:

```sh
sudo install -o root -g root -m 0755 /opt/cf-robot/deploy/cf-robot-update.sh /usr/local/sbin/cf-robot-update
sudo install -o root -g root -m 0644 /opt/cf-robot/deploy/cf-robot-update.service /etc/systemd/system/
sudo install -o root -g root -m 0644 /opt/cf-robot/deploy/cf-robot-update.path /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now cf-robot-update.path
```

This is a one-time privileged setup. The existing Access application protects
the entire hostname, including `/update`, so no additional Tunnel route is
needed. After this setup, ordinary code and dependency updates can be
requested from the Mac over WSS without SSH.

Check `journalctl -u cf-robot-follower -f` for failures. Confirm both this
service and the separately installed `cloudflared` service are enabled and
healthy after a Pi reboot. `GET /health` reports process availability and
whether a control session is occupied; it does **not** test the arm or camera.
If the GitHub repository is private, configure read-only SSH deploy-key
access for the account performing the clone and use its SSH URL instead.
Never put a personal access token in a clone URL, shell history, or service
unit.

#### Record and use the follower parking pose

These helpers record the follower's current joint positions as a parking pose
and later move the follower back to it. They do not move the leader. The saved
pose is `/var/lib/cf-robot/parking/so101_follower.json`.

Use an unloaded arm, clear workspace, and physical emergency stop. The parking
move follows a straight interpolation in joint space; it does not detect
obstacles or guarantee a collision-free path. The helper disconnects when
finished, so do not rely on the servos to hold a load or keep the arm parked.

To record a pose, manually place the follower in a safe parking position.
The Pi helpers read the serial path already saved in
`/etc/cf-robot/follower.env`, so they do not prompt for the USB device ID.
Stop the follower service, then run the recorder from your normal SSH account:

```sh
sudo systemctl stop cf-robot-follower
/opt/cf-robot/scripts/parking_record
```

Check the displayed joint positions, then type `RECORD` to save them. The
helper will refuse to run while the follower service is active. Restart the
service after recording:

```sh
sudo systemctl start cf-robot-follower
```

To move the follower to the saved pose later, first stop the service again
and run the setter:

```sh
sudo systemctl stop cf-robot-follower
/opt/cf-robot/scripts/parking_set
```

Review the current and saved positions, clear the workspace, then type `MOVE`
to confirm. The script moves one servo at a time in steps of at most 0.1
degree, with a 0.1-second pause between steps (about 1 degree per second).
It moves shoulder, elbow, and gripper joints first, then wrist roll and wrist
flex last.
This creates a smoother, slower ramp, but the intermediate poses can still
collide with the workspace. After it finishes, place the leader manually at
the same pose before starting teleoperation, then restart the follower service:

```sh
sudo systemctl start cf-robot-follower
```

#### Normal startup and restart

After the one-time installation, simply power on the Pi: `systemd` starts
the follower service and the separately installed `cloudflared` service.
The operator then starts a control session from the laptop. No venv
activation, monitor, desktop session, Pi login, or Pi command is required.
The follower unit has `Restart=always`, so `systemd` also restarts its
process after an unexpected exit. LeRobot calibration must already exist
for the `robot` user: service startup and control sessions never prompt for
interactive input. The camera uses headless OpenCV.

The service becomes available after boot, but **it does not reconnect a
leader session or issue motion commands automatically**. The operator
must initiate a new session from the laptop and check arm alignment.
If a deliberate service restart is needed for maintenance, first stop any
control session and make the arm safe, then run:

```sh
sudo systemctl restart cf-robot-follower
```

Use `systemctl restart`, not `systemctl reload`: this unit has no
`ExecReload`. The service restart does not automatically resume arm motion.
`Restart=always` covers process exits, not a process stuck indefinitely
inside a hardware call; a hardware watchdog would require additional
implementation and validation. Missing calibration or persistent hardware
faults also cannot be repaired automatically by restarting.

#### Upgrade the Pi from GitHub

Schedule upgrades while the follower is clear and **no control session is
active**. For a code-only change (no new Python dependencies or changes to
the systemd unit), the editable installation already points to the Git
checkout. Stop the service, then fast-forward the Pi's checkout:

```sh
sudo systemctl stop cf-robot-follower &&
sudo git -C /opt/cf-robot pull --ff-only
```

If dependencies or package entry points change, repeat the Pi dependency
installation commands above **before starting** the service. Only if the
service unit changes, run
`sudo cp /opt/cf-robot/deploy/cf-robot-follower.service /etc/systemd/system/`
and `sudo systemctl daemon-reload` before starting. Do not repeat those
steps for an ordinary code-only update. If any update step fails, leave
the service stopped until it is resolved.

Only after the update completes successfully, start and verify the service:

```sh
sudo systemctl start cf-robot-follower
sudo systemctl status cf-robot-follower
curl http://127.0.0.1:8765/health; echo
```

Stop if `git pull` or dependency installation fails; resolve the problem
before restarting and resuming operation. Keep the laptop installation on
the same tested revision; reinstall there only if dependencies or entry
points change.
The environment file at `/etc/cf-robot/follower.env` and the follower's
calibration under `/var/lib/cf-robot` are outside the Git checkout: **do
not overwrite them with `deploy/follower.env.example` during upgrades**.
Check release changes for new configuration or calibration requirements
before re-enabling control. `--ff-only` refuses a non-fast-forward update
instead of discarding local changes.

### Cloudflare dashboard setup

#### Map the local app to a public URL

In the Cloudflare dashboard, select the Pi's Tunnel and add a public hostname
(published application route).
Choose an available subdomain and a domain in your Cloudflare zone, for example `robot.example.com`. Set the service type to **HTTP** and its origin URL to **`http://127.0.0.1:8765`**. This address is the follower service on the **Pi**. 

The controller laptop will use `https://robot.example.com` for health checks or the web
console and `wss://robot.example.com/control` for CLI control; video uses
`wss://robot.example.com/video`. 

#### Protect the hostname with Access

Configure Access for the entire hostname before operating the arm:

1. Create a Cloudflare Access **self-hosted application** covering the whole
   hostname, including `/health`, `/control`, `/video`, and `/update`. Create a
   service token and a **Service Auth** policy allowing only that token.

2. Copy the token's client ID and secret for entry into the laptop's private
   controller config file. The secret is shown once; never put it in a URL,
   shell history, browser JavaScript, or the Pi environment file. The
   laptop also needs `CF_ROBOT_ORIGIN_KEY`; do not put any of these values
   in browser JavaScript.
3. Verify the public hostname rejects a request without Access credentials,
   then verify `https://robot.example.com/health` succeeds when a client
   supplies the two Access headers. A successful health response does not
   verify the WebSocket origin key or USB devices.

Cloudflare documentation: [Tunnel public hostnames](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/get-started/create-remote-tunnel/),
[WebSockets](https://developers.cloudflare.com/network/websockets/), and
[Access service tokens](https://developers.cloudflare.com/cloudflare-one/access-controls/service-credentials/service-tokens/).

### Leader side: controller laptop

#### Set up and calibrate the leader

The suggested folder location is `~/Developer/lerobot-slave`. Run the
commands below from its root.

This project supports Python 3.12 and 3.13; the commands here use Python
3.13. A virtual environment uses the interpreter that creates it, so do not
use `python3 -m venv` if `python3 --version` reports 3.14. Check both:

```sh
python3 --version
python3.13 --version
```

If `python3.13` is not installed, install it separately. For Homebrew:

```sh
brew install python@3.13
```

This installs a Python 3.13 interpreter; it does not change the macOS
`python3` default. You will use `python3.13 -m venv .venv` later to create the
project environment with the supported interpreter.

Create a folder for the project and clone this repo:

```sh
mkdir -p "$HOME/Developer"
git clone https://github.com/Davide-Grandis/lerobot-slave.git "$HOME/Developer/lerobot-arms"
cd "$HOME/Developer/lerobot-arms"
```

Find the leader's serial device on the Mac with the arm connected:

```sh
ls /dev/tty.usb*
```

Use the full `/dev/tty.usb...` path shown. If several adapters are
connected, unplug the leader's USB cable, run the command again, reconnect
it, and run the command once more; the newly appearing `tty` path is the
leader. Use this same path for calibration and both controller commands below.

Create a virtual environment with Python 3.13, confirm its version, and
install the project with LeRobot and Feetech support:

```sh
python3.13 -m venv .venv
.venv/bin/python --version
.venv/bin/python -m pip install -e . 'lerobot[feetech]==0.6.1'
```

Run LeRobot's interactive calibration for the SO-ARM101 leader:

```sh
.venv/bin/lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --teleop.id=so101_leader
```

Replace `/dev/tty.usbmodemYOUR_LEADER_DEVICE` with the path found above. The
`so101_leader` value passed to `--teleop.id` is the calibration ID; will use the same later as   `--arm-id`. Calibration is stored by LeRobot.

#### Update the controller laptop (macOS)

Run these commands from the Mac checkout whenever you need the latest
controller code. `git pull --ff-only` fast-forwards without overwriting local
changes; the editable install makes source-code-only changes available
immediately:

```sh
cd "$HOME/Developer/lerobot-arms"
git pull --ff-only
```

After changes to `pyproject.toml` dependencies or console-script entry points,
refresh the editable install as well. This creates new commands such as
`.venv/bin/cf-robot-update`; do not recreate the virtual environment:

```sh
.venv/bin/python -m pip install -e . 'lerobot[feetech]==0.6.1'
```

Verify that the WSS update and parking commands are installed:

```sh
ls .venv/bin/cf-robot-update \
  .venv/bin/cf-robot-parking-record \
  .venv/bin/cf-robot-parking-set
```

#### CLI (phase 1)

The SO-ARM101 leader arm is connected directly, via USB, to the controller laptop.

Keep these credentials in a private configuration file outside the
repository. The controller reads them from its process environment, so load
the file in the same Terminal session before starting the CLI or web console.

1. Retrieve the `CF_ROBOT_ORIGIN_KEY` key that was generated on the Pi.

2. In a Mac Terminal, create a private config directory and open a file in
   the editor:

   ```sh
   mkdir -p "$HOME/.config/cf-robot"
   chmod 700 "$HOME/.config/cf-robot"
   umask 077
   nano "$HOME/.config/cf-robot/controller.env"
   ```

   Put these four lines in the file, replacing each placeholder with its
   value. Keep the single quotes:

   ```sh
   export CF_ACCESS_CLIENT_ID='YOUR_CLOUDFLARE_CLIENT_ID'
   export CF_ACCESS_CLIENT_SECRET='YOUR_CLOUDFLARE_CLIENT_SECRET'
   export CF_ROBOT_ORIGIN_KEY='YOUR_PI_ORIGIN_KEY'
   export CF_ROBOT_CONTROL_URL='wss://YOUR_PUBLIC_HOSTNAME/control'
   ```

3. Set the file permissions:

   ```sh
   chmod 600 "$HOME/.config/cf-robot/controller.env"
   ```

4. Before each controller session, load the file and verify that the
   credentials and WSS URL are present without printing them:

   ```sh
   source "$HOME/.config/cf-robot/controller.env"
   test -n "$CF_ACCESS_CLIENT_ID" && test -n "$CF_ACCESS_CLIENT_SECRET" \
     && test -n "$CF_ROBOT_ORIGIN_KEY" && test -n "$CF_ROBOT_CONTROL_URL" \
     && echo "Controller settings loaded"
   ```

Before using this controller CLI, complete the leader setup above, including
the one-time leader calibration with `--teleop.id=so101_leader`. This CLI
connects using that saved calibration; it does not calibrate the leader.

- Run this from the repository root in the Terminal where you loaded the
  credentials; `.venv/bin/cf-robot-leader` is installed in that checkout.
- Replace `robot.example.com` with the public hostname configured for the
  Pi's Tunnel, followed by `/control`.
- Replace `/dev/tty.usbmodemYOUR_LEADER_DEVICE` with the Mac leader's full
  serial path found in the leader setup above. It must be the same port used
  when calibrating the leader, not the Pi follower's port.
- `so101_leader` is the leader calibration ID from `--teleop.id` in the
  leader setup above; keep it identical.
- `--fps 30` sends leader samples at up to 30 frames per second. It is
  optional (30 is the default); valid values are 1 through 60.

Then run from the repository root:

```sh
.venv/bin/cf-robot-leader \
  --url wss://robot.example.com/control \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader --fps 30
```

Press Ctrl-C to stop. The CLI does not silently reconnect after a dropped
session: inspect both arms, then start a new session. Startup sync will offer
to move the follower to the leader's initial pose if needed.

#### Remote follower parking commands (WSS)

These Mac commands use the authenticated `/control` WebSocket. Set
`CF_ROBOT_CONTROL_URL` in `~/.config/cf-robot/controller.env` to
`wss://<your-public-hostname>/control`, then source that file in the current
Terminal as described above. They use the same Access credentials and origin
key as teleoperation.

The follower's USB path is stored on the Pi in
`/etc/cf-robot/follower.env`; it is not needed on the Mac. The recorded
parking pose for these Mac commands is saved locally at
`~/.config/cf-robot/follower-parking.json`. The Pi-local `parking_record`
and `parking_set` scripts instead use `/var/lib/cf-robot/parking/`.

The Pi service must also be updated to a version that includes the measured
pose in its WSS `ready` message. First, update the Mac checkout from its root:

```sh
git pull --ff-only
```

Then refresh the editable installation to create the new command entry points:

```sh
.venv/bin/python -m pip install -e . 'lerobot[feetech]==0.6.1'
```

To record the follower's current measured pose without moving it, run:

```sh
.venv/bin/cf-robot-parking-record
```

Review the displayed pose and type `RECORD` to save it locally. This opens a
follower control session, so do not run it during teleoperation.

To move the follower toward the saved pose, run:

```sh
.venv/bin/cf-robot-parking-set
```

Clear the workspace and type `MOVE` when prompted. It sends the move as
small, one-servo-at-a-time action updates over WSS, with the wrist joints
last; the server's per-step limit and control watchdog remain active. This
command does not move the leader. Afterward, manually align the leader to the
follower before starting teleoperation.

#### Update the Pi over WSS

After the one-time updater setup on the Pi, source `controller.env` in this
Mac Terminal and run:

```sh
.venv/bin/cf-robot-update
```

The client requests the fixed `origin/main` fast-forward update and polls its
status over the same WSS tunnel. It accepts no arbitrary shell commands or
refs. Stop teleoperation and close the camera stream first; the Pi rejects
updates while either session is active.
If an update fails, the status reports the failed stage; local checkout
changes are refused.

The root-owned systemd path watcher triggers the updater. It updates only the
configured GitHub repository's `main` branch, refreshes dependencies, and
restarts the follower. The Mac does not need the Pi's follower USB path; that
remains in the Pi's `FOLLOWER_PORT` setting.

#### Cloud-hosted operator app (phase 2)

The operator UI is deployed as Cloudflare Worker Static Assets and protected
by Cloudflare Access. D1 stores the robot display record and low-volume
session/audit metadata; it is not used for video or per-frame commands.
The browser pairs with the controller laptop's loopback agent, which retains
LeRobot, USB access, Access credentials, and the direct WSS path to the Pi.
Follow [the operator app setup and runbook](docs/operator-app.md) to create
the Worker/D1 resources, protect the app, deploy it, pair the browser, and
start control.

### Configuration reference

| Setting | Where | Meaning |
| --- | --- | --- |
| `FOLLOWER_PORT` | Pi `/etc/cf-robot/follower.env` | Follower USB serial path passed to `--port`; prefer `/dev/serial/by-id/...`. |
| `FOLLOWER_ID` | Pi environment file | Follower LeRobot calibration ID passed to `--arm-id`. |
| `CAMERA_DEVICE` | Pi environment file | Camera path passed to `--camera`; prefer `/dev/v4l/by-id/...`. |
| `CF_ROBOT_ORIGIN_KEY` | Pi environment file **and** laptop environment | Same random key on both ends (minimum 32 characters); sent only during the control/video WebSocket handshakes. |
| `CF_ACCESS_CLIENT_ID`, `CF_ACCESS_CLIENT_SECRET` | Laptop environment only | Access service token used for public `wss://` requests; never place these on the Pi or in browser code. |
| `CF_ROBOT_OPERATOR_ORIGIN` | Laptop environment or `cf-robot-web --operator-origin` | Exact HTTPS origin allowed to pair with the loopback controller agent. |
| `CF_ROBOT_BRIDGE_TOKEN` | Optional laptop environment | Pairing code for the hosted UI; if unset, a one-run code is generated and printed locally. Never put it in D1 or a URL. |
| `CF_ROBOT_CONTROL_URL` | Laptop `controller.env` | Public WSS control URL used by remote parking commands, for example `wss://robot.example.com/control`. |
| `HF_HOME` | Pi service unit and calibration command | LeRobot/Hugging Face data directory for the `robot` user. |
| `--listen-port` | Pi follower CLI | Loopback HTTP/WebSocket port, default `8765`; update Tunnel origin if changed. |
| `--camera-width`, `--camera-height`, `--camera-fps` | Pi follower CLI | Video capture defaults `640`, `480`, `10`; fps must be 1-30. Update the unit if changing them. |
| `--fps` | Laptop leader CLI or web CLI | Maximum leader action sampling rate, default `30`, accepted range 1-60; not a guaranteed end-to-end rate. |
| `--listen-port` | Laptop web CLI | Loopback controller API port, default `8080`. |
| `--operator-origin` | Laptop web CLI | Exact HTTPS origin of the Cloudflare-hosted operator app. |

The Pi offers `GET /health`, `WS /control`, `WS /video`, and `WS /update` behind the same
Access hostname. The laptop agent listens only on loopback and exposes the
operator UI plus status, video-frame, start/stop, emergency-stop, mode, leader,
manual-jog, and heartbeat APIs. The hosted page polls measured state and the
latest JPEG frame. A missing browser heartbeat stops an active control session
after approximately three seconds; stale video also stops control.

## Operating runbook

1. Check the follower's physical emergency stop, workspace, camera view, and
   any load the arm could drop if torque is lost. Have someone able to stop
   the follower locally. Do not begin with an unobserved arm.
2. From the laptop, confirm `/health` is reachable through Access and, when
   using the operator app, pair the local agent and verify the live camera.
   Check that laptop/Pi clocks are synchronized. The
   protected health request verifies the service/Tunnel path, not that the
   USB arm or camera is ready. Pi login and `systemctl` commands are only
   needed for initial setup or troubleshooting, not each startup.
3. Keep the follower's workspace clear and verify the live camera, then start
   **one** control session. In the Mac leader CLI, startup sync is enabled by
   default: if the initial poses differ by more than 1 unit on any joint, it
   shows the measured poses and waits for Enter before moving the follower
   toward the leader. Hold the leader still and watch the follower until sync
   completes. Without sync (`--no-startup-sync`), align within 5 units on every
   joint before streaming. Manual mode starts from the measured follower pose.
   The Pi connects to the follower on demand; connecting alone sends no motion
   target.
4. Move the leader slowly or jog one follower joint at a time while watching
   both the measured pose and camera. If the session reports an error, stalls,
   or the follower behaves unexpectedly, use the physical stop first. In
   normal operation press Stop or Ctrl-C; verify the follower is stationary
   before leaving.
5. After a timeout, disconnect, Pi reboot, or rejected joint jump, inspect
   both arms and start a **new** session. The CLI can synchronize the initial
   follower pose after confirmation; it does not automatically reconnect or
   replay previous commands.

When the control WebSocket closes, the follower session disconnects. After
the first command, a 500 ms gap also ends it; the Pi does not continue
streaming the last target. A browser heartbeat failure requests laptop-side
shutdown after about three seconds, while the Pi's shorter command watchdog
still applies if action traffic stops. Neither timing mechanism is a
certified emergency stop, and a blocking USB call or driver behavior can
delay software shutdown.

### Troubleshooting

| Symptom | Check |
| --- | --- |
| `cf-robot-follower` fails at startup | `journalctl -u cf-robot-follower -f`, the unit's `/opt/cf-robot/venv` path, `/etc/cf-robot/follower.env`, and a 32+ character origin key. |
| Follower fails when control starts | Correct `FOLLOWER_PORT`, LeRobot calibration under `robot`/`HF_HOME`, serial permissions, and whether another process owns the USB device. |
| Camera shows unavailable | `CAMERA_DEVICE`, `video` group, whether another camera client holds it, and Pi logs. Video is limited to one client. |
| Access returns 403 or login HTML | Service Auth policy, hostname coverage, service-token ID/secret, expiry, and the public `wss://` URL. |
| WebSocket upgrade fails but health works | Tunnel origin must be HTTP on `127.0.0.1:8765`; check Access protection, the matching origin key, and WebSocket support at the zone. |
| TLS certificate verification fails on macOS with WARP connected | Switch the WARP profile to DNS-only mode and retry; the WARP traffic proxy can interfere with TLS verification. |
| `stale command or unsynchronized clocks` | NTP on both hosts and actual network delay/jitter; do not loosen freshness checks casually. |
| `joint jump exceeds limit` | The error lists each offending joint, its prior reference, current leader command, and difference in servo units. The CLI offers a confirmed startup sync for initial mismatch; during motion, reduce per-sample movement and retry. |
| Web control stops when tab is hidden or closed | The page must continue sending heartbeats and receiving camera frames. Reopen it, inspect the follower, re-align if using the leader, and restart. |

## Tests and limitations

Run unit tests from the repository root:

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -v
```

`tests/test_protocol.py`, `test_auth.py`, and `test_control.py` cover
validation, credentials, idle behavior, alignment, hardware failure, and
watchdog behavior without attached arms. The optional WebSocket/video/UI
smoke tests in `tests/test_integration.py` require the project dependencies
and `httpx` (install with `pip install -e '.[test]' 'lerobot[feetech]==0.6.1'`
in a compatible environment); they are skipped when dependencies are
missing. They use fake hardware, not a Cloudflare account.
