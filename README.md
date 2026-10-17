# SO-ARM101 remote teleoperation over Cloudflare Tunnel

This project splits LeRobot teleoperation across two computers. The operator's
laptop reads an SO-ARM101 leader over USB; a Debian Raspberry Pi drives an
SO-ARM101 follower over USB and reads its USB camera. The laptop connects to
the Pi through a Cloudflare-protected public FQDN. Phase 1 is a CLI; phase 2
adds a laptop-local browser console with follower-camera video. Both modes
use the same control implementation.

This is a thin integration around LeRobot hardware drivers, not a LeRobot
fork. Cloudflare Tunnel, Access, DNS, and credentials are configured manually;
this repository does not provision Cloudflare resources.
Source repository: [Davide-Grandis/lerobot-slave](https://github.com/Davide-Grandis/lerobot-slave).

## Requirements

| Area | Requirement |
| --- | --- |
| Laptop | SO-ARM101 leader attached by USB; Python supported by the selected LeRobot release; serial-device access; network access to the public FQDN. |
| Raspberry Pi | Headless 64-bit Debian (AArch64); SO-ARM101 follower and UVC-compatible camera attached by USB; serial and video-device access; `systemd`. See the Pi hardware assumptions below. |
| Both hosts | This package, compatible LeRobot/Feetech dependencies, separately calibrated arms, and synchronized clocks (NTP). Use the same tested LeRobot release on both hosts. |
| Cloudflare | An account with an active zone/FQDN, a named Tunnel with a `cloudflared` connector on the Pi, an Access self-hosted application, and an Access service token. |
| Safety | A clear workspace, a physical emergency stop, and an operator able to observe the follower. Do not use internet teleoperation for safety-critical motion. |

The package and pinned LeRobot release require Python `>=3.12,<3.14`.
Check `python3 --version` before installing. Network latency, USB
permissions, camera availability, and ARM-compatible dependencies must
be verified on the actual Pi before operation.

## Architecture

```text
Laptop                                             Cloudflare / Raspberry Pi
SO-ARM101 leader --USB--> LeRobot SO101Leader
                             | actions
                   CLI or local web server
                             | WSS /control       Access -> Tunnel -> 127.0.0.1:8765
                             +-------------------------------> Pi follower server
                                                              | one hardware thread
                                                              +--> SO101Follower --USB--> arm

Browser <--local /ws/video-- laptop video relay <--WSS /video-- Pi OpenCVCamera
            127.0.0.1:8080                                    --USB--> follower camera
```

`cloudflared` routes the **HTTP** public hostname to the Pi's
`http://127.0.0.1:8765`; the control and video routes upgrade to WebSockets.
The Pi server never listens on a LAN/public address. The local web server
listens only on `127.0.0.1:8080`. The browser does not read USB devices,
contact the Pi directly, or receive Cloudflare credentials. No Workers or
cloud-hosted UI are involved.

| Component | Responsibility |
| --- | --- |
| `src/cf_robot/hardware.py` | Construct LeRobot `SO101Leader`, `SO101Follower`, and `OpenCVCamera`; extract joint observations. |
| `src/cf_robot/leader.py` | Connect the laptop leader without interactive calibration; send actions at a configured maximum rate (default 30 fps); stop rather than auto-reconnect after failure. |
| `src/cf_robot/follower.py` | Serve `/health`, `/control`, and `/video`; authenticate WebSocket upgrades; allow one control and one video connection. |
| `src/cf_robot/control.py` | Own the follower in one thread; retain only the latest target; check alignment, command gap, and joint jumps; disconnect on stop/error. |
| `src/cf_robot/protocol.py` | Define protocol version 1, joint names, action validation, sequence and timestamp checks. |
| `src/cf_robot/auth.py` | Validate the remote URL and attach Access and origin-key headers from the laptop environment. |
| `src/cf_robot/web.py` and `src/cf_robot/index.html` | Serve local controls/status, monitor browser heartbeat, and relay JPEG frames to the browser. |
| `deploy/` | Example Pi environment file and `systemd` unit. |

### Control and video protocol

- The Pi accepts `/control` only after checking `X-CF-Robot-Key`. It creates
  one `FollowerSession`, connects the arm with `calibrate=False`, reads its
  current joint positions, then sends `{"type":"ready","version":1}`.
- Each subsequent control message is JSON with `version`, `type`, increasing
  `seq`, Unix-seconds `sent_at`, and exactly six numeric joint positions:
  `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`,
  `wrist_flex.pos`, `wrist_roll.pos`, and `gripper.pos`.
  For example (illustrative values only; `sent_at` must be current):

  ```json
  {"version":1,"type":"action","seq":1,"sent_at":1730000000.123,"action":{"shoulder_pan.pos":0,"shoulder_lift.pos":0,"elbow_flex.pos":0,"wrist_flex.pos":0,"wrist_roll.pos":0,"gripper.pos":0}}
  ```

- The Pi rejects a duplicate/out-of-order sequence, missing/extra joint,
  non-finite or non-numeric position, position outside +/-360, command older
  than 500 ms, or timestamp more than 250 ms ahead of its clock. The
  timestamp is a freshness check, not a clock-synchronization mechanism.
  A protocol failure ends the session; there is no automatic replay.
- The follower thread replaces an unprocessed target with the newest one.
  It checks each accepted target against the initial measured pose or the
  previously sent target (maximum 5 degrees per joint). It is **not** a
  real-time servo loop, calibrated joint-limit model, or guaranteed
  velocity/force limiter. The Pi sends periodic status messages and errors
  over the control socket.
- `/video` is a separate WebSocket. The Pi opens the camera on demand and
  sends binary JPEG frames (default 640x480, 10 fps, JPEG quality 70).
  The laptop relays them via local `/ws/video`; a video session does not arm
  the follower. Camera capture, JPEG encoding, and network bandwidth can
  affect latency; no audio or WebRTC is implemented.

### Trust boundaries

Cloudflare Access should reject unauthenticated requests to **all** Pi
paths. The laptop supplies `CF-Access-Client-Id` and
`CF-Access-Client-Secret` on the WebSocket upgrade. Independently, the Pi
requires a shared `CF_ROBOT_ORIGIN_KEY` (at least 32 characters) as
`X-CF-Robot-Key` on both WebSocket routes. This limits the damage of an
accidentally unprotected hostname; it does not replace Access or physical
safety controls. `/health` is HTTP and has no origin-key check, so protect
it with the same Access application. The laptop's web server restricts
hosts to loopback names and requires a matching browser Origin for its
control POSTs and video WebSocket. Never expose the local web server with
a reverse proxy.

## Before using hardware

- Keep people clear of the follower and provide a **physical emergency stop**.
  Cloudflare, this software, and a browser Stop button are not safety systems.
- Calibrate each arm with LeRobot on the machine and under the user account
  that will run it. Use the same calibration ID when starting this software.
  Align the arms to within **5 degrees on every joint** before Start; a larger
  jump is rejected instead of being sent to the follower.
- Synchronize the laptop and Pi clocks with NTP. Commands more than 500 ms old
  (or over 250 ms in the future) are rejected. If latency or jitter approaches
  this limit, do not operate the robot through this path.
- A lost connection or 500 ms gap after the first command ends the session.
  LeRobot's `disconnect()` behavior, including whether torque is released,
  depends on the hardware/driver. Verify the arm cannot fall or strike anything
  on disconnect or Pi reboot. Do not rely on software to hold a load.
- Only one control client and one video client are allowed at a time. The
  follower never automatically resumes movement after a failure or reboot.

## Install and calibrate

Use a `python3` version compatible with both `pyproject.toml` and your chosen
LeRobot release. This project pins **LeRobot 0.6.1** on both hosts; its
hardware APIs have been checked against that release, but the complete
system still needs a physical-arm test. Pi hardware operation is not yet
verified.

Find stable Pi devices with `ls -l /dev/serial/by-id/ /dev/v4l/by-id/`. The
leader on macOS usually appears as `/dev/tty.usbmodem...`. Check permissions
before starting either program; the Pi service user needs serial (`dialout`)
and camera (`video`) access.

### Laptop

From this repository's root, under the same account that will run the
CLI/web console:

```sh
python3 -m venv .venv
.venv/bin/python -m pip install -e . 'lerobot[feetech]==0.6.1'
.venv/bin/lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --teleop.id=so101_leader
```

Replace `python3` with your compatible interpreter as needed. The leader ID
must match the ID used later with `--arm-id`. Calibration is stored by
LeRobot for this local user; do not copy the leader calibration to the Pi.

### Raspberry Pi (Debian)

These instructions assume a Raspberry Pi 4 with 4 GB RAM and 64-bit
Debian (AArch64). The dependency commands below use Python 3.13; check
that it is available on your Pi before continuing.

#### Step 1: Configure the Cloudflare Tunnel

In the Cloudflare dashboard, create a named Tunnel for the Pi and follow
the dashboard-provided **Debian installation instructions** to install
`cloudflared` as a service on the headless Pi.
Confirm the connector is connected and the service starts automatically
after a Pi reboot. 

#### Step 2: Install the code (suggested path)

Use **`/opt/cf-robot`** for the Git checkout. This is the path already
configured in the `systemd` unit, so no service changes are needed. For a
fresh installation on the Pi:

```sh
sudo git clone https://github.com/Davide-Grandis/lerobot-slave.git /opt/cf-robot
```

Do not clone over an existing `/opt/cf-robot`; use the upgrade procedure
below instead. Keep the paths separate:

- `/opt/cf-robot`: application code.
- `/opt/cf-robot/venv`: Python venv.
- `/etc/cf-robot/follower.env`: local configuration and origin key.
- `/var/lib/cf-robot`: service user's persistent calibration data.



#### Install dependencies

On the Pi, check the installed Python version and venv package:

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

Then install this project and LeRobot's Feetech support:

```sh
sudo env TMPDIR=/var/tmp /opt/cf-robot/venv/bin/python -m pip install --no-cache-dir \
  -e /opt/cf-robot 'lerobot[feetech]==0.6.1'
/opt/cf-robot/venv/bin/python -m pip check
```

#### Calibrate the follower arm

1. **Retrieve the USB serial device ID.** Connect the follower arm's USB
   adapter to the Pi. With no control session running, list the stable
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

3. **Calibrate using that path.** Paste the full path at the prompt.
   The command verifies that it is a character device and that the
   `robot` service user can access it before starting LeRobot's
   interactive calibration:

   ```sh
   read -r -p 'Follower serial path (/dev/serial/by-id/...): ' FOLLOWER_PORT
   if [ -c "$FOLLOWER_PORT" ] &&
      sudo -u robot test -r "$FOLLOWER_PORT" &&
      sudo -u robot test -w "$FOLLOWER_PORT"; then
     sudo -u robot env HF_HOME=/var/lib/cf-robot /opt/cf-robot/venv/bin/lerobot-calibrate \
       --robot.type=so101_follower \
       --robot.port="$FOLLOWER_PORT" \
       --robot.id=so101_follower
   else
     echo "Wrong serial path or robot user lacks device access: $FOLLOWER_PORT" >&2
   fi
   ```

When calibration finishes, LeRobot saves it automatically under
`/var/lib/cf-robot/lerobot/calibration/robots/so_follower/so101_follower.json`.
There is no separate save command. Record that file path in your setup
notes if you need it for a backup; do not move the file. Verify that it
exists on the Pi:

```sh
sudo -u robot test -s /var/lib/cf-robot/lerobot/calibration/robots/so_follower/so101_follower.json \
  && echo 'Follower calibration found'
```

If the message does not appear, stop and resolve the missing calibration
before setting up the service. `--robot.id=so101_follower` names that
calibration; it is not the USB device ID. Also record the full serial path
you used for the configuration below.

The follower service runs with `HF_HOME=/var/lib/cf-robot` and uses
`connect(calibrate=False)`; it must not attempt interactive
calibration at boot or when a client connects. The camera is configured by
device path and does not need a LeRobot arm-calibration ID. Perform this
one-time terminal calibration over SSH on a headless Pi; it is not part of
normal startup.

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
   and `YOUR_GENERATED_KEY` with the output from step 2. Do not leave
   `CHANGE_ME` or any `YOUR_...` placeholders in the file. Put the same
   origin key in the laptop's secret manager; never commit or share it.

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

Check `journalctl -u cf-robot-follower -f` for failures. Confirm both this
service and the separately installed `cloudflared` service are enabled and
healthy after a Pi reboot. `GET /health` reports process availability and
whether a control session is occupied; it does **not** test the arm or camera.
If the GitHub repository is private, configure read-only SSH deploy-key
access for the account performing the clone and use its SSH URL instead.
Never put a personal access token in a clone URL, shell history, or service
unit.

### Normal startup and restart

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

### Upgrade the Pi from GitHub

Schedule upgrades while the follower is clear and **no control session is
active**. For a code-only change (no new Python dependencies or changes to
the systemd unit), the editable installation already points to the Git
checkout. On the Pi:

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
curl http://127.0.0.1:8765/health
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

## Cloudflare dashboard setup


### Step 3: Map the local app to a public URL (suggested hostname)

In the Cloudflare dashboard, select the Pi's Tunnel and add a public hostname
(published application route). Choose an available subdomain and a domain in
your Cloudflare zone, for example `robot.example.com`. Set the service type
to **HTTP** and its origin URL to **`http://127.0.0.1:8765`**. This address is
the follower service on the **Pi**, not the laptop. Route the whole hostname
without restricting it to a path: `/health`, `/control`, and `/video` share
the same origin. Do not expose port 8765 on the Pi's network interface.

The laptop will use `https://robot.example.com` for health checks or the web
console and `wss://robot.example.com/control` for CLI control; video uses
`wss://robot.example.com/video`. Replace the example hostname with your
chosen FQDN in all commands. A connected Tunnel does not mean the local
follower service is running; requests can fail until the origin starts.

### Protect the hostname with Access

Configure Access for the entire hostname before operating the arm:

1. Create a Cloudflare Access **self-hosted application** covering the whole
   hostname, not just one path (`/health`, `/control`, and `/video`). Create a
   service token and a **Service Auth** policy allowing only that token. Do not add a
   bypass policy or publish an unprotected alternate hostname. Where the
   Tunnel dashboard offers Access JWT validation at the connector, enable it
   for the application's audience as defense in depth.
2. Copy the token's client ID and secret to the laptop's environment or a
   local secret manager. The secret is shown once; never put it in a URL,
   shell history, browser JavaScript, or the Pi environment file. The
   laptop also needs `CF_ROBOT_ORIGIN_KEY`; do not put any of these values
   in browser JavaScript.
3. Verify the public hostname rejects a request without Access credentials,
   then verify `https://robot.example.com/health` succeeds when a client
   supplies the two Access headers. A successful health response does not
   verify the WebSocket origin key or USB devices. Do not start control until
   the unauthenticated request is denied. Avoid exposing secrets in shell
   history or command arguments while testing.

Cloudflare documentation: [Tunnel public hostnames](https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/get-started/create-remote-tunnel/),
[WebSockets](https://developers.cloudflare.com/network/websockets/), and
[Access service tokens](https://developers.cloudflare.com/cloudflare-one/access-controls/service-credentials/service-tokens/).

### Configuration reference

| Setting | Where | Meaning |
| --- | --- | --- |
| `FOLLOWER_PORT` | Pi `/etc/cf-robot/follower.env` | Follower USB serial path passed to `--port`; prefer `/dev/serial/by-id/...`. |
| `FOLLOWER_ID` | Pi environment file | Follower LeRobot calibration ID passed to `--arm-id`. |
| `CAMERA_DEVICE` | Pi environment file | Camera path passed to `--camera`; prefer `/dev/v4l/by-id/...`. |
| `CF_ROBOT_ORIGIN_KEY` | Pi environment file **and** laptop environment | Same random key on both ends (minimum 32 characters); sent only during the control/video WebSocket handshakes. |
| `CF_ACCESS_CLIENT_ID`, `CF_ACCESS_CLIENT_SECRET` | Laptop environment only | Access service token used for public `wss://` requests; never place these on the Pi or in browser code. |
| `HF_HOME` | Pi service unit and calibration command | LeRobot/Hugging Face data directory for the `robot` user. |
| `--listen-port` | Pi follower CLI | Loopback HTTP/WebSocket port, default `8765`; update Tunnel origin if changed. |
| `--camera-width`, `--camera-height`, `--camera-fps` | Pi follower CLI | Video capture defaults `640`, `480`, `10`; fps must be 1-30. Update the unit if changing them. |
| `--fps` | Laptop leader CLI or web CLI | Maximum leader action sampling rate, default `30`, accepted range 1-60; not a guaranteed end-to-end rate. |
| `--listen-port` | Laptop web CLI | Loopback web-console port, default `8080`. |

The Pi offers `GET /health`, `WS /control`, and `WS /video` behind the same
Access hostname. The laptop web console offers `GET /`, `GET /api/status`,
`POST /api/start`, `POST /api/stop`, `POST /api/heartbeat`, and
`WS /ws/video` **only on localhost**. Its browser sends a heartbeat once
per second; a missing heartbeat ends a running web control session after
approximately three seconds. The video connection is independent of the
control session.

## Laptop: CLI (phase 1)

Set `CF_ACCESS_CLIENT_ID`, `CF_ACCESS_CLIENT_SECRET`, and
`CF_ROBOT_ORIGIN_KEY` in the laptop process environment from a local secret
store. The Access token and origin key must match the Pi/Cloudflare setup.
Run from the repository root:

```sh
.venv/bin/cf-robot-leader \
  --url wss://robot.example.com/control \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader --fps 30
```

Press Ctrl-C to stop. The CLI does not silently reconnect after a dropped
session: inspect the follower, realign both arms, then start again.

## Laptop: browser console (phase 2)

With the same laptop environment variables:

```sh
.venv/bin/cf-robot-web \
  --url https://robot.example.com \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader
```

Open `http://127.0.0.1:8080` on the laptop. The page shows the follower
camera even when control is idle. Its Start and Stop buttons manage the
leader-arm session. Closing the browser or losing its heartbeat stops control
after approximately three seconds; use Stop to request a deliberate
shutdown rather than relying on that timeout.
The local server only accepts loopback hosts and same-origin control
requests; do not reverse-proxy it to the internet.

## Operating runbook

1. Check the follower's physical emergency stop, workspace, camera view, and
   any load the arm could drop if torque is lost. Have someone able to stop
   the follower locally. Do not begin with an unobserved arm.
2. From the laptop, confirm `/health` is reachable through Access and, when
   using the web console, that the camera view works. Check that laptop/Pi
   clocks are synchronized. The
   protected health request verifies the service/Tunnel path, not that the
   USB arm or camera is ready. Pi login and `systemctl` commands are only
   needed for initial setup or troubleshooting, not each startup.
3. Align the leader and follower poses (all six joints within 5 degrees),
   keep the follower clear, then start **one** CLI or web control session.
   The Pi connects to the follower on demand. The web camera may already
   be visible, but that alone does not start motion.
4. Move the leader slowly while watching the follower. If the session
   reports an error, stalls, or the follower behaves unexpectedly, use the
   physical stop first. In normal operation press Stop or Ctrl-C; verify
   the follower is stationary before leaving.
5. After a timeout, disconnect, Pi reboot, or rejected joint jump, inspect
   both arms, re-align, and start a **new** session. The code does not
   automatically reconnect or replay previous commands.

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
| `stale command or unsynchronized clocks` | NTP on both hosts and actual network delay/jitter; do not loosen freshness checks casually. |
| `joint jump exceeds limit` | Stop, inspect both arms, align them within 5 degrees, and restart the session; fast leader movement may also exceed the per-command limit. |
| Web control stops when tab is hidden or closed | The page must continue sending heartbeats. Reopen it, inspect the follower, re-align, and restart. |

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

Before live use, perform a supervised two-arm/camera acceptance test for
calibration, Access denial, video, network loss, Pi power cycle, and torque
behavior on disconnect. This repository has **not** verified those
conditions on your hardware. Internet latency is variable; this system
does not guarantee a 30 Hz control rate. Video is JPEG-over-WebSocket, not
low-latency WebRTC. Lower its resolution/fps if it competes with control
traffic. The implementation supports one SO-ARM101 pair and one camera;
it does not provide recording, force feedback, autonomous recovery, or
a certified emergency stop.
