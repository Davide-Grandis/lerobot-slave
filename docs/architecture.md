# Architecture

This document describes the teleoperation topology, components, protocol, and
trust boundaries.

```text
Controller laptop                          Cloudflare                    Raspberry Pi
SO-ARM101 leader --USB--> LeRobot          Worker / static UI
                             |               | D1: config + audit          Access
                  local controller API <-- browser                         |
                     |                                                       Tunnel
                     | WSS /control ------------------------------------> 127.0.0.1:8765
                     | WSS /video    <----------------------------------- Pi follower
                     +--> measured/commanded joints                        |
                                                                          one hardware thread
                                                                          +--> follower --USB--> arm
                                                                          +--> camera --USB--> Pi
```

`cloudflared` routes the Pi's public hostname to `http://127.0.0.1:8765`;
control and video upgrade to WebSockets. The Pi never listens on a
LAN/public address. The Worker hosts the browser UI and uses D1 for
low-volume operator/session records; it is not in the motion or video path.
The controller API binds only to loopback and holds all Access and origin-key
credentials. The hosted page reaches that API through a configured browser
origin and a one-run pairing code. It does not read USB directly or contact
the Pi.

| Component | Responsibility |
| --- | --- |
| [`src/cf_robot/hardware.py`](../src/cf_robot/hardware.py) | Construct LeRobot `SO101Leader`, `SO101Follower`, and `OpenCVCamera`; extract joint observations. |
| [`src/cf_robot/leader.py`](../src/cf_robot/leader.py) | Connect the laptop leader without interactive calibration; send actions at a configured maximum rate (default 30 fps); stop rather than auto-reconnect after failure. |
| [`src/cf_robot/follower.py`](../src/cf_robot/follower.py) | Serve `/health`, `/control`, `/video`, `/update`, `/calibrate`, `/maintenance`, and `/monitor`; authenticate WebSocket upgrades; allow one control and one video connection. |
| [`src/cf_robot/control.py`](../src/cf_robot/control.py) | Own the follower in one thread; retain only the latest target; refresh measured servo positions; check command gap and joint jumps; disconnect on stop/error. |
| [`src/cf_robot/gripper_control.py`](../src/cf_robot/gripper_control.py) | Isolate J6 through direct-bus adapters, without whole-arm configuration or J1-J5 goal/torque writes; check J6 calibration/alignment and release only J6 on stop. |
| [`src/cf_robot/teleop_runtime.py`](../src/cf_robot/teleop_runtime.py) | Shared feedback validation, timing diagnostics, bounded sequential startup sync and measured settling for normal CLI, browser leader mode and isolated tests. |
| [`src/cf_robot/teleop_motion.py`](../src/cf_robot/teleop_motion.py) | Shared per-joint applied-goal windows and direction-aware no-progress detection, using degrees for J1-J5 and percent for J6. |
| [`src/cf_robot/teleop_hardware.py`](../src/cf_robot/teleop_hardware.py) | Shared saved/live calibration preflight and direct-bus adapters; seed selected goals before enabling motors, preserve calibration and hold a full arm on stop. |
| [`src/cf_robot/protocol.py`](../src/cf_robot/protocol.py) | Define protocol version 1, joint names, action validation, sequence and timestamp checks. |
| [`src/cf_robot/auth.py`](../src/cf_robot/auth.py) | Validate the remote URL and attach Access and origin-key headers from the laptop environment. |
| [`src/cf_robot/operator_session.py`](../src/cf_robot/operator_session.py) | Own one remote control socket; route either manual jog targets or leader samples, never both; refresh the manual target to satisfy the follower watchdog. |
| [`src/cf_robot/web.py`](../src/cf_robot/web.py) | Serve the loopback controller API, relay JPEG frames, pair with the hosted page, and stop on stale browser/video heartbeats. |
| [`src/cf_robot/calibration_inspection.py`](../src/cf_robot/calibration_inspection.py) | Read saved/live calibration, actual SDK units and identity; capture and analyse independent physical evidence without writes. |
| [`src/cf_robot/calibration_restore.py`](../src/cf_robot/calibration_restore.py) | Own session-scoped, one-use previews for J6-only saved calibration restore; require torque OFF, verify position/configuration, restrict writes and verify readback without auto-retry or rollback. |
| [`src/cf_robot/web_calibration.py`](../src/cf_robot/web_calibration.py) | Run one explicitly confirmed, browser-supervised guided calibration, locally for the leader or through the Pi's existing authenticated calibration route. |
| [`src/cf_robot/operator/`](../src/cf_robot/operator/) and [`operator/worker/`](../operator/worker/) | Serve the operator UI from Worker Static Assets; keep D1 for robot/session/audit records; render follower/leader pose from URDF joint transforms. |
| [`src/cf_robot/parking_client.py`](../src/cf_robot/parking_client.py) | Record the follower's measured pose locally on the Mac and send a saved parking pose to the Pi over the authenticated control WebSocket. |
| [`deploy/`](../deploy/) | Example Pi environment, follower service, and the root-owned WSS update watcher/service. |

## Control and video protocol

See [Shared Teleoperation Policy](teleoperation.md). Joint-only mode selects
the joints; it does not select a separate tracking/feedback/sync algorithm.

The CLI's `--J6-only` mode uses authenticated `WS /control/gripper`,
sharing the normal control/maintenance lock. Its ready message explicitly
acknowledges `mode=gripper-only` and `joints=["gripper.pos"]`; every action and
observation contains only that joint. Other-joint frames are rejected rather
than padded with fixed targets. The existing `/control` route still requires
all six joints. Gripper-only startup is read-only until the first aligned target;
there is no full-arm SDK connect/configure or whole-arm startup sync. After
confirmation, the Mac seeds the measured J6 position, ramps only J6 toward a
fixed leader startup snapshot and waits for fresh measured settling before
live following. The Pi's first-target guard remains unchanged.

J1-J5 isolated tests use `WS /control/joint/{id}` and ready metadata
`mode=joint-only` with exactly the selected position key. These share the same
hardware owner, interlocks, feedback holds and motion policy. They hold the
selected axis on stop rather than releasing a gravity-loaded wrist/arm.
The monitor's `angular_preview` / `angular_apply` commands program selected
J1-J5 MIN/MAX EEPROM and matching angular/SDK files, with explicit hardware-write
intent, a one-use nonce, readback and backups. Homing, torque, goal registers and
other joints are never written; arbitrary paths are not accepted.
The monitor stops after apply and flushes its
final result before the terminal frame so clients cannot retain old mappings.

The Pose Monitor supports independent local leader or remote follower sessions,
as well as comparison. Calibration maintenance, monitoring and teleoperation are
mutually exclusive in the controller; Pi routes retain their separate hardware
interlocks. The paired `/api/calibration/*` routes accept only a fixed arm
selection and guided-calibrator replies, not shell commands or arbitrary
register/file writes. Browser heartbeat loss cancels calibration and retains
local ownership through process cleanup. Cancellation does not roll back motor
settings. See [Arm Calibration](arm-calibration.md) for the physical procedure,
evidence limits and setting safeguards.

- The Pi accepts `/control` only after checking `X-CF-Robot-Key`. It creates
  one `FollowerSession`, performs shared read-only calibration preflight, reads its
  current joint positions, then sends a versioned `ready` message containing
  that measured pose. The Mac parking client records this pose locally or
  uses it as the starting point when sending a saved parking pose.
- Each subsequent control message is JSON with `version`, `type`, increasing
  `seq`, Unix-seconds `sent_at`, and exactly six numeric joint positions:
  `shoulder_pan.pos`, `shoulder_lift.pos`, `elbow_flex.pos`,
  `wrist_flex.pos`, `wrist_roll.pos`, and `gripper.pos`.
  For example (illustrative values only; `sent_at` must be current):

  ```json
  {"version":1,"type":"action","seq":1,"sent_at":1730000000.123,"action":{"shoulder_pan.pos":0,"shoulder_lift.pos":0,"elbow_flex.pos":0,"wrist_flex.pos":0,"wrist_roll.pos":0,"gripper.pos":0}}
  ```

- Updated ready messages advertise `capabilities=["hold-current-position"]`.
  During a brief live feedback gap the client can send
  `{"version":1,"type":"hold","seq":2,"sent_at":1730000000.2}` with no joint
  values. The sole hardware-owner thread reads and holds its own selected
  joints. It never arms an unarmed session. Holds obey the same sequence,
  timestamp and 500 ms command watchdog checks as actions. Without this
  negotiated capability the client fails closed at the original feedback
  threshold; it never substitutes an old far-away target.

- The Pi rejects a duplicate/out-of-order sequence, missing/extra joint,
  non-finite or non-numeric position, position outside +/-360, command older
  than 500 ms, or timestamp more than 250 ms ahead of its clock. The
  timestamp is a freshness check, not a clock-synchronization mechanism.
  A protocol failure ends the session; there is no automatic replay.
- The follower thread replaces an unprocessed target with the newest one.
  Initial alignment is checked; shared adapters then bound applied goals
  against measured positions (maximum 5 degrees or percentage points) and check
  progress and saved ranges. A distant leader request does not cause false lag
  faults. This is **not** a real-time servo loop, collision model or guaranteed
  velocity/force limiter. The Pi reads measured servo positions in the same
  single-owner hardware thread and sends them in status messages about every
  100 ms. These are feedback values, not target echoes.
- `/video` is a separate WebSocket. The Pi opens the camera on demand and
  sends binary JPEG frames (default 640x480, 10 fps, JPEG quality 70).
  The laptop relays the latest frame through its loopback `/api/video/frame`
  endpoint. A fresh camera frame is required before control starts; a stale
  feed stops an active browser session. Camera capture, JPEG encoding, and
  network bandwidth can affect latency; no audio or WebRTC is implemented.
- `/update` accepts only fixed versioned update and status messages. The
  unprivileged follower writes a request marker under `/var/lib/cf-robot`; a
  preinstalled root-owned `systemd` path/service performs a fast-forward-only
  update from `origin/main`, refreshes dependencies, and restarts the
  follower. The client polls update status over the same WSS route; no shell
  command, branch, or path is accepted from the client.
- `/calibrate` bridges only LeRobot's fixed follower-calibration command to
  the Mac. Prompt text and user responses are streamed over the authenticated
  WSS connection; the serial port and calibration ID are taken from the Pi's
  service configuration, never from client input. Calibration is interactive
  and still requires someone physically at the follower to move its joints.
- `/maintenance` accepts only fixed servo-limit-read and go-home operations.
  It reads raw MIN/MAX registers through LeRobot, stores them under
  `/var/lib/cf-robot`, and requires a separate confirmation before the fixed
  SDK-style home pose. It accepts no client-supplied serial path or raw target.
- `/monitor` streams normalized follower readback, raw servo counts, servo
  limits, and torque state without changing torque at startup. It accepts only
  validated per-joint tick/degree/gripper-percent targets and explicit `TE`/
  `TD` torque commands, including all-joint `J* TE`/`J* TD`, with configured
  limits and bounded single-joint steps. It blocks control, calibration,
  maintenance, and update operations while connected.

## Trust boundaries

Cloudflare Access should reject unauthenticated requests to **all** Pi
paths, including `/update`, `/calibrate`, `/maintenance`, and `/monitor`. The laptop supplies `CF-Access-Client-Id` and
`CF-Access-Client-Secret` on the WebSocket upgrade. Independently, the Pi
requires a shared `CF_ROBOT_ORIGIN_KEY` (at least 32 characters) as
`X-CF-Robot-Key` on all Pi WebSocket routes. This limits the damage of an
accidentally unprotected hostname; it does not replace Access or physical
safety controls. `/health` is HTTP and has no origin-key check, so protect
it with the same Access application. The app Worker and its metadata API are
public and do not use Access identity. New D1 session/audit records are
attributed to `anonymous`; they are untrusted metadata, not operator identity
or authorization. Anonymous session UUIDs allow updates to those records;
historical identified records are not editable through that path. D1 never
stores the Access token, pairing code, or origin key. The laptop API
restricts Host to loopback, allows only the exact configured Worker origin
for cross-origin requests, and requires a one-run pairing code in a request
header. It still binds only to `127.0.0.1`; never expose it with a reverse
proxy. A software-stop latch is a UI/control-session safeguard, not a
certified emergency stop.
