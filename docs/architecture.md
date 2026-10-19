# Architecture

This document describes the teleoperation topology, components, protocol, and
trust boundaries.

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
| [`src/cf_robot/hardware.py`](../src/cf_robot/hardware.py) | Construct LeRobot `SO101Leader`, `SO101Follower`, and `OpenCVCamera`; extract joint observations. |
| [`src/cf_robot/leader.py`](../src/cf_robot/leader.py) | Connect the laptop leader without interactive calibration; send actions at a configured maximum rate (default 30 fps); stop rather than auto-reconnect after failure. |
| [`src/cf_robot/follower.py`](../src/cf_robot/follower.py) | Serve `/health`, `/control`, `/video`, and `/update`; authenticate WebSocket upgrades; allow one control and one video connection. |
| [`src/cf_robot/control.py`](../src/cf_robot/control.py) | Own the follower in one thread; retain only the latest target; check alignment, command gap, and joint jumps; disconnect on stop/error. |
| [`src/cf_robot/protocol.py`](../src/cf_robot/protocol.py) | Define protocol version 1, joint names, action validation, sequence and timestamp checks. |
| [`src/cf_robot/auth.py`](../src/cf_robot/auth.py) | Validate the remote URL and attach Access and origin-key headers from the laptop environment. |
| [`src/cf_robot/web.py`](../src/cf_robot/web.py) and [`src/cf_robot/index.html`](../src/cf_robot/index.html) | Serve local controls/status, monitor browser heartbeat, and relay JPEG frames to the browser. |
| [`src/cf_robot/parking_client.py`](../src/cf_robot/parking_client.py) | Record the follower's measured pose locally on the Mac and send a saved parking pose to the Pi over the authenticated control WebSocket. |
| [`deploy/`](../deploy/) | Example Pi environment, follower service, and the root-owned WSS update watcher/service. |

## Control and video protocol

- The Pi accepts `/control` only after checking `X-CF-Robot-Key`. It creates
  one `FollowerSession`, connects the arm with `calibrate=False`, reads its
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
- `/update` accepts only a fixed versioned update request. The unprivileged
  follower writes a request marker under `/var/lib/cf-robot`; a preinstalled
  root-owned `systemd` path/service performs a fast-forward-only update from
  `origin/main`, refreshes dependencies, and restarts the follower. It accepts
  no shell command, branch, or path from the client.

## Trust boundaries

Cloudflare Access should reject unauthenticated requests to **all** Pi
paths, including `/update`. The laptop supplies `CF-Access-Client-Id` and
`CF-Access-Client-Secret` on the WebSocket upgrade. Independently, the Pi
requires a shared `CF_ROBOT_ORIGIN_KEY` (at least 32 characters) as
`X-CF-Robot-Key` on both WebSocket routes. This limits the damage of an
accidentally unprotected hostname; it does not replace Access or physical
safety controls. `/health` is HTTP and has no origin-key check, so protect
it with the same Access application. The laptop's web server restricts
hosts to loopback names and requires a matching browser Origin for its
control POSTs and video WebSocket. Never expose the local web server with
a reverse proxy.
