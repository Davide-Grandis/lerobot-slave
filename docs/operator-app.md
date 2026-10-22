# Cloud-hosted operator app

Phase 2 hosts the browser UI on a Cloudflare Worker while keeping the SO-ARM101
hardware and the Pi's service-token credentials on the controller laptop.

```text
Browser --HTTPS--> Worker Static Assets + API --D1 (session/audit data)
   |
   | loopback HTTP + one-run pairing code
   v
Controller laptop: cf-robot-web --WSS/Access--> Pi follower
   |                                                    |
   +-- LeRobot leader USB                              +-- follower USB
   +-- video relay                                      +-- camera USB
```

The Worker/D1 path stores low-volume robot and session metadata only. Motion
commands, servo feedback, and JPEG frames stay between the browser/controller
and the Pi; D1 is not a telemetry or video store.

## Cloudflare setup

Use a Node.js 22.13+ environment with Wrangler 4. The project has no frontend
framework or runtime npm dependencies.

1. Create a D1 database from `operator/worker`:

   ```sh
   cd operator/worker
   wrangler d1 create cf-robot-operator
   ```

   Copy the returned database ID into `d1_databases[0].database_id` in
   `wrangler.jsonc`, replacing the all-zero local placeholder.

2. Apply the migration to the production database:

   ```sh
   wrangler d1 migrations apply cf-robot-operator --remote
   ```

   For local development, use `wrangler d1 migrations apply
   cf-robot-operator --local`. Local Wrangler storage is separate from the
   production database.

3. Deploy the Worker and its static assets:

   ```sh
   wrangler deploy
   ```

4. Assign the Worker a custom domain in the Cloudflare dashboard. The hosted
   app and its metadata API are public; no Cloudflare Access identity or login
   is required. The Pi's separate hostname remains protected by Access.

`/api/me` reports `PUBLIC OPERATOR` with `authenticated: false`. New D1 session
and audit records use the literal `anonymous` in the existing `operator_email`
and `actor_email` columns; no schema migration is needed. Historical identified
records remain unchanged and cannot be updated through the anonymous API.

These are unauthenticated metadata records, not proof of who operated a robot.
Anyone can submit public session records, and possession of an anonymous
session's UUID permits updating it. There is no public session/audit listing.
Use edge rate limiting if abuse becomes a concern. The API never grants robot
access, accepts controller pairing codes, or stores Pi credentials: hardware
control still requires the local loopback bridge and its pairing code.

## Start the controller agent

The controller laptop must have the existing private environment configured
with:

- `CF_ACCESS_CLIENT_ID`
- `CF_ACCESS_CLIENT_SECRET`
- `CF_ROBOT_ORIGIN_KEY`

Update the Pi to this project version first. The operator UI requires the
follower's periodic measured-position status messages as well as the existing
versioned `/control` and `/video` WebSockets.

Set `CF_ROBOT_OPERATOR_ORIGIN` to the exact HTTPS origin of the Worker, with no
path or trailing slash. Example:

```sh
export CF_ROBOT_OPERATOR_ORIGIN=https://operator.example.com
source "$HOME/.config/cf-robot/controller.env"

.venv/bin/cf-robot-web \
  --url https://robot.example.com \
  --port /dev/tty.usbmodemYOUR_LEADER_DEVICE \
  --arm-id so101_leader \
  --operator-origin https://operator.example.com
```

The leader port is optional when operating only with on-screen controls. The
controller agent continues to bind only to `127.0.0.1`; do not expose it through
a router, Tunnel, or reverse proxy. On startup, it prints a one-run pairing
code consisting of two random lowercase words joined by a hyphen, such as
`quiet-otter`. Enter that code in the hosted app's pairing panel. A new code
is generated on each controller restart unless `CF_ROBOT_BRIDGE_TOKEN` is
explicitly set. The code is held in browser memory only and is not stored in
D1 or browser local storage.

These memorable codes have less entropy than long random tokens. Keep the
controller loopback-only and the hosted origin restricted as described here;
set `CF_ROBOT_BRIDGE_TOKEN` to a longer secret if stronger codes are required.

The hosted page uses cross-origin requests to the local controller API. Use a
current Chromium-based desktop browser and grant its local-network permission
if prompted. The controller accepts that browser origin only when it exactly
matches `CF_ROBOT_OPERATOR_ORIGIN`, and each API call uses the pairing code in
a request header. If the browser blocks local-network access, use the
controller-local page at `http://127.0.0.1:8080` while resolving the browser
permission issue.

The video path is follower `/video` WebSocket -> local controller
`/api/video/frame` -> browser JPEG decoding. It does not pass through the
hosted Worker or D1. The camera opens when the paired Teleoperation tab is
shown; pairing on Pose Monitor alone does not open it. Only successfully
decoded frames count as live in browser heartbeats. Missing/stale follower
frames or loss of browser video stops the active controller session and
latches the software stop; recovery never automatically restarts motion.

Use **Rotate 90°** below the video to rotate the browser image clockwise.
Four clicks return to the original orientation. The video window follows the
decoded image's aspect ratio, swapping width/height proportions on quarter
turns instead of leaving empty bars in a fixed 4:3 window. The entire image
remains visible and the orientation is remembered in this browser across refreshes;
this does not change the camera stream, joint coordinates or robot commands.

## Operator workflow

The app always opens on **01 / Teleoperation**, including after a page refresh.
Follower video and Measured pose share the top row, with Operate underneath.
On narrow screens the model stacks below the video. Start/stop, readiness and
input-mode controls are grouped in Operate rather than a separate Session box.

Use **Disconnect controller** in the paired-controller bar to end pairing.
The browser stops polling, clears its in-memory pairing code and video, and
requests cancellation of teleoperation, monitoring and guided calibration.
An updated local bridge also closes its follower video connection, releasing
the camera; the bridge process itself stays running and can be paired again
using its existing code. Restart the updated local bridge to enable this
endpoint. Older bridges receive the existing stop requests instead, but must
be restarted to release their camera relay. If the bridge cannot be reached,
the UI warns that stopping is unconfirmed; inspect the robot and rely on the
heartbeat watchdogs and physical stop as appropriate.

1. Pair the hosted page with the controller laptop and wait for the follower
   camera to show fresh frames.
2. In Operate, select **Start teleoperation** after the camera is live.
   This asks the local controller to open its Pi session; the browser still
   uses only the local controller endpoint. Starting does not select leader
   following automatically. **Stop teleoperation** ends that session without
   unpairing the browser.
3. Choose one input mode:
   - **On-screen:** use the per-joint `+`/`−` jog controls and a bounded step
     size. The UI and local agent enforce the nominal SO-ARM101 URDF joint
     bounds; each target is checked again by the Pi's per-step guard.
   - **Leader arm:** connect the leader, align the leader and follower within
     five units on each joint, then select leader streaming. Manual and leader
     input modes are exclusive.
4. Watch both the follower camera and the measured-pose display. Browser
   disconnect, stale camera frames, or loss of the control socket stops the
   session; it never reconnects into motion automatically.
5. A software-stop latch requires typing `RE-ARM` after inspecting the robot.
   It does not replace the physical emergency stop.

The Measured pose canvas and both Pose Monitor views use the same approximate
SO-ARM101 model, retaining URDF joint transforms and current servo feedback.
J1 is the vertical base bearing; J2/J3/J4 are the shoulder, elbow and wrist
bends; J5 rolls the whole gripper axially; J6 moves only the opening jaw against
the fixed jaw. Two long forked arm links connect J2-J3 and J3-J4, followed by a
compact wrist yoke. White shells, dark servo bodies and orange joint accents
identify the follower; the leader uses teal. Joint callouts J1-J6 are drawn on
the models.

Before connecting or after disconnecting, the main view shows a bent,
open-gripper illustration reference based on the arm's physical structure, labelled
`HOME REFERENCE / NOT LIVE`. This is a visual placeholder, not the physical
arm's measured position, a zero-angle pose, or its calibration-dependent
go-home target; joint readings remain blank. Old feedback during an active session is labelled
`LAST SAMPLE / NOT LIVE`, while fresh feedback is labelled `LIVE FOLLOWER / MEASURED`.
The model's zero offsets and mounting frame must be verified against the
physical arms before treating its spatial overlay as precise; the numerical
servo readback and the live camera remain the operational references. Angle
ranges printed on product pictures are not used as software limits or
calibration offsets. The approximate tool readout follows the fixed hand frame;
opening J6 does not invent another wrist rotation or move that reference.

## Development and checks

```sh
cd operator/worker
npm test
wrangler dev
```

`wrangler dev` uses local D1 state. Do not use `--remote` for routine
development. The local app UI can also be served by the Python controller
agent, which is useful for testing the bridge without deploying a Worker.
When exercising the Wrangler-hosted page against a local controller agent,
set `CF_ROBOT_OPERATOR_ORIGIN=http://localhost:8787` and enter the pairing
code printed by the agent. Production uses the public hosted HTTPS origin.

Run the Python regression suite from the project root:

```sh
.venv/bin/python -m unittest discover -s tests
```
