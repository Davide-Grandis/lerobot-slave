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

Use a Node.js environment with Wrangler 4. The project has no frontend
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

4. Assign the Worker a custom domain in the Cloudflare dashboard and protect
   the Worker with Cloudflare Access. Restrict the Access policy to the
   intended operators. Protect the Worker itself, including its API routes,
   rather than only protecting the HTML page.

The `access.dev` identity in the Wrangler config is only for local Worker
development; it does not authenticate production requests. Production API
handlers require the Access identity.

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
code. Enter that code in the hosted app's pairing panel. The code is held in
browser memory only and is not stored in D1 or browser local storage.

The hosted page uses cross-origin requests to the local controller API. Use a
current Chromium-based desktop browser and grant its local-network permission
if prompted. The controller accepts that browser origin only when it exactly
matches `CF_ROBOT_OPERATOR_ORIGIN`, and each API call uses the pairing code in
a request header. If the browser blocks local-network access, use the
controller-local page at `http://127.0.0.1:8080` while resolving the browser
permission issue.

## Operator workflow

1. Pair the hosted page with the controller laptop and wait for the follower
   camera to show fresh frames.
2. Start a control session. The follower connection opens only after the
   camera is live; connecting the camera/session by itself sends no motion.
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

The canvas renders a kinematic skeleton from SO-ARM101 URDF joint transforms
and current servo feedback. It also draws the leader as a dashed comparison
pose. The model's zero offsets and mounting frame must be verified against the
physical arms before treating its spatial overlay as precise; the numerical
servo readback and the live camera remain the operational references.

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
code printed by the agent. Production uses the Access-protected HTTPS origin.

Run the Python regression suite from the project root:

```sh
.venv/bin/python -m unittest discover -s tests
```
