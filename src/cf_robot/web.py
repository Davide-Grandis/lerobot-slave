"""Loopback controller API for the hosted operator UI."""

import argparse
import asyncio
import contextlib
from importlib.resources import files
import hmac
import json
import logging
import os
import secrets
import time
from urllib.parse import urlsplit

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response
from fastapi.middleware.trustedhost import TrustedHostMiddleware
import uvicorn
from websockets.asyncio.client import connect

from cf_robot.auth import access_headers
from cf_robot.operator_session import OperatorSession
from cf_robot.protocol import validate_action
from cf_robot.web_pose import BrowserPoseMonitor, install_pose_routes

LOG = logging.getLogger(__name__)
VIDEO_STALE_S = 1.5
BROWSER_FRAME_STALE_S = 2.0
ASSETS = {
    "app.js": ("text/javascript; charset=utf-8", "operator/app.js"),
    "app.css": ("text/css; charset=utf-8", "operator/app.css"),
    "arm-viewer.js": ("text/javascript; charset=utf-8", "operator/arm-viewer.js"),
    "pose-monitor.js": ("text/javascript; charset=utf-8", "operator/pose-monitor.js"),
}


def create_app(
    url,
    port,
    arm_id,
    fps=30,
    runner=None,
    heartbeat_timeout=3,
    operator_origin=None,
    bridge_token=None,
    controller_factory=OperatorSession,
):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["127.0.0.1", "localhost"])
    control_url = url.rstrip("/") + "/control"
    video_url = url.rstrip("/") + "/video"
    access_headers(control_url)
    hosted_origin = (operator_origin or os.environ.get("CF_ROBOT_OPERATOR_ORIGIN", "")).rstrip("/")
    local_bridge_token = bridge_token or os.environ.get("CF_ROBOT_BRIDGE_TOKEN") or secrets.token_urlsafe(32)
    task = None
    runtime = None
    stop_event = None
    video_task = None
    video_stop = asyncio.Event()
    last_heartbeat = 0.0
    last_browser_frame_at = None
    stop_reason = None
    estop_latched = False
    legacy_runner = runner is not None
    status = {
        "state": "idle",
        "error": None,
        "mode": "manual",
        "leader_connected": False,
        "leader_positions": None,
        "follower_positions": None,
        "follower_observed_at": None,
        "aligned": False,
        "alignment_errors": {},
        "commands_sent": 0,
        "camera_state": "idle",
        "camera_received_at": None,
        "estop_latched": False,
        "running": False,
    }
    video_frame = {"data": None, "received_at": None}
    pose_monitor = BrowserPoseMonitor(url, port, arm_id, heartbeat_timeout)

    def origin_allowed(origin, host):
        if not origin:
            return False
        parsed = urlsplit(origin)
        if (
            parsed.scheme == "http"
            and parsed.netloc == host
            and parsed.hostname in ("localhost", "127.0.0.1")
        ):
            return True
        return bool(hosted_origin and origin.rstrip("/") == hosted_origin)

    def require_origin(request):
        origin = request.headers.get("origin", "")
        host = request.headers.get("host", "")
        if not origin_allowed(origin, host):
            raise HTTPException(status_code=403, detail="untrusted browser origin")
        if hosted_origin and origin.rstrip("/") == hosted_origin:
            supplied = request.headers.get("x-cf-robot-bridge", "")
            if not hmac.compare_digest(supplied, local_bridge_token):
                raise HTTPException(status_code=403, detail="controller pairing code required")

    install_pose_routes(
        app, pose_monitor, require_origin,
        lambda: task is not None and not task.done(),
        lambda: estop_latched,
    )

    @app.middleware("http")
    async def local_network_cors(request: Request, call_next):
        origin = request.headers.get("origin", "")
        allowed = origin_allowed(origin, request.headers.get("host", ""))
        if request.method == "OPTIONS" and allowed:
            response = Response(status_code=204)
            response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
            response.headers["Access-Control-Allow-Headers"] = (
                "Content-Type, X-CF-Robot-Bridge"
            )
            response.headers["Access-Control-Max-Age"] = "600"
            if request.headers.get("access-control-request-private-network") == "true":
                response.headers["Access-Control-Allow-Private-Network"] = "true"
        else:
            response = await call_next(request)
        if allowed:
            response.headers["Access-Control-Allow-Origin"] = origin
            response.headers["Vary"] = "Origin"
        return response

    def apply_runtime_status(message):
        status.update(message)
        status["running"] = task is not None and not task.done()
        if "error" not in message and message.get("state") not in ("error",):
            status["error"] = None

    def finished(done):
        nonlocal estop_latched
        try:
            done.result()
        except asyncio.CancelledError:
            return
        except Exception as exc:
            LOG.info("Operator session ended: %s", exc)
            status.update(state="error", error=str(exc), running=False)
            estop_latched = True
        else:
            status.update(
                state="idle",
                error=stop_reason,
                running=False,
                leader_connected=False,
                mode="manual",
            )
        status["estop_latched"] = estop_latched

    async def relay_video():
        while not video_stop.is_set():
            status["camera_state"] = "connecting"
            try:
                async with connect(
                    video_url,
                    additional_headers=access_headers(video_url),
                    open_timeout=10,
                    ping_interval=10,
                    ping_timeout=5,
                    max_size=2 * 1024 * 1024,
                ) as remote:
                    async for frame in remote:
                        if video_stop.is_set():
                            return
                        if isinstance(frame, bytes):
                            now = time.time()
                            video_frame.update(data=frame, received_at=now)
                            status.update(camera_state="live", camera_received_at=now)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                status.update(camera_state="offline", camera_error=str(exc))
                LOG.info("Follower video relay ended: %s", exc)
            if not video_stop.is_set():
                await asyncio.sleep(1)

    def start_video_relay():
        nonlocal video_task
        if video_task is None or video_task.done():
            video_task = asyncio.create_task(relay_video())

    async def watch_browser(current_task, current_stop, require_video):
        nonlocal stop_reason, estop_latched
        while not current_task.done():
            await asyncio.sleep(min(0.25, heartbeat_timeout / 2))
            if current_task.done():
                return
            if time.monotonic() - last_heartbeat > heartbeat_timeout:
                stop_reason = "Browser heartbeat lost; control stopped."
            elif require_video and (
                video_frame["received_at"] is None
                or time.time() - video_frame["received_at"] > VIDEO_STALE_S
            ):
                stop_reason = "Follower camera feed lost; control stopped."
            elif require_video and (
                last_browser_frame_at is None
                or time.monotonic() - last_browser_frame_at > BROWSER_FRAME_STALE_S
            ):
                stop_reason = "Browser stopped receiving follower video; control stopped."
            else:
                continue
            status.update(state="stopping", error=stop_reason)
            estop_latched = True
            status["estop_latched"] = True
            current_stop.set()
            return

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return files("cf_robot").joinpath("operator", "index.html").read_text(encoding="utf-8")

    @app.get("/{name}")
    async def asset(name: str):
        if name not in ASSETS:
            raise HTTPException(status_code=404, detail="asset not found")
        content_type, path = ASSETS[name]
        content = files("cf_robot").joinpath(path).read_bytes()
        return Response(content, media_type=content_type, headers={"Cache-Control": "no-cache"})

    @app.get("/api/config")
    async def get_config(request: Request):
        require_origin(request)
        return {"api_version": 1}

    @app.get("/api/status")
    async def get_status(request: Request):
        require_origin(request)
        result = dict(status)
        result["running"] = task is not None and not task.done()
        result["camera_live"] = (
            video_frame["received_at"] is not None
            and time.time() - video_frame["received_at"] <= VIDEO_STALE_S
        )
        return result

    @app.get("/api/video/frame")
    async def get_video_frame(request: Request):
        require_origin(request)
        start_video_relay()
        if (
            video_frame["data"] is None
            or video_frame["received_at"] is None
            or time.time() - video_frame["received_at"] > VIDEO_STALE_S
        ):
            raise HTTPException(status_code=503, detail="follower camera is not live")
        return Response(
            video_frame["data"],
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store, max-age=0"},
        )

    @app.post("/api/start")
    async def start(request: Request):
        nonlocal task, runtime, stop_event, last_heartbeat, last_browser_frame_at, stop_reason
        require_origin(request)
        if estop_latched:
            raise HTTPException(status_code=409, detail="re-arm explicitly after inspecting the robot")
        if task is not None and not task.done():
            raise HTTPException(status_code=409, detail="session already running")
        if pose_monitor.running:
            raise HTTPException(status_code=409, detail="stop the pose monitor before teleoperation")
        if not legacy_runner:
            try:
                start_body = await request.json()
            except (ValueError, json.JSONDecodeError):
                start_body = {}
            if not isinstance(start_body, dict) or start_body.get("camera_live") is not True:
                raise HTTPException(status_code=409, detail="confirm the live camera is visible before starting")
            start_video_relay()
            if (
                video_frame["received_at"] is None
                or time.time() - video_frame["received_at"] > VIDEO_STALE_S
            ):
                raise HTTPException(status_code=409, detail="wait for a live follower camera before starting")
        if estop_latched or pose_monitor.running or (task is not None and not task.done()):
            raise HTTPException(status_code=409, detail="another hardware session is running")
        stop_event = asyncio.Event()
        last_heartbeat = time.monotonic()
        last_browser_frame_at = time.monotonic() if not legacy_runner else None
        stop_reason = None
        status.update(state="connecting", error=None, running=True, estop_latched=False)
        if legacy_runner:
            task = asyncio.create_task(
                runner(
                    control_url,
                    port,
                    arm_id,
                    fps=fps,
                    stop=stop_event,
                    on_status=apply_runtime_status,
                )
            )
        else:
            runtime = controller_factory(
                control_url,
                port,
                arm_id,
                fps=fps,
                on_status=apply_runtime_status,
            )
            task = asyncio.create_task(runtime.run())
        task.add_done_callback(finished)
        asyncio.create_task(watch_browser(task, stop_event, not legacy_runner))
        return {"state": "connecting"}

    @app.post("/api/heartbeat")
    async def heartbeat(request: Request):
        nonlocal last_heartbeat, last_browser_frame_at
        require_origin(request)
        last_heartbeat = time.monotonic()
        if not legacy_runner:
            try:
                body = await request.json()
            except (ValueError, json.JSONDecodeError):
                body = {}
            last_browser_frame_at = (
                time.monotonic()
                if isinstance(body, dict) and body.get("camera_live") is True
                else None
            )
        return {"ok": True}

    @app.post("/api/stop")
    async def stop(request: Request):
        nonlocal stop_reason
        require_origin(request)
        if runtime is not None and task is not None and not task.done():
            stop_reason = "Stopped by operator."
            runtime.stop_event.set()
        elif stop_event is not None:
            stop_event.set()
        status["state"] = "stopping" if task is not None and not task.done() else "idle"
        return {"state": status["state"]}

    @app.post("/api/estop")
    async def emergency_stop(request: Request):
        nonlocal stop_reason, estop_latched
        require_origin(request)
        estop_latched = True
        pose_monitor.stop()
        stop_reason = "Software stop requested. Inspect the robot before re-arming."
        if runtime is not None and task is not None and not task.done():
            runtime.stop_event.set()
        elif stop_event is not None:
            stop_event.set()
        status.update(
            state="stopping" if task is not None and not task.done() else "idle",
            error=stop_reason,
            estop_latched=True,
        )
        return {"state": status["state"], "estop_latched": True}

    @app.post("/api/estop/rearm")
    async def rearm(request: Request):
        nonlocal estop_latched
        require_origin(request)
        try:
            body = await request.json()
        except (ValueError, json.JSONDecodeError):
            body = {}
        if not isinstance(body, dict) or body.get("confirmation") != "RE-ARM":
            raise HTTPException(status_code=400, detail="type RE-ARM after inspecting the robot")
        if pose_monitor.running or (task is not None and not task.done()):
            raise HTTPException(status_code=409, detail="stop the current session before re-arming")
        estop_latched = False
        status.update(state="idle", error=None, estop_latched=False)
        return {"state": "idle", "estop_latched": False}

    @app.post("/api/mode")
    async def set_mode(request: Request):
        require_origin(request)
        if runtime is None or task is None or task.done():
            raise HTTPException(status_code=409, detail="connect the follower first")
        try:
            body = await request.json()
            if not isinstance(body, dict):
                raise ValueError("request body must be a JSON object")
            result = runtime.set_mode(body.get("mode"))
        except (ValueError, RuntimeError, AttributeError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return result

    @app.post("/api/manual")
    async def manual_action(request: Request):
        require_origin(request)
        if runtime is None or task is None or task.done():
            raise HTTPException(status_code=409, detail="connect the follower first")
        try:
            body = await request.json()
            if not isinstance(body, dict):
                raise ValueError("request body must be a JSON object")
            await runtime.submit_manual(body.get("action"))
        except (ValueError, RuntimeError, TypeError, AttributeError, asyncio.TimeoutError) as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
        return {"queued": True}

    @app.post("/api/leader/connect")
    async def connect_leader(request: Request):
        require_origin(request)
        if runtime is None or task is None or task.done():
            raise HTTPException(status_code=409, detail="connect the follower first")
        try:
            return await runtime.connect_leader()
        except Exception as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/leader/disconnect")
    async def disconnect_leader(request: Request):
        require_origin(request)
        if runtime is None:
            raise HTTPException(status_code=409, detail="leader is not connected")
        await runtime.disconnect_leader()
        return dict(runtime.status)

    @app.on_event("shutdown")
    async def shutdown():
        pose_monitor.stop()
        if pose_monitor.task is not None:
            await pose_monitor.task
        video_stop.set()
        if video_task is not None:
            video_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await video_task
        if runtime is not None:
            runtime.stop_event.set()
        if stop_event is not None:
            stop_event.set()
        if task is not None:
            with contextlib.suppress(Exception):
                await task

    return app


def main():
    parser = argparse.ArgumentParser(description="SO-ARM101 browser operator bridge")
    parser.add_argument("--url", required=True, help="https://<public-fqdn> or wss://<public-fqdn>")
    parser.add_argument("--port", help="Local leader serial device (optional for manual-only control)")
    parser.add_argument("--arm-id", required=True, help="LeRobot leader calibration ID")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--listen-port", type=int, default=8080)
    parser.add_argument(
        "--operator-origin",
        default=os.environ.get("CF_ROBOT_OPERATOR_ORIGIN"),
        help="Exact browser origin of the Cloudflare-hosted operator app",
    )
    args = parser.parse_args()
    base = args.url.rstrip("/")
    if base.startswith("https://"):
        base = "wss://" + base[len("https://"):]
    elif base.startswith("http://"):
        base = "ws://" + base[len("http://"):]
    if urlsplit(base).path not in ("", "/"):
        parser.error("--url must be a hostname without a path")
    bridge_token = os.environ.get("CF_ROBOT_BRIDGE_TOKEN") or secrets.token_urlsafe(32)
    app = create_app(
        base,
        args.port,
        args.arm_id,
        args.fps,
        operator_origin=args.operator_origin,
        bridge_token=bridge_token,
    )
    if args.operator_origin:
        print(f"Allowing hosted operator origin: {args.operator_origin}")
        print("Enter this one-run controller pairing code in the hosted operator UI:")
        print(bridge_token)
    uvicorn.run(app, host="127.0.0.1", port=args.listen_port)


if __name__ == "__main__":
    main()
