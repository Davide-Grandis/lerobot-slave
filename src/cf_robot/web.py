"""Local web console; the browser never receives Cloudflare Access credentials."""

import argparse
import asyncio
import contextlib
from importlib.resources import files
import json
import logging
import time
from urllib.parse import urlsplit

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.responses import HTMLResponse
import uvicorn
from websockets.asyncio.client import connect

from cf_robot.auth import access_headers
from cf_robot.leader import run_teleop

LOG = logging.getLogger(__name__)


def create_app(url, port, arm_id, fps=30, runner=run_teleop, heartbeat_timeout=3):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(TrustedHostMiddleware, allowed_hosts=["127.0.0.1", "localhost"])
    control_url = url.rstrip("/") + "/control"
    video_url = url.rstrip("/") + "/video"
    access_headers(control_url)
    task = None
    stop_event = None
    last_heartbeat = 0.0
    stop_reason = None
    status = {"state": "idle", "error": None}

    def same_origin(origin, host):
        parsed = urlsplit(origin or "")
        return parsed.scheme == "http" and parsed.netloc == host and parsed.hostname in ("localhost", "127.0.0.1")

    def update(message):
        status["state"] = message["state"]
        status["error"] = None

    def finished(done):
        try:
            done.result()
        except Exception as exc:
            status.update(state="error", error=str(exc))
        else:
            status.update(state="idle", error=stop_reason)

    async def watch_browser(current_task, current_stop):
        nonlocal stop_reason
        while not current_task.done():
            await asyncio.sleep(min(0.5, heartbeat_timeout / 2))
            if not current_task.done() and time.monotonic() - last_heartbeat > heartbeat_timeout:
                stop_reason = "Browser heartbeat lost; control stopped."
                status["state"] = "stopping"
                current_stop.set()
                return

    @app.get("/", response_class=HTMLResponse)
    async def index():
        return files("cf_robot").joinpath("index.html").read_text(encoding="utf-8")

    @app.get("/api/status")
    async def get_status():
        return {**status, "running": task is not None and not task.done()}

    @app.post("/api/start")
    async def start(request: Request):
        nonlocal task, stop_event, last_heartbeat, stop_reason
        if not same_origin(request.headers.get("origin"), request.headers.get("host", "")):
            raise HTTPException(status_code=403, detail="invalid origin")
        if task is not None and not task.done():
            raise HTTPException(status_code=409, detail="session already running")
        stop_event = asyncio.Event()
        last_heartbeat = time.monotonic()
        stop_reason = None
        status.update(state="connecting", error=None)
        task = asyncio.create_task(runner(control_url, port, arm_id, fps=fps, stop=stop_event, on_status=update))
        task.add_done_callback(finished)
        asyncio.create_task(watch_browser(task, stop_event))
        return {"state": "connecting"}

    @app.post("/api/heartbeat")
    async def heartbeat(request: Request):
        nonlocal last_heartbeat
        if not same_origin(request.headers.get("origin"), request.headers.get("host", "")):
            raise HTTPException(status_code=403, detail="invalid origin")
        last_heartbeat = time.monotonic()
        return {"ok": True}

    @app.post("/api/stop")
    async def stop(request: Request):
        if not same_origin(request.headers.get("origin"), request.headers.get("host", "")):
            raise HTTPException(status_code=403, detail="invalid origin")
        if stop_event is not None:
            stop_event.set()
        status["state"] = "stopping" if task is not None and not task.done() else "idle"
        return {"state": status["state"]}

    @app.websocket("/ws/video")
    async def video(ws: WebSocket):
        if not same_origin(ws.headers.get("origin"), ws.headers.get("host", "")):
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            async with connect(video_url, additional_headers=access_headers(video_url),
                               open_timeout=10, ping_interval=10, ping_timeout=5,
                               max_size=2 * 1024 * 1024) as remote:
                async for frame in remote:
                    if isinstance(frame, bytes):
                        await ws.send_bytes(frame)
        except Exception as exc:
            LOG.info("Video relay ended: %s", exc)
            with contextlib.suppress(RuntimeError, OSError, WebSocketDisconnect):
                await ws.send_text(json.dumps({"error": str(exc)}))
        finally:
            try:
                await ws.close()
            except (RuntimeError, OSError, WebSocketDisconnect):
                pass

    @app.on_event("shutdown")
    async def shutdown():
        if stop_event is not None:
            stop_event.set()
        if task is not None:
            with contextlib.suppress(Exception):
                await task

    return app


def main():
    parser = argparse.ArgumentParser(description="Local SO-ARM101 web console")
    parser.add_argument("--url", required=True, help="https://<public-fqdn> (or wss://<public-fqdn>)")
    parser.add_argument("--port", required=True, help="Local leader serial device")
    parser.add_argument("--arm-id", required=True, help="LeRobot leader calibration ID")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--listen-port", type=int, default=8080)
    args = parser.parse_args()
    base = args.url.rstrip("/")
    if base.startswith("https://"):
        base = "wss://" + base[len("https://"):]
    elif base.startswith("http://"):
        base = "ws://" + base[len("http://"):]
    if urlsplit(base).path not in ("", "/"):
        parser.error("--url must be a hostname without a path")
    app = create_app(base, args.port, args.arm_id, args.fps)
    uvicorn.run(app, host="127.0.0.1", port=args.listen_port)


if __name__ == "__main__":
    main()
