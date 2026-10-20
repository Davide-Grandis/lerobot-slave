"""Browser-owned pose monitor; hardware stays behind the paired loopback API."""

import asyncio
import contextlib
import json
import threading
import time

from fastapi import HTTPException, Request
from websockets.asyncio.client import connect

from cf_robot.auth import access_headers
from cf_robot.pose_monitor import stream_leader_pose, validate_pose_command
from cf_robot.protocol import VERSION


class BrowserPoseMonitor:
    def __init__(self, url, port, arm_id, heartbeat_timeout=3, leader_reader=None):
        self.url = url.rstrip("/") + "/monitor"
        self.port = port
        self.arm_id = arm_id
        self.heartbeat_timeout = heartbeat_timeout
        self.leader_reader = leader_reader or stream_leader_pose
        self.task = None
        self.socket = None
        self.stop_event = threading.Event()
        self.last_heartbeat = 0
        self.command_id = 0
        self.pending_since = None
        self.status = {"state": "idle", "error": None}

    @property
    def running(self):
        return self.task is not None and not self.task.done()

    def snapshot(self):
        return {**self.status, "running": self.running}

    def start(self):
        if self.running:
            raise RuntimeError("pose monitor is already running")
        self.stop_event = threading.Event()
        self.last_heartbeat = time.monotonic()
        self.pending_since = None
        self.status = {
            "state": "connecting", "error": None,
            "leader": None, "follower": None, "leader_torque": None,
            "follower_torque": None, "command": None, "command_pending": False,
        }
        self.task = asyncio.create_task(self.run())

    def stop(self):
        if self.running:
            self.status["state"] = "stopping"
            self.stop_event.set()

    def publish(self, message):
        kind = message.get("type")
        if kind == "pose_sample" and message.get("side") in ("leader", "follower"):
            self.status[message["side"]] = message
        elif kind in ("leader_torque", "follower_torque"):
            self.status[kind] = message
        elif kind in ("command_queued", "command_progress", "command_result"):
            self.status["command"] = message
            if kind == "command_result" and message.get("command_id") == self.command_id:
                self.pending_since = None
                self.status["command_pending"] = False

    async def command(self, body):
        if not self.running or self.status["state"] != "ready" or self.socket is None:
            raise RuntimeError("start the pose monitor before sending commands")
        if self.status["command_pending"]:
            raise RuntimeError("wait for the current command to finish")
        if not isinstance(body, dict) or set(body) not in (
            {"joint_id", "mode", "value"}, {"joint_id", "mode", "enabled"}
        ):
            raise ValueError("provide one joint, mode, and value or enabled flag")
        command = validate_pose_command({
            **body, "version": VERSION, "type": "command",
            "command_id": self.command_id + 1,
        })
        self.command_id = command["command_id"]
        self.status["command_pending"] = True
        self.pending_since = time.monotonic()
        self.status["command"] = {
            "type": "command_queued", "command_id": self.command_id,
            "joint_id": command["joint_id"], "state": "sending",
        }
        try:
            await self.socket.send(json.dumps(command))
        except Exception:
            self.stop()
            raise
        return {"command_id": self.command_id}

    async def run(self):
        leader_task = None
        watchdog = asyncio.create_task(self.watch_browser())
        try:
            async with connect(
                self.url, additional_headers=access_headers(self.url),
                open_timeout=10, ping_interval=10, ping_timeout=5,
                max_size=2 * 1024 * 1024,
            ) as socket:
                self.socket = socket
                await socket.send(json.dumps({"version": VERSION, "type": "start"}))
                started = json.loads(await asyncio.wait_for(socket.recv(), 15))
                if (
                    started.get("type") != "monitor_started"
                    or started.get("version") != VERSION
                ):
                    raise RuntimeError(started.get("message", "invalid monitor response"))
                if self.port and not self.stop_event.is_set():
                    loop = asyncio.get_running_loop()

                    def publish(message):
                        loop.call_soon_threadsafe(self.publish, message)

                    leader_task = asyncio.create_task(asyncio.to_thread(
                        self.leader_reader, self.port, self.arm_id,
                        self.stop_event, publish,
                    ))
                if not self.stop_event.is_set():
                    self.status["state"] = "ready"
                stop_sent_at = None
                while True:
                    if leader_task is not None and leader_task.done():
                        leader_task.result()
                        if not self.stop_event.is_set():
                            raise RuntimeError("leader sampling stopped unexpectedly")
                    if self.stop_event.is_set() and stop_sent_at is None:
                        await socket.send(json.dumps({"version": VERSION, "type": "stop"}))
                        stop_sent_at = time.monotonic()
                    if stop_sent_at is not None and time.monotonic() - stop_sent_at > 7:
                        raise RuntimeError("follower did not confirm monitor shutdown")
                    try:
                        message = json.loads(await asyncio.wait_for(socket.recv(), 0.2))
                    except TimeoutError:
                        continue
                    if message.get("type") in ("error", "monitor_cancelled"):
                        raise RuntimeError(message.get("message", "monitor cancelled"))
                    if message.get("type") == "monitor_finished":
                        break
                    self.publish(message)
        except Exception as exc:  # noqa: BLE001 - surface hardware failures without losing cleanup
            self.status.update(state="error", error=str(exc))
        finally:
            self.socket = None
            self.stop_event.set()
            # Retain ownership until serial I/O has actually disconnected.
            if leader_task is not None:
                try:
                    await leader_task
                except Exception as exc:  # noqa: BLE001 - serial adapters use SDK-specific errors
                    self.status.update(state="error", error=str(exc))
            watchdog.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await watchdog
            self.status["command_pending"] = False
            if self.status["state"] != "error":
                self.status["state"] = "idle"

    async def watch_browser(self):
        while True:
            await asyncio.sleep(0.2)
            now = time.monotonic()
            if now - self.last_heartbeat > self.heartbeat_timeout:
                self.status["error"] = "Browser heartbeat lost; pose monitor stopped."
                self.stop()
                return
            if self.pending_since is not None and now - self.pending_since > 15:
                self.status["error"] = "Command result timed out; inspect the follower."
                self.stop()
                return


def install_pose_routes(app, monitor, require_origin, control_busy, stop_latched):
    @app.get("/api/pose/status")
    async def status(request: Request):
        require_origin(request)
        return monitor.snapshot()

    @app.post("/api/pose/start")
    async def start(request: Request):
        require_origin(request)
        if control_busy() or stop_latched():
            raise HTTPException(409, "stop teleoperation and clear its safety latch first")
        try:
            body = await request.json()
            if body != {}:
                raise ValueError("pose monitor startup preserves the current torque state")
            # Recheck after reading the request: another start may have won the race.
            if control_busy() or stop_latched():
                raise RuntimeError("teleoperation is active or safety-latched")
            monitor.start()
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(409, str(exc)) from exc
        return monitor.snapshot()

    @app.post("/api/pose/stop")
    async def stop(request: Request):
        require_origin(request)
        monitor.stop()
        return monitor.snapshot()

    @app.post("/api/pose/heartbeat")
    async def heartbeat(request: Request):
        require_origin(request)
        monitor.last_heartbeat = time.monotonic()
        return {"ok": True}

    @app.post("/api/pose/command")
    async def command(request: Request):
        require_origin(request)
        try:
            return await monitor.command(await request.json())
        except (ValueError, RuntimeError) as exc:
            raise HTTPException(409, str(exc)) from exc
