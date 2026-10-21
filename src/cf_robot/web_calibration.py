"""Explicit, browser-supervised LeRobot calibration, one arm at a time."""

import asyncio
import contextlib
import json
import os
import sys
import time
from pathlib import Path

from fastapi import HTTPException, Request
from websockets.asyncio.client import connect

from cf_robot.auth import access_headers
from cf_robot.protocol import VERSION


class BrowserCalibration:
    def __init__(self, url, port, arm_id, heartbeat_timeout=3):
        self.url = url.rstrip("/") + "/calibrate"
        self.port = port
        self.arm_id = arm_id
        self.heartbeat_timeout = heartbeat_timeout
        self.task = None
        self.stop_event = asyncio.Event()
        self.inputs = asyncio.Queue(maxsize=1)
        self.status = {"state": "idle", "output": "", "error": None}
        self.last_heartbeat = 0

    @property
    def running(self):
        return self.task is not None and not self.task.done()

    def snapshot(self):
        return {**self.status, "running": self.running}

    def append(self, text):
        self.status["output"] = (self.status["output"] + text)[-64000:]

    def start(self, side):
        if self.running:
            raise ValueError("calibration already running")
        if side not in ("leader", "follower"):
            raise ValueError("select one arm to calibrate")
        if side == "leader" and not self.port:
            raise ValueError("configure the leader serial port first")
        self.stop_event = asyncio.Event()
        self.inputs = asyncio.Queue(maxsize=1)
        self.last_heartbeat = time.monotonic()
        self.status = {"state": "starting", "side": side, "output": "", "error": None}
        self.task = asyncio.create_task(self.run(side))

    def stop(self):
        self.stop_event.set()
        if self.running:
            self.status["state"] = "stopping"

    async def supervise(self, operation):
        task = asyncio.create_task(operation)
        try:
            while not task.done():
                if self.stop_event.is_set():
                    self.status["state"] = "cancelled"
                    return
                if time.monotonic() - self.last_heartbeat > self.heartbeat_timeout:
                    raise RuntimeError("Browser heartbeat lost; calibration cancelled. Inspect the arm.")
                await asyncio.wait({task}, timeout=0.1)
            await task
        finally:
            if not task.done():
                task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def local(self):
        executable = Path(sys.executable).with_name("lerobot-calibrate")
        if not executable.is_file():
            raise RuntimeError("lerobot-calibrate missing from the controller environment")
        env = {key: value for key, value in os.environ.items() if key not in (
            "CF_ROBOT_ORIGIN_KEY", "CF_ACCESS_CLIENT_ID",
            "CF_ACCESS_CLIENT_SECRET", "CF_ROBOT_BRIDGE_TOKEN",
        )}
        env["PYTHONUNBUFFERED"] = "1"
        process = await asyncio.create_subprocess_exec(
            str(executable), "--teleop.type=so101_leader",
            f"--teleop.port={self.port}", f"--teleop.id={self.arm_id}",
            stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT, env=env,
        )

        async def output():
            while chunk := await process.stdout.read(1024):
                self.append(chunk.decode("utf-8", errors="replace"))

        async def input_lines():
            while True:
                text = await self.inputs.get()
                process.stdin.write((text + "\n").encode())
                await process.stdin.drain()

        tasks = [asyncio.create_task(output()), asyncio.create_task(input_lines())]
        try:
            self.status["state"] = "running"
            wait = asyncio.create_task(process.wait())
            tasks.append(wait)
            while not wait.done():
                done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                for task in done:
                    task.result()
                    if task is tasks[1]:
                        raise RuntimeError("calibration input closed unexpectedly")
                if tasks[0].done() and not wait.done():
                    await wait
            code = wait.result()
            await tasks[0]
            if code:
                raise RuntimeError(f"LeRobot calibration exited with code {code}")
        finally:
            if process.returncode is None:
                process.terminate()
                try:
                    await asyncio.wait_for(process.wait(), 3)
                except TimeoutError:
                    process.kill()
                    await process.wait()
            for task in tasks:
                task.cancel()
            for task in tasks:
                with contextlib.suppress(asyncio.CancelledError, OSError):
                    await task

    async def remote(self):
        async with connect(
            self.url, additional_headers=access_headers(self.url),
            open_timeout=10, ping_interval=10, ping_timeout=5,
        ) as socket:
            await socket.send(json.dumps({"version": VERSION, "type": "start"}))

            async def input_lines():
                while True:
                    await socket.send(json.dumps({
                        "version": VERSION, "type": "input", "text": await self.inputs.get(),
                    }))

            sender = asyncio.create_task(input_lines())
            receiver = None
            try:
                while True:
                    receiver = asyncio.create_task(socket.recv())
                    done, _ = await asyncio.wait(
                        {sender, receiver}, timeout=15 if self.status["state"] == "starting" else None,
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    if not done:
                        raise RuntimeError("Follower calibration did not acknowledge startup")
                    if sender in done:
                        sender.result()
                        raise RuntimeError("Follower calibration input stopped")
                    message = json.loads(receiver.result())
                    if not isinstance(message, dict):
                        raise RuntimeError("Invalid follower calibration response")
                    kind = message.get("type")
                    if kind == "calibration_output":
                        self.append(message.get("text", ""))
                    elif kind == "calibration_started":
                        if message.get("version") != VERSION:
                            raise RuntimeError("Unsupported follower calibration version")
                        self.status["state"] = "running"
                    elif kind == "calibration_finished":
                        if message.get("exit_code") != 0:
                            raise RuntimeError("Follower calibration failed; inspect output.")
                        return
                    elif kind == "error":
                        raise RuntimeError(message.get("message", "Follower rejected calibration"))
                    else:
                        raise RuntimeError("Unsupported follower calibration response")
            finally:
                for task in (sender, receiver):
                    if task is not None:
                        task.cancel()
                await asyncio.gather(
                    *(task for task in (sender, receiver) if task is not None),
                    return_exceptions=True,
                )
                with contextlib.suppress(Exception):
                    await socket.send(json.dumps({"version": VERSION, "type": "cancel"}))

    async def run(self, side):
        try:
            await self.supervise(self.local() if side == "leader" else self.remote())
            if self.status["state"] != "cancelled":
                self.status["state"] = "finished"
        except Exception as exc:  # noqa: BLE001 - preserve SDK/process failures and cleanup.
            self.status.update(state="error", error=str(exc))
        finally:
            self.append("\nInspect the arm. Restart monitoring to reload and verify calibration.\n")


def install_calibration_routes(app, calibration, require_origin, hardware_busy, stop_latched):
    @app.get("/api/calibration/status")
    async def status(request: Request):
        require_origin(request)
        return calibration.snapshot()

    @app.post("/api/calibration/start")
    async def start(request: Request):
        require_origin(request)
        try:
            body = await request.json()
        except ValueError as exc:
            raise HTTPException(409, "provide a calibration request as JSON") from exc
        if not isinstance(body, dict) or set(body) != {"side", "confirmation"} or body["confirmation"] != "CALIBRATE":
            raise HTTPException(409, "type CALIBRATE after securing the selected arm")
        if hardware_busy() or stop_latched():
            raise HTTPException(409, "stop monitoring/teleoperation and clear the safety latch first")
        try:
            calibration.start(body["side"])
        except ValueError as exc:
            raise HTTPException(409, str(exc)) from exc
        return calibration.snapshot()

    @app.post("/api/calibration/input")
    async def input_line(request: Request):
        require_origin(request)
        try:
            body = await request.json()
        except ValueError as exc:
            raise HTTPException(409, "provide a calibration reply as JSON") from exc
        text = body.get("text") if isinstance(body, dict) else None
        if (
            not calibration.running or calibration.stop_event.is_set()
            or calibration.status["state"] != "running"
        ):
            raise HTTPException(409, "calibration is not accepting input")
        if not isinstance(text, str) or len(text) > 200 or any(not c.isprintable() for c in text):
            raise HTTPException(409, "provide one printable input line of at most 200 characters")
        try:
            calibration.inputs.put_nowait(text)
        except asyncio.QueueFull as exc:
            raise HTTPException(409, "wait for the previous input") from exc
        return {"ok": True}

    @app.post("/api/calibration/heartbeat")
    async def heartbeat(request: Request):
        require_origin(request)
        calibration.last_heartbeat = time.monotonic()
        return {"ok": True}

    @app.post("/api/calibration/stop")
    async def stop(request: Request):
        require_origin(request)
        calibration.stop()
        return calibration.snapshot()
