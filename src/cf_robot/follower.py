"""Loopback-only follower server, intended to sit behind Cloudflare Tunnel."""

import argparse
import asyncio
import contextlib
import hmac
import json
import logging
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import uvicorn

from cf_robot.control import FollowerSession
from cf_robot.hardware import make_camera, make_follower
from cf_robot.protocol import VERSION, validate_command

LOG = logging.getLogger(__name__)
UPDATE_REQUEST = Path("/var/lib/cf-robot/update-request")
UPDATE_STATUS = Path("/var/lib/cf-robot/update-status.json")
CALIBRATION_ID = "so101_follower"
CALIBRATOR = Path(sys.executable).with_name("lerobot-calibrate")


def read_update_status():
    try:
        return json.loads(UPDATE_STATUS.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError):
        return {"state": "unavailable"}


def create_app(port, arm_id, camera_device=None, width=640, height=480, video_fps=10,
               factory=None, camera_factory=None, origin_key=None):
    if not origin_key or len(origin_key) < 32:
        raise ValueError("CF_ROBOT_ORIGIN_KEY must contain at least 32 characters")
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    control_lock = asyncio.Lock()
    video_lock = asyncio.Lock()
    update_requested = UPDATE_REQUEST.exists()
    calibration_requested = False
    follower_factory = factory or (lambda: make_follower(port, arm_id))
    camera_factory = camera_factory or (lambda: make_camera(camera_device, width, height, video_fps))

    @app.get("/health")
    async def health():
        return {
            "service": "cf-robot-follower",
            "version": VERSION,
            "control_busy": control_lock.locked(),
            "calibration_busy": calibration_requested,
            "update": read_update_status(),
        }

    @app.websocket("/control")
    async def control(ws: WebSocket):
        if not hmac.compare_digest(ws.headers.get("x-cf-robot-key", ""), origin_key):
            await ws.close(code=1008)
            return
        if update_requested or calibration_requested:
            await ws.close(code=1013, reason="follower maintenance in progress")
            return
        await ws.accept()
        if update_requested or calibration_requested:
            await ws.close(code=1013, reason="follower maintenance in progress")
            return
        if control_lock.locked():
            await ws.close(code=1008, reason="another operator is connected")
            return
        async with control_lock:
            session = FollowerSession(follower_factory)
            session.start()
            try:
                if not await asyncio.to_thread(session.wait_ready):
                    await ws.send_json({"type": "error", "message": session.error or "follower connection timed out"})
                    return
                await ws.send_json(
                    {
                        "type": "ready",
                        "version": VERSION,
                        "observation": session.initial_observation,
                    }
                )
                last_seq = 0
                last_status = 0.0
                while True:
                    if session.error:
                        await ws.send_json({"type": "error", "message": session.error})
                        return
                    try:
                        raw = await asyncio.wait_for(ws.receive_text(), timeout=0.05)
                    except asyncio.TimeoutError:
                        raw = None
                    if raw is not None:
                        try:
                            seq, action = validate_command(json.loads(raw), last_seq)
                            session.submit(action)
                            last_seq = seq
                        except (ValueError, TypeError) as exc:
                            await ws.send_json({"type": "error", "message": str(exc)})
                            return
                    if time.monotonic() - last_status >= 0.1:
                        with session.lock:
                            status = {
                                "type": "status",
                                "state": session.state,
                                "observation": dict(session.observation or {}),
                                "observed_at": session.observed_at,
                            }
                        await ws.send_json(status)
                        last_status = time.monotonic()
            except WebSocketDisconnect:
                pass
            finally:
                await asyncio.to_thread(session.stop)
                try:
                    await ws.close()
                except (RuntimeError, OSError, WebSocketDisconnect):
                    pass

    @app.websocket("/update")
    async def update(ws: WebSocket):
        nonlocal update_requested
        if not hmac.compare_digest(ws.headers.get("x-cf-robot-key", ""), origin_key):
            await ws.close(code=1008)
            return
        await ws.accept()

        async def reject(message):
            await ws.send_json({"type": "error", "message": message})
            await ws.close(code=1008)

        try:
            raw = await asyncio.wait_for(ws.receive_text(), timeout=10)
            message = json.loads(raw)
        except asyncio.TimeoutError:
            await reject("timed out waiting for an update request")
            return
        except json.JSONDecodeError:
            await reject("invalid update request")
            return
        except WebSocketDisconnect:
            return

        if (
            isinstance(message, dict)
            and message.get("version") == VERSION
            and message.get("type") == "update_status"
            and isinstance(message.get("request_id"), str)
            and len(message["request_id"]) == 32
            and all(char in "0123456789abcdef" for char in message["request_id"])
            and set(message) == {"version", "type", "request_id"}
        ):
            request_id = message["request_id"]
            status = read_update_status()
            if not status or status.get("request_id") != request_id:
                queued_id = ""
                try:
                    queued_id = UPDATE_REQUEST.read_text(encoding="ascii").strip()
                except OSError:
                    pass
                state = "queued" if queued_id == request_id else "pending"
                status = {"request_id": request_id, "state": state}
            await ws.send_json({"type": "update_status", "version": VERSION, **status})
            await ws.close()
            return

        if message != {"version": VERSION, "type": "update"}:
            await reject("unsupported update request")
            return
        if update_requested or UPDATE_REQUEST.exists():
            await reject("an update is already queued")
            return
        if calibration_requested:
            await reject("follower calibration is in progress")
            return
        if control_lock.locked() or video_lock.locked():
            await reject("stop control and video sessions before updating")
            return
        updater = await asyncio.to_thread(
            subprocess.run,
            ["systemctl", "is-active", "--quiet", "cf-robot-update.path"],
            check=False,
        )
        if updater.returncode != 0:
            await reject("remote updater is not installed or enabled; run one-time setup over SSH")
            return

        request_id = uuid.uuid4().hex
        temporary = UPDATE_REQUEST.with_name(f"{UPDATE_REQUEST.name}.{request_id}.tmp")

        def stage_request():
            temporary.write_text(request_id, encoding="ascii")
            temporary.chmod(0o600)

        try:
            await asyncio.to_thread(stage_request)
            if (
                update_requested
                or calibration_requested
                or UPDATE_REQUEST.exists()
                or control_lock.locked()
                or video_lock.locked()
            ):
                temporary.unlink(missing_ok=True)
                await reject("follower became busy; retry when no session is active")
                return
            update_requested = True
            await ws.send_json({"type": "update_queued", "request_id": request_id})
            await asyncio.to_thread(os.replace, temporary, UPDATE_REQUEST)
        except Exception:
            update_requested = False
            temporary.unlink(missing_ok=True)
            raise
        finally:
            try:
                await ws.close()
            except (RuntimeError, OSError, WebSocketDisconnect):
                pass

    @app.websocket("/calibrate")
    async def calibrate(ws: WebSocket):
        nonlocal calibration_requested
        if not hmac.compare_digest(ws.headers.get("x-cf-robot-key", ""), origin_key):
            await ws.close(code=1008)
            return
        await ws.accept()

        async def reject(message):
            await ws.send_json({"type": "error", "message": message})
            await ws.close(code=1008)

        try:
            raw = await asyncio.wait_for(ws.receive_text(), timeout=10)
            message = json.loads(raw)
        except asyncio.TimeoutError:
            await reject("timed out waiting for calibration request")
            return
        except json.JSONDecodeError:
            await reject("invalid calibration request")
            return
        except WebSocketDisconnect:
            return
        if message != {"version": VERSION, "type": "start"}:
            await reject("unsupported calibration request")
            return
        if calibration_requested or update_requested or UPDATE_REQUEST.exists():
            await reject("follower maintenance is already in progress")
            return
        if control_lock.locked():
            await reject("stop the follower control session before calibration")
            return
        if arm_id != CALIBRATION_ID:
            await reject(f"this endpoint only calibrates {CALIBRATION_ID}")
            return
        if not port:
            await reject("follower serial port is not configured")
            return
        if not CALIBRATOR.is_file() or not os.access(CALIBRATOR, os.X_OK):
            await reject("LeRobot calibrator is missing from the Pi virtual environment")
            return

        calibration_requested = True
        process = None
        output_task = None
        receive_task = None
        wait_task = None
        client_disconnected = False
        input_error = None

        async def relay_output():
            while True:
                chunk = await process.stdout.read(1024)
                if not chunk:
                    return
                await ws.send_json(
                    {
                        "type": "calibration_output",
                        "text": chunk.decode("utf-8", errors="replace"),
                    }
                )

        async def stop_child():
            if process is None or process.returncode is not None:
                return
            process.terminate()
            try:
                await asyncio.wait_for(process.wait(), timeout=3)
            except asyncio.TimeoutError:
                process.kill()
                await process.wait()

        try:
            child_env = os.environ.copy()
            for name in (
                "CF_ROBOT_ORIGIN_KEY",
                "CF_ACCESS_CLIENT_ID",
                "CF_ACCESS_CLIENT_SECRET",
                "CF_ROBOT_BRIDGE_TOKEN",
            ):
                child_env.pop(name, None)
            child_env["HF_HOME"] = "/var/lib/cf-robot"
            child_env["PYTHONUNBUFFERED"] = "1"
            process = await asyncio.create_subprocess_exec(
                str(CALIBRATOR),
                "--robot.type=so101_follower",
                f"--robot.port={port}",
                f"--robot.id={CALIBRATION_ID}",
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                cwd="/opt/cf-robot",
                env=child_env,
            )
            await ws.send_json({"type": "calibration_started", "version": VERSION})
            output_task = asyncio.create_task(relay_output())
            wait_task = asyncio.create_task(process.wait())
            receive_task = asyncio.create_task(ws.receive_text())

            while not wait_task.done():
                done, _ = await asyncio.wait(
                    {wait_task, receive_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if wait_task in done:
                    break
                try:
                    incoming = json.loads(receive_task.result())
                except json.JSONDecodeError:
                    input_error = "invalid calibration input message"
                    await stop_child()
                    break
                except WebSocketDisconnect:
                    client_disconnected = True
                    await stop_child()
                    break

                if incoming == {"version": VERSION, "type": "cancel"}:
                    await stop_child()
                    break
                if (
                    not isinstance(incoming, dict)
                    or incoming.get("version") != VERSION
                    or incoming.get("type") != "input"
                    or set(incoming) != {"version", "type", "text"}
                    or not isinstance(incoming.get("text"), str)
                    or len(incoming["text"]) > 256
                    or "\n" in incoming["text"]
                    or "\r" in incoming["text"]
                ):
                    input_error = "invalid calibration input"
                    await stop_child()
                    break
                if process.stdin is None or process.returncode is not None:
                    input_error = "calibration process is no longer accepting input"
                    break
                process.stdin.write((incoming["text"] + "\n").encode("utf-8"))
                try:
                    await process.stdin.drain()
                except (BrokenPipeError, ConnectionResetError):
                    input_error = "calibration process closed its input"
                    break
                receive_task = asyncio.create_task(ws.receive_text())

            if wait_task is not None:
                await wait_task
            if output_task is not None:
                await output_task
            if input_error and not client_disconnected:
                await ws.send_json({"type": "error", "message": input_error})
            elif not client_disconnected:
                await ws.send_json(
                    {
                        "type": "calibration_finished",
                        "exit_code": process.returncode,
                    }
                )
        except WebSocketDisconnect:
            client_disconnected = True
            await stop_child()
        except Exception:
            LOG.exception("Remote follower calibration failed")
            await stop_child()
            if not client_disconnected:
                try:
                    await ws.send_json(
                        {"type": "error", "message": "calibration process failed; inspect its output"}
                    )
                except (RuntimeError, OSError, WebSocketDisconnect):
                    pass
        finally:
            if process is not None and process.returncode is None:
                await stop_child()
            if receive_task is not None and not receive_task.done():
                receive_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, WebSocketDisconnect):
                    await receive_task
            if output_task is not None and not output_task.done():
                output_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, WebSocketDisconnect):
                    await output_task
            calibration_requested = False
            try:
                await ws.close()
            except (RuntimeError, OSError, WebSocketDisconnect):
                pass

    @app.websocket("/video")
    async def video(ws: WebSocket):
        if not hmac.compare_digest(ws.headers.get("x-cf-robot-key", ""), origin_key):
            await ws.close(code=1008)
            return
        if update_requested:
            await ws.close(code=1013, reason="follower update in progress")
            return
        await ws.accept()
        if camera_device is None:
            await ws.close(code=1008, reason="camera not configured")
            return
        if video_lock.locked():
            await ws.close(code=1008, reason="camera already in use")
            return
        async with video_lock:
            camera = None
            camera_connected = False
            disconnected = None
            try:
                camera = camera_factory()
                await asyncio.to_thread(camera.connect)
                camera_connected = True
                import cv2

                async def watch_disconnect():
                    while True:
                        event = await ws.receive()
                        if event["type"] == "websocket.disconnect":
                            return

                disconnected = asyncio.create_task(watch_disconnect())
                while True:
                    start = time.monotonic()
                    frame = await asyncio.to_thread(camera.read)
                    if disconnected.done():
                        break
                    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    if not ok:
                        raise RuntimeError("JPEG encoding failed")
                    await ws.send_bytes(encoded.tobytes())
                    await asyncio.sleep(max(0, 1 / video_fps - (time.monotonic() - start)))
            except (WebSocketDisconnect, RuntimeError, OSError):
                LOG.info("Video stream ended", exc_info=True)
            finally:
                if disconnected is not None:
                    disconnected.cancel()
                    with contextlib.suppress(asyncio.CancelledError):
                        await disconnected
                if camera is not None and camera_connected:
                    try:
                        await asyncio.to_thread(camera.disconnect)
                    except Exception:
                        LOG.exception("Could not disconnect camera")
                try:
                    await ws.close()
                except (RuntimeError, OSError, WebSocketDisconnect):
                    pass

    return app


def main():
    parser = argparse.ArgumentParser(description="SO-ARM101 follower service (Cloudflare Tunnel origin)")
    parser.add_argument("--port", required=True, help="Follower serial device, preferably /dev/serial/by-id/...")
    parser.add_argument("--arm-id", required=True, help="LeRobot follower calibration ID")
    parser.add_argument("--listen-port", type=int, default=8765)
    parser.add_argument("--camera", help="Follower USB camera, preferably /dev/v4l/by-id/...")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=10)
    args = parser.parse_args()
    if args.camera_fps < 1 or args.camera_fps > 30:
        parser.error("--camera-fps must be between 1 and 30")
    logging.basicConfig(level=logging.INFO)
    uvicorn.run(
        create_app(args.port, args.arm_id, args.camera, args.camera_width, args.camera_height,
                   args.camera_fps, origin_key=os.environ.get("CF_ROBOT_ORIGIN_KEY")),
        host="127.0.0.1",
        port=args.listen_port,
    )


if __name__ == "__main__":
    main()
