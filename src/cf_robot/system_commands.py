"""Fixed Linux operations exposed through the authenticated follower socket."""

import asyncio
import hmac
import json
import uuid
from pathlib import Path

from fastapi import WebSocket, WebSocketDisconnect

from cf_robot.protocol import VERSION

POWER_REQUEST = Path("/var/lib/cf-robot/power-request")
POWER_STATUS = Path("/var/lib/cf-robot/power-status.json")
COMMANDS = ("ip -details address show", "sudo reboot", "sudo shutdown -h now")


async def run_command(*args):
    process = await asyncio.create_subprocess_exec(
        *args,
        stdin=asyncio.subprocess.DEVNULL,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=10)
    finally:
        if process.returncode is None:
            process.kill()
            await process.wait()
    return {
        "exit_code": process.returncode,
        "stdout": stdout.decode("utf-8", errors="replace"),
        "stderr": stderr.decode("utf-8", errors="replace"),
    }


async def queue_power(command, request_id):
    watcher = await run_command(
        "/usr/bin/systemctl", "is-active", "--quiet", "cf-robot-power.path"
    )
    if watcher["exit_code"] != 0:
        raise RuntimeError("power helper is not enabled; complete the one-time Pi setup")
    if POWER_REQUEST.exists():
        raise RuntimeError("a power request is already queued")
    temporary = POWER_REQUEST.with_name(f".power-{request_id}.tmp")
    try:
        temporary.write_text(
            json.dumps({"command": command, "request_id": request_id}), encoding="ascii"
        )
        temporary.chmod(0o600)
        temporary.replace(POWER_REQUEST)
    finally:
        temporary.unlink(missing_ok=True)


async def power_result(request_id):
    for _ in range(100):
        try:
            result = json.loads(POWER_STATUS.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            result = None
        if isinstance(result, dict) and result.get("request_id") == request_id:
            return result
        await asyncio.sleep(0.1)
    raise TimeoutError(
        "no power-helper result received; outcome is unknown, do not automatically retry"
    )


def register_system_commands(app, origin_key, reserve_power, release_power):
    """Register a one-request socket; reserve_power blocks new robot sessions."""
    command_lock = asyncio.Lock()

    @app.websocket("/system")
    async def system_command(ws: WebSocket):
        if not hmac.compare_digest(ws.headers.get("x-cf-robot-key", ""), origin_key):
            await ws.close(code=1008)
            return
        await ws.accept()
        queued = False
        reserved = False
        try:
            raw = await asyncio.wait_for(ws.receive_text(), timeout=10)
            if len(raw) > 1024:
                raise ValueError("system request is too large")
            request = json.loads(raw)
            if (
                not isinstance(request, dict)
                or set(request) != {"version", "type", "command", "confirmed"}
                or request["version"] != VERSION
                or request["type"] != "system_command"
                or request["command"] not in COMMANDS
                or type(request["confirmed"]) is not bool
            ):
                raise ValueError("unsupported system command")
            command = request["command"]
            power = command != COMMANDS[0]
            if power and not request["confirmed"]:
                raise ValueError("power commands require explicit confirmation")
            if command_lock.locked():
                raise RuntimeError("another system command is running")
            async with command_lock:
                request_id = uuid.uuid4().hex
                if power:
                    reserve_power()
                    reserved = True
                    await queue_power(command, request_id)
                    queued = True
                    await ws.send_json({
                        "type": "system_queued", "request_id": request_id,
                        "message": "Power request queued; waiting for the Pi helper.",
                    })
                    result = await power_result(request_id)
                    if result["exit_code"] != 0 and not result.get("outcome_unknown"):
                        release_power()
                        reserved = False
                else:
                    result = await run_command("/usr/sbin/ip", "-details", "address", "show")
                await ws.send_json({
                    **result, "version": VERSION, "type": "system_result",
                    "request_id": request_id, "command": command,
                })
        except WebSocketDisconnect:
            pass
        except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
            await ws.send_json({"type": "error", "message": str(exc)})
        finally:
            # Once queued, a lost connection must not undo the safety interlock.
            if reserved and not queued:
                release_power()
            try:
                await ws.close()
            except (RuntimeError, WebSocketDisconnect):
                pass
