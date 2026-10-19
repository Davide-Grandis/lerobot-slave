"""Laptop CLI: read the local SO-ARM101 leader and stream to the Pi."""

import argparse
import asyncio
import contextlib
import json
import time

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.hardware import make_leader
from cf_robot.protocol import VERSION, action_message


async def stream_status(socket, on_status):
    async for raw in socket:
        message = json.loads(raw)
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "follower error"))
        if message.get("type") == "status":
            on_status(message)
    raise ConnectionError("follower connection closed")


async def run_teleop(url, port, arm_id, fps=30, stop=None, on_status=None, leader_factory=None):
    if not 1 <= fps <= 60:
        raise ValueError("fps must be between 1 and 60")
    stop = stop if stop is not None else asyncio.Event()
    on_status = on_status or (lambda message: None)
    leader_factory = leader_factory or (lambda: make_leader(port, arm_id))
    headers = access_headers(url)
    leader = None
    reader = None
    async with connect(url, additional_headers=headers, open_timeout=10, ping_interval=10, ping_timeout=5) as socket:
        first = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
        if first.get("type") != "ready" or first.get("version") != VERSION:
            raise RuntimeError(first.get("message", "follower did not become ready"))
        on_status({"type": "status", "state": "connecting leader"})
        try:
            leader = leader_factory()
            await asyncio.to_thread(leader.connect, calibrate=False)
            reader = asyncio.create_task(stream_status(socket, on_status))
            on_status({"type": "status", "state": "running"})
            seq = 0
            period = 1 / fps
            while not stop.is_set():
                started = time.monotonic()
                if reader.done():
                    reader.result()
                action = await asyncio.to_thread(leader.get_action)
                if reader.done():
                    reader.result()
                seq += 1
                await socket.send(json.dumps(action_message(seq, action), separators=(",", ":")))
                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    try:
                        await asyncio.wait_for(stop.wait(), timeout=remaining)
                    except asyncio.TimeoutError:
                        pass
        finally:
            if reader is not None:
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionError, RuntimeError):
                    await reader
            if leader is not None:
                await asyncio.to_thread(leader.disconnect)


def main():
    parser = argparse.ArgumentParser(description="Teleoperate a remote SO-ARM101 follower")
    parser.add_argument("--url", required=True, help="wss://<public-fqdn>/control")
    parser.add_argument("--port", required=True, help="Local leader serial device")
    parser.add_argument("--arm-id", required=True, help="LeRobot leader calibration ID")
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    try:
        asyncio.run(run_teleop(args.url, args.port, args.arm_id, args.fps, on_status=lambda m: print(m["state"])))
    except KeyboardInterrupt:
        print("Stopped")
    except (OSError, ValueError, RuntimeError, ConnectionError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Teleoperation stopped:\n{exc}\n")


if __name__ == "__main__":
    main()
