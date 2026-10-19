"""Mac-side WSS clients for recording and applying a follower parking pose."""

import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
import json
import os
from pathlib import Path
import sys

from cf_robot.auth import access_headers
from cf_robot.parking import (
    PARKING_STEP_DELAY_S,
    PARKING_STEP_DEG,
    parking_steps,
    print_pose,
)
from cf_robot.protocol import VERSION, action_message, validate_action

PARKING_FILE = Path.home() / ".config/cf-robot/follower-parking.json"


@asynccontextmanager
async def follower_control(url):
    from websockets.asyncio.client import connect

    async with connect(
        url,
        additional_headers=access_headers(url),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
    ) as socket:
        first = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
        if first.get("type") == "error":
            raise RuntimeError(first.get("message", "follower failed to connect"))
        if first.get("type") != "ready" or first.get("version") != VERSION:
            raise RuntimeError("follower did not send a compatible ready message")
        try:
            observation = validate_action(first["observation"])
        except (KeyError, TypeError, ValueError) as exc:
            raise RuntimeError(
                "Pi follower does not provide its measured pose; update its code"
            ) from exc
        yield socket, observation


def save_parking_pose(pose):
    PARKING_FILE.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    PARKING_FILE.parent.chmod(0o700)
    temporary = PARKING_FILE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(
            {"version": 1, "arm_id": "so101_follower", "joint_positions": pose},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.chmod(0o600)
    temporary.replace(PARKING_FILE)


def load_parking_pose():
    try:
        saved = json.loads(PARKING_FILE.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read local parking pose {PARKING_FILE}: {exc}") from exc
    if not isinstance(saved, dict):
        raise RuntimeError("local parking pose must contain a JSON object")
    if saved.get("version") != 1 or saved.get("arm_id") != "so101_follower":
        raise RuntimeError("local parking pose has an unsupported version or arm ID")
    try:
        return validate_action(saved["joint_positions"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid local parking pose: {exc}") from exc


async def receive_errors(socket):
    async for raw in socket:
        message = json.loads(raw)
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "follower rejected parking command"))
    raise ConnectionError("follower closed the control WebSocket")


async def record_pose(url):
    async with follower_control(url) as (_, observation):
        print_pose("Measured follower pose:", observation)
        if PARKING_FILE.exists():
            print(f"This will replace {PARKING_FILE}.")
        if input("Type RECORD to save this pose on the Mac: ").strip() != "RECORD":
            print("Not saved.")
            return
        save_parking_pose(observation)
    print(f"Parking pose saved locally to {PARKING_FILE}")


async def set_parking_pose(url):
    target = load_parking_pose()
    async with follower_control(url) as (socket, current):
        print_pose("Current follower pose:", current)
        print_pose("Saved parking pose:", target)
        steps = list(parking_steps(current, target, max_step=PARKING_STEP_DEG))
        if not steps:
            print("Follower is already at the saved parking pose.")
            return
        print(
            f"Move one servo at a time in {len(steps)} small steps, at most "
            f"{PARKING_STEP_DEG:g} degree per step."
        )
        if input("Clear the workspace, then type MOVE to proceed: ").strip() != "MOVE":
            print("Move cancelled.")
            return

        reader = asyncio.create_task(receive_errors(socket))
        try:
            for sequence, (_, pose) in enumerate(steps, start=1):
                if reader.done():
                    reader.result()
                message = action_message(sequence, pose)
                await socket.send(json.dumps(message, separators=(",", ":")))
                await asyncio.sleep(PARKING_STEP_DELAY_S)
            await asyncio.sleep(0.15)
            if reader.done():
                reader.result()
            print("Parking commands sent. Inspect the follower before proceeding.")
        finally:
            reader.cancel()
            with suppress(asyncio.CancelledError, ConnectionError, RuntimeError):
                await reader


def main(mode):
    parser = argparse.ArgumentParser(description=f"Remote follower parking {mode} over WSS")
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")
    try:
        operation = record_pose if mode == "record" else set_parking_pose
        asyncio.run(operation(args.url))
    except KeyboardInterrupt:
        parser.exit(130, "\nInterrupted; check that the follower is safe.\n")
    except Exception as exc:
        parser.exit(1, f"Parking {mode} stopped: {exc}\n")


def record_main():
    main("record")


def set_main():
    main("set")


if __name__ == "__main__":
    record_main()
