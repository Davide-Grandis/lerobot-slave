"""Controller-laptop CLI for fixed follower servo-maintenance operations."""

import argparse
import asyncio
import json
import os
from pathlib import Path
import tempfile
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.protocol import JOINTS, VERSION

LOCAL_LIMITS_FILE = Path.home() / ".config/cf-robot/follower-servo-limits.json"


def maintenance_url(control_url):
    parsed = urlsplit(control_url)
    if parsed.scheme != "wss" or not parsed.hostname or parsed.path not in ("", "/", "/control"):
        raise ValueError("control URL must be wss://<public-hostname>/control")
    return urlunsplit(("wss", parsed.netloc, "/maintenance", "", ""))


def save_local_limits(record):
    LOCAL_LIMITS_FILE.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    LOCAL_LIMITS_FILE.parent.chmod(0o700)
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=LOCAL_LIMITS_FILE.parent,
        prefix=f".{LOCAL_LIMITS_FILE.name}.",
        suffix=".tmp",
        delete=False,
    ) as temporary:
        json.dump(record, temporary, indent=2)
        temporary.write("\n")
        temporary_path = Path(temporary.name)
    temporary_path.chmod(0o600)
    temporary_path.replace(LOCAL_LIMITS_FILE)
    return LOCAL_LIMITS_FILE


async def _connect(url):
    return await connect(
        url,
        additional_headers=access_headers(url),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
        max_size=2 * 1024 * 1024,
    )


def _print_limits(record, current=None):
    print("Follower servo limits (raw encoder ticks):")
    print(f"{'Joint':24} {'Min':>8} {'Max':>8} {'Current':>9}")
    limits = record["limits"]
    for joint in JOINTS:
        entry = limits[joint]
        position = current[joint] if current is not None else "-"
        print(f"{joint:24} {entry['min']:8d} {entry['max']:8d} {position:>9}")


async def read_limits(control_url):
    async with await _connect(maintenance_url(control_url)) as socket:
        await socket.send(json.dumps({"version": VERSION, "type": "read_limits"}))
        response = json.loads(await socket.recv())
    if response.get("type") == "error":
        raise RuntimeError(response.get("message", "Pi rejected servo-limit read"))
    if response.get("type") != "servo_limits":
        raise RuntimeError("Pi returned an invalid servo-limit response")
    return response["record"], response["current"]


async def limits_command(control_url):
    record, current = await read_limits(control_url)
    local_path = save_local_limits(record)
    _print_limits(record, current)
    print(f"Saved on Pi and Mac: {local_path}")


async def home_command(control_url, speed_percent):
    endpoint = maintenance_url(control_url)
    async with await _connect(endpoint) as socket:
        await socket.send(
            json.dumps(
                {
                    "version": VERSION,
                    "type": "go_home",
                    "speed_percent": speed_percent,
                },
                separators=(",", ":"),
            )
        )
        plan = json.loads(await socket.recv())
        if plan.get("type") == "error":
            raise RuntimeError(plan.get("message", "Pi rejected go-home request"))
        if plan.get("type") != "home_plan":
            raise RuntimeError("Pi returned an invalid go-home plan")

        record = plan["record"]
        current = plan["current"]
        target = plan["target"]
        save_local_limits(record)
        _print_limits(record, current)
        print("SDK-compatible home targets (raw encoder ticks):")
        for joint in JOINTS:
            print(f"  {joint}: {current[joint]} -> {target[joint]}")
        print(
            "This moves all joints toward the selected home pose at 10% servo "
            f"speed, then restores {speed_percent}% speed."
        )
        if input("Clear the workspace and type MOVE to proceed: ").strip() != "MOVE":
            await socket.send(json.dumps({"version": VERSION, "type": "cancel_home"}))
            result = json.loads(await socket.recv())
            print(result.get("reason", "Go-home cancelled."))
            return

        await socket.send(json.dumps({"version": VERSION, "type": "confirm_home"}))
        while True:
            response = json.loads(await socket.recv())
            if response.get("type") == "error":
                raise RuntimeError(response.get("message", "go-home failed"))
            if response.get("type") == "home_progress":
                if response.get("state") == "moving":
                    present = response.get("present")
                    if present:
                        remaining = max(
                            abs(target[joint] - present[joint]) for joint in JOINTS
                        )
                        print(f"Go-home in progress; largest residual: {remaining} raw ticks")
                continue
            if response.get("type") == "home_finished":
                if response.get("state") == "succeeded":
                    print("Follower reached the SDK home pose.")
                    return
                if response.get("state") == "cancelled":
                    print("Go-home cancelled; follower motion was stopped.")
                    return
                raise RuntimeError(f"go-home ended unexpectedly: {response}")
            raise RuntimeError("Pi returned an invalid go-home status")


def _main(operation):
    parser = argparse.ArgumentParser(description=operation["description"])
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    if operation["kind"] == "home":
        parser.add_argument(
            "--speed-percent",
            type=int,
            default=75,
            help="servo speed to restore after homing (default: 75)",
        )
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")
    try:
        if operation["kind"] == "limits":
            asyncio.run(limits_command(args.url))
        else:
            asyncio.run(home_command(args.url, args.speed_percent))
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Follower servo operation failed: {exc}\n")


def limits_main():
    _main(
        {
            "kind": "limits",
            "description": "Read and store the Pi follower's raw servo min/max registers",
        }
    )


def home_main():
    _main(
        {
            "kind": "home",
            "description": "Move the Pi follower to the SDK-compatible home pose",
        }
    )


if __name__ == "__main__":
    limits_main()
