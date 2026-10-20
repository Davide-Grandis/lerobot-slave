"""Run allowlisted Linux commands on the Pi over WSS."""

import argparse
import asyncio
import json
import os
import sys
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.protocol import VERSION
from cf_robot.system_commands import COMMANDS

ALIASES = {"interfaces": COMMANDS[0], "reboot": COMMANDS[1], "shutdown": COMMANDS[2]}


def system_url(control_url):
    parsed = urlsplit(control_url)
    if (
        parsed.scheme != "wss" or not parsed.hostname
        or parsed.username or parsed.password or parsed.query or parsed.fragment
        or parsed.path not in ("", "/", "/control")
    ):
        raise ValueError("control URL must be wss://<public-hostname>/control")
    return urlunsplit(("wss", parsed.netloc, "/system", "", ""))


async def request_command(control_url, command, confirmed=False):
    url = system_url(control_url)
    async with connect(
        url, additional_headers=access_headers(url),
        open_timeout=10, ping_interval=10, ping_timeout=5,
    ) as socket:
        await socket.send(json.dumps({
            "version": VERSION, "type": "system_command",
            "command": command, "confirmed": confirmed,
        }))
        request_id = None
        while True:
            response = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
            if not isinstance(response, dict) or response.get("type") not in (
                "error", "system_queued", "system_result"
            ):
                raise RuntimeError("Pi returned an invalid system response")
            if response.get("type") == "error":
                raise RuntimeError(response.get("message", "Pi rejected the command"))
            if response.get("type") == "system_queued":
                request_id = response.get("request_id")
                print(response.get("message", "Power request queued."), flush=True)
                continue
            if (
                response.get("type") != "system_result"
                or response.get("version") != VERSION
                or response.get("command") != command
                or (request_id and response.get("request_id") != request_id)
                or type(response.get("exit_code")) is not int
                or not isinstance(response.get("stdout"), str)
                or not isinstance(response.get("stderr"), str)
            ):
                raise RuntimeError("Pi returned an invalid system result")
            return response


def main():
    parser = argparse.ArgumentParser(description="Run fixed Pi Linux commands over WSS")
    parser.add_argument("command", help="interfaces, reboot, shutdown, or a quoted supported command")
    parser.add_argument("--url", default=os.environ.get("CF_ROBOT_CONTROL_URL"))
    parser.add_argument("--yes", action="store_true", help="confirm reboot/shutdown without a prompt")
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")
    command = ALIASES.get(args.command, args.command)
    if command not in COMMANDS:
        parser.error(f"supported commands: {', '.join(COMMANDS)}")
    try:
        system_url(args.url)
        if command != COMMANDS[0] and not args.yes:
            print("This schedules Pi power-off/reboot in one minute. Secure the arm first.")
            if input("Type YES to proceed: ").strip() != "YES":
                print("Cancelled; no command sent.")
                return
        result = asyncio.run(request_command(args.url, command, confirmed=command != COMMANDS[0]))
        print(result["stdout"], end="", flush=True)
        print(result["stderr"], end="", file=sys.stderr, flush=True)
        print(f"Pi exit status: {result['exit_code']}", flush=True)
        if result["exit_code"] == 0 and command != COMMANDS[0]:
            print("Pi accepted the power schedule; completion cannot be verified over WSS.")
        if result["exit_code"] != 0:
            parser.exit(result["exit_code"] if 0 < result["exit_code"] < 256 else 1)
    except (OSError, ValueError, RuntimeError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Pi command failed (do not retry power commands blindly): {exc}\n")
    except (KeyboardInterrupt, EOFError):
        parser.exit(1, "\nCancelled; an already-queued power request may still execute.\n")


if __name__ == "__main__":
    main()
