"""Request a fixed, authenticated Pi update over the Cloudflare Tunnel."""

import argparse
import asyncio
import json
import os
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit, urlunsplit
from urllib.request import Request, urlopen

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.protocol import VERSION

POLL_INTERVAL_S = 2
UPDATE_TIMEOUT_S = 600


def update_urls(control_url):
    parsed = urlsplit(control_url)
    if parsed.scheme != "wss" or not parsed.hostname or parsed.path not in ("", "/", "/control"):
        raise ValueError("CF_ROBOT_CONTROL_URL must be wss://<public-hostname>/control")
    update_url = urlunsplit(("wss", parsed.netloc, "/update", "", ""))
    health_url = urlunsplit(("https", parsed.netloc, "/health", "", ""))
    return update_url, health_url


async def request_update(update_url):
    async with connect(
        update_url,
        additional_headers=access_headers(update_url),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
    ) as socket:
        await socket.send(json.dumps({"version": VERSION, "type": "update"}))
        response = json.loads(await asyncio.wait_for(socket.recv(), timeout=15))
        if response.get("type") == "error":
            raise RuntimeError(response.get("message", "Pi rejected the update request"))
        if response.get("type") != "update_queued" or not response.get("request_id"):
            raise RuntimeError("Pi returned an invalid update response")
        return response["request_id"]


def read_update_status(health_url, access):
    request = Request(health_url, headers=access)
    with urlopen(request, timeout=5) as response:
        payload = json.loads(response.read())
    return payload.get("update")


def wait_for_update(health_url, request_id, access):
    deadline = time.monotonic() + UPDATE_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            status = read_update_status(health_url, access)
        except HTTPError as exc:
            if exc.code in (401, 403):
                raise RuntimeError(f"Access rejected the update status request: HTTP {exc.code}") from exc
            status = None
        except (URLError, TimeoutError, OSError, json.JSONDecodeError):
            status = None

        if status and status.get("request_id") == request_id:
            state = status.get("state")
            if state == "succeeded":
                return status
            if state == "failed":
                stage = status.get("stage", "unknown stage")
                raise RuntimeError(
                    f"Pi update failed during {stage}; inspect the Pi's "
                    "cf-robot-update.service journal"
                )
        time.sleep(POLL_INTERVAL_S)
    raise TimeoutError("timed out waiting for the Pi update; check /health and its service status")


def main():
    parser = argparse.ArgumentParser(
        description="Update the Pi follower from the configured GitHub main branch over WSS"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")

    try:
        update_url, health_url = update_urls(args.url)
        request_id = asyncio.run(request_update(update_url))
        print(f"Update accepted ({request_id}); waiting for the Pi service to restart...")
        headers = access_headers(update_url)
        access = {
            "CF-Access-Client-Id": headers["CF-Access-Client-Id"],
            "CF-Access-Client-Secret": headers["CF-Access-Client-Secret"],
        }
        result = wait_for_update(health_url, request_id, access)
        print(f"Pi update succeeded at commit {result.get('revision', 'unknown')}.")
    except (OSError, ValueError, RuntimeError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Pi update failed: {exc}\n")


if __name__ == "__main__":
    main()
