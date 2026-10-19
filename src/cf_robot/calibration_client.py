"""Interactive follower calibration tunneled from the controller over WSS."""

import argparse
import asyncio
import json
import os
import sys
import threading
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.protocol import VERSION


def calibration_url(control_url):
    parsed = urlsplit(control_url)
    if parsed.scheme != "wss" or not parsed.hostname or parsed.path not in ("", "/", "/control"):
        raise ValueError("control URL must be wss://<public-hostname>/control")
    return urlunsplit(("wss", parsed.netloc, "/calibrate", "", ""))


async def relay_calibration(calibration_endpoint):
    loop = asyncio.get_running_loop()
    input_queue = asyncio.Queue()
    stop_input = threading.Event()

    def read_terminal():
        while not stop_input.is_set():
            line = sys.stdin.readline()
            if not line:
                loop.call_soon_threadsafe(input_queue.put_nowait, None)
                return
            loop.call_soon_threadsafe(input_queue.put_nowait, line.rstrip("\r\n"))

    async with connect(
        calibration_endpoint,
        additional_headers=access_headers(calibration_endpoint),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
        max_size=2 * 1024 * 1024,
    ) as socket:
        await socket.send(json.dumps({"version": VERSION, "type": "start"}))
        input_thread = threading.Thread(target=read_terminal, name="calibration-input", daemon=True)
        input_thread.start()
        output_task = asyncio.create_task(socket.recv())
        input_task = asyncio.create_task(input_queue.get())
        try:
            while True:
                done, _ = await asyncio.wait(
                    {output_task, input_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if output_task in done:
                    response = json.loads(output_task.result())
                    message_type = response.get("type")
                    if message_type == "error":
                        raise RuntimeError(response.get("message", "Pi rejected calibration"))
                    if message_type == "calibration_output":
                        sys.stdout.write(response.get("text", ""))
                        sys.stdout.flush()
                    elif message_type == "calibration_started":
                        pass
                    elif message_type == "calibration_finished":
                        if response.get("exit_code") != 0:
                            raise RuntimeError(
                                f"LeRobot calibration exited with code {response.get('exit_code')}"
                            )
                        print("\nFollower calibration completed.")
                        return
                    else:
                        raise RuntimeError("Pi returned an invalid calibration response")
                    output_task = asyncio.create_task(socket.recv())

                if input_task in done:
                    line = input_task.result()
                    if line is None:
                        await socket.send(
                            json.dumps({"version": VERSION, "type": "cancel"})
                        )
                        raise RuntimeError("terminal input closed; calibration cancelled")
                    await socket.send(
                        json.dumps(
                            {"version": VERSION, "type": "input", "text": line},
                            separators=(",", ":"),
                        )
                    )
                    input_task = asyncio.create_task(input_queue.get())
        finally:
            stop_input.set()
            for task in (output_task, input_task):
                if not task.done():
                    task.cancel()
            for task in (output_task, input_task):
                try:
                    await task
                except (asyncio.CancelledError, WebSocketException, OSError):
                    pass


def main():
    parser = argparse.ArgumentParser(
        description="Run the Pi follower's interactive LeRobot calibration over WSS"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")

    print(
        "Follower calibration disables servo torque and requires someone at "
        "the arm to move each joint. Keep the workspace clear and physical "
        "emergency stop available."
    )
    if input("Type CALIBRATE to start the remote follower calibration: ").strip() != "CALIBRATE":
        print("Calibration cancelled.")
        return

    try:
        asyncio.run(relay_calibration(calibration_url(args.url)))
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Follower calibration failed: {exc}\n")


if __name__ == "__main__":
    main()
