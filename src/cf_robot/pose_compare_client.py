"""Read-only live comparison of leader action and follower pose over WSS."""

import argparse
import asyncio
import contextlib
import json
import os
import sys
import threading
import time
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.hardware import joint_positions, make_leader
from cf_robot.protocol import JOINTS, VERSION

SAMPLE_INTERVAL_S = 0.1


def monitor_url(control_url):
    parsed = urlsplit(control_url)
    if parsed.scheme != "wss" or not parsed.hostname or parsed.path not in ("", "/", "/control"):
        raise ValueError("control URL must be wss://<public-hostname>/control")
    return urlunsplit(("wss", parsed.netloc, "/monitor", "", ""))


def is_follower_sample(message):
    return message.get("type") == "pose_sample" and message.get("side") == "follower"


def _read_leader(leader, loop, samples, stop):
    bus = leader.bus
    connected = False

    def publish(message):
        def enqueue():
            if samples.full():
                with contextlib.suppress(asyncio.QueueEmpty):
                    samples.get_nowait()
            samples.put_nowait(message)

        loop.call_soon_threadsafe(enqueue)

    try:
        # Do not call leader.connect(): it configures the arm and disables torque.
        bus.connect()
        connected = True
        while not stop.is_set():
            started = time.monotonic()
            action = joint_positions(leader.get_action())
            raw = bus.sync_read("Present_Position", normalize=False, num_retry=2)
            publish(
                {
                    "type": "leader_sample",
                    "values": action,
                    "raw": {
                        joint: int(raw[joint.removesuffix(".pos")])
                        for joint in JOINTS
                    },
                    "sampled_at": time.time(),
                }
            )
            remaining = SAMPLE_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    except Exception as exc:
        publish({"type": "leader_error", "message": str(exc)})
    finally:
        if connected:
            try:
                bus.disconnect(disable_torque=False)
            except Exception as exc:
                publish({"type": "leader_error", "message": f"read-only disconnect failed: {exc}"})


def _render(leader_sample, follower_sample):
    if sys.stdout.isatty():
        sys.stdout.write("\033[H\033[J")
    print("READ-ONLY POSE MONITOR -- Ctrl-C to stop (no torque or goal writes)")
    print("Leader action values below are the normalized values teleoperation sends.")
    print(
        f"{'Joint':24} {'Leader target':>14} {'Follower pose':>14} "
        f"{'Delta':>9} {'L raw':>7} {'F raw':>7}"
    )
    for joint in JOINTS:
        leader_value = leader_sample["values"][joint]
        follower_value = follower_sample["values"][joint]
        delta = leader_value - follower_value
        leader_raw = leader_sample["raw"][joint]
        follower_raw = follower_sample["raw"][joint]
        print(
            f"{joint:24} {leader_value:14.2f} {follower_value:14.2f} "
            f"{delta:+9.2f} {leader_raw:7d} {follower_raw:7d}"
        )
    print(
        f"Samples: leader={leader_sample['sampled_at']:.3f}, "
        f"follower={follower_sample['sampled_at']:.3f}"
    )
    sys.stdout.flush()


async def monitor_poses(url, port, arm_id):
    leader = make_leader(port, arm_id)
    stop = threading.Event()
    samples = asyncio.Queue(maxsize=1)
    leader_thread = None

    async with connect(
        url,
        additional_headers=access_headers(url),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
        max_size=2 * 1024 * 1024,
    ) as socket:
        await socket.send(json.dumps({"version": VERSION, "type": "start"}))
        started = json.loads(await socket.recv())
        if started.get("type") == "error":
            raise RuntimeError(started.get("message", "Pi rejected pose monitoring"))
        if started.get("type") != "monitor_started":
            raise RuntimeError("Pi returned an invalid pose-monitor response")

        loop = asyncio.get_running_loop()
        leader_thread = threading.Thread(
            target=_read_leader,
            args=(leader, loop, samples, stop),
            name="read-only-leader-monitor",
            daemon=True,
        )
        leader_thread.start()
        follower_sample = None
        leader_sample = None
        receive_task = asyncio.create_task(socket.recv())
        leader_task = asyncio.create_task(samples.get())
        try:
            while True:
                done, _ = await asyncio.wait(
                    {receive_task, leader_task},
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if receive_task in done:
                    message = json.loads(receive_task.result())
                    message_type = message.get("type")
                    if message_type == "error":
                        raise RuntimeError(message.get("message", "pose monitor failed"))
                    if is_follower_sample(message):
                        follower_sample = message
                    elif message_type == "monitor_finished":
                        return
                    else:
                        raise RuntimeError("Pi returned an invalid pose-monitor sample")
                    receive_task = asyncio.create_task(socket.recv())

                if leader_task in done:
                    message = leader_task.result()
                    message_type = message.get("type")
                    if message_type == "leader_error":
                        raise RuntimeError(message.get("message", "leader read failed"))
                    if message_type != "leader_sample":
                        raise RuntimeError("invalid local leader sample")
                    leader_sample = message
                    leader_task = asyncio.create_task(samples.get())

                if leader_sample is not None and follower_sample is not None:
                    _render(leader_sample, follower_sample)
        finally:
            stop.set()
            with contextlib.suppress(Exception):
                await socket.send(json.dumps({"version": VERSION, "type": "stop"}))
            for task in (receive_task, leader_task):
                if not task.done():
                    task.cancel()
            for task in (receive_task, leader_task):
                with contextlib.suppress(asyncio.CancelledError, WebSocketException, OSError):
                    await task
            if leader_thread is not None:
                await asyncio.to_thread(leader_thread.join, 5)


def main():
    parser = argparse.ArgumentParser(
        description="Continuously compare leader action values with follower readback without commanding either arm"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    parser.add_argument("--port", required=True, help="Local leader serial device")
    parser.add_argument("--arm-id", default="so101_leader", help="Leader calibration ID")
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")

    print(
        "Read-only monitor: it reads Present_Position on both arms and does not "
        "configure torque or send Goal_Position commands."
    )
    try:
        asyncio.run(monitor_poses(monitor_url(args.url), args.port, args.arm_id))
    except KeyboardInterrupt:
        print("\nPose monitoring stopped.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Pose monitor stopped: {exc}\n")


if __name__ == "__main__":
    main()
