"""Live leader/follower pose comparison with optional torque release."""

import argparse
import asyncio
import contextlib
import json
import math
import os
import re
import sys
import threading
import time
import termios
import tty
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.hardware import joint_positions, make_leader
from cf_robot.protocol import JOINTS, VERSION
from cf_robot.pose_monitor import prepare_bus_torque
from cf_robot.servo_maintenance import read_servo_limits

SAMPLE_INTERVAL_S = 0.1


def monitor_url(control_url):
    parsed = urlsplit(control_url)
    if parsed.scheme != "wss" or not parsed.hostname or parsed.path not in ("", "/", "/control"):
        raise ValueError("control URL must be wss://<public-hostname>/control")
    return urlunsplit(("wss", parsed.netloc, "/monitor", "", ""))


def is_follower_sample(message):
    return message.get("type") == "pose_sample" and message.get("side") == "follower"


def parse_pose_command(line, command_id):
    text = line.strip()
    if text.lower() in ("", "help", "?"):
        return {"kind": "help"}
    if text.lower() in ("q", "quit", "exit"):
        return {"kind": "stop"}

    match = re.fullmatch(r"J([1-6])\s+(TE|TD)", text, re.IGNORECASE)
    if match:
        return {
            "kind": "command",
            "message": {
                "version": VERSION,
                "type": "command",
                "command_id": command_id,
                "joint_id": int(match.group(1)),
                "mode": "torque",
                "enabled": match.group(2).upper() == "TE",
            },
        }

    match = re.fullmatch(r"J([1-6])\s+T(\d+)", text, re.IGNORECASE)
    if match:
        ticks = int(match.group(2))
        if ticks > 4095:
            raise ValueError("raw ticks must be from 0 through 4095")
        return {
            "kind": "command",
            "message": {
                "version": VERSION,
                "type": "command",
                "command_id": command_id,
                "joint_id": int(match.group(1)),
                "mode": "ticks",
                "value": ticks,
            },
        }

    match = re.fullmatch(r"J([1-5])\s+D([+-]?(?:\d+(?:\.\d*)?|\.\d+))", text, re.IGNORECASE)
    if match:
        value = float(match.group(2))
        if not math.isfinite(value):
            raise ValueError("angle must be finite")
        return {
            "kind": "command",
            "message": {
                "version": VERSION,
                "type": "command",
                "command_id": command_id,
                "joint_id": int(match.group(1)),
                "mode": "degrees",
                "value": value,
            },
        }

    match = re.fullmatch(r"J6\s+%([+-]?(?:\d+(?:\.\d*)?|\.\d+))", text, re.IGNORECASE)
    if match:
        value = float(match.group(1))
        if not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError("J6 gripper percentage must be between 0 and 100")
        return {
            "kind": "command",
            "message": {
                "version": VERSION,
                "type": "command",
                "command_id": command_id,
                "joint_id": 6,
                "mode": "percent",
                "value": value,
            },
        }

    raise ValueError(
        "use J1-J6 T<ticks>, J1-J5 D<degrees>, J6 %<0-100>, or J1-J6 TE/TD"
    )


class TerminalInput:
    """Small raw-mode input reader so live redraws do not eat typed commands."""

    def __init__(self, loop):
        if not sys.stdin.isatty():
            raise RuntimeError("pose compare needs an interactive terminal")
        self.loop = loop
        self.fd = sys.stdin.fileno()
        self.original = termios.tcgetattr(self.fd)
        self.buffer = ""
        self.lines = asyncio.Queue()
        self.changed = asyncio.Event()
        self.stop_requested = asyncio.Event()
        tty.setraw(self.fd)
        loop.add_reader(self.fd, self._read_ready)

    def _read_ready(self):
        try:
            data = os.read(self.fd, 64)
        except OSError:
            self.stop_requested.set()
            return
        for byte in data:
            if byte in (3, 4):
                self.stop_requested.set()
            elif byte in (10, 13):
                self.lines.put_nowait(self.buffer)
                self.buffer = ""
            elif byte in (8, 127):
                self.buffer = self.buffer[:-1]
            elif 32 <= byte < 127:
                self.buffer += chr(byte)
        self.changed.set()

    def close(self):
        self.loop.remove_reader(self.fd)
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original)
        sys.stdout.write("\n")
        sys.stdout.flush()


def _read_leader(leader, loop, samples, stop, release_torque):
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
        # Avoid LeRobot's configure path; only the explicit release mode writes torque.
        bus.connect()
        connected = True
        limits = read_servo_limits(bus)
        prepare_bus_torque(bus, "leader", release_torque, publish)
        while not stop.is_set():
            started = time.monotonic()
            action = joint_positions(leader.get_action())
            raw = bus.sync_read("Present_Position", normalize=False, num_retry=2)
            torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
            publish(
                {
                    "type": "leader_sample",
                    "values": action,
                    "raw": {
                        joint: int(raw[joint.removesuffix(".pos")])
                        for joint in JOINTS
                    },
                    "torque": {
                        joint: bool(torque[joint.removesuffix(".pos")])
                        for joint in JOINTS
                    },
                    "limits": limits,
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


def _torque_states(states):
    if not states:
        return "not read"
    return ", ".join(
        f"{joint.removesuffix('.pos')}={'ON' if states[joint] else 'off'}"
        for joint in JOINTS
    )


def _render(
    leader_sample,
    follower_sample,
    leader_torque=None,
    follower_torque=None,
    command_buffer="",
    status_message="",
):
    if sys.stdout.isatty():
        sys.stdout.write("\033[H\033[J")
    print("ARMS MONITOR -- Ctrl-C or q to stop")
    print(f"Torque before -> after: leader: {_torque_states(leader_torque and leader_torque['before'])}")
    print(f"                         follower: {_torque_states(follower_torque and follower_torque['before'])}")
    print(
        "Current torque: leader "
        f"{_torque_states(leader_sample['torque'])}; follower "
        f"{_torque_states(follower_sample['torque'])}"
    )
    print(
        "Units: J1-J5 values/delta=degrees; J6=0-100 normalized; "
        "raw counts and MIN/MAX=encoder ticks; torque 1=on, 0=off."
    )
    print("Leader values are the normalized action values teleoperation sends.")
    print(
        f"{'Joint':24} {'Unit':>6} {'Leader':>9} {'Follower':>9} {'Delta':>8} "
        f"{'L raw':>6} {'L MIN..MAX':>13} {'F raw':>6} {'F MIN..MAX':>13} {'L T':>4} {'F T':>4}"
    )
    for joint in JOINTS:
        leader_value = leader_sample["values"][joint]
        follower_value = follower_sample["values"][joint]
        delta = leader_value - follower_value
        leader_raw = leader_sample["raw"][joint]
        follower_raw = follower_sample["raw"][joint]
        unit = "0-100" if joint == "gripper.pos" else "deg"
        leader_limits = leader_sample["limits"][joint]
        follower_limits = follower_sample["limits"][joint]
        print(
            f"{joint:24} {unit:>6} {leader_value:9.2f} {follower_value:9.2f} "
            f"{delta:+8.2f} {leader_raw:6d} "
            f"{leader_limits['min']:5d}..{leader_limits['max']:<5d} {follower_raw:6d} "
            f"{follower_limits['min']:5d}..{follower_limits['max']:<5d} "
            f"{int(leader_sample['torque'][joint]):4d} {int(follower_sample['torque'][joint]):4d}"
        )
    print(
        f"Samples: leader={leader_sample['sampled_at']:.3f}, "
        f"follower={follower_sample['sampled_at']:.3f}"
    )
    print(
        "J1=shoulder_pan J2=shoulder_lift J3=elbow_flex J4=wrist_flex "
        "J5=wrist_roll J6=gripper"
    )
    print("Follower commands: J1-J6 T<ticks> | J1-J5 D<degrees> | J6 %<0-100> | J1-J6 TE/TD | help | q")
    print("TE sets the current position as the hold target; TD releases torque and may let the joint sag.")
    if status_message:
        print(status_message)
    print(f"> {command_buffer}", end="")
    sys.stdout.flush()


async def monitor_poses(url, port, arm_id, release_torque):
    leader = make_leader(port, arm_id)
    stop = threading.Event()
    samples = asyncio.Queue(maxsize=4)
    leader_thread = None

    async with connect(
        url,
        additional_headers=access_headers(url),
        open_timeout=10,
        ping_interval=10,
        ping_timeout=5,
        max_size=2 * 1024 * 1024,
    ) as socket:
        await socket.send(
            json.dumps(
                {
                    "version": VERSION,
                    "type": "start",
                    "release_torque": release_torque,
                }
            )
        )
        started = json.loads(await socket.recv())
        if started.get("type") == "error":
            raise RuntimeError(started.get("message", "Pi rejected pose monitoring"))
        if started.get("type") == "torque_release_confirmation_required":
            await socket.send(
                json.dumps({"version": VERSION, "type": "confirm_torque_release"})
            )
            started = json.loads(await socket.recv())
            if started.get("type") == "error":
                raise RuntimeError(started.get("message", "Pi rejected torque release"))
        if started.get("type") == "monitor_cancelled":
            raise RuntimeError(started.get("reason", "torque release was cancelled"))
        if started.get("type") != "monitor_started":
            raise RuntimeError("Pi returned an invalid pose-monitor response")
        if started.get("release_torque") != release_torque:
            raise RuntimeError("Pi pose-monitor torque mode did not match the request")

        loop = asyncio.get_running_loop()
        terminal = TerminalInput(loop)
        leader_thread = threading.Thread(
            target=_read_leader,
            args=(leader, loop, samples, stop, release_torque),
            name="leader-pose-monitor",
            daemon=True,
        )
        leader_thread.start()
        follower_sample = None
        leader_sample = None
        follower_torque = None
        leader_torque = None
        status_message = ""
        command_id = 0
        receive_task = asyncio.create_task(socket.recv())
        leader_task = asyncio.create_task(samples.get())
        command_task = asyncio.create_task(terminal.lines.get())
        changed_task = asyncio.create_task(terminal.changed.wait())
        stop_task = asyncio.create_task(terminal.stop_requested.wait())
        stop_sent = False
        server_finished = False
        try:
            while True:
                waiting = {
                    receive_task,
                    leader_task,
                    command_task,
                    changed_task,
                }
                if stop_task is not None:
                    waiting.add(stop_task)
                done, _ = await asyncio.wait(
                    waiting,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if stop_task is not None and stop_task in done:
                    await socket.send(json.dumps({"version": VERSION, "type": "stop"}))
                    stop_sent = True
                    stop.set()
                    status_message = "Stopping monitor..."
                    stop_task = None

                if receive_task in done:
                    message = json.loads(receive_task.result())
                    message_type = message.get("type")
                    if message_type == "error":
                        raise RuntimeError(message.get("message", "pose monitor failed"))
                    if is_follower_sample(message):
                        follower_sample = message
                    elif message_type == "follower_torque":
                        follower_torque = message
                    elif message_type == "monitor_finished":
                        server_finished = True
                        return
                    elif message_type in ("command_queued", "command_progress", "command_result"):
                        joint_id = message.get("joint_id")
                        state = message.get("state", message_type)
                        status_message = f"J{joint_id}: {state}"
                        if message.get("message"):
                            status_message += f" - {message['message']}"
                        elif message_type == "command_progress":
                            status_message += (
                                f" ({message.get('current', 0):.2f} -> "
                                f"{message.get('target', 0):.2f} {message.get('unit', '')})"
                            )
                    else:
                        raise RuntimeError("Pi returned an invalid pose-monitor sample")
                    receive_task = asyncio.create_task(socket.recv())

                if leader_task in done:
                    message = leader_task.result()
                    message_type = message.get("type")
                    if message_type == "leader_error":
                        raise RuntimeError(message.get("message", "leader read failed"))
                    if message_type == "leader_torque":
                        leader_torque = message
                        leader_task = asyncio.create_task(samples.get())
                        continue
                    if message_type != "leader_sample":
                        raise RuntimeError("invalid local leader sample")
                    leader_sample = message
                    leader_task = asyncio.create_task(samples.get())

                if command_task in done:
                    line = command_task.result()
                    if line is not None and not stop_sent:
                        command_id += 1
                        try:
                            parsed = parse_pose_command(line, command_id)
                        except ValueError as exc:
                            status_message = str(exc)
                        else:
                            if parsed["kind"] == "help":
                                status_message = (
                                    "J1 pan J2 lift J3 elbow J4 wrist-flex J5 wrist-roll J6 gripper; "
                                    "T=ticks, D=degrees (J1-J5), %=gripper (J6), TE/TD=torque."
                                )
                            elif parsed["kind"] == "stop":
                                await socket.send(
                                    json.dumps({"version": VERSION, "type": "stop"})
                                )
                                stop_sent = True
                                stop.set()
                                status_message = "Stopping monitor..."
                            else:
                                await socket.send(
                                    json.dumps(
                                        parsed["message"],
                                        separators=(",", ":"),
                                    )
                                )
                                status_message = (
                                    f"J{parsed['message']['joint_id']}: command sent"
                                )
                    command_task = asyncio.create_task(terminal.lines.get())

                if changed_task in done:
                    terminal.changed.clear()
                    changed_task = asyncio.create_task(terminal.changed.wait())

                if leader_sample is not None and follower_sample is not None:
                    _render(
                        leader_sample,
                        follower_sample,
                        leader_torque,
                        follower_torque,
                        terminal.buffer,
                        status_message,
                    )
        finally:
            stop.set()
            if not stop_sent and not server_finished:
                if receive_task is not None and not receive_task.done():
                    receive_task.cancel()
                    with contextlib.suppress(asyncio.CancelledError, WebSocketException, OSError):
                        await receive_task
                with contextlib.suppress(Exception):
                    await socket.send(json.dumps({"version": VERSION, "type": "stop"}))
                with contextlib.suppress(Exception, asyncio.TimeoutError):
                    while True:
                        message = json.loads(await asyncio.wait_for(socket.recv(), timeout=2))
                        if message.get("type") == "monitor_finished":
                            server_finished = True
                            break
            for task in (receive_task, leader_task, command_task, changed_task, stop_task):
                if task is not None and not task.done():
                    task.cancel()
            for task in (receive_task, leader_task, command_task, changed_task, stop_task):
                if task is not None:
                    with contextlib.suppress(asyncio.CancelledError, WebSocketException, OSError):
                        await task
            if leader_thread is not None:
                await asyncio.to_thread(leader_thread.join, 5)
            terminal.close()


def main():
    parser = argparse.ArgumentParser(
        description="Continuously compare leader action values with follower readback"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    parser.add_argument("--port", required=True, help="Local leader serial device")
    parser.add_argument("--arm-id", default="so101_leader", help="Leader calibration ID")
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="read torque state without disabling enabled servos",
    )
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")

    release_torque = not args.keep_torque
    if release_torque:
        print(
            "The monitor reads both arms and sends no position goals. It will "
            "disable torque only on servos currently enabled; the arms may sag."
        )
        if input("Physically support both arms and type RELEASE to proceed: ").strip() != "RELEASE":
            print("Pose monitoring cancelled; no torque state was changed.")
            return
    else:
        print(
            "Read-only mode: torque state will not be changed. Manually moving "
            "an enabled servo may meet resistance."
        )
    try:
        asyncio.run(
            monitor_poses(
                monitor_url(args.url),
                args.port,
                args.arm_id,
                release_torque,
            )
        )
    except KeyboardInterrupt:
        print("\nPose monitoring stopped.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Pose monitor stopped: {exc}\n")


if __name__ == "__main__":
    main()
