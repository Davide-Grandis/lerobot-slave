"""Live leader/follower monitor with explicit follower commands."""

import argparse
import asyncio
import contextlib
import json
import math
import os
import re
import shlex
import subprocess
import sys
import termios
import threading
import time
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot import __version__
from cf_robot.auth import access_headers
from cf_robot.calibration_inspection import capture_validation, inspect_calibration
from cf_robot.hardware import make_leader
from cf_robot.pose_monitor import _sample
from cf_robot.protocol import JOINTS, VERSION
from cf_robot.servo_maintenance import read_servo_limits

SAMPLE_INTERVAL_S = 0.1


def git_build_version():
    repo_root = Path(__file__).resolve().parents[2]

    def git(*args):
        result = subprocess.run(
            ["git", *args],
            cwd=repo_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        return result.stdout.strip()

    try:
        commit_count = git("rev-list", "--count", "HEAD")
        commit_hash = git("rev-parse", "--short=7", "HEAD")
        dirty = bool(git("status", "--porcelain"))
        if not commit_count.isdigit() or not commit_hash:
            raise ValueError("git returned no build revision")
        return f"{commit_count}-g{commit_hash}{'*' if dirty else ''}"
    except (OSError, subprocess.SubprocessError, ValueError):
        return f"pkg-{__version__}"


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
    match = re.fullmatch(r"INSPECT\s+([LF])", text, re.IGNORECASE)
    if match:
        return {"kind": "inspect", "side": "leader" if match[1].upper() == "L" else "follower"}
    if text.upper().startswith("SAVE "):
        parts = shlex.split(text)
        if len(parts) != 3 or parts[1].upper() not in ("L", "F"):
            raise ValueError("use SAVE L/F <path>; quote paths containing spaces")
        return {
            "kind": "save_inspection",
            "side": "leader" if parts[1].upper() == "L" else "follower",
            "path": parts[2],
        }
    match = re.fullmatch(
        r"CAPTURE\s+([LF])\s+J([1-6])\s+([+-]?[\d.]+)\s+([\d.]+)\s+(.+)",
        text, re.IGNORECASE,
    )
    if match:
        return {"kind": "capture", "body": {
            "side": "leader" if match[1].upper() == "L" else "follower",
            "joint": JOINTS[int(match[2]) - 1], "expected": float(match[3]),
            "tolerance": float(match[4]), "label": match[5], "confirmed": True,
        }}

    match = re.fullmatch(r"J(\*|[1-6])\s+(TE|TD)", text, re.IGNORECASE)
    if match:
        return {
            "kind": "command",
            "message": {
                "version": VERSION,
                "type": "command",
                "command_id": command_id,
                "joint_id": match.group(1) if match.group(1) == "*" else int(match.group(1)),
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
        "use INSPECT L/F, SAVE L/F <path>, J1-J6 T<ticks>, J1-J5 D<degrees>, "
        "J6 %<0-100>, or J1-J6/J* TE/TD"
    )


class InspectionReports:
    """Retain explicit inspection snapshots independently of live screen redraws."""

    def __init__(self):
        self.reports = {}

    def command(self, command, samples):
        side = command["side"]
        report = (samples.get(side) or {}).get("calibration")
        if not report:
            return f"{side.capitalize()} inspection unavailable; start monitoring that arm."
        content = json.dumps(report, indent=2)
        self.reports[side] = content
        if command["kind"] == "save_inspection":
            path = Path(command["path"]).expanduser()
            try:
                # Exclusive creation protects previous exports from accidental overwrite.
                with path.open("x", encoding="utf-8") as output:
                    output.write(content + "\n")
            except OSError as exc:
                return f"Could not save inspection: {exc}"
            return f"Saved {side} inspection to {path.resolve()}"
        return content

    def print_on_exit(self):
        if not self.reports:
            return
        print("\n" + "=" * 72)
        print("INSPECTION REPORTS -- retained after monitor exit; safe to copy")
        for side, content in self.reports.items():
            print(f"\n{side.upper()} INSPECTION")
            print(content)
        print("=" * 72)
        sys.stdout.flush()


class TerminalInput:
    """Read commands without echo while preserving normal terminal line output."""

    def __init__(self, loop):
        if not sys.stdin.isatty():
            raise RuntimeError("pose compare needs an interactive terminal")
        self.loop = loop
        self.fd = sys.stdin.fileno()
        self.original = termios.tcgetattr(self.fd)
        self.restore = [*self.original]
        self.restore[6] = list(self.original[6])
        # Recover a usable shell mode if an earlier raw-mode run was interrupted.
        if not self.restore[1] & termios.OPOST:
            self.restore[1] |= termios.OPOST
            if hasattr(termios, "ONLCR"):
                self.restore[1] |= termios.ONLCR
            self.restore[3] |= termios.ECHO | termios.ICANON | termios.ISIG
            if hasattr(termios, "IEXTEN"):
                self.restore[3] |= termios.IEXTEN
        self.buffer = ""
        self.lines = asyncio.Queue()
        self.changed = asyncio.Event()
        self.stop_requested = asyncio.Event()
        attributes = termios.tcgetattr(self.fd)
        attributes[3] &= ~termios.ICANON
        attributes[3] &= ~termios.ECHO
        attributes[1] |= termios.OPOST
        if hasattr(termios, "ONLCR"):
            attributes[1] |= termios.ONLCR
        attributes[6][termios.VMIN] = 1
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSADRAIN, attributes)
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
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.restore)
        sys.stdout.write("\n")
        sys.stdout.flush()


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
        # Avoid LeRobot's configure path; this monitor only reads the leader bus.
        bus.connect()
        connected = True
        limits = read_servo_limits(bus, strict=False)
        calibration = inspect_calibration(leader, "leader", limits)
        previous = None
        while not stop.is_set():
            started = time.monotonic()
            sample = _sample(leader, "leader", limits, calibration, previous)
            publish({**sample, "type": "leader_sample"})
            previous = sample
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


def _render(build_version, leader_sample, follower_sample, command_buffer="", status_message=""):
    if sys.stdout.isatty():
        sys.stdout.write("\033[H\033[J")
    print(f"ARMS MONITOR [{build_version}] -- Ctrl-C or q to stop")
    print("Leader values are the normalized target values teleoperation sends.")
    header = (
        f"{'Joint':17}   {'Leader':>10}   {'Follower':>10}   {'Delta':>10}   "
        f"{'LT':>3}   {'FT':>3}"
    )
    separator = "=" * len(header)
    print(separator)
    print(header)
    print(separator)
    for index, joint in enumerate(JOINTS, start=1):
        def number(sample, joint=joint):
            value = (sample or {}).get("values", {}).get(joint)
            return value if isinstance(value, (int, float)) and math.isfinite(value) else None

        leader_value, follower_value = number(leader_sample), number(follower_sample)
        modes = [
            ((sample or {}).get("calibration") or {}).get("joints", {}).get(joint, {}).get("mode")
            for sample in (leader_sample, follower_sample)
        ]
        compatible = not all(modes) or modes[0] == modes[1]
        delta = (
            leader_value - follower_value
            if compatible and leader_value is not None and follower_value is not None else None
        )

        def formatted(value, signed=False):
            return "--".rjust(10) if value is None else f"{value:+10.2f}" if signed else f"{value:10.2f}"

        def torque(sample, joint=joint):
            value = (sample or {}).get("torque", {}).get(joint)
            return "--" if value is None else "V" if value else "X"

        print(
            f"J{index} {joint.removesuffix('.pos'):13} "
            f"   {formatted(leader_value)}   {formatted(follower_value)}   {formatted(delta, True)}   "
            f"{torque(leader_sample):>3}   {torque(follower_sample):>3}"
        )
        def raw_description(sample, joint=joint):
            if not sample:
                return "--"
            bounds = sample["limits"][joint]
            return f"{sample['raw'][joint]:4d} [{bounds['min']}..{bounds['max']}]"

        print(f"   raw ticks: L {raw_description(leader_sample):24}   F {raw_description(follower_sample)}")
        print()
    print(separator)
    print(
        f"Samples: leader={leader_sample['sampled_at'] if leader_sample else '--'}, "
        f"follower={follower_sample['sampled_at'] if follower_sample else '--'}"
    )
    print(separator)
    print("Units: J1-J5 deg, J6 % (0-100); raw/MIN/MAX ticks. LT/FT: V=ON, X=off.")
    for side, sample in (("leader", leader_sample), ("follower", follower_sample)):
        inspection = (sample or {}).get("calibration")
        if inspection:
            print(f"{side.capitalize()} calibration: {inspection['state']} (not physically validated)")
            if any(item["unit"] != ("%" if key == "gripper.pos" else "deg") for key, item in inspection["joints"].items()):
                print("WARNING: actual normalization differs from the units note; use INSPECT.")
        for warning in (sample or {}).get("motion_warnings", []):
            print(f"WARNING ({side}): {warning}")
    print("Follower: J1-J6 T<ticks>; J1-J5 D<degrees>; J6 %<0-100>")
    print("          J1-J6 or J* TE/TD (TD may sag); help; q")
    print("TE holds present position; TD releases torque and may drop the joint.")
    print("INSPECT L/F (reprinted on exit); SAVE L/F <path> (JSON, no overwrite)")
    print("CAPTURE L/F J<n> <known value> <tolerance> <pose label>")
    print("CAPTURE confirms an independently measured physical pose; it never moves an arm.")
    if status_message:
        print(status_message)
    print(f"> {command_buffer}", end="")
    sys.stdout.flush()


async def monitor_poses(url, port, arm_id, build_version):
    leader = make_leader(port, arm_id) if port else None
    stop = threading.Event()
    samples = asyncio.Queue(maxsize=4)
    leader_thread = None
    inspections = InspectionReports()

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
        terminal = TerminalInput(loop)
        if leader is not None:
            leader_thread = threading.Thread(
                target=_read_leader,
                args=(leader, loop, samples, stop),
                name="leader-pose-monitor",
                daemon=True,
            )
            leader_thread.start()
        follower_sample = None
        leader_sample = None
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
                    elif message_type in ("leader_torque", "follower_torque"):
                        # Ignore startup-state notifications from older Pi builds.
                        pass
                    elif message_type == "monitor_finished":
                        server_finished = True
                        return
                    elif message_type == "command_result" and message.get("joint_results"):
                        status_message = " | ".join(
                            f"{result['joint_name']} (J{result['joint_id']}): "
                            f"{result['message']}"
                            for result in message["joint_results"]
                        )
                    elif message_type in ("command_queued", "command_progress", "command_result"):
                        joint_id = message.get("joint_id")
                        if isinstance(joint_id, int) and 1 <= joint_id <= len(JOINTS):
                            joint_name = (
                                JOINTS[joint_id - 1]
                                .removesuffix(".pos")
                                .replace("_", " ")
                                .title()
                            )
                            joint_label = f"{joint_name} (J{joint_id})"
                        else:
                            joint_label = f"J{joint_id}"
                        state = message.get("state", message_type)
                        status_message = f"{joint_label}: {state}"
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
                                    "T=ticks, D=degrees (J1-J5), %=gripper (J6), "
                                    "J1-J6 or J* TE/TD=torque."
                                )
                            elif parsed["kind"] == "stop":
                                await socket.send(
                                    json.dumps({"version": VERSION, "type": "stop"})
                                )
                                stop_sent = True
                                stop.set()
                                status_message = "Stopping monitor..."
                            elif parsed["kind"] in ("inspect", "save_inspection"):
                                status_message = inspections.command(
                                    parsed, {"leader": leader_sample, "follower": follower_sample},
                                )
                            elif parsed["kind"] == "capture":
                                try:
                                    evidence = capture_validation(
                                        {"leader": leader_sample, "follower": follower_sample}, parsed["body"],
                                    )
                                    status_message = json.dumps(evidence, indent=2)
                                except ValueError as exc:
                                    status_message = str(exc)
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

                if follower_sample is not None and (leader_sample is not None or leader is None):
                    _render(
                        build_version,
                        leader_sample,
                        follower_sample,
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
                await asyncio.to_thread(leader_thread.join)
            terminal.close()
            inspections.print_on_exit()


async def monitor_leader_only(port, arm_id, build_version):
    """Inspect the local arm without opening any connection to the follower."""
    leader = make_leader(port, arm_id)
    loop = asyncio.get_running_loop()
    samples = asyncio.Queue(maxsize=4)
    stop = threading.Event()
    terminal = TerminalInput(loop)
    thread = threading.Thread(
        target=_read_leader, args=(leader, loop, samples, stop),
        name="leader-pose-monitor", daemon=True,
    )
    thread.start()
    tasks = {
        "sample": asyncio.create_task(samples.get()),
        "command": asyncio.create_task(terminal.lines.get()),
        "changed": asyncio.create_task(terminal.changed.wait()),
        "stop": asyncio.create_task(terminal.stop_requested.wait()),
    }
    sample, status = None, "Leader-only: read-only. No follower connection."
    inspections = InspectionReports()
    try:
        while True:
            done, _ = await asyncio.wait(tasks.values(), return_when=asyncio.FIRST_COMPLETED)
            if tasks["stop"] in done:
                return
            if tasks["sample"] in done:
                sample = tasks["sample"].result()
                if sample.get("type") == "leader_error":
                    raise RuntimeError(sample["message"])
                tasks["sample"] = asyncio.create_task(samples.get())
            if tasks["command"] in done:
                try:
                    command = parse_pose_command(tasks["command"].result(), 1)
                    if command["kind"] == "stop":
                        return
                    if command["kind"] in ("inspect", "save_inspection"):
                        status = inspections.command(command, {"leader": sample})
                    elif command["kind"] == "capture":
                        status = json.dumps(capture_validation({"leader": sample}, command["body"]), indent=2)
                    else:
                        status = "Leader-only: INSPECT L; CAPTURE L J<n> <known value> <tolerance> <label>; q"
                except ValueError as exc:
                    status = str(exc)
                tasks["command"] = asyncio.create_task(terminal.lines.get())
            if tasks["changed"] in done:
                terminal.changed.clear()
                tasks["changed"] = asyncio.create_task(terminal.changed.wait())
            if sample:
                _render(build_version, sample, None, terminal.buffer, status)
    finally:
        stop.set()
        for task in tasks.values():
            task.cancel()
        for task in tasks.values():
            with contextlib.suppress(asyncio.CancelledError):
                await task
        # Do not return serial ownership while the hardware thread is still alive.
        await asyncio.to_thread(thread.join)
        terminal.close()
        inspections.print_on_exit()


def main():
    parser = argparse.ArgumentParser(
        description="Continuously compare leader action values with follower readback"
    )
    parser.add_argument(
        "--url",
        default=os.environ.get("CF_ROBOT_CONTROL_URL"),
        help="WSS control URL (defaults to CF_ROBOT_CONTROL_URL)",
    )
    parser.add_argument("--port", help="Local leader serial device (required for leader/both)")
    parser.add_argument("--side", choices=("leader", "follower", "both"), default="both")
    parser.add_argument("--arm-id", default="so101_leader", help="Leader calibration ID")
    args = parser.parse_args()
    if args.side != "follower" and not args.port:
        parser.error("pass --port for leader or both arms")
    if args.side != "leader" and not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL in controller.env")

    try:
        if args.side == "leader":
            asyncio.run(monitor_leader_only(args.port, args.arm_id, git_build_version()))
        else:
            asyncio.run(monitor_poses(
                monitor_url(args.url),
                args.port if args.side == "both" else None,
                args.arm_id,
                git_build_version(),
            ))
    except KeyboardInterrupt:
        print("\nPose monitoring stopped.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Pose monitor stopped: {exc}\n")


if __name__ == "__main__":
    main()
