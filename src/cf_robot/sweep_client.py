"""Interactive, single-joint follower sweeps over the authenticated monitor."""

import argparse
import asyncio
import contextlib
import json
import math
import os
import sys
import time

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.pose_compare_client import monitor_url
from cf_robot.protocol import JOINTS, SAFE_MONITOR_MOVES_CAPABILITY, VERSION

MAX_CONFIRMED_RETRIES = 2


def load_description(message):
    raw = message.get("present_load_raw")
    if type(raw) is int:
        return f"load={raw:+d} raw ({raw / 10:+.1f}% est.)"
    error = message.get("present_load_error")
    return "load=unavailable" + (f" ({error})" if error else "")


class SweepCommandFailed(RuntimeError):
    def __init__(self, result):
        self.result = result
        super().__init__("follower command failed: " + json.dumps(result))


def sweep_plan(sample, joint_ids):
    if (
        not joint_ids or len(set(joint_ids)) != len(joint_ids)
        or any(type(v) is not int or not 1 <= v <= 6 for v in joint_ids)
    ):
        raise ValueError("select distinct follower joints J1-J6")
    calibration = sample.get("calibration") or {}
    if calibration.get("state") != "consistent":
        raise ValueError("follower calibration must be consistent before a sweep")
    plan = []
    for joint_id in joint_ids:
        joint = JOINTS[joint_id - 1]
        item = calibration["joints"][joint]
        limits = sample["limits"][joint]
        low, high = limits["min"], limits["max"]
        if (
            item.get("state") != "consistent"
            or any(type(v) is not int for v in (low, high))
            or not 0 <= low < high <= 4095
            or any(
                (item.get(key) or {}).get("range_min") != low
                or (item.get(key) or {}).get("range_max") != high
                for key in ("saved", "live")
            )
        ):
            raise ValueError(f"follower J{joint_id} limits are not verified")
        if joint_id == 6:
            if item.get("mode") != "range_0_100":
                raise ValueError("follower J6 must use calibrated percent")
            zero, label = low, "0% opening (MIN, not degrees)"
        else:
            angular = item.get("angular_calibration") or {}
            zero = angular.get("zero_ticks")
            if (
                item.get("mode") != "degrees" or angular.get("state") != "verified"
                or angular.get("pending_zero") or angular.get("pending_range_max")
                or type(zero) not in (int, float) or not math.isfinite(zero)
                or not low <= zero <= high
            ):
                raise ValueError(f"follower J{joint_id} needs a verified physical zero and range")
            zero, label = math.floor(zero + 0.5), "0 degrees"
        plan.append({
            "id": joint_id, "joint": joint, "min": low, "max": high,
            "zero": zero, "zero_label": label,
        })
    return plan


async def confirm_enter(prompt):
    """Read terminal input without a thread that can hang shutdown on Ctrl-C."""
    loop = asyncio.get_running_loop()
    answer = loop.create_future()
    print(
        prompt + "\nPress ENTER to confirm or press S to skip (then ENTER); q or Ctrl-C cancels: ",
        end="", flush=True,
    )

    def ready():
        if not answer.done():
            answer.set_result(sys.stdin.readline())

    loop.add_reader(sys.stdin.fileno(), ready)
    try:
        line = await answer
    finally:
        loop.remove_reader(sys.stdin.fileno())
    if line.strip().lower() == "s":
        return "skip"
    if line == "" or line.strip():
        raise RuntimeError("sweep cancelled; no further movements requested")


class SweepSession:
    def __init__(self, socket):
        self.socket = socket
        self.sample = None
        self.generation = 0
        self.changed = asyncio.Event()
        self.pending = None
        self.command_id = 0
        self.stopping = False
        self.reader = asyncio.create_task(self.read())

    async def read(self):
        async for raw in self.socket:
            message = json.loads(raw)
            kind = message.get("type")
            if kind == "error":
                raise RuntimeError(f"follower arm: {message.get('message', 'monitor failed')}")
            if kind == "monitor_finished":
                return
            if kind == "pose_sample" and message.get("side") == "follower":
                self.sample = message
                self.generation += 1
                self.changed.set()
            elif kind == "command_result":
                if self.stopping:
                    continue
                if message.get("command_id") != self.command_id or self.pending is None:
                    raise RuntimeError("unexpected follower command result; stopping")
                if not self.pending.done():
                    self.pending.set_result(message)
                if "present_load_raw" in message:
                    print(
                        f"  J{message['joint_id']}: {message.get('state')} | {load_description(message)}",
                        flush=True,
                    )
            elif kind == "command_progress":
                print(
                    f"  J{message['joint_id']}: {message.get('current', 0):.2f} -> "
                    f"{message.get('target', 0):.2f} {message.get('unit', '')} | "
                    f"{load_description(message)}",
                    flush=True,
                )

    async def guard(self, awaitable, timeout=None):
        task = asyncio.ensure_future(awaitable)
        try:
            done, _ = await asyncio.wait(
                {task, self.reader}, timeout=timeout,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if self.reader in done:
                self.reader.result()
                raise RuntimeError("follower monitor ended; no further movements requested")
            if task not in done:
                raise TimeoutError("follower response timed out; command will not be retried")
            return task.result()
        finally:
            if not task.done():
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task

    async def fresh_sample(self, after=0):
        async def wait():
            while self.generation <= after:
                self.changed.clear()
                await self.changed.wait()
            if not -0.25 <= time.time() - self.sample["sampled_at"] <= 1.5:
                raise RuntimeError("follower sample is stale; stopping")
            return self.sample

        return await self.guard(wait(), timeout=10)

    async def command(self, joint_id, mode, **fields):
        self.command_id += 1
        self.pending = asyncio.get_running_loop().create_future()
        await self.socket.send(json.dumps({
            "version": VERSION, "type": "command", "command_id": self.command_id,
            "joint_id": joint_id, "mode": mode, **fields,
        }))
        try:
            result = await self.guard(self.pending, timeout=600)
            if result.get("state") != "succeeded":
                raise SweepCommandFailed(result)
            return await self.fresh_sample(after=self.generation)
        finally:
            self.pending = None

    async def close(self):
        self.stopping = True
        try:
            if not self.reader.done():
                await asyncio.wait_for(self.socket.send(json.dumps({
                    "version": VERSION, "type": "stop",
                })), timeout=2)
            await asyncio.wait_for(asyncio.shield(self.reader), timeout=10)
        finally:
            if not self.reader.done():
                self.reader.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await self.reader


def check_torque(sample, enabled_joint=None):
    expected = {joint: joint == enabled_joint for joint in JOINTS}
    if sample.get("torque") != expected:
        raise RuntimeError("follower torque state is not the confirmed single-joint state; stopping")


async def move_with_retry(session, item, target, confirm):
    retries = 0
    while True:
        try:
            sample = await session.command(item["id"], "ticks", value=target)
            check_torque(sample, item["joint"])
            return sample
        except SweepCommandFailed as exc:
            result = exc.result
            snapshot = result.get("servo_snapshot") or {}
            held = result.get("held_raw")
            if (
                result.get("state") != "failed" or result.get("joint_id") != item["id"]
                or type(held) is not int or not item["min"] <= held <= item["max"]
                or snapshot.get("Status", 0) != 0
            ):
                raise
            if retries >= MAX_CONFIRMED_RETRIES:
                print("Retry limit reached; stopping without advancing.", flush=True)
                raise
            check_torque(await session.fresh_sample(after=session.generation), item["joint"])
            print(str(exc), flush=True)
            if await session.guard(confirm(
                f"Follower J{item['id']} held at {held} ticks after the failed move.\n"
                "Support the links and inspect for obstruction, strain or heat before retrying.\n"
                f"Retry {retries + 1}/{MAX_CONFIRMED_RETRIES}: confirm to retry the SAME "
                f"target {target} ticks at unchanged reduced speed.\n"
                "S skips this movement task; q cancels. Do not retry a straining joint."
            )) == "skip":
                return None
            check_torque(await session.fresh_sample(), item["joint"])
            retries += 1


async def run_sweep(url, joint_ids=(1, 2, 3, 4, 5, 6), confirm=confirm_enter):
    endpoint = monitor_url(url)
    async with connect(
        endpoint, additional_headers=access_headers(endpoint), open_timeout=15,
        ping_interval=10, ping_timeout=20, close_timeout=5, max_size=2 * 1024 * 1024,
    ) as socket:
        await socket.send(json.dumps({"version": VERSION, "type": "start"}))
        started = json.loads(await asyncio.wait_for(socket.recv(), timeout=15))
        if (
            started.get("type") != "monitor_started" or started.get("version") != VERSION
            or SAFE_MONITOR_MOVES_CAPABILITY not in started.get("capabilities", [])
        ):
            raise RuntimeError(
                "follower needs the safe-monitor-moves update before sweeping; "
                + started.get("message", "no torque or movement commands sent")
            )
        session = SweepSession(socket)
        failed = False
        try:
            plan = sweep_plan(await session.fresh_sample(), joint_ids)
            print(
                "FOLLOWER SWEEP: full verified ranges, one powered joint at a time.\n"
                "Other joints will be UNPOWERED and can fall. Physically support the arm.\n"
                "No collision planning or force limiting. Keep the physical stop accessible.\n"
                "Confirmed held failures offer manual retry/skip; uncertain failures stop.\n"
                "Ctrl-C never automatically releases torque.",
                flush=True,
            )
            for item in plan:
                joint_id, joint = item["id"], item["joint"]
                label = f"Follower J{joint_id} {joint.removesuffix('.pos')}"
                print(
                    f"\n{label}: MIN={item['min']}, MAX={item['max']}, "
                    f"{item['zero_label']}={item['zero']} ticks.",
                    flush=True,
                )
                if await session.guard(confirm(
                    f"{label}: support ALL joints, including the previously tested joint.\n"
                    "Confirm to RELEASE ALL torque for manual positioning.\n"
                    "Skipping this release skips this joint."
                )) == "skip":
                    print(f"{label}: skipped; no release or movements requested for this joint.", flush=True)
                    continue
                check_torque(await session.command("*", "torque", enabled=False))
                skip_sweep = await session.guard(confirm(
                    f"{label}: manually position the unpowered links so the ENTIRE sweep is clear.\n"
                    f"Confirm to enable ONLY J{joint_id}, move to MIN {item['min']}, "
                    f"then sweep to MAX {item['max']}.\n"
                    "Keep hands out of the tested joint's path and gripper jaws."
                )) == "skip"
                powered = False
                sweep_complete = False
                if skip_sweep:
                    print(f"{label}: MIN/MAX sweep skipped; joint remains unpowered.", flush=True)
                else:
                    check_torque(await session.fresh_sample())
                    check_torque(await session.command(joint_id, "torque", enabled=True), joint)
                    powered = True
                    for phase in ("min", "max"):
                        print(f"{label}: moving to {phase.upper()} {item[phase]} ticks.", flush=True)
                        if await move_with_retry(session, item, item[phase], confirm) is None:
                            print(f"{label}: remaining MIN/MAX sweep skipped; joint remains holding.", flush=True)
                            break
                    else:
                        sweep_complete = True
                if not powered:
                    state = "sweep skipped; joint is unpowered"
                elif sweep_complete:
                    state = "MAX reached within tolerance; joint remains powered"
                else:
                    state = "sweep incomplete; joint is holding its measured position"
                if await session.guard(confirm(
                    f"{label}: {state}.\n"
                    + (f"Confirm to enable ONLY J{joint_id} and return to " if not powered else "Confirm to return to ")
                    + f"{item['zero_label']} ({item['zero']} ticks)."
                )) == "skip":
                    print(f"{label}: zero task skipped; torque state unchanged.", flush=True)
                    continue
                check_torque(await session.fresh_sample(), joint if powered else None)
                if not powered:
                    check_torque(await session.command(joint_id, "torque", enabled=True), joint)
                if await move_with_retry(session, item, item["zero"], confirm) is not None:
                    print(f"{label}: zero reached within tolerance; holding until confirmed release.", flush=True)
                else:
                    print(f"{label}: zero movement skipped; joint remains holding.", flush=True)
            released = await session.guard(confirm(
                "All selected joint tasks finished. Support ALL joints.\n"
                "Confirm to RELEASE ALL torque and finish."
            )) != "skip"
            if released:
                check_torque(await session.command("*", "torque", enabled=False))
        except BaseException:
            failed = True
            raise
        finally:
            try:
                await session.close()
            except Exception as exc:
                if not failed:
                    raise
                print(f"Monitor shutdown unconfirmed: {exc}", file=sys.stderr)
        if released:
            print("Sweep session complete. All follower torque is OFF; keep the arm supported.", flush=True)
        else:
            print("Sweep finished. Final release skipped; existing torque states retained. Keep the arm supported.", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Interactive follower MIN/MAX/zero sweeps over WSS")
    parser.add_argument("--url", default=os.environ.get("CF_ROBOT_CONTROL_URL"))
    parser.add_argument("--joint", type=int, choices=range(1, 7), help="test only this joint; default J1 through J6")
    args = parser.parse_args()
    if not args.url:
        parser.error("pass --url or set CF_ROBOT_CONTROL_URL")
    if not sys.stdin.isatty():
        parser.error("an interactive terminal is required; piped confirmations are not allowed")
    try:
        asyncio.run(run_sweep(args.url, (args.joint,) if args.joint else (1, 2, 3, 4, 5, 6)))
    except KeyboardInterrupt:
        print("\nSweep cancelled. The tested joint may still hold torque; support the arm.")
    except (OSError, RuntimeError, ValueError, WebSocketException) as exc:
        parser.exit(1, f"Sweep stopped: {exc}\nNo automatic retry or torque release. Support the arm.\n")


if __name__ == "__main__":
    main()
