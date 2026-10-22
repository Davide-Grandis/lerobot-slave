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
    print(prompt + "\nPress ENTER to confirm; q or Ctrl-C cancels: ", end="", flush=True)

    def ready():
        if not answer.done():
            answer.set_result(sys.stdin.readline())

    loop.add_reader(sys.stdin.fileno(), ready)
    try:
        line = await answer
    finally:
        loop.remove_reader(sys.stdin.fileno())
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
            elif kind == "command_progress":
                print(
                    f"  J{message['joint_id']}: {message.get('current', 0):.2f} -> "
                    f"{message.get('target', 0):.2f} {message.get('unit', '')}",
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
                raise RuntimeError(
                    "follower command failed; sweep stopped without retry: "
                    + json.dumps(result)
                )
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
                "Any failed move stops the routine; Ctrl-C never automatically releases torque.",
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
                await session.guard(confirm(
                    f"{label}: support ALL joints, including the previously tested joint.\n"
                    "Confirm to RELEASE ALL torque for manual positioning."
                ))
                check_torque(await session.command("*", "torque", enabled=False))
                await session.guard(confirm(
                    f"{label}: manually position the unpowered links so the ENTIRE sweep is clear.\n"
                    f"Confirm to enable ONLY J{joint_id}, move to MIN {item['min']}, "
                    f"then sweep to MAX {item['max']}.\n"
                    "Keep hands out of the tested joint's path and gripper jaws."
                ))
                check_torque(await session.fresh_sample())
                check_torque(await session.command(joint_id, "torque", enabled=True), joint)
                for phase in ("min", "max"):
                    print(f"{label}: moving to {phase.upper()} {item[phase]} ticks.", flush=True)
                    check_torque(await session.command(joint_id, "ticks", value=item[phase]), joint)
                await session.guard(confirm(
                    f"{label}: MAX reached within tolerance; joint remains powered.\n"
                    f"Confirm to return to {item['zero_label']} ({item['zero']} ticks)."
                ))
                check_torque(await session.fresh_sample(), joint)
                check_torque(await session.command(joint_id, "ticks", value=item["zero"]), joint)
                print(f"{label}: zero reached within tolerance; holding until confirmed release.", flush=True)
            await session.guard(confirm(
                "All selected sweeps completed. Support ALL joints.\n"
                "Confirm to RELEASE ALL torque and finish."
            ))
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
        print("Sweep complete. All follower torque is OFF; keep the arm supported.", flush=True)


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
