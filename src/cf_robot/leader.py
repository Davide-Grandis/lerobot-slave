"""Laptop CLI: read the local SO-ARM101 leader and stream to the Pi."""

import argparse
import asyncio
import contextlib
import json
import time

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.hardware import joint_positions, make_leader
from cf_robot.parking import parking_steps
from cf_robot.protocol import JOINTS, VERSION, action_message, validate_action

STARTUP_SYNC_TOLERANCE = 1.0
# Half the follower's per-target guard, at a faster ramp than parking motion.
STARTUP_SYNC_STEP_UNITS = 2.5
STARTUP_SYNC_STEP_DELAY_S = 0.1
STARTUP_SYNC_KEEPALIVE_S = 0.2
STARTUP_SYNC_SETTLE_TIMEOUT_S = 30.0


async def stream_status(socket, on_status):
    async for raw in socket:
        message = json.loads(raw)
        if message.get("type") == "error":
            raise RuntimeError(message.get("message", "follower error"))
        if message.get("type") == "status":
            on_status(message)
    raise ConnectionError("follower connection closed")


async def run_teleop(
    url,
    port,
    arm_id,
    fps=30,
    stop=None,
    on_status=None,
    leader_factory=None,
    sync_at_start=False,
):
    if not 1 <= fps <= 60:
        raise ValueError("fps must be between 1 and 60")
    stop = stop if stop is not None else asyncio.Event()
    on_status = on_status or (lambda message: None)
    leader_factory = leader_factory or (lambda: make_leader(port, arm_id))
    headers = access_headers(url)
    leader = None
    reader = None
    operation_error = None
    async with connect(url, additional_headers=headers, open_timeout=10, ping_interval=10, ping_timeout=5) as socket:
        first = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
        if first.get("type") != "ready" or first.get("version") != VERSION:
            raise RuntimeError(first.get("message", "follower did not become ready"))
        follower_pose = None
        if sync_at_start:
            try:
                follower_pose = validate_action(first["observation"])
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    "Pi must report its measured follower pose before startup sync"
                ) from exc
        on_status({"type": "status", "state": "connecting leader"})
        try:
            leader = leader_factory()
            await asyncio.to_thread(leader.connect, calibrate=False)
            if getattr(leader, "is_connected", True) is False:
                raise RuntimeError(
                    "leader connect() returned but the driver reports disconnected; "
                    "check the USB port and close any other process using the leader"
                )
            on_status({"type": "status", "state": "leader connected"})
            latest_feedback = {"observation": None, "observed_at": None}

            def handle_status(message):
                if message.get("type") == "status" and message.get("observation"):
                    try:
                        latest_feedback["observation"] = validate_action(message["observation"])
                        latest_feedback["observed_at"] = float(message["observed_at"])
                    except (KeyError, TypeError, ValueError):
                        pass
                on_status(message)

            reader = asyncio.create_task(stream_status(socket, handle_status))
            seq = 0
            period = 1 / fps
            if sync_at_start:
                initial_leader_pose = validate_action(
                    joint_positions(await asyncio.to_thread(leader.get_action))
                )
                initial_gap = max(
                    abs(initial_leader_pose[joint] - follower_pose[joint])
                    for joint in JOINTS
                )
                if initial_gap > STARTUP_SYNC_TOLERANCE:
                    sync_steps = list(
                        parking_steps(
                            follower_pose,
                            initial_leader_pose,
                            max_step=STARTUP_SYNC_STEP_UNITS,
                        )
                    )
                    duration = len(sync_steps) * STARTUP_SYNC_STEP_DELAY_S
                    print("Measured follower and leader startup poses (servo units):")
                    for joint in JOINTS:
                        print(
                            f"  {joint}: follower={follower_pose[joint]:.2f}, "
                            f"leader={initial_leader_pose[joint]:.2f}, "
                            f"delta={initial_leader_pose[joint] - follower_pose[joint]:+.2f}"
                        )
                    print(
                        f"Startup sync will move one servo at a time in at most "
                        f"{STARTUP_SYNC_STEP_UNITS:g}-unit steps, taking at least "
                        f"{duration:.1f} seconds."
                    )
                    confirmation = await asyncio.to_thread(
                        input,
                        "Clear the workspace, hold the leader still, then type SYNC to proceed: ",
                    )
                    if confirmation.strip() != "SYNC":
                        raise RuntimeError("startup sync cancelled; no targets were sent")

                    active_joint = None
                    last_target_sent_at = None
                    for joint, target in sync_steps:
                        if reader.done():
                            reader.result()
                        if joint != active_joint:
                            on_status(
                                {"type": "status", "state": f"synchronizing {joint}"}
                            )
                            active_joint = joint
                        seq += 1
                        await socket.send(
                            json.dumps(
                                action_message(seq, target),
                                separators=(",", ":"),
                            )
                        )
                        last_target_sent_at = time.time()
                        await asyncio.sleep(STARTUP_SYNC_STEP_DELAY_S)

                    deadline = time.monotonic() + STARTUP_SYNC_SETTLE_TIMEOUT_S
                    last_action_sent_at = time.monotonic()
                    while time.monotonic() < deadline:
                        if reader.done():
                            reader.result()
                        measured = latest_feedback["observation"]
                        observed_at = latest_feedback["observed_at"]
                        if (
                            measured is not None
                            and observed_at is not None
                            and last_target_sent_at is not None
                            and observed_at >= last_target_sent_at
                            and all(
                                abs(measured[joint] - initial_leader_pose[joint])
                                <= STARTUP_SYNC_TOLERANCE
                                for joint in JOINTS
                            )
                        ):
                            break
                        if time.monotonic() - last_action_sent_at >= STARTUP_SYNC_KEEPALIVE_S:
                            seq += 1
                            await socket.send(
                                json.dumps(
                                    action_message(seq, initial_leader_pose),
                                    separators=(",", ":"),
                                )
                            )
                            last_target_sent_at = time.time()
                            last_action_sent_at = time.monotonic()
                        await asyncio.sleep(0.05)
                    else:
                        raise RuntimeError(
                            "follower did not reach the startup leader pose within "
                            f"{STARTUP_SYNC_SETTLE_TIMEOUT_S:g} seconds; inspect both arms"
                        )
                on_status({"type": "status", "state": "startup sync complete"})

            on_status({"type": "status", "state": "running"})
            while not stop.is_set():
                started = time.monotonic()
                if reader.done():
                    reader.result()
                action = await asyncio.to_thread(leader.get_action)
                if reader.done():
                    reader.result()
                seq += 1
                await socket.send(json.dumps(action_message(seq, action), separators=(",", ":")))
                remaining = period - (time.monotonic() - started)
                if remaining > 0:
                    try:
                        await asyncio.wait_for(stop.wait(), timeout=remaining)
                    except asyncio.TimeoutError:
                        pass
        except BaseException as exc:
            operation_error = exc
            raise
        finally:
            if reader is not None:
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionError, RuntimeError):
                    await reader
            if leader is not None:
                try:
                    await asyncio.to_thread(leader.disconnect)
                except Exception:
                    if operation_error is None:
                        raise


def main():
    parser = argparse.ArgumentParser(description="Teleoperate a remote SO-ARM101 follower")
    parser.add_argument("--url", required=True, help="wss://<public-fqdn>/control")
    parser.add_argument("--port", required=True, help="Local leader serial device")
    parser.add_argument("--arm-id", required=True, help="LeRobot leader calibration ID")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--no-startup-sync",
        action="store_true",
        help="disable the confirmed gradual follower-to-leader sync when needed",
    )
    args = parser.parse_args()
    last_printed_state = None

    def report_status(message):
        nonlocal last_printed_state
        state = message.get("state")
        if state and state != last_printed_state:
            print(state)
            last_printed_state = state

    try:
        asyncio.run(
            run_teleop(
                args.url,
                args.port,
                args.arm_id,
                args.fps,
                on_status=report_status,
                sync_at_start=not args.no_startup_sync,
            )
        )
    except KeyboardInterrupt:
        print("Stopped")
    except (OSError, ValueError, RuntimeError, ConnectionError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Teleoperation stopped:\n{exc}\n")


if __name__ == "__main__":
    main()
