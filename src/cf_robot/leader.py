"""Laptop CLI: read the local SO-ARM101 leader and stream to the Pi."""

import argparse
import asyncio
import contextlib
import json
import time

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.gripper_control import GripperLeader, gripper_control_url
from cf_robot.hardware import joint_positions, make_leader
from cf_robot.parking import parking_steps
from cf_robot.protocol import GRIPPER_JOINTS, JOINTS, VERSION, action_message, validate_action

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


async def sync_gripper(socket, reader, feedback, target, seq, stop, on_status):
    """Synchronize only J6 to a fixed startup snapshot using measured readback."""
    on_status({"type": "status", "state": "synchronizing gripper.pos"})
    deadline = time.monotonic() + STARTUP_SYNC_SETTLE_TIMEOUT_S
    seeded = False
    final_generation = None
    last_report = 0
    while not stop.is_set():
        if reader.done():
            reader.result()
        received_at = feedback["received_at"]
        if received_at is None or time.monotonic() - received_at > 0.5:
            raise RuntimeError("fresh J6 follower readback lost during startup sync")
        measured = feedback["observation"]["gripper.pos"]
        if (
            final_generation is not None
            and feedback["generation"] > final_generation
            and abs(target - measured) <= STARTUP_SYNC_TOLERANCE
        ):
            on_status({"type": "status", "state": "J6 startup sync complete"})
            return seq
        if time.monotonic() >= deadline:
            raise RuntimeError(
                f"J6 startup sync timed out: follower={measured:.2f}%, "
                f"leader startup target={target:.2f}%; inspect the gripper"
            )
        if not seeded:
            # The Pi's first-target alignment guard remains intact: arm while
            # holding the measured follower position, then begin the J6 ramp.
            goal = measured
            seeded = True
        else:
            goal = max(
                measured - STARTUP_SYNC_STEP_UNITS, min(measured + STARTUP_SYNC_STEP_UNITS, target),
            )
        if goal == target and final_generation is None:
            final_generation = feedback["generation"]
        seq += 1
        await socket.send(json.dumps(
            action_message(seq, {"gripper.pos": goal}, GRIPPER_JOINTS),
            separators=(",", ":"),
        ))
        if time.monotonic() - last_report >= 0.25:
            on_status({
                "type": "gripper_sample", "leader_percent": target,
                "follower_percent": measured,
            })
            last_report = time.monotonic()
        try:
            await asyncio.wait_for(stop.wait(), timeout=STARTUP_SYNC_STEP_DELAY_S)
        except TimeoutError:
            pass
    return seq


async def run_teleop(
    url,
    port,
    arm_id,
    fps=30,
    stop=None,
    on_status=None,
    leader_factory=None,
    sync_at_start=False,
    gripper_only=False,
):
    if not 1 <= fps <= 60:
        raise ValueError("fps must be between 1 and 60")
    stop = stop if stop is not None else asyncio.Event()
    on_status = on_status or (lambda message: None)
    leader_factory = leader_factory or (lambda: make_leader(port, arm_id))
    joints = GRIPPER_JOINTS if gripper_only else JOINTS
    sync_gripper_at_start = gripper_only and sync_at_start
    if gripper_only:
        url = gripper_control_url(url)
        sync_at_start = False
    headers = access_headers(url)
    leader = None
    reader = None
    operation_error = None
    async with connect(url, additional_headers=headers, open_timeout=10, ping_interval=10, ping_timeout=5) as socket:
        first = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
        if first.get("type") != "ready" or first.get("version") != VERSION:
            raise RuntimeError(first.get("message", "follower did not become ready"))
        if gripper_only and (
            first.get("mode") != "gripper-only" or first.get("joints") != list(GRIPPER_JOINTS)
        ):
            raise RuntimeError("Pi did not confirm J6-only isolation; refusing to send any targets")
        follower_pose = None
        if sync_at_start or gripper_only:
            try:
                follower_pose = validate_action(first["observation"], joints)
            except (KeyError, TypeError, ValueError) as exc:
                raise RuntimeError(
                    "Pi must report its measured follower pose before startup sync"
                ) from exc
        on_status({"type": "status", "state": "connecting leader"})
        try:
            leader = leader_factory()
            if gripper_only:
                leader = GripperLeader(leader)
            await asyncio.to_thread(leader.connect, calibrate=False)
            if getattr(leader, "is_connected", True) is False:
                raise RuntimeError(
                    "leader connect() returned but the driver reports disconnected; "
                    "check the USB port and close any other process using the leader"
                )
            on_status({"type": "status", "state": "leader connected"})
            latest_feedback = {"observation": None, "received_at": None, "generation": 0}

            def handle_status(message):
                if message.get("type") == "status" and message.get("observation"):
                    try:
                        if gripper_only:
                            observed_at = message.get("observed_at")
                            if (
                                not isinstance(observed_at, (int, float))
                                or not -0.25 <= time.time() - observed_at <= 0.5
                            ):
                                return
                        latest_feedback["observation"] = validate_action(message["observation"], joints)
                        latest_feedback["received_at"] = time.monotonic()
                        latest_feedback["generation"] += 1
                    except (KeyError, TypeError, ValueError):
                        pass
                on_status(message)

            reader = asyncio.create_task(stream_status(socket, handle_status))
            seq = 0
            period = 1 / fps
            if gripper_only:
                initial = await asyncio.to_thread(leader.get_action)
                print(
                    f"J6 ONLY: leader={initial['gripper.pos']:.2f}%, "
                    f"follower={follower_pose['gripper.pos']:.2f}%. "
                    "No J1-J5 goals or torque changes will be sent."
                )
                confirmation = await asyncio.to_thread(
                    input,
                    "Secure both arms and clear jaws/objects. "
                    + (
                        "The follower gripper will sync to the leader; hold the leader still. "
                        if sync_gripper_at_start else "Align grippers within 1%. "
                    )
                    + "J6 will be powered while streaming and released on stop. "
                    "Press Enter to continue (Ctrl-C cancels): ",
                )
                if confirmation.strip():
                    raise RuntimeError("gripper-only test cancelled without arming")
                if sync_gripper_at_start:
                    on_status({"type": "status", "state": "waiting for J6 startup sync"})
                deadline = time.monotonic() + 2
                while (
                    latest_feedback["received_at"] is None
                    or time.monotonic() - latest_feedback["received_at"] > 0.5
                ):
                    if reader.done():
                        reader.result()
                    if time.monotonic() > deadline:
                        raise RuntimeError("fresh J6 follower readback required before arming")
                    await asyncio.sleep(0.05)
                initial = await asyncio.to_thread(leader.get_action)
                measured = latest_feedback["observation"]
                if sync_gripper_at_start:
                    print(
                        f"J6 startup sync: follower={measured['gripper.pos']:.2f}% -> "
                        f"leader={initial['gripper.pos']:.2f}%"
                    )
                    seq = await sync_gripper(
                        socket, reader, latest_feedback, initial["gripper.pos"],
                        seq, stop, on_status,
                    )
                    if stop.is_set():
                        return
                elif abs(initial["gripper.pos"] - measured["gripper.pos"]) > STARTUP_SYNC_TOLERANCE:
                    raise RuntimeError(
                        f"align J6 before starting: leader={initial['gripper.pos']:.2f}%, "
                        f"follower={measured['gripper.pos']:.2f}%; no startup movement was sent"
                    )
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
                    await asyncio.to_thread(
                        input,
                        "Clear the workspace, hold the leader still, then press Enter to proceed (Ctrl-C cancels): ",
                    )

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
                        last_target_sent_at = time.monotonic()
                        await asyncio.sleep(STARTUP_SYNC_STEP_DELAY_S)

                    deadline = time.monotonic() + STARTUP_SYNC_SETTLE_TIMEOUT_S
                    last_action_sent_at = time.monotonic()
                    last_progress_at = last_action_sent_at
                    while time.monotonic() < deadline:
                        if reader.done():
                            reader.result()
                        measured = latest_feedback["observation"]
                        feedback_received_at = latest_feedback["received_at"]
                        if (
                            measured is not None
                            and feedback_received_at is not None
                            and last_target_sent_at is not None
                            and feedback_received_at >= last_target_sent_at
                            and all(
                                abs(measured[joint] - initial_leader_pose[joint])
                                <= STARTUP_SYNC_TOLERANCE
                                for joint in JOINTS
                            )
                        ):
                            break
                        now = time.monotonic()
                        if now - last_progress_at >= 2.0:
                            if measured is None:
                                print("Waiting for measured follower readback...")
                            else:
                                remaining = [
                                    f"{joint}: follower={measured[joint]:.2f}, "
                                    f"leader={initial_leader_pose[joint]:.2f}, "
                                    f"delta={initial_leader_pose[joint] - measured[joint]:+.2f}"
                                    for joint in JOINTS
                                    if abs(measured[joint] - initial_leader_pose[joint])
                                    > STARTUP_SYNC_TOLERANCE
                                ]
                                if remaining:
                                    print("Follower still settling: " + "; ".join(remaining))
                            last_progress_at = now
                        if time.monotonic() - last_action_sent_at >= STARTUP_SYNC_KEEPALIVE_S:
                            seq += 1
                            await socket.send(
                                json.dumps(
                                    action_message(seq, initial_leader_pose),
                                    separators=(",", ":"),
                                )
                            )
                            last_target_sent_at = time.monotonic()
                            last_action_sent_at = time.monotonic()
                        await asyncio.sleep(0.05)
                    else:
                        measured = latest_feedback["observation"]
                        if measured is None:
                            residual = "no measured follower feedback received"
                        else:
                            offsets = [
                                f"{joint}: follower={measured[joint]:.2f}, "
                                f"leader={initial_leader_pose[joint]:.2f}, "
                                f"delta={initial_leader_pose[joint] - measured[joint]:+.2f}"
                                for joint in JOINTS
                                if abs(measured[joint] - initial_leader_pose[joint])
                                > STARTUP_SYNC_TOLERANCE
                            ]
                            residual = (
                                "; ".join(offsets)
                                if offsets
                                else "readback was not refreshed after the final target"
                            )
                        raise RuntimeError(
                            "follower did not reach the startup leader pose within "
                            f"{STARTUP_SYNC_SETTLE_TIMEOUT_S:g} seconds; "
                            f"final readback: {residual}; inspect both arms"
                        )
                on_status({"type": "status", "state": "startup sync complete"})

            on_status({"type": "status", "state": "running"})
            last_gripper_report = 0
            while not stop.is_set():
                started = time.monotonic()
                if reader.done():
                    reader.result()
                action = await asyncio.to_thread(leader.get_action)
                if reader.done():
                    reader.result()
                if gripper_only and time.monotonic() - latest_feedback["received_at"] > 0.5:
                    raise RuntimeError("fresh J6 follower readback lost; gripper-only test stopped")
                seq += 1
                await socket.send(json.dumps(action_message(seq, action, joints), separators=(",", ":")))
                if gripper_only and time.monotonic() - last_gripper_report >= 0.25:
                    on_status({
                        "type": "gripper_sample", "leader_percent": action["gripper.pos"],
                        "follower_percent": latest_feedback["observation"]["gripper.pos"],
                    })
                    last_gripper_report = time.monotonic()
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
        "--J6-only", dest="gripper_only", action="store_true",
        help="isolated J6 test; never send goals or torque changes to J1-J5",
    )
    parser.add_argument(
        "--no-startup-sync",
        action="store_true",
        help="disable the confirmed gradual follower-to-leader sync when needed",
    )
    args = parser.parse_args()
    last_printed_state = None
    waiting_for_sync = False

    def report_status(message):
        nonlocal last_printed_state, waiting_for_sync
        state = message.get("state")
        if args.gripper_only:
            if state in ("waiting for J6 startup sync", "synchronizing gripper.pos"):
                waiting_for_sync = True
                state = "WAIT - syncing J6; hold the leader still."
            elif state == "running":
                waiting_for_sync = False
                state = "Ready for teleop"
            elif state in ("ready", "J6 startup sync complete"):
                # Pi hardware readiness is not permission to start teleoperating.
                return
        if message.get("type") == "gripper_sample":
            leader_percent = message["leader_percent"]
            follower_percent = message["follower_percent"]
            print(
                f"{'WAIT - ' if waiting_for_sync else ''}"
                f"J6 leader={leader_percent:.2f}%  follower={follower_percent:.2f}%  "
                f"delta={leader_percent - follower_percent:+.2f} percentage points"
            )
            return
        if state and state != last_printed_state:
            print(state, flush=True)
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
                gripper_only=args.gripper_only,
            )
        )
    except KeyboardInterrupt:
        print("Stopped")
    except (OSError, ValueError, RuntimeError, ConnectionError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Teleoperation stopped:\n{exc}\n")


if __name__ == "__main__":
    main()
