"""Laptop CLI: read the local SO-ARM101 leader and stream to the Pi."""

import argparse
import asyncio
import contextlib
import json
import time

from websockets.asyncio.client import connect
from websockets.exceptions import WebSocketException

from cf_robot.auth import access_headers
from cf_robot.gripper_control import GripperLeader, joint_control_url, wrist_gripper_control_url
from cf_robot.hardware import make_leader
from cf_robot.protocol import HOLD_CAPABILITY, JOINTS, VERSION, WRIST_GRIPPER_JOINTS, action_message, validate_action
from cf_robot.teleop_hardware import TeleopLeader
from cf_robot.teleop_motion import joint_label
from cf_robot.teleop_runtime import Feedback, recover_live_feedback, synchronize
from cf_robot.teleop_runtime import SYNC_STEP as STARTUP_SYNC_STEP_UNITS  # noqa: F401 - compatibility export
from cf_robot.teleop_runtime import SYNC_TIMEOUT_S as STARTUP_SYNC_SETTLE_TIMEOUT_S
from cf_robot.teleop_runtime import SYNC_TOLERANCE as STARTUP_SYNC_TOLERANCE


async def stream_status(socket, on_status):
    async for raw in socket:
        message = json.loads(raw)
        if message.get("type") == "error":
            raise RuntimeError(f"follower arm: {message.get('message', 'unknown error')}")
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
    gripper_only=False,
    joint_only=None,
    wrist_gripper_only=False,
):
    if not 1 <= fps <= 60:
        raise ValueError("fps must be between 1 and 60")
    stop = stop if stop is not None else asyncio.Event()
    on_status = on_status or (lambda message: None)
    leader_factory = leader_factory or (lambda: make_leader(port, arm_id))
    if sum((bool(gripper_only), joint_only is not None, bool(wrist_gripper_only))) > 1:
        raise ValueError("select only one joint-only mode")
    selected_id = 6 if gripper_only else joint_only
    if selected_id is not None and (type(selected_id) is not int or not 1 <= selected_id <= 6):
        raise ValueError("joint-only mode requires J1-J6")
    gripper_only = selected_id == 6
    joints = WRIST_GRIPPER_JOINTS if wrist_gripper_only else (
        (JOINTS[selected_id - 1],) if selected_id is not None else JOINTS
    )
    isolated = selected_id is not None or wrist_gripper_only
    isolation_label = "J5-J6" if wrist_gripper_only else f"J{selected_id}"
    if wrist_gripper_only:
        url = wrist_gripper_control_url(url)
    elif selected_id is not None:
        url = joint_control_url(url, selected_id)
    headers = access_headers(url)
    leader = None
    reader = None
    operation_error = None
    async with connect(url, additional_headers=headers, open_timeout=10, ping_interval=10, ping_timeout=5) as socket:
        first = json.loads(await asyncio.wait_for(socket.recv(), timeout=25))
        if first.get("type") != "ready" or first.get("version") != VERSION:
            raise RuntimeError(f"follower arm: {first.get('message', 'did not become ready')}")
        if isolated and (
            first.get("mode") != ("gripper-only" if gripper_only else "joint-only")
            or first.get("joints") != list(joints)
        ):
            raise RuntimeError(f"Pi did not confirm {isolation_label}-only isolation; refusing to send any targets")
        supports_hold = HOLD_CAPABILITY in first.get("capabilities", [])
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
            elif hasattr(leader, "bus"):
                leader = TeleopLeader(leader, joints)
            await asyncio.to_thread(leader.connect, calibrate=False)
            if getattr(leader, "is_connected", True) is False:
                raise RuntimeError(
                    "leader connect() returned but the driver reports disconnected; "
                    "check the USB port and close any other process using the leader"
                )
            on_status({"type": "status", "state": "leader connected"})
            latest_feedback = Feedback(joints)

            def handle_status(message):
                latest_feedback.accept(message)
                on_status(message)

            reader = asyncio.create_task(stream_status(socket, handle_status))
            seq = 0
            period = 1 / fps
            initial = validate_action(await asyncio.to_thread(leader.get_action), joints)
            if gripper_only:
                print(
                    f"J6 ONLY: leader={initial['gripper.pos']:.2f}%, "
                    f"follower={follower_pose['gripper.pos']:.2f}%. "
                    "No J1-J5 goals or torque changes will be sent."
                )
            elif selected_id is not None:
                joint = joints[0]
                print(
                    f"J{selected_id} ONLY: leader={initial[joint]:.2f} deg, "
                    f"follower={follower_pose[joint]:.2f} deg. Other joints are untouched."
                )
            else:
                print(
                    "J5-J6 ONLY: wrist roll in degrees; gripper in percent. J1-J4 remain untouched."
                    if wrist_gripper_only else "Full-arm teleoperation: J1-J5 degrees; J6 percent."
                )
                for joint in joints:
                    print(
                        f"  {joint_label(joint)} leader={initial[joint]:.2f}, "
                        f"follower={follower_pose[joint]:.2f}"
                    )
            confirmation = await asyncio.to_thread(
                input,
                "Secure both arms and clear the workspace/jaws. "
                + (
                    "The follower will sync selected joints to the leader; hold the leader still. "
                    if sync_at_start else "Manually align selected joints within 1 unit. "
                )
                + (
                    "J5 will hold position; J6 will be released on stop. " if wrist_gripper_only else
                    "J6 will be released on stop. " if gripper_only
                    else "Selected follower joints will hold position on stop; keep physical support available. "
                )
                + "Press Enter to continue (Ctrl-C cancels): ",
            )
            if confirmation.strip():
                raise RuntimeError("teleoperation cancelled without arming")
            if sync_at_start:
                on_status({
                    "type": "status",
                    "state": "waiting for J6 startup sync" if gripper_only else "waiting for startup sync",
                })
            deadline = time.monotonic() + 2
            while not latest_feedback.fresh:
                if reader.done():
                    reader.result()
                if time.monotonic() > deadline:
                    raise latest_feedback.error("before arming")
                await asyncio.sleep(0.05)
            initial = validate_action(await asyncio.to_thread(leader.get_action), joints)
            if sync_at_start:
                seq = await synchronize(
                    socket, reader, latest_feedback, initial, seq, stop, on_status,
                    timeout=STARTUP_SYNC_SETTLE_TIMEOUT_S,
                )
                if stop.is_set():
                    return
            elif any(
                abs(initial[joint] - latest_feedback["observation"][joint]) > STARTUP_SYNC_TOLERANCE
                for joint in joints
            ):
                raise RuntimeError("manually align selected joints before starting; no startup movement was sent")
            on_status({"type": "status", "state": "running"})
            last_gripper_report = 0
            while not stop.is_set():
                started = time.monotonic()
                if reader.done():
                    reader.result()
                action = await asyncio.to_thread(leader.get_action)
                if reader.done():
                    reader.result()
                seq, paused = await recover_live_feedback(
                    socket, reader, latest_feedback, seq, stop, on_status, supports_hold,
                )
                if paused:
                    continue  # Re-read the leader; never resume with a stale queued target.
                seq += 1
                await socket.send(json.dumps(action_message(seq, action, joints), separators=(",", ":")))
                if time.monotonic() - last_gripper_report >= 0.25:
                    if gripper_only:
                        on_status({
                            "type": "gripper_sample", "leader_percent": action["gripper.pos"],
                            "follower_percent": latest_feedback["observation"]["gripper.pos"],
                        })
                    else:
                        on_status({
                            "type": "teleop_sample", "target": action,
                            "observation": latest_feedback["observation"],
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
    selected = parser.add_mutually_exclusive_group()
    for joint_id in range(1, 6):
        selected.add_argument(
            f"--J{joint_id}-only", dest="joint_only", action="store_const", const=joint_id,
            help=f"isolated J{joint_id} test; other joints remain untouched",
        )
    selected.add_argument(
        "--J6-only", dest="gripper_only", action="store_true",
        help="isolated J6 test; never send goals or torque changes to J1-J5",
    )
    selected.add_argument(
        "--J5-J6-only", dest="wrist_gripper_only", action="store_true",
        help="isolated wrist-roll and gripper control; never change J1-J4 goals or torque",
    )
    parser.add_argument(
        "--no-startup-sync",
        action="store_true",
        help="disable the confirmed gradual follower-to-leader sync when needed",
    )
    args = parser.parse_args()
    last_printed_state = None
    waiting_for_sync = False
    selected_label = "J5-J6" if args.wrist_gripper_only else (
        f"J{6 if args.gripper_only else args.joint_only}" if args.gripper_only or args.joint_only else None
    )

    def report_status(message):
        nonlocal last_printed_state, waiting_for_sync
        state = message.get("state")
        if state in ("waiting for J6 startup sync", "waiting for startup sync") or (
            isinstance(state, str) and state.startswith("synchronizing ")
        ):
            waiting_for_sync = True
            selected = selected_label or (
                state.removeprefix("synchronizing ") if state.startswith("synchronizing ") else "selected joints"
            )
            state = f"WAIT - syncing {selected}; hold the leader still."
        elif state in ("waiting for J6 feedback", "waiting for follower feedback"):
            waiting_for_sync = True
            selected = selected_label or "selected-joint"
            state = f"WAIT - feedback delayed; holding the last {selected} sync goal."
        elif state == "waiting for live feedback":
            waiting_for_sync = True
            state = "WAIT - feedback delayed; Pi holding current measured position."
        elif state == "running":
            waiting_for_sync = False
            state = "Ready for teleop"
        elif state in ("ready", "J6 startup sync complete", "startup sync complete"):
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
        if message.get("type") == "teleop_sample":
            target, observed = message["target"], message["observation"]
            joint = max(target, key=lambda name: abs(target[name] - observed[name]))
            print(
                f"{'WAIT - ' if waiting_for_sync else ''}{joint_label(joint)} "
                f"leader={target[joint]:.2f} follower={observed[joint]:.2f} "
                f"delta={target[joint] - observed[joint]:+.2f} (largest error)"
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
                joint_only=args.joint_only,
                wrist_gripper_only=args.wrist_gripper_only,
            )
        )
    except KeyboardInterrupt:
        print("Stopped")
    except (OSError, ValueError, RuntimeError, ConnectionError, TimeoutError, WebSocketException) as exc:
        parser.exit(1, f"Teleoperation stopped:\n{exc}\n")


if __name__ == "__main__":
    main()
