"""Record and move the follower to a manually selected parking pose."""

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import time

from cf_robot.hardware import joint_positions, make_follower
from cf_robot.protocol import JOINTS, validate_action

CALIBRATION_FILE = Path(
    "/var/lib/cf-robot/lerobot/calibration/robots/so_follower/so101_follower.json"
)
PARKING_FILE = Path("/var/lib/cf-robot/parking/so101_follower.json")
PARKING_STEP_DEG = 1.0
PARKING_STEP_DELAY_S = 0.1


def ensure_service_stopped():
    if shutil.which("systemctl") is None:
        raise RuntimeError("systemctl not found; these helpers are for the Raspberry Pi")
    result = subprocess.run(
        ["systemctl", "is-active", "cf-robot-follower"],
        capture_output=True,
        text=True,
        check=False,
    )
    state = result.stdout.strip()
    if state not in ("inactive", "failed"):
        raise RuntimeError(
            "stop cf-robot-follower before using a parking helper "
            f"(service state: {state or 'unknown'})"
        )


def check_calibration():
    if not CALIBRATION_FILE.is_file():
        raise RuntimeError(
            f"follower calibration not found at {CALIBRATION_FILE}; calibrate the arm first"
        )


def prompt_port():
    port = input("Follower serial path (/dev/serial/by-id/...): ").strip()
    try:
        mode = os.stat(port).st_mode
    except OSError as exc:
        raise RuntimeError(f"cannot access serial path {port!r}: {exc}") from exc
    if not stat.S_ISCHR(mode):
        raise RuntimeError(f"not a serial character device: {port}")
    if not os.access(port, os.R_OK | os.W_OK):
        raise RuntimeError(f"robot user cannot read and write the serial device: {port}")
    return port


@contextmanager
def connected_follower(port):
    robot = make_follower(port, "so101_follower")
    try:
        robot.connect(calibrate=False)
        yield robot
    finally:
        try:
            robot.disconnect()
        except Exception as exc:
            print(f"Warning: could not disconnect follower cleanly: {exc}", file=sys.stderr)


def observe_pose(robot):
    return validate_action(joint_positions(robot.get_observation()))


def print_pose(title, pose):
    print(title)
    for name in JOINTS:
        print(f"  {name}: {pose[name]:.2f} deg")


def write_parking_pose(pose):
    PARKING_FILE.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
    temporary = PARKING_FILE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(
            {
                "version": 1,
                "arm_id": "so101_follower",
                "joint_positions": pose,
            },
            indent=2,
        )
        + "\n"
    )
    temporary.chmod(0o600)
    temporary.replace(PARKING_FILE)


def load_parking_pose():
    try:
        saved = json.loads(PARKING_FILE.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"cannot read parking pose at {PARKING_FILE}: {exc}") from exc
    if saved.get("version") != 1 or saved.get("arm_id") != "so101_follower":
        raise RuntimeError("parking file has an unsupported version or arm ID")
    try:
        return validate_action(saved["joint_positions"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid parking pose in {PARKING_FILE}: {exc}") from exc


def interpolated_targets(start, end, max_step=PARKING_STEP_DEG):
    largest_delta = max(abs(end[name] - start[name]) for name in JOINTS)
    steps = max(1, math.ceil(largest_delta / max_step))
    for index in range(1, steps + 1):
        fraction = index / steps
        yield {
            name: start[name] + (end[name] - start[name]) * fraction
            for name in JOINTS
        }


def record_pose():
    ensure_service_stopped()
    check_calibration()
    port = prompt_port()
    with connected_follower(port) as robot:
        pose = observe_pose(robot)
        print_pose("Current follower pose:", pose)
        if PARKING_FILE.exists():
            print(f"This will replace the existing parking pose at {PARKING_FILE}.")
        if input("Type RECORD to save this pose: ").strip() != "RECORD":
            print("Not saved.")
            return
        write_parking_pose(pose)
    print(f"Parking pose saved to {PARKING_FILE}")


def move_to_parking_pose():
    ensure_service_stopped()
    check_calibration()
    target = load_parking_pose()
    port = prompt_port()

    with connected_follower(port) as robot:
        start = observe_pose(robot)
        print_pose("Current follower pose:", start)
        print_pose("Recorded parking pose:", target)
        largest_delta = max(abs(target[name] - start[name]) for name in JOINTS)
        steps = max(1, math.ceil(largest_delta / PARKING_STEP_DEG))
        print(
            f"Move to the saved pose in {steps} small steps, at most "
            f"{PARKING_STEP_DEG:g} degree per joint per step."
        )
        if input("Clear the workspace, then type MOVE to proceed: ").strip() != "MOVE":
            print("Move cancelled.")
            return

        for next_pose in interpolated_targets(start, target):
            robot.send_action(next_pose)
            time.sleep(PARKING_STEP_DELAY_S)

        time.sleep(0.3)
        final_pose = observe_pose(robot)
        print_pose("Follower pose after parking move:", final_pose)
        remaining = max(abs(target[name] - final_pose[name]) for name in JOINTS)
        if remaining > 5.0:
            print(
                f"Warning: follower is still {remaining:.2f} degrees from the "
                "saved pose; inspect it before continuing."
            )


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in {"record", "set"}:
        print("Usage: python -m cf_robot.parking {record|set}", file=sys.stderr)
        return 2
    try:
        if sys.argv[1] == "record":
            record_pose()
        else:
            move_to_parking_pose()
    except KeyboardInterrupt:
        print("\nInterrupted. Check that the follower is safe.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Parking helper failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
