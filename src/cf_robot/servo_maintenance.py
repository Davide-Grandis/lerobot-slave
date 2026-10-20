"""Fixed follower servo-limit inspection and SDK-compatible go-home motion."""

import json
import logging
import math
from pathlib import Path
import time

from cf_robot.hardware import make_follower
from cf_robot.protocol import JOINTS

LOG = logging.getLogger(__name__)

FOLLOWER_ID = "so101_follower"
LIMITS_FILE = Path("/var/lib/cf-robot/servo-limits/so101_follower.json")
MOTORS = tuple(joint.removesuffix(".pos") for joint in JOINTS)
MAX_RAW_POSITION = 4095
MIN_USABLE_RANGE = 100

# Match the SDK's low-speed home move; the CLI supplies the restore percentage.
HOME_SPEED = max(1, round(0.10 * 1023))
HOME_POSITION_TOLERANCE = 20
HOME_TIMEOUT_S = 180
PLAN_DRIFT_TOLERANCE = 20


def calculate_home_positions(limits):
    """Return the SDK demo's per-servo home pose in raw encoder ticks."""
    home = {}
    for index, joint in enumerate(JOINTS):
        joint_limits = limits.get(joint)
        low = int(joint_limits["min"]) if joint_limits else 0
        high = int(joint_limits["max"]) if joint_limits else MAX_RAW_POSITION
        if index == 0:  # shoulder pan: center
            position = math.floor((low + high) / 2 + 0.5)
        elif index == 2:  # elbow flex: maximum
            position = high
        elif index == 4:  # wrist roll: one quarter from its minimum
            position = math.floor(low + 0.25 * (high - low) + 0.5)
        else:  # shoulder lift, wrist flex, and gripper: minimum
            position = low
        home[joint] = position
    return home


def _read_raw(robot, register, motor):
    return int(robot.bus.read(register, motor, normalize=False, num_retry=2))


def _read_limits(robot):
    limits = {}
    for joint, motor in zip(JOINTS, MOTORS, strict=True):
        low = _read_raw(robot, "Min_Position_Limit", motor)
        high = _read_raw(robot, "Max_Position_Limit", motor)
        if low > high:
            low, high = high, low
        if (
            low < 0
            or high > MAX_RAW_POSITION
            or high - low <= MIN_USABLE_RANGE
        ):
            raise RuntimeError(
                f"invalid raw servo limits for {joint}: min={low}, max={high}"
            )
        limits[joint] = {"min": low, "max": high}
    return limits


def _read_present_positions(robot):
    return {
        joint: _read_raw(robot, "Present_Position", motor)
        for joint, motor in zip(JOINTS, MOTORS, strict=True)
    }


def _write_limits_record(limits):
    LIMITS_FILE.parent.mkdir(mode=0o750, parents=True, exist_ok=True)
    record = {
        "version": 1,
        "arm_id": FOLLOWER_ID,
        "units": "raw_servo_ticks",
        "read_at": time.time(),
        "limits": limits,
    }
    temporary = LIMITS_FILE.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    temporary.chmod(0o600)
    temporary.replace(LIMITS_FILE)
    return record


def read_limits_and_pose(port, arm_id):
    if arm_id != FOLLOWER_ID:
        raise RuntimeError(f"servo-limit tools only support {FOLLOWER_ID}")
    robot = make_follower(port, arm_id)
    bus = robot.bus
    connected = False
    try:
        # Read the same raw registers as the SDK without changing torque state.
        bus.connect()
        connected = True
        limits = _read_limits(robot)
        current = _read_present_positions(robot)
        return {
            "record": _write_limits_record(limits),
            "current": current,
        }
    finally:
        if connected:
            bus.disconnect(disable_torque=False)


def move_to_home(port, arm_id, planned_limits, planned_current, home, speed_percent, cancelled, report):
    if arm_id != FOLLOWER_ID:
        raise RuntimeError(f"go-home only supports {FOLLOWER_ID}")
    if not 1 <= speed_percent <= 100:
        raise ValueError("speed percent must be from 1 through 100")

    robot = make_follower(port, arm_id)
    connected = False
    speeds_changed = False
    try:
        robot.config.disable_torque_on_disconnect = False
        robot.connect(calibrate=False)
        connected = True
        limits = _read_limits(robot)
        if limits != planned_limits:
            raise RuntimeError("servo limits changed since the home preview; request a new preview")
        target = calculate_home_positions(limits)
        if target != home:
            raise RuntimeError("home target changed since the preview; request a new preview")
        current = _read_present_positions(robot)
        moved_since_preview = [
            joint
            for joint in JOINTS
            if abs(current[joint] - planned_current[joint]) > PLAN_DRIFT_TOLERANCE
        ]
        if moved_since_preview:
            raise RuntimeError(
                "follower moved since the home preview; request a new preview for "
                + ", ".join(moved_since_preview)
            )
        if cancelled.is_set():
            return {"state": "cancelled", "present": current, "target": target}

        speeds_changed = True
        for motor in MOTORS:
            robot.bus.write("Goal_Velocity", motor, HOME_SPEED, normalize=False, num_retry=2)
        if cancelled.is_set():
            return {"state": "cancelled", "present": current, "target": target}
        for joint, motor in zip(JOINTS, MOTORS, strict=True):
            robot.bus.write("Goal_Position", motor, target[joint], normalize=False, num_retry=2)
        report({"type": "home_progress", "state": "moving", "target": target})

        deadline = time.monotonic() + HOME_TIMEOUT_S
        stable_samples = 0
        last_report = 0.0
        while time.monotonic() < deadline:
            present = _read_present_positions(robot)
            if cancelled.is_set():
                for joint, motor in zip(JOINTS, MOTORS, strict=True):
                    robot.bus.write(
                        "Goal_Position", motor, present[joint], normalize=False, num_retry=1
                    )
                return {"state": "cancelled", "present": present, "target": target}

            remaining = {
                joint: target[joint] - present[joint]
                for joint in JOINTS
            }
            if all(abs(value) <= HOME_POSITION_TOLERANCE for value in remaining.values()):
                stable_samples += 1
                if stable_samples >= 3:
                    return {"state": "succeeded", "present": present, "target": target}
            else:
                stable_samples = 0

            now = time.monotonic()
            if now - last_report >= 1:
                report(
                    {
                        "type": "home_progress",
                        "state": "moving",
                        "present": present,
                        "target": target,
                    }
                )
                last_report = now
            time.sleep(0.1)

        present = _read_present_positions(robot)
        residuals = {
            joint: target[joint] - present[joint]
            for joint in JOINTS
            if abs(target[joint] - present[joint]) > HOME_POSITION_TOLERANCE
        }
        raise TimeoutError(f"follower did not reach the home pose; raw-tick residuals: {residuals}")
    finally:
        if connected:
            if speeds_changed:
                for motor in MOTORS:
                    try:
                        robot.bus.write(
                            "Goal_Velocity",
                            motor,
                            max(1, math.floor((speed_percent / 100) * 1023 + 0.5)),
                            normalize=False,
                            num_retry=1,
                        )
                    except Exception:
                        LOG.exception("Could not restore servo speed for %s", motor)
            try:
                robot.disconnect()
            except Exception:
                LOG.exception("Could not disconnect follower after go-home")
