"""LeRobot pose sampling and bounded, explicit follower servo commands."""

import math
import queue
import time

from cf_robot.hardware import make_follower, make_leader, joint_positions
from cf_robot.protocol import JOINTS
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits

POSE_MONITOR_INTERVAL_S = 0.1
MAX_COMMAND_STEP_UNITS = 2.5
COMMAND_SETTLE_TOLERANCE = 1.0
COMMAND_SETTLE_TIMEOUT_S = 5.0
MOTORS = tuple(joint.removesuffix(".pos") for joint in JOINTS)


def _sample(device, side, limits):
    bus = device.bus
    if side == "leader":
        # This is the same normalized action value used by leader teleoperation.
        values = joint_positions(device.get_action())
    else:
        observed = bus.sync_read("Present_Position", num_retry=2)
        values = joint_positions(
            {f"{motor}.pos": value for motor, value in observed.items()}
        )

    raw = bus.sync_read("Present_Position", normalize=False, num_retry=2)
    torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
    return {
        "type": "pose_sample",
        "side": side,
        "values": values,
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


def report_bus_torque(bus, side, publish):
    """Report torque state without changing it."""
    state = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
    publish(
        {
            "type": f"{side}_torque",
            "before": {
                joint: bool(state[joint.removesuffix(".pos")])
                for joint in JOINTS
            },
            "after": {
                joint: bool(state[joint.removesuffix(".pos")])
                for joint in JOINTS
            },
        }
    )


def validate_pose_command(message):
    if not isinstance(message, dict) or message.get("type") != "command":
        raise ValueError("unsupported monitor command")
    if message.get("version") != 1:
        raise ValueError("unsupported monitor command version")
    if (
        not isinstance(message.get("command_id"), int)
        or isinstance(message.get("command_id"), bool)
        or message["command_id"] < 1
    ):
        raise ValueError("command_id must be a positive integer")
    joint_id = message.get("joint_id")
    mode = message.get("mode")
    if mode == "torque":
        if (
            set(message) != {"version", "type", "command_id", "joint_id", "mode", "enabled"}
            or not isinstance(message.get("enabled"), bool)
        ):
            raise ValueError("torque command requires enabled=true or false")
        if joint_id != "*" and (
            not isinstance(joint_id, int)
            or isinstance(joint_id, bool)
            or not 1 <= joint_id <= 6
        ):
            raise ValueError("torque command joint_id must be J1-J6 or *")
        return dict(message)

    if not isinstance(joint_id, int) or isinstance(joint_id, bool) or not 1 <= joint_id <= 6:
        raise ValueError("joint_id must be from 1 through 6")
    if (
        set(message) != {"version", "type", "command_id", "joint_id", "mode", "value"}
        or mode not in ("ticks", "degrees", "percent")
        or not isinstance(message.get("value"), (int, float))
        or isinstance(message.get("value"), bool)
        or not math.isfinite(message["value"])
    ):
        raise ValueError("position command must contain a finite value and valid mode")
    if mode == "ticks":
        if not isinstance(message["value"], int) or not 0 <= message["value"] <= 4095:
            raise ValueError("tick targets must be integers from 0 through 4095")
    elif mode == "degrees" and joint_id == 6:
        raise ValueError("J6 is the gripper; use percent or raw ticks")
    elif mode == "percent" and joint_id != 6:
        raise ValueError("percent targets are valid only for J6 gripper")
    elif mode == "percent" and not 0 <= message["value"] <= 100:
        raise ValueError("gripper percent must be from 0 through 100")
    return dict(message)


def _publish_command_result(report, command, state, **details):
    joint_id = command["joint_id"]
    if isinstance(joint_id, int):
        details.setdefault(
            "joint_name",
            JOINTS[joint_id - 1].removesuffix(".pos").replace("_", " ").title(),
        )
    report(
        {
            "type": "command_result",
            "command_id": command["command_id"],
            "joint_id": joint_id,
            "state": state,
            **details,
        }
    )


def execute_pose_command(device, limits, command, stop, report):
    """Execute one validated command on one follower joint only."""
    bus = device.bus
    if command["mode"] == "torque":
        joint_ids = range(1, 7) if command["joint_id"] == "*" else (command["joint_id"],)
        joint_results = []
        for joint_id in joint_ids:
            joint = JOINTS[joint_id - 1]
            motor = MOTORS[joint_id - 1]
            try:
                torque_enabled = bool(
                    bus.read("Torque_Enable", motor, normalize=False, num_retry=2)
                )
                if command["enabled"] and not torque_enabled:
                    present = int(
                        bus.read("Present_Position", motor, normalize=False, num_retry=2)
                    )
                    bus.write("Goal_Position", motor, present, normalize=False, num_retry=2)
                    bus.write("Torque_Enable", motor, 1, normalize=False, num_retry=2)
                elif not command["enabled"] and torque_enabled:
                    bus.write("Torque_Enable", motor, 0, normalize=False, num_retry=2)
                torque_enabled = bool(
                    bus.read("Torque_Enable", motor, normalize=False, num_retry=2)
                )
                joint_results.append(
                    {
                        "joint_id": joint_id,
                        "joint_name": joint.removesuffix(".pos").replace("_", " ").title(),
                        "torque_enabled": torque_enabled,
                        "message": "torque enabled" if torque_enabled else "torque released",
                    }
                )
            except Exception as exc:
                joint_results.append(
                    {
                        "joint_id": joint_id,
                        "joint_name": joint.removesuffix(".pos").replace("_", " ").title(),
                        "message": f"failed: {exc}",
                    }
                )
                break
        succeeded = all(not result["message"].startswith("failed:") for result in joint_results)
        details = "; ".join(
            f"{result['joint_name']} (J{result['joint_id']}): {result['message']}"
            for result in joint_results
        )
        _publish_command_result(
            report,
            command,
            "succeeded" if succeeded else "partial",
            message=details,
            joint_results=joint_results,
        )
        return

    joint = JOINTS[command["joint_id"] - 1]
    motor = MOTORS[command["joint_id"] - 1]
    torque_enabled = bool(bus.read("Torque_Enable", motor, normalize=False, num_retry=2))
    if not torque_enabled:
        _publish_command_result(
            report,
            command,
            "rejected",
            message=f"{joint} torque is off; use J{command['joint_id']} TE before moving it",
        )
        return

    motor_id = bus.motors[motor].id
    if command["mode"] == "ticks":
        target_raw = int(command["value"])
    else:
        target_raw = int(
            bus._unnormalize({motor_id: float(command["value"])})[motor_id]
        )
    limit = limits[joint]
    if not limit["min"] <= target_raw <= limit["max"]:
        _publish_command_result(
            report,
            command,
            "rejected",
            message=(
                f"{joint} target {target_raw} ticks is outside configured "
                f"limits {limit['min']}..{limit['max']}"
            ),
        )
        return

    target_units = float(bus._normalize({motor_id: target_raw})[motor_id])
    unit = "percent" if command["joint_id"] == 6 else "degrees"
    current_raw = int(bus.read("Present_Position", motor, normalize=False, num_retry=2))
    current_units = float(bus.read("Present_Position", motor, normalize=True, num_retry=2))
    delta = target_units - current_units
    steps = max(1, math.ceil(abs(delta) / MAX_COMMAND_STEP_UNITS))
    previous_speed = int(bus.read("Goal_Velocity", motor, normalize=False, num_retry=2))
    report(
        {
            "type": "command_progress",
            "command_id": command["command_id"],
            "joint_id": command["joint_id"],
            "state": "moving",
            "current": current_units,
            "target": target_units,
            "unit": unit,
            "steps": steps,
        }
    )
    try:
        bus.write("Goal_Velocity", motor, HOME_SPEED, normalize=False, num_retry=2)
        for index in range(1, steps + 1):
            if stop.is_set():
                present_raw = int(
                    bus.read("Present_Position", motor, normalize=False, num_retry=2)
                )
                bus.write(
                    "Goal_Position", motor, present_raw, normalize=False, num_retry=1
                )
                _publish_command_result(
                    report,
                    command,
                    "cancelled",
                    current=float(bus.read("Present_Position", motor, normalize=True, num_retry=2)),
                    target=target_units,
                )
                return
            step_raw = math.floor(current_raw + (target_raw - current_raw) * index / steps + 0.5)
            bus.write("Goal_Position", motor, step_raw, normalize=False, num_retry=2)
            deadline = time.monotonic() + COMMAND_SETTLE_TIMEOUT_S
            while time.monotonic() < deadline:
                present_raw = int(
                    bus.read("Present_Position", motor, normalize=False, num_retry=2)
                )
                present_units = float(
                    bus.read("Present_Position", motor, normalize=True, num_retry=2)
                )
                if stop.is_set():
                    bus.write(
                        "Goal_Position", motor, present_raw, normalize=False, num_retry=1
                    )
                    _publish_command_result(
                        report,
                        command,
                        "cancelled",
                        current=present_units,
                        target=target_units,
                    )
                    return
                step_units = float(bus._normalize({motor_id: step_raw})[motor_id])
                if abs(present_units - step_units) <= COMMAND_SETTLE_TOLERANCE:
                    break
                time.sleep(0.05)
            else:
                _publish_command_result(
                    report,
                    command,
                    "failed",
                    message=f"{joint} did not reach an intermediate target",
                    current=present_units,
                    target=target_units,
                )
                return
            report(_sample(device, "follower", limits))
            report(
                {
                    "type": "command_progress",
                    "command_id": command["command_id"],
                    "joint_id": command["joint_id"],
                    "state": "moving",
                    "current": present_units,
                    "target": target_units,
                    "unit": unit,
                    "step": index,
                    "steps": steps,
                }
            )

        present_units = float(bus.read("Present_Position", motor, normalize=True, num_retry=2))
        state = "cancelled" if stop.is_set() else "succeeded"
        _publish_command_result(
            report,
            command,
            state,
            message="position reached" if state == "succeeded" else "positioning cancelled",
            current=present_units,
            target=target_units,
        )
    finally:
        bus.write("Goal_Velocity", motor, previous_speed, normalize=False, num_retry=1)


def stream_leader_pose(port, arm_id, stop, publish):
    """Sample the leader bus without configuring or changing torque."""
    leader = make_leader(port, arm_id)
    bus = leader.bus
    connected = False
    try:
        bus.connect()
        connected = True
        limits = read_servo_limits(bus)
        report_bus_torque(bus, "leader", publish)
        while not stop.is_set():
            started = time.monotonic()
            publish(_sample(leader, "leader", limits))
            remaining = POSE_MONITOR_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    finally:
        if connected:
            bus.disconnect(disable_torque=False)


def stream_follower_pose(
    port,
    arm_id,
    stop,
    publish,
    commands=None,
    command_busy=None,
):
    """Read follower positions and process commands without startup torque changes."""
    follower = make_follower(port, arm_id)
    bus = follower.bus
    connected = False
    try:
        bus.connect()
        connected = True
        limits = read_servo_limits(bus)
        report_bus_torque(bus, "follower", publish)
        while not stop.is_set():
            if commands is not None:
                try:
                    command = commands.get_nowait()
                except queue.Empty:
                    command = None
                if command is not None:
                    if command_busy is not None:
                        command_busy.set()
                    try:
                        execute_pose_command(
                            follower,
                            limits,
                            command,
                            stop,
                            publish,
                        )
                    except Exception as exc:
                        _publish_command_result(
                            publish,
                            command,
                            "failed",
                            message=str(exc),
                        )
                    finally:
                        if command_busy is not None:
                            command_busy.clear()
            started = time.monotonic()
            publish(_sample(follower, "follower", limits))
            remaining = POSE_MONITOR_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    finally:
        if connected:
            bus.disconnect(disable_torque=False)
