"""LeRobot pose sampling and bounded, explicit follower servo commands."""

import math
import queue
import time

from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.calibration_restore import GripperCalibrationRestore
from cf_robot.angular_install import AngularInstall
from cf_robot.encoder_rebase import EncoderRebase
from cf_robot.hardware import make_follower, make_leader
from cf_robot.protocol import JOINTS
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits

POSE_MONITOR_INTERVAL_S = 0.1
MAX_COMMAND_STEP_UNITS = 2.5
COMMAND_SETTLE_TOLERANCE = 1.0
COMMAND_CYCLE_S = 0.05
COMMAND_REPORT_INTERVAL_S = 0.25
# Moves end on lack of measured progress, not on elapsed travel time.
COMMAND_NO_PROGRESS_TIMEOUT_S = 3.0
COMMAND_MIN_PROGRESS_TICKS = 3
COMMAND_MAX_DURATION_S = 120.0
STALL_DIAGNOSTIC_REGISTERS = (
    "Present_Load", "Present_Current", "Present_Voltage", "Present_Temperature",
    "Status", "Moving", "Torque_Limit", "Goal_Velocity", "Acceleration",
)
MOTORS = tuple(joint.removesuffix(".pos") for joint in JOINTS)
DIAGNOSTIC_REGISTERS = (
    "Present_Position", "Goal_Position", "Goal_Position_2",
    "Torque_Enable", "Goal_Velocity", "Present_Velocity", "Acceleration",
    "Present_Load", "Present_Current", "Present_Voltage", "Present_Temperature",
    "Status", "Moving", "Torque_Limit", "Max_Torque_Limit", "Protective_Torque",
    "P_Coefficient", "I_Coefficient", "D_Coefficient",
    "Minimum_Startup_Force", "CW_Dead_Zone", "CCW_Dead_Zone",
    "Operating_Mode", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
    "Min_Voltage_Limit", "Max_Voltage_Limit", "Max_Temperature_Limit",
    "Unloading_Condition", "Overload_Torque", "Protection_Current",
    "Protection_Time", "Over_Current_Protection_Time",
)


def _load_fields(value):
    if type(value) is int:
        return {
            "present_load_raw": value,
            "present_load_percent": round(value / 10, 1),
        }
    return {
        "present_load_raw": None, "present_load_percent": None,
        "present_load_error": str(value),
    }


def read_present_load(bus, motor):
    """Read the SDK-decoded signed load; this is not a calibrated torque sensor."""
    try:
        value = int(bus.read("Present_Load", motor, normalize=False, num_retry=0))
    except Exception as exc:
        value = f"unavailable: {exc}"
    return _load_fields(value)


def read_servo_diagnostics(device, joint_id):
    """Read a fixed STS3215 register snapshot without writes or normalization."""
    if type(joint_id) is not int or not 1 <= joint_id <= 6:
        raise ValueError("select one follower joint J1-J6")
    motor = MOTORS[joint_id - 1]
    configured = device.bus.motors[motor]
    if configured.id != joint_id or configured.model != "sts3215":
        raise ValueError("diagnostics require the configured STS3215 joint identity")
    values, errors = {}, {}
    for register in DIAGNOSTIC_REGISTERS:
        try:
            values[register] = int(device.bus.read(
                register, motor, normalize=False, num_retry=0,
            ))
        except Exception as exc:  # SDK/firmware-specific failures stay explicit.
            errors[register] = str(exc)
    return {
        "side": "follower", "joint_id": joint_id, "motor": motor,
        "sampled_at": time.time(), "units": "raw register values",
        "registers": values, "errors": errors,
    }


def _sample(device, side, limits, calibration=None, previous=None):
    bus = device.bus
    raw = bus.sync_read("Present_Position", normalize=False, num_retry=2)
    torque = bus.sync_read("Torque_Enable", normalize=False, num_retry=2)
    values, errors = {}, {}
    for joint, motor in zip(JOINTS, MOTORS, strict=True):
        try:
            # Use the SDK conversion on this very encoder sample, not a second read.
            motor_id = bus.motors[motor].id
            value = float(bus._normalize({motor_id: raw[motor]})[motor_id])
            if not math.isfinite(value):
                raise ValueError("non-finite normalized position")
            values[joint] = value
        except (KeyError, ValueError, RuntimeError) as exc:
            values[joint] = None
            errors[joint] = str(exc)
    warnings = []
    for joint, motor in zip(JOINTS, MOTORS, strict=True):
        item = (calibration or {}).get("joints", {}).get(joint, {})
        saved = item.get("saved")
        working = item.get("working_range")
        if working and not working["min"] <= raw[motor] <= working["max"]:
            warnings.append(f"{motor}: outside angular calibration working range.")
        if (
            saved and all(type(saved.get(key)) is int for key in ("range_min", "range_max"))
            and not saved["range_min"] <= raw[motor] <= saved["range_max"]
        ):
            warnings.append(f"{motor}: outside saved range; range normalization may clip.")
        resolution = item.get("resolution")
        if previous and resolution and abs(raw[motor] - previous["raw"][joint]) > resolution / 2:
            warnings.append(f"{motor}: possible encoder wrap/discontinuity or rapid movement.")
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
        "calibration": calibration,
        "conversion_errors": errors,
        "motion_warnings": warnings,
        "sampled_at": time.time(),
    }


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
    if mode == "diagnostics":
        if (
            type(joint_id) is not int or not 1 <= joint_id <= 6
            or set(message) != {"version", "type", "command_id", "joint_id", "mode"}
        ):
            raise ValueError("read-only diagnostics require exactly one follower joint J1-J6")
        return dict(message)
    if mode in ("rebase_preview", "rebase_apply"):
        if type(joint_id) is not int or not 1 <= joint_id <= 5:
            raise ValueError("encoder rebasing supports follower J1-J5 only")
        keys = {"version", "type", "command_id", "joint_id", "mode"}
        if mode == "rebase_preview":
            fields = {"old_start", "new_start", "expected_offset"}
            endpoint = {"old_end"} if "old_end" in message else {"provisional_max"}
            if (
                set(message) != keys | fields | endpoint
                or any(type(message.get(key)) is not int for key in fields | endpoint)
                or not all(0 <= message[key] <= 4095 for key in (fields | endpoint) - {"expected_offset"})
                or not -2047 <= message["expected_offset"] <= 2047
            ):
                raise ValueError("rebase preview requires integer old/new start, expected offset and old end or provisional maximum")
        elif (
            set(message) != keys | {"token", "confirmation"}
            or message.get("confirmation") != f"REBASE FOLLOWER J{joint_id}"
            or not isinstance(message.get("token"), str)
            or len(message["token"]) != 32
            or any(character not in "0123456789abcdef" for character in message["token"])
        ):
            raise ValueError("rebase apply requires a matching preview and explicit joint confirmation")
        return dict(message)
    if mode in ("angular_preview", "angular_apply"):
        if type(joint_id) is not int or not 1 <= joint_id <= 5:
            raise ValueError("angular profiles support follower J1-J5 only; J6 is unchanged")
        keys = {"version", "type", "command_id", "joint_id", "mode"}
        if mode == "angular_preview":
            fields = {"range_min", "range_max", "negative_ticks", "positive_ticks"}
            if (
                set(message) != keys | fields | {"write_hardware_limits"}
                or message.get("write_hardware_limits") is not True
                or any(type(message.get(key)) is not int for key in fields)
            ):
                raise ValueError(
                    "angular preview requires integer MIN/MAX/-90/+90 references and "
                    "write_hardware_limits=true; use the updated client"
                )
        else:
            token = message.get("token")
            if (
                set(message) != keys | {"token", "confirmation"}
                or message.get("confirmation") != f"APPLY ANGULAR AND LIMITS J{joint_id}"
                or not isinstance(token, str) or len(token) != 32
                or any(character not in "0123456789abcdef" for character in token)
            ):
                raise ValueError("angular apply requires a matching preview and explicit joint confirmation")
        return dict(message)
    if mode in ("calibration_preview", "calibration_restore"):
        if type(joint_id) is not int or joint_id != 6:
            raise ValueError("targeted calibration restore supports follower J6 only")
        keys = {"version", "type", "command_id", "joint_id", "mode"}
        if mode == "calibration_preview":
            if set(message) != keys:
                raise ValueError("J6 preview takes no register values")
        else:
            token = message.get("token")
            if (
                set(message) != keys | {"token", "confirmation"}
                or message.get("confirmation") != "RESTORE J6"
                or not isinstance(token, str) or len(token) != 32
                or any(character not in "0123456789abcdef" for character in token)
            ):
                raise ValueError("confirm RESTORE J6 with the token from a fresh preview")
        return dict(message)
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
            "mode": command["mode"],
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
    probe_raw = target_raw + (100 if target_raw + 100 <= limit["max"] else -100)
    units_per_tick = abs(float(bus._normalize({motor_id: probe_raw})[motor_id]) - target_units) / 100
    if not math.isfinite(units_per_tick) or units_per_tick <= 0:
        raise ValueError(f"follower {joint} conversion has no usable tick scale")
    # Bound each goal by physical units (degrees, or J6 percentage points).
    step_ticks = max(1, int(MAX_COMMAND_STEP_UNITS / units_per_tick))
    tolerance_ticks = max(2, round(COMMAND_SETTLE_TOLERANCE / units_per_tick))
    steps = max(1, math.ceil(abs(target_raw - current_raw) / step_ticks))
    previous_speed = int(bus.read("Goal_Velocity", motor, normalize=False, num_retry=2))
    if not 0 <= previous_speed <= 32767:
        raise ValueError(f"follower {joint} has an invalid speed register")
    # Zero is unrestricted speed. Retain the cap even after accepting a goal
    # within tolerance, while the servo may still be physically settling.
    movement_speed = min(previous_speed or HOME_SPEED, HOME_SPEED)
    speed_capped = False
    hold_attempted = False

    def hold_current():
        nonlocal hold_attempted
        hold_attempted = True
        try:
            raw = int(bus.read("Present_Position", motor, normalize=False, num_retry=2))
            if not limit["min"] <= raw <= limit["max"]:
                raise ValueError("measured position is outside verified limits")
            bus.write("Goal_Position", motor, raw, normalize=False, num_retry=0)
            return raw
        except Exception as exc:
            raise RuntimeError(
                f"follower {joint} safe hold failed: {exc}; keep the arm supported "
                "and use the physical stop if necessary. Torque was not released."
            ) from exc

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
            **read_present_load(bus, motor),
        }
    )
    try:
        bus.write("Goal_Velocity", motor, movement_speed, normalize=False, num_retry=2)
        speed_capped = True

        def units(raw):
            return float(bus._normalize({motor_id: raw})[motor_id])

        def cancelled():
            held_raw = hold_current()
            _publish_command_result(
                report, command, "cancelled", message="positioning cancelled",
                current=units(held_raw), target=target_units, held_raw=held_raw,
                **read_present_load(bus, motor),
            )

        started = progress_at = last_report = time.monotonic()
        best_remaining = abs(target_raw - current_raw)
        goal_raw = None
        while True:
            if stop.is_set():
                cancelled()
                return
            present_raw = int(bus.read("Present_Position", motor, normalize=False, num_retry=2))
            remaining = target_raw - present_raw
            if goal_raw == target_raw and abs(remaining) <= tolerance_ticks:
                break
            now = time.monotonic()
            if abs(remaining) <= best_remaining - COMMAND_MIN_PROGRESS_TICKS:
                best_remaining, progress_at = abs(remaining), now
            stalled = now - progress_at >= COMMAND_NO_PROGRESS_TIMEOUT_S
            if stalled or now - started >= COMMAND_MAX_DURATION_S:
                snapshot = {}
                for register in STALL_DIAGNOSTIC_REGISTERS:
                    try:
                        snapshot[register] = int(bus.read(register, motor, normalize=False, num_retry=0))
                    except Exception as exc:  # Firmware-specific registers stay explicit.
                        snapshot[register] = f"unavailable: {exc}"
                details = {
                    "current": units(present_raw), "target": target_units,
                    "actual_raw": present_raw, "step_target_raw": goal_raw,
                    "residual_ticks": remaining, "tolerance_ticks": tolerance_ticks,
                    "speed": movement_speed, "elapsed_s": round(now - started, 2),
                    "servo_snapshot": snapshot,
                    **_load_fields(snapshot.get("Present_Load", "unavailable")),
                }
                if goal_raw == target_raw and abs(remaining) <= step_ticks:
                    # Load/friction steady-state error: keep pushing toward the
                    # target, which is already within one bounded step.
                    _publish_command_result(
                        report, command, "settled_short",
                        message=(
                            f"follower {joint} stopped {abs(remaining)} ticks short of the target "
                            "(likely load or friction); target goal retained"
                        ),
                        **details,
                    )
                    return
                held_raw = hold_current()
                _publish_command_result(
                    report, command, "failed",
                    message=(
                        f"follower {joint} made no measured progress for "
                        f"{COMMAND_NO_PROGRESS_TIMEOUT_S:g}s; holding measured position. "
                        "Check servo_snapshot load/status/voltage and for obstruction."
                        if stalled else
                        f"follower {joint} exceeded {COMMAND_MAX_DURATION_S:g}s; holding measured position"
                    ),
                    held_raw=held_raw, **details,
                )
                return
            next_goal = present_raw + max(-step_ticks, min(step_ticks, remaining))
            if next_goal != goal_raw:
                bus.write("Goal_Position", motor, next_goal, normalize=False, num_retry=2)
                goal_raw = next_goal
            if now - last_report >= COMMAND_REPORT_INTERVAL_S:
                report(_sample(device, "follower", limits))
                report({
                    "type": "command_progress", "command_id": command["command_id"],
                    "joint_id": command["joint_id"], "state": "moving",
                    "current": units(present_raw), "target": target_units, "unit": unit,
                    "remaining_ticks": remaining, "elapsed_s": round(now - started, 2),
                    "steps": steps,
                    **read_present_load(bus, motor),
                })
                last_report = now
            time.sleep(COMMAND_CYCLE_S)

        report(_sample(device, "follower", limits))
        if stop.is_set():
            cancelled()
            return
        present_raw = int(bus.read("Present_Position", motor, normalize=False, num_retry=2))
        _publish_command_result(
            report, command, "succeeded", message="position reached",
            current=units(present_raw), target=target_units,
            actual_raw=present_raw, elapsed_s=round(time.monotonic() - started, 2),
            **read_present_load(bus, motor),
        )
    except Exception:
        if speed_capped and not hold_attempted:
            hold_current()
        raise


def stream_leader_pose(port, arm_id, stop, publish):
    """Sample the leader bus without configuring or changing torque."""
    leader = make_leader(port, arm_id)
    bus = leader.bus
    connected = False
    try:
        bus.connect()
        connected = True
        limits = read_servo_limits(bus, strict=False)
        calibration = inspect_calibration(leader, "leader", limits)
        previous = None
        while not stop.is_set():
            started = time.monotonic()
            sample = _sample(leader, "leader", limits, calibration, previous)
            publish(sample)
            previous = sample
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
        limits = read_servo_limits(bus, strict=False)
        calibration = inspect_calibration(follower, "follower", limits)
        restore = GripperCalibrationRestore()
        angular = AngularInstall()
        rebase = EncoderRebase("follower")
        previous = None
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
                        mode = command["mode"]
                        if mode not in ("rebase_preview", "rebase_apply"):
                            rebase.plan = None
                        if mode == "rebase_preview":
                            angular.plan = None
                            restore.plan = None
                            if restore.tainted:
                                raise ValueError("resolve the incomplete J6 restore before rebasing")
                            preview = rebase.preview(
                                follower, command["joint_id"], command["old_start"],
                                command.get("old_end"), command["new_start"],
                                command["expected_offset"],
                                provisional_max=command.get("provisional_max"),
                            )
                            _publish_command_result(
                                publish, command, "preview",
                                message="Read-only homing-offset/MIN/MAX rebase preview; zero stays pending.",
                                rebase_preview=preview,
                            )
                        elif mode == "rebase_apply":
                            angular.plan = None
                            restore.plan = None
                            try:
                                if not rebase.plan or rebase.plan["joint_id"] != command["joint_id"]:
                                    raise ValueError("no matching unused encoder-rebase preview")
                                result = rebase.apply(follower, command["token"], stop)
                                _publish_command_result(
                                    publish, command, "succeeded",
                                    message="Encoder rebase verified; torque stays OFF. Restart for raw measurements.",
                                    encoder_rebase=result,
                                )
                            finally:
                                stop.set()
                        elif mode == "angular_preview":
                            restore.plan = None
                            if restore.tainted:
                                raise ValueError("resolve the incomplete J6 restore before installing angular files")
                            preview = angular.preview(follower, command)
                            _publish_command_result(
                                publish, command, "preview",
                                message="Read-only preview of MIN/MAX EEPROM writes and matching files; review before confirming.",
                                angular_preview=preview,
                            )
                        elif mode == "angular_apply":
                            restore.plan = None
                            try:
                                result = angular.apply(follower, command, stop)
                                _publish_command_result(
                                    publish, command, "succeeded",
                                    message="Selected hardware MIN/MAX and matching files saved/verified; homing and other joints unchanged. Restart monitoring.",
                                    angular_install=result,
                                )
                            finally:
                                stop.set()  # Never continue with the old in-memory calibration.
                        elif mode == "calibration_preview":
                            angular.plan = None
                            preview = restore.preview(follower)
                            _publish_command_result(
                                publish, command, "preview",
                                message="Read-only J6 preview. Review before confirming; expires in 60 seconds.",
                                calibration_preview=preview,
                            )
                        elif mode == "calibration_restore":
                            angular.plan = None
                            result = restore.apply(follower, command["token"], stop)
                            _publish_command_result(
                                publish, command, "succeeded",
                                message="J6 saved calibration restored and verified. Torque remains OFF; J1-J5 unchanged.",
                                calibration_restore=result,
                            )
                        elif mode == "diagnostics":
                            angular.plan = None
                            restore.plan = None
                            snapshot = read_servo_diagnostics(follower, command["joint_id"])
                            _publish_command_result(
                                publish, command, "succeeded",
                                message="Read-only follower servo snapshot; unavailable registers appear in errors.",
                                servo_diagnostics=snapshot,
                            )
                        else:
                            angular.plan = None
                            restore.plan = None
                            if restore.tainted and (
                                mode != "torque" or command["enabled"]
                            ):
                                raise ValueError("J6 restore incomplete; only torque OFF and restore commands are allowed")
                            if mode in ("ticks", "degrees", "percent") and calibration["state"] != "consistent":
                                raise ValueError("resolve calibration inspector warnings before position commands")
                            execute_pose_command(
                                follower, limits, command, stop, publish,
                            )
                    except Exception as exc:
                        _publish_command_result(
                            publish,
                            command,
                            "failed",
                            message=str(exc),
                        )
                    finally:
                        if command["mode"] in ("calibration_preview", "calibration_restore"):
                            limits = read_servo_limits(bus, strict=False)
                            calibration = inspect_calibration(follower, "follower", limits)
                            if restore.tainted:
                                calibration["state"] = "warning"
                                calibration["warnings"].append("J6 restore incomplete; inspect before further operation.")
                            previous = None
                        if command_busy is not None:
                            command_busy.clear()
                    if stop.is_set():
                        break
            started = time.monotonic()
            sample = _sample(follower, "follower", limits, calibration, previous)
            publish(sample)
            previous = sample
            remaining = POSE_MONITOR_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    finally:
        if connected:
            bus.disconnect(disable_torque=False)
