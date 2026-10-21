"""Calibrated direct-bus teleoperation for full-arm and joint-isolated sessions."""

from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.protocol import GRIPPER_JOINTS, JOINTS, validate_action
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits
from cf_robot.teleop_motion import BoundedMotion, joint_label


class TeleopLeader:
    def __init__(self, device, joints=JOINTS):
        self.device = device
        self.bus = device.bus
        self.joints = joints
        self.is_connected = False
        self.saved_records = {}
        self.raw_limits = {}
        self.unit_limits = {}
        self.side = "leader"

    def connect(self, calibrate=False):
        if calibrate:
            raise ValueError("teleoperation does not calibrate interactively")
        for joint in self.joints:
            name = joint.removesuffix(".pos")
            motor = self.bus.motors[name]
            if motor.id != JOINTS.index(joint) + 1 or motor.model != "sts3215":
                raise ValueError(f"{joint_label(joint)} motor identity is invalid")
        self.is_connected = True
        self.bus.connect()
        report = inspect_calibration(
            self.device, self.side, read_servo_limits(self.bus, strict=False),
        )
        warnings = list(report["warnings"])
        for joint in self.joints:
            item = report["joints"][joint]
            warnings.extend(f"{joint_label(joint)}: {warning}" for warning in item["warnings"])
            self.saved_records[joint] = item["saved"]
            if item.get("working_range"):
                self.raw_limits[joint] = item["working_range"]
            elif item["saved"]:
                self.raw_limits[joint] = {
                    "min": item["saved"]["range_min"], "max": item["saved"]["range_max"],
                }
            if item.get("converted_range") is not None:
                self.unit_limits[joint] = sorted(item["converted_range"])
        if warnings:
            raise ValueError("selected-joint calibration must be consistent: " + "; ".join(warnings))
        if self.side == "leader" or self.joints == GRIPPER_JOINTS:
            for joint in self.joints:
                if self.bus.read(
                    "Torque_Enable", joint.removesuffix(".pos"), normalize=False, num_retry=2,
                ):
                    raise ValueError(f"{joint_label(joint)} torque must already be OFF")
        self.get_action()

    def raw_positions(self):
        if len(self.joints) == 1:
            name = self.joints[0].removesuffix(".pos")
            raw = {name: self.bus.read("Present_Position", name, normalize=False, num_retry=2)}
        else:
            raw = self.bus.sync_read("Present_Position", normalize=False, num_retry=2)
        result = {}
        for joint in self.joints:
            value = int(raw[joint.removesuffix(".pos")])
            limits = self.raw_limits[joint]
            if not limits["min"] <= value <= limits["max"]:
                raise ValueError(
                    f"{joint_label(joint)} is outside its saved range; inspect before moving"
                )
            result[joint] = value
        return result

    def get_action(self):
        return self.convert_positions(self.raw_positions())

    def convert_positions(self, raw):
        ids = {joint: self.bus.motors[joint.removesuffix(".pos")].id for joint in self.joints}
        converted = self.bus._normalize({ids[joint]: raw[joint] for joint in self.joints})
        values = validate_action({joint: converted[ids[joint]] for joint in self.joints}, self.joints)
        if "gripper.pos" in values and not 0 <= values["gripper.pos"] <= 100:
            raise ValueError("gripper value must be between 0 and 100 percent")
        return values

    def disconnect(self):
        if self.is_connected:
            try:
                self.bus.disconnect(disable_torque=False)
            finally:
                self.is_connected = False


class TeleopFollower(TeleopLeader):
    bounded_following = True

    def __init__(self, device, joints=JOINTS, clock=None):
        super().__init__(device, joints)
        self.side = "follower"
        self.changed = False
        self.previous_speeds = {}
        self.speeds = {}
        self.motion = BoundedMotion(joints, clock)

    def get_observation(self):
        return self.get_action()

    def hold_position(self):
        raw = self.raw_positions()
        measured = self.convert_positions(raw)
        if self.changed:
            for joint, value in raw.items():
                self.bus.write(
                    "Goal_Position", joint.removesuffix(".pos"), value,
                    normalize=False, num_retry=0,
                )
            self.motion.committed(measured)
        return measured

    def send_action(self, action):
        action = validate_action(action, self.joints)
        for joint, value in action.items():
            low, high = self.unit_limits[joint]
            if not low <= value <= high:
                raise ValueError(
                    f"{joint_label(joint)} target {value:.2f} is outside saved range {low:.2f}..{high:.2f}"
                )
        measured = self.get_action()
        try:
            applied = self.motion.plan(action, measured)
        except RuntimeError as exc:
            raise RuntimeError(f"{exc} Speed registers={self.speeds}") from exc
        if not self.changed:
            raw = self.raw_positions()
            for joint in self.joints:
                name = joint.removesuffix(".pos")
                speed = int(self.bus.read("Goal_Velocity", name, normalize=False, num_retry=2))
                if not 0 <= speed <= 1023:
                    raise ValueError(f"{joint_label(joint)} has an unexpected speed register")
                self.previous_speeds[name] = speed
            if self.joints == GRIPPER_JOINTS and self.bus.read(
                "Torque_Enable", "gripper", normalize=False, num_retry=2,
            ):
                raise ValueError(f"{joint_label(self.joints[0])} torque changed before arming")
            self.changed = True
            # Seed every selected goal before enabling any selected motor.
            for joint in self.joints:
                self.bus.write(
                    "Goal_Position", joint.removesuffix(".pos"), raw[joint],
                    normalize=False, num_retry=0,
                )
            for joint in self.joints:
                name = joint.removesuffix(".pos")
                self.speeds[name] = min(self.previous_speeds[name] or HOME_SPEED, HOME_SPEED)
                self.bus.write("Goal_Velocity", name, self.speeds[name], normalize=False, num_retry=0)
                self.bus.write("Torque_Enable", name, 1, normalize=False, num_retry=0)
        for joint, value in applied.items():
            self.bus.write("Goal_Position", joint.removesuffix(".pos"), value, num_retry=0)
        self.motion.committed(applied)

    def disconnect(self):
        errors = []
        safe_speed_restore = set()
        try:
            if self.is_connected and self.changed:
                if self.joints == GRIPPER_JOINTS:
                    try:
                        self.bus.write("Torque_Enable", "gripper", 0, normalize=False, num_retry=2)
                        safe_speed_restore.add("gripper")
                    except Exception as exc:  # noqa: BLE001 - attempt remaining cleanup.
                        errors.append(str(exc))
                else:
                    # A full arm may be supporting a load: hold, do not drop it.
                    for joint in self.joints:
                        name = joint.removesuffix(".pos")
                        try:
                            raw = int(self.bus.read("Present_Position", name, normalize=False, num_retry=2))
                            limits = self.raw_limits[joint]
                            if not limits["min"] <= raw <= limits["max"]:
                                raise ValueError(f"{name} cannot safely hold outside saved limits")
                            self.bus.write("Goal_Position", name, raw, normalize=False, num_retry=2)
                            safe_speed_restore.add(name)
                        except Exception as exc:  # noqa: BLE001 - leave last bounded goal, never release.
                            errors.append(str(exc))
                for name, speed in self.previous_speeds.items():
                    if name not in safe_speed_restore:
                        continue
                    try:
                        self.bus.write("Goal_Velocity", name, speed, normalize=False, num_retry=2)
                    except Exception as exc:  # noqa: BLE001 - preserve disconnect cleanup.
                        errors.append(str(exc))
        finally:
            super().disconnect()
        if errors:
            raise RuntimeError("teleoperation cleanup incomplete: " + "; ".join(errors))
