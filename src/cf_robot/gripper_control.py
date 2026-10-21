"""Isolated J6 adapters: never configure or command the other five joints."""

from urllib.parse import urlsplit, urlunsplit

from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.protocol import GRIPPER_JOINTS, validate_action
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits

MOTOR = "gripper"
JOINT = "gripper.pos"
MAX_TRACKING_ERROR_PERCENT = 5


def gripper_control_url(url):
    parsed = urlsplit(url)
    if parsed.path not in ("", "/", "/control"):
        raise ValueError("pass the normal /control URL with --J6-only")
    return urlunsplit((parsed.scheme, parsed.netloc, "/control/gripper", "", ""))


class GripperLeader:
    """Read J6 directly without the SDK's whole-arm configure/disconnect path."""

    def __init__(self, device):
        self.device = device
        self.bus = device.bus
        self.is_connected = False
        self.saved = None
        self.side = "leader"

    def connect(self, calibrate=False):
        if calibrate:
            raise ValueError("gripper-only mode does not calibrate")
        motor = self.bus.motors[MOTOR]
        if motor.id != 6 or motor.model != "sts3215":
            raise ValueError("gripper-only mode requires the STS3215 gripper on motor ID 6")
        self.is_connected = True
        self.bus.connect()
        report = inspect_calibration(
            self.device, self.side, read_servo_limits(self.bus, strict=False),
        )
        item = report["joints"][JOINT]
        if report["warnings"] or item["state"] != "consistent":
            raise ValueError(f"J6 calibration must be consistent: {report['warnings'] + item['warnings']}")
        if self.bus.read("Torque_Enable", MOTOR, normalize=False, num_retry=2):
            raise ValueError("J6 torque must already be OFF before gripper-only mode")
        self.saved = item["saved"]
        self.get_action()

    def _raw_position(self):
        raw = int(self.bus.read("Present_Position", MOTOR, normalize=False, num_retry=2))
        if not self.saved["range_min"] <= raw <= self.saved["range_max"]:
            raise ValueError("J6 is outside its saved range; with torque OFF move gently inward")
        return raw

    def get_action(self):
        raw = self._raw_position()
        motor_id = self.bus.motors[MOTOR].id
        percent = self.bus._normalize({motor_id: raw})[motor_id]
        return validate_action({JOINT: percent}, GRIPPER_JOINTS)

    def disconnect(self):
        if self.is_connected:
            try:
                self.bus.disconnect(disable_torque=False)
            finally:
                self.is_connected = False


class GripperFollower(GripperLeader):
    """Enable only J6 on the first aligned target; release J6 on stop/watchdog."""

    def __init__(self, device):
        super().__init__(device)
        self.changed = False
        self.previous_speed = None
        self.side = "follower"

    def get_observation(self):
        return self.get_action()

    def send_action(self, action):
        target = validate_action(action, GRIPPER_JOINTS)[JOINT]
        measured = self.get_action()[JOINT]
        if abs(target - measured) > MAX_TRACKING_ERROR_PERCENT:
            raise ValueError("J6 target exceeds 5% measured tracking/alignment limit; move more slowly")
        if not self.changed:
            raw = self._raw_position()
            if self.bus.read("Torque_Enable", MOTOR, normalize=False, num_retry=2):
                raise ValueError("J6 torque changed before arming")
            self.previous_speed = int(
                self.bus.read("Goal_Velocity", MOTOR, normalize=False, num_retry=2)
            )
            if not 0 <= self.previous_speed <= 1023:
                raise ValueError("unexpected J6 speed register value")
            self.changed = True
            self.bus.write(
                "Goal_Position", MOTOR, raw, normalize=False, num_retry=0,
            )
            speed = min(self.previous_speed or HOME_SPEED, HOME_SPEED)
            self.bus.write("Goal_Velocity", MOTOR, speed, normalize=False, num_retry=0)
            # An uncertain enable acknowledgement still requires torque-OFF cleanup.
            self.bus.write("Torque_Enable", MOTOR, 1, normalize=False, num_retry=0)
        self.bus.write("Goal_Position", MOTOR, target, num_retry=0)

    def disconnect(self):
        try:
            if self.is_connected and self.changed:
                self.bus.write("Torque_Enable", MOTOR, 0, normalize=False, num_retry=2)
                if self.previous_speed is not None:
                    self.bus.write(
                        "Goal_Velocity", MOTOR, self.previous_speed,
                        normalize=False, num_retry=2,
                    )
        finally:
            super().disconnect()
