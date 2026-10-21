"""Isolated J6 adapters: never configure or command the other five joints."""

from time import monotonic
from urllib.parse import urlsplit, urlunsplit

from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.protocol import GRIPPER_JOINTS, validate_action
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits

MOTOR = "gripper"
JOINT = "gripper.pos"
MAX_GOAL_LEAD_PERCENT = 5
PROGRESS_TOLERANCE_PERCENT = 1.0
MIN_PROGRESS_PERCENT = 0.25
NO_PROGRESS_TIMEOUT_S = 1.0


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
        self.last_goal = None
        self.progress_position = None
        self.progress_at = None
        self.progress_direction = 0
        self.speed = None

    def get_observation(self):
        return self.get_action()

    def send_action(self, action):
        target = validate_action(action, GRIPPER_JOINTS)[JOINT]
        measured = self.get_action()[JOINT]
        if not self.changed and abs(target - measured) > MAX_GOAL_LEAD_PERCENT:
            raise ValueError(
                f"J6 startup alignment exceeds 5 percentage points: "
                f"leader={target:.2f}%, follower={measured:.2f}%"
            )
        self._check_progress(target, measured)
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
            self.speed = min(self.previous_speed or HOME_SPEED, HOME_SPEED)
            self.bus.write("Goal_Velocity", MOTOR, self.speed, normalize=False, num_retry=0)
            # An uncertain enable acknowledgement still requires torque-OFF cleanup.
            self.bus.write("Torque_Enable", MOTOR, 1, normalize=False, num_retry=0)
        # Bound the applied goal, not the leader's requested position. Normal
        # servo lag must not be mistaken for a calibration/alignment failure.
        applied = measured + max(
            -MAX_GOAL_LEAD_PERCENT, min(MAX_GOAL_LEAD_PERCENT, target - measured),
        )
        self.bus.write("Goal_Position", MOTOR, applied, num_retry=0)
        self.last_goal = applied

    def _check_progress(self, target, measured):
        now = monotonic()
        direction = 1 if target > measured else -1
        pending = (
            self.last_goal is not None
            and abs(self.last_goal - measured) > PROGRESS_TOLERANCE_PERCENT
            and (self.last_goal - measured) * (target - measured) > 0
            and abs(target - measured) > PROGRESS_TOLERANCE_PERCENT
        )
        if not pending or direction != self.progress_direction:
            self.progress_position, self.progress_at = measured, now
            self.progress_direction = direction
        elif (measured - self.progress_position) * direction >= MIN_PROGRESS_PERCENT:
            self.progress_position, self.progress_at = measured, now
        elif now - self.progress_at >= NO_PROGRESS_TIMEOUT_S:
            raise RuntimeError(
                f"J6 no measured progress for {NO_PROGRESS_TIMEOUT_S:g}s: "
                f"leader={target:.2f}%, follower={measured:.2f}%, "
                f"previous goal={self.last_goal:.2f}%, speed register={self.speed}. "
                "Stop and check for obstruction or an unexpectedly low speed setting."
            )

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
