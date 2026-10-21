"""Isolated J6 adapters: never configure or command the other five joints."""

from time import monotonic
from urllib.parse import urlsplit, urlunsplit

from cf_robot.protocol import GRIPPER_JOINTS
from cf_robot.teleop_hardware import TeleopFollower, TeleopLeader


def gripper_control_url(url):
    parsed = urlsplit(url)
    if parsed.path not in ("", "/", "/control"):
        raise ValueError("pass the normal /control URL with --J6-only")
    return urlunsplit((parsed.scheme, parsed.netloc, "/control/gripper", "", ""))


class GripperLeader(TeleopLeader):
    def __init__(self, device):
        super().__init__(device, GRIPPER_JOINTS)


class GripperFollower(TeleopFollower):
    def __init__(self, device):
        super().__init__(device, GRIPPER_JOINTS, clock=lambda: monotonic())

    @property
    def last_goal(self):
        return None if self.motion.last_goals is None else self.motion.last_goals["gripper.pos"]
