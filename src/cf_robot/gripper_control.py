"""Isolated J6 adapters: never configure or command the other five joints."""

from time import monotonic
from urllib.parse import urlsplit, urlunsplit

from cf_robot.protocol import GRIPPER_JOINTS
from cf_robot.teleop_hardware import TeleopFollower, TeleopLeader


def gripper_control_url(url):
    return joint_control_url(url, 6)


def joint_control_url(url, joint_id):
    if type(joint_id) is not int or not 1 <= joint_id <= 6:
        raise ValueError("joint-only mode requires J1-J6")
    parsed = urlsplit(url)
    if parsed.path not in ("", "/", "/control"):
        raise ValueError(f"pass the normal /control URL with --J{joint_id}-only")
    path = "/control/gripper" if joint_id == 6 else f"/control/joint/{joint_id}"
    return urlunsplit((parsed.scheme, parsed.netloc, path, "", ""))


class GripperLeader(TeleopLeader):
    def __init__(self, device):
        super().__init__(device, GRIPPER_JOINTS)


class GripperFollower(TeleopFollower):
    def __init__(self, device):
        super().__init__(device, GRIPPER_JOINTS, clock=lambda: monotonic())

    @property
    def last_goal(self):
        return None if self.motion.last_goals is None else self.motion.last_goals["gripper.pos"]
