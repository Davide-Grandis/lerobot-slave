import json
import threading
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from cf_robot.protocol import JOINTS
from cf_robot.servo_maintenance import (
    HOME_SPEED,
    calculate_home_positions,
    move_to_home,
    read_limits_and_pose,
)


class FakeBus:
    def __init__(self, limits, present):
        self.limits = limits
        self.present = dict(present)
        self.speeds = {joint: 500 for joint in present}
        self.writes = []
        self.connected = False
        self.disconnected = False

    def connect(self):
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.disconnected = True

    def read(self, register, motor, *, normalize=True, num_retry=0):
        if register == "Min_Position_Limit":
            return self.limits[motor]["min"]
        if register == "Max_Position_Limit":
            return self.limits[motor]["max"]
        if register == "Present_Position":
            return self.present[motor]
        if register == "Goal_Velocity":
            return self.speeds[motor]
        raise AssertionError(f"unexpected read: {register}")

    def write(self, register, motor, value, *, normalize=True, num_retry=0):
        self.writes.append((register, motor, value))
        if register == "Goal_Velocity":
            self.speeds[motor] = value
        elif register == "Goal_Position":
            self.present[motor] = value
        else:
            raise AssertionError(f"unexpected write: {register}")


class FakeRobot:
    def __init__(self, bus):
        self.bus = bus
        self.config = SimpleNamespace(disable_torque_on_disconnect=True)
        self.connected = False
        self.disconnected = False

    def connect(self, calibrate=False):
        self.connected = True

    def disconnect(self):
        self.disconnected = True


def raw_limits():
    return {
        joint.removesuffix(".pos"): {"min": 100 + index, "max": 3900 - index}
        for index, joint in enumerate(JOINTS)
    }


class ServoMaintenanceTests(unittest.TestCase):
    def test_home_pose_matches_sdk_joint_formula_and_js_rounding(self):
        limits = {
            "shoulder_pan.pos": {"min": 20, "max": 81},
            "shoulder_lift.pos": {"min": 101, "max": 201},
            "elbow_flex.pos": {"min": 7, "max": 10},
            "wrist_flex.pos": {"min": 40, "max": 90},
            "wrist_roll.pos": {"min": 10, "max": 17},
            "gripper.pos": {"min": 2, "max": 25},
        }
        self.assertEqual(
            calculate_home_positions(limits),
            {
                "shoulder_pan.pos": 51,
                "shoulder_lift.pos": 101,
                "elbow_flex.pos": 10,
                "wrist_flex.pos": 40,
                "wrist_roll.pos": 12,
                "gripper.pos": 2,
            },
        )

    def test_home_fallbacks_match_sdk_when_limits_are_missing(self):
        self.assertEqual(
            calculate_home_positions({}),
            {
                "shoulder_pan.pos": 2048,
                "shoulder_lift.pos": 0,
                "elbow_flex.pos": 4095,
                "wrist_flex.pos": 0,
                "wrist_roll.pos": 1024,
                "gripper.pos": 0,
            },
        )

    def test_read_limits_stores_raw_values_and_present_pose(self):
        limits = raw_limits()
        present = {
            joint.removesuffix(".pos"): 2000 + index
            for index, joint in enumerate(JOINTS)
        }
        robot = FakeRobot(FakeBus(limits, present))
        with TemporaryDirectory() as temp, patch(
            "cf_robot.servo_maintenance.LIMITS_FILE", Path(temp) / "limits.json"
        ), patch("cf_robot.servo_maintenance.make_follower", return_value=robot):
            snapshot = read_limits_and_pose("/dev/fake", "so101_follower")
            saved = json.loads((Path(temp) / "limits.json").read_text())

        expected_limits = {
            joint: limits[joint.removesuffix(".pos")]
            for joint in JOINTS
        }
        expected_present = {
            joint: present[joint.removesuffix(".pos")]
            for joint in JOINTS
        }
        self.assertEqual(snapshot["record"]["limits"], expected_limits)
        self.assertEqual(snapshot["current"], expected_present)
        self.assertEqual(saved["limits"], expected_limits)
        self.assertTrue(robot.bus.disconnected)
        self.assertFalse(robot.disconnected)

    def test_go_home_uses_low_speed_then_restores_selected_speed(self):
        limits = raw_limits()
        limits_by_joint = {
            joint: limits[joint.removesuffix(".pos")]
            for joint in JOINTS
        }
        current = {joint: 2000 for joint in JOINTS}
        motor_present = {
            joint.removesuffix(".pos"): current[joint]
            for joint in JOINTS
        }
        robot = FakeRobot(FakeBus(limits, motor_present))
        target = calculate_home_positions(limits_by_joint)
        progress = []
        with patch("cf_robot.servo_maintenance.make_follower", return_value=robot), patch(
            "cf_robot.servo_maintenance.time.sleep", return_value=None
        ):
            result = move_to_home(
                "/dev/fake",
                "so101_follower",
                limits_by_joint,
                current,
                target,
                75,
                threading.Event(),
                progress.append,
            )

        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(result["target"], target)
        self.assertTrue(robot.disconnected)
        speed_writes = [write for write in robot.bus.writes if write[0] == "Goal_Velocity"]
        self.assertEqual(speed_writes[0][2], HOME_SPEED)
        self.assertTrue(all(write[2] == 767 for write in speed_writes[6:]))
        position_writes = [write for write in robot.bus.writes if write[0] == "Goal_Position"]
        self.assertEqual([write[2] for write in position_writes], list(target.values()))
        self.assertTrue(progress)


if __name__ == "__main__":
    unittest.main()
