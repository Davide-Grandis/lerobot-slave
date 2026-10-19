import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from cf_robot.parking import parking_steps
from cf_robot.parking_client import load_parking_pose, save_parking_pose
from cf_robot.protocol import JOINTS


class ParkingTests(unittest.TestCase):
    def test_parking_moves_one_joint_at_a_time_in_small_steps(self):
        start = {joint: 0.0 for joint in JOINTS}
        target = {
            "shoulder_pan.pos": 0.4,
            "shoulder_lift.pos": -0.3,
            "elbow_flex.pos": 0.2,
            "wrist_flex.pos": -0.3,
            "wrist_roll.pos": 0.2,
            "gripper.pos": 1.0,
        }

        steps = list(parking_steps(start, target))

        self.assertEqual(steps[-1][1], target)
        previous = start
        changed_order = []
        for changed_joint, pose in steps:
            changed = [
                joint
                for joint in JOINTS
                if abs(pose[joint] - previous[joint]) > 1e-9
            ]
            self.assertEqual(changed, [changed_joint])
            self.assertLessEqual(
                abs(pose[changed_joint] - previous[changed_joint]), 0.1 + 1e-9
            )
            if not changed_order or changed_order[-1] != changed_joint:
                changed_order.append(changed_joint)
            previous = pose
        self.assertEqual(
            changed_order,
            [
                "shoulder_pan.pos",
                "shoulder_lift.pos",
                "elbow_flex.pos",
                "gripper.pos",
                "wrist_roll.pos",
                "wrist_flex.pos",
            ],
        )

    def test_no_motion_still_yields_target(self):
        pose = {joint: 0.0 for joint in JOINTS}
        self.assertEqual(list(parking_steps(pose, pose)), [])

    def test_remote_parking_pose_is_saved_with_private_file_permissions(self):
        pose = {joint: 1.25 for joint in JOINTS}
        with TemporaryDirectory() as tmp:
            parking_file = Path(tmp) / "cf-robot" / "follower-parking.json"
            with patch("cf_robot.parking_client.PARKING_FILE", parking_file):
                save_parking_pose(pose)
                self.assertEqual(load_parking_pose(), pose)
            self.assertEqual(parking_file.stat().st_mode & 0o777, 0o600)


if __name__ == "__main__":
    unittest.main()
