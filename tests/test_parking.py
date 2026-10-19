import unittest

from cf_robot.parking import interpolated_targets
from cf_robot.protocol import JOINTS


class ParkingTests(unittest.TestCase):
    def test_interpolation_limits_each_joint_step(self):
        start = {joint: 0.0 for joint in JOINTS}
        target = {joint: 0.0 for joint in JOINTS}
        target["shoulder_pan.pos"] = 2.5
        target["wrist_flex.pos"] = -1.5

        poses = list(interpolated_targets(start, target, max_step=1.0))

        self.assertEqual(poses[-1], target)
        previous = start
        for pose in poses:
            for joint in JOINTS:
                self.assertLessEqual(abs(pose[joint] - previous[joint]), 1.0)
            previous = pose

    def test_no_motion_still_yields_target(self):
        pose = {joint: 0.0 for joint in JOINTS}
        self.assertEqual(list(interpolated_targets(pose, pose)), [pose])


if __name__ == "__main__":
    unittest.main()
