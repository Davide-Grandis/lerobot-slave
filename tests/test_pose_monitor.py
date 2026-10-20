import threading
import unittest
from unittest.mock import patch

from cf_robot.protocol import JOINTS
from cf_robot.pose_monitor import stream_follower_pose, stream_leader_pose


class FakeBus:
    def __init__(self):
        self.connected = False
        self.disconnect_torque_args = []
        self.writes = []

    def connect(self):
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.disconnect_torque_args.append(disable_torque)
        self.connected = False

    def sync_read(self, register, *, normalize=True, num_retry=0):
        if register != "Present_Position":
            raise AssertionError(f"unexpected register read: {register}")
        if normalize:
            return {
                joint.removesuffix(".pos"): float(index + 0.5)
                for index, joint in enumerate(JOINTS)
            }
        return {
            joint.removesuffix(".pos"): 1000 + index
            for index, joint in enumerate(JOINTS)
        }

    def write(self, *args, **kwargs):
        self.writes.append((args, kwargs))
        raise AssertionError("read-only monitor must not write servo registers")


class FakeArm:
    def __init__(self):
        self.bus = FakeBus()

    def get_action(self):
        return {
            joint: float(index + 0.5)
            for index, joint in enumerate(JOINTS)
        }


class PoseMonitorTests(unittest.TestCase):
    def _check_stream(self, stream, factory_path):
        arm = FakeArm()
        stop = threading.Event()
        samples = []

        def publish(sample):
            samples.append(sample)
            stop.set()

        with patch(factory_path, return_value=arm):
            stream("/dev/fake", "so101_leader", stop, publish)

        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["values"]["shoulder_pan.pos"], 0.5)
        self.assertEqual(samples[0]["raw"]["shoulder_pan.pos"], 1000)
        self.assertEqual(arm.bus.disconnect_torque_args, [False])
        self.assertEqual(arm.bus.writes, [])

    def test_leader_stream_reads_action_without_configuring_or_writing(self):
        self._check_stream(stream_leader_pose, "cf_robot.pose_monitor.make_leader")

    def test_follower_stream_reads_observation_without_configuring_or_writing(self):
        self._check_stream(stream_follower_pose, "cf_robot.pose_monitor.make_follower")


if __name__ == "__main__":
    unittest.main()
