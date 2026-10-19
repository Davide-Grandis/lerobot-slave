import time
import unittest

from cf_robot.control import FollowerSession
from cf_robot.protocol import JOINTS


class FakeFollower:
    def __init__(self):
        self.connected = False
        self.disconnected = False
        self.actions = []

    def connect(self, calibrate=True):
        if calibrate:
            raise AssertionError("service must not calibrate interactively")
        self.connected = True

    def get_observation(self):
        return {joint: 0.0 for joint in JOINTS}

    def send_action(self, action):
        self.actions.append(action)

    def disconnect(self):
        self.disconnected = True


class FailingFollower(FakeFollower):
    def send_action(self, action):
        raise OSError("serial device disconnected")


def wait_for(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    return predicate()


class ControlTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeFollower()
        self.session = FollowerSession(lambda: self.robot, watchdog=0.1)
        self.session.start()
        self.assertTrue(self.session.wait_ready(1))
        self.assertEqual(self.session.initial_observation, {joint: 0.0 for joint in JOINTS})

    def tearDown(self):
        self.session.stop()
        self.assertTrue(self.robot.disconnected)

    def test_idle_never_moves(self):
        time.sleep(0.15)
        self.assertEqual(self.robot.actions, [])
        self.assertEqual(self.session.state, "ready")

    def test_command_then_watchdog_disarms(self):
        self.session.submit({joint: 1.0 for joint in JOINTS})
        self.assertTrue(wait_for(lambda: len(self.robot.actions) == 1))
        self.assertTrue(wait_for(lambda: self.session.state == "error"))
        self.assertIn("timed out", self.session.error)

    def test_initial_jump_is_rejected(self):
        action = {joint: 0.0 for joint in JOINTS}
        action["wrist_roll.pos"] = 20.0
        action["elbow_flex.pos"] = -6.0
        self.session.submit(action)
        self.assertTrue(wait_for(lambda: self.session.state == "error"))
        self.assertEqual(self.robot.actions, [])
        self.assertIn("\n  - elbow_flex.pos:", self.session.error)
        self.assertIn("\n  - wrist_roll.pos:", self.session.error)
        self.assertIn("follower=0.00", self.session.error)
        self.assertIn("leader=20.00", self.session.error)
        self.assertIn("delta=+20.00", self.session.error)

    def test_midstream_jump_compares_with_previous_leader_target(self):
        self.session.watchdog = 1.0
        self.session.submit({joint: 0.0 for joint in JOINTS})
        self.assertTrue(wait_for(lambda: len(self.robot.actions) == 1))

        action = {joint: 0.0 for joint in JOINTS}
        action["wrist_flex.pos"] = -5.01
        self.session.submit(action)

        self.assertTrue(wait_for(lambda: self.session.state == "error"))
        self.assertEqual(len(self.robot.actions), 1)
        self.assertIn("previous leader=0.00", self.session.error)
        self.assertIn("leader=-5.01", self.session.error)
        self.assertIn("delta=-5.01", self.session.error)
        self.assertIn("reduce per-sample movement", self.session.error)

    def test_hardware_error_ends_session(self):
        self.session.stop()
        self.robot = FailingFollower()
        self.session = FollowerSession(lambda: self.robot, watchdog=0.1)
        self.session.start()
        self.assertTrue(self.session.wait_ready(1))
        self.session.submit({joint: 1.0 for joint in JOINTS})
        self.assertTrue(wait_for(lambda: self.session.state == "error"))
        self.assertIn("serial device disconnected", self.session.error)


if __name__ == "__main__":
    unittest.main()
