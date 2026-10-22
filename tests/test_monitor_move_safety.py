import threading
import unittest
from unittest.mock import patch

from test_pose_monitor import FakeArm

from cf_robot.pose_monitor import execute_pose_command
from cf_robot.servo_maintenance import HOME_SPEED, read_servo_limits


class MonitorMoveSafetyTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeArm()
        self.bus = self.robot.bus
        self.bus.speeds["elbow_flex"] = 0
        self.command = {
            "version": 1, "type": "command", "command_id": 1,
            "joint_id": 3, "mode": "ticks", "value": 2050,
        }
        self.results = []

    def run_move(self, stop=None):
        execute_pose_command(
            self.robot, read_servo_limits(self.bus), self.command,
            stop or threading.Event(), self.results.append,
        )

    def test_success_does_not_restore_unrestricted_speed(self):
        self.run_move()
        self.assertEqual(self.results[-1]["state"], "succeeded")
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)
        self.assertEqual(self.bus.present["elbow_flex"], 2050)

    def test_lower_existing_speed_is_preserved(self):
        self.bus.speeds["elbow_flex"] = 20
        self.run_move()
        self.assertEqual(self.bus.speeds["elbow_flex"], 20)

    def test_stall_holds_measured_position_before_reporting_failure(self):
        def write(register, motor, value, **kwargs):
            self.bus.writes.append(((register, motor, value), kwargs))
            if register == "Goal_Velocity":
                self.bus.speeds[motor] = value

        with patch.object(self.bus, "write", side_effect=write), patch(
            "cf_robot.pose_monitor.time.monotonic", side_effect=range(100),
        ), patch("cf_robot.pose_monitor.time.sleep"):
            self.run_move()
        self.assertEqual(self.results[-1]["state"], "failed")
        self.assertEqual(self.bus.writes[-1][0], ("Goal_Position", "elbow_flex", 2000))
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)
        self.assertEqual(self.results[-1]["actual_raw"], 2000)
        self.assertIn("step_target_raw", self.results[-1])

    def test_cancel_holds_without_releasing_torque_or_restoring_full_speed(self):
        stop = threading.Event()
        stop.set()
        self.run_move(stop)
        self.assertEqual(self.results[-1]["state"], "cancelled")
        self.assertEqual(self.bus.writes[-1][0], ("Goal_Position", "elbow_flex", 2000))
        self.assertTrue(self.bus.torque["elbow_flex"])
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)

    def test_goal_write_failure_attempts_one_hold_without_other_joint_writes(self):
        original = self.bus.write
        failed = False

        def write(register, motor, value, **kwargs):
            nonlocal failed
            if register == "Goal_Position" and not failed:
                failed = True
                raise OSError("goal acknowledgement lost")
            original(register, motor, value, **kwargs)

        with patch.object(self.bus, "write", side_effect=write):
            with self.assertRaisesRegex(OSError, "acknowledgement lost"):
                self.run_move()
        self.assertEqual(self.bus.writes[-1][0], ("Goal_Position", "elbow_flex", 2000))
        self.assertTrue(all(entry[0][1] == "elbow_flex" for entry in self.bus.writes))
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)

    def test_cancel_during_last_step_reporting_still_replaces_unsettled_goal(self):
        stop = threading.Event()
        self.command["value"] = 2010

        def write(register, motor, value, **kwargs):
            self.bus.writes.append(((register, motor, value), kwargs))
            if register == "Goal_Velocity":
                self.bus.speeds[motor] = value

        def publish(message):
            self.results.append(message)
            if message.get("type") == "pose_sample":
                stop.set()

        with patch.object(self.bus, "write", side_effect=write):
            execute_pose_command(
                self.robot, read_servo_limits(self.bus), self.command, stop, publish,
            )
        self.assertEqual(self.results[-1]["state"], "cancelled")
        self.assertEqual(self.bus.writes[-1][0], ("Goal_Position", "elbow_flex", 2000))

    def test_failed_hold_is_reported_without_speed_restore_or_torque_release(self):
        with patch.object(self.bus, "write", side_effect=OSError("bus lost")):
            with self.assertRaises(OSError):
                self.run_move()
        self.assertEqual(self.bus.speeds["elbow_flex"], 0)
        self.assertTrue(self.bus.torque["elbow_flex"])

    def test_hold_failure_after_stall_keeps_cap_and_does_not_claim_safe_stop(self):
        def write(register, motor, value, **kwargs):
            self.bus.writes.append(((register, motor, value), kwargs))
            if register == "Goal_Velocity":
                self.bus.speeds[motor] = value
            elif register == "Goal_Position" and value == 2000:
                raise OSError("hold not acknowledged")

        with patch.object(self.bus, "write", side_effect=write), patch(
            "cf_robot.pose_monitor.time.monotonic", side_effect=range(100),
        ), patch("cf_robot.pose_monitor.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "safe hold failed"):
                self.run_move()
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)
        self.assertTrue(self.bus.torque["elbow_flex"])
        self.assertEqual(sum(entry[0] == ("Goal_Position", "elbow_flex", 2000) for entry in self.bus.writes), 1)
        self.assertFalse(any(m.get("state") == "failed" for m in self.results))
