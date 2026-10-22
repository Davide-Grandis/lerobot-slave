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

    def follow_goals(self, response):
        """Simulate a servo that moves `response(present, goal)` ticks per bus cycle."""
        goal = {"value": self.bus.present["elbow_flex"]}
        original_read = self.bus.read

        def write(register, motor, value, **kwargs):
            self.bus.writes.append(((register, motor, value), kwargs))
            if register == "Goal_Velocity":
                self.bus.speeds[motor] = value
            elif register == "Goal_Position":
                goal["value"] = value

        def read(register, motor, **kwargs):
            if register == "Present_Position" and motor == "elbow_flex" and not kwargs.get("normalize", True):
                self.bus.present[motor] += response(self.bus.present[motor], goal["value"])
            return original_read(register, motor, **kwargs)

        return goal, write, read

    def test_slow_but_moving_joint_is_not_cut_off_by_elapsed_time(self):
        self.command["value"] = 2900
        clock = iter(range(0, 10_000))

        def creep(present, goal):
            # One tick per cycle: far slower than the old 5 s per-step allowance.
            return (goal > present) - (goal < present)

        _, write, read = self.follow_goals(creep)
        with patch.object(self.bus, "write", side_effect=write), patch.object(
            self.bus, "read", side_effect=read,
        ), patch("cf_robot.pose_monitor.time.monotonic", side_effect=lambda: next(clock) * 0.05), patch(
            "cf_robot.pose_monitor.time.sleep",
        ):
            self.run_move()
        self.assertEqual(self.results[-1]["state"], "succeeded")
        self.assertGreater(self.results[-1]["elapsed_s"], 5)
        self.assertLessEqual(abs(self.bus.present["elbow_flex"] - 2900), 11)

    def test_load_steady_state_error_settles_short_and_keeps_target_goal(self):
        def sag(present, goal):
            # Gravity leaves the joint 20 ticks (~1.8 degrees) below its goal.
            return 1 if present < goal - 20 else 0

        goal, write, read = self.follow_goals(sag)
        clock = iter(range(0, 10_000))
        with patch.object(self.bus, "write", side_effect=write), patch.object(
            self.bus, "read", side_effect=read,
        ), patch("cf_robot.pose_monitor.time.monotonic", side_effect=lambda: next(clock) * 0.05), patch(
            "cf_robot.pose_monitor.time.sleep",
        ):
            self.run_move()
        result = self.results[-1]
        self.assertEqual(result["state"], "settled_short")
        self.assertEqual(goal["value"], 2050)
        self.assertEqual(result["residual_ticks"], 20)
        self.assertIn("servo_snapshot", result)
        self.assertEqual(self.bus.speeds["elbow_flex"], HOME_SPEED)

    def test_far_stall_reports_diagnostics_and_holds_measured_position(self):
        self.command["value"] = 2900

        def blocked(present, goal):
            return 1 if present < 2300 and goal > present else 0

        _, write, read = self.follow_goals(blocked)
        clock = iter(range(0, 10_000))
        with patch.object(self.bus, "write", side_effect=write), patch.object(
            self.bus, "read", side_effect=read,
        ), patch("cf_robot.pose_monitor.time.monotonic", side_effect=lambda: next(clock) * 0.05), patch(
            "cf_robot.pose_monitor.time.sleep",
        ):
            self.run_move()
        result = self.results[-1]
        self.assertEqual(result["state"], "failed")
        self.assertIn("no measured progress", result["message"])
        self.assertEqual(result["held_raw"], 2300)
        self.assertEqual(self.bus.writes[-1][0], ("Goal_Position", "elbow_flex", 2300))
        self.assertIn("Goal_Velocity", result["servo_snapshot"])
        self.assertTrue(all(entry[0][1] == "elbow_flex" for entry in self.bus.writes))

    def test_each_goal_stays_within_one_bounded_step_of_measured_position(self):
        self.command["value"] = 2900
        goals = []

        def lag(present, goal):
            return max(-5, min(5, goal - present))

        _, write, read = self.follow_goals(lag)

        def record(register, motor, value, **kwargs):
            if register == "Goal_Position":
                goals.append((self.bus.present[motor], value))
            write(register, motor, value, **kwargs)

        with patch.object(self.bus, "write", side_effect=record), patch.object(
            self.bus, "read", side_effect=read,
        ), patch("cf_robot.pose_monitor.time.sleep"):
            self.run_move()
        self.assertEqual(self.results[-1]["state"], "succeeded")
        self.assertTrue(all(abs(goal - present) <= 28 for present, goal in goals))
