from io import StringIO
import itertools
import json
import threading
import unittest
from unittest.mock import Mock, patch

from test_pose_monitor import FakeArm

from cf_robot.pose_monitor import (
    DIAGNOSTIC_REGISTERS, execute_pose_command, read_present_load,
)
from cf_robot.servo_maintenance import read_servo_limits
from cf_robot.sweep_client import SweepSession, load_description


class LoadTelemetryTests(unittest.TestCase):
    def test_signed_sdk_read_is_preserved_without_decoding_again(self):
        bus = Mock()
        bus.read.return_value = -520
        result = read_present_load(bus, "elbow_flex")
        self.assertEqual(result["present_load_raw"], -520)
        self.assertEqual(result["present_load_percent"], -52.0)
        bus.read.assert_called_once_with("Present_Load", "elbow_flex", normalize=False, num_retry=0)
        bus.write.assert_not_called()

    def test_failed_load_read_is_unavailable_not_zero(self):
        bus = Mock()
        bus.read.side_effect = OSError("no response")
        result = read_present_load(bus, "elbow_flex")
        self.assertIsNone(result["present_load_raw"])
        self.assertIsNone(result["present_load_percent"])
        self.assertIn("no response", result["present_load_error"])
        bus.write.assert_not_called()

    def test_status_includes_protection_time_readback(self):
        self.assertIn("Protection_Time", DIAGNOSTIC_REGISTERS)
        self.assertIn("Over_Current_Protection_Time", DIAGNOSTIC_REGISTERS)

    def test_sweep_progress_reads_load_on_selected_joint_and_keeps_goal_behavior(self):
        robot = FakeArm()
        bus = robot.bus
        original = bus.read
        load_reads = []

        def read(register, motor, **kwargs):
            if register == "Present_Load":
                load_reads.append(motor)
                return 125
            return original(register, motor, **kwargs)

        messages = []
        clock = itertools.count(step=0.05)
        with patch.object(bus, "read", side_effect=read), patch(
            "cf_robot.pose_monitor.time.monotonic", side_effect=lambda: next(clock),
        ), patch("cf_robot.pose_monitor.time.sleep"):
            execute_pose_command(
                robot, read_servo_limits(bus),
                {"version": 1, "type": "command", "command_id": 1,
                 "joint_id": 3, "mode": "ticks", "value": 2900},
                threading.Event(), messages.append,
            )
        progress = [m for m in messages if m.get("type") == "command_progress"]
        self.assertGreater(len(progress), 1)
        self.assertTrue(all(m["present_load_raw"] == 125 for m in progress))
        self.assertEqual(messages[-1]["state"], "succeeded")
        self.assertEqual(messages[-1]["present_load_percent"], 12.5)
        self.assertTrue(all(motor == "elbow_flex" for motor in load_reads))
        self.assertLessEqual(len(load_reads), len(progress) + 1)
        self.assertTrue(all(entry[0][0] in ("Goal_Position", "Goal_Velocity") for entry in bus.writes))

    def test_load_display_distinguishes_zero_negative_and_missing_readings(self):
        self.assertIn("+0 raw", load_description({"present_load_raw": 0}))
        negative = load_description({"present_load_raw": -520})
        self.assertIn("-520 raw", negative)
        self.assertIn("-52.0%", negative)
        self.assertIn("unavailable", load_description({}))
        self.assertNotIn("0 raw", load_description({}))


class SweepLoadOutputTests(unittest.IsolatedAsyncioTestCase):
    async def test_live_progress_output_contains_load(self):
        class Socket:
            def __aiter__(self):
                return self.messages()

            async def messages(self):
                yield json.dumps({
                    "type": "command_progress", "joint_id": 3,
                    "current": 10, "target": 20, "unit": "degrees",
                    "present_load_raw": -520, "present_load_percent": -52.0,
                })
                yield json.dumps({"type": "monitor_finished"})

        output = StringIO()
        with patch("sys.stdout", output):
            session = SweepSession(Socket())
            await session.reader
        self.assertIn("J3:", output.getvalue())
        self.assertIn("-520 raw", output.getvalue())
        self.assertIn("-52.0%", output.getvalue())
