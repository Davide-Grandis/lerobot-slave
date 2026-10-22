import queue
import threading
import unittest
from unittest.mock import patch

from cf_robot.pose_compare_client import parse_pose_command
from cf_robot.pose_monitor import (
    DIAGNOSTIC_REGISTERS,
    read_servo_diagnostics,
    stream_follower_pose,
    validate_pose_command,
)

from test_pose_monitor import FakeArm


class ServoDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.device = FakeArm()
        for motor in self.device.bus.motors.values():
            motor.model = "sts3215"
        self.command = {
            "version": 1, "type": "command", "command_id": 1,
            "joint_id": 2, "mode": "diagnostics",
        }

    def test_cli_and_protocol_accept_only_one_joint_without_write_parameters(self):
        parsed = parse_pose_command("J2 STATUS", 1)
        self.assertEqual(parsed["message"], self.command)
        self.assertEqual(validate_pose_command(self.command), self.command)
        for change in (
            {"joint_id": "*"}, {"joint_id": True}, {"joint_id": 7},
            {"value": 100}, {"enabled": True}, {"register": "ID"},
        ):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_pose_command({**self.command, **change})

    def test_snapshot_reads_selected_motor_only_without_writes_or_normalization(self):
        with patch.object(self.device.bus, "read", return_value=42) as read:
            result = read_servo_diagnostics(self.device, 2)
        self.assertEqual(result["side"], "follower")
        self.assertEqual(result["registers"], dict.fromkeys(DIAGNOSTIC_REGISTERS, 42))
        self.assertEqual(result["errors"], {})
        for call in read.call_args_list:
            self.assertEqual(call.args[1], "shoulder_lift")
            self.assertEqual(call.kwargs, {"normalize": False, "num_retry": 0})
        self.assertEqual(self.device.bus.writes, [])

    def test_failed_register_is_explicit_and_does_not_hide_other_readings(self):
        def read(register, motor, **kwargs):
            if register == "Present_Current":
                raise OSError("no response")
            return 7

        with patch.object(self.device.bus, "read", side_effect=read):
            result = read_servo_diagnostics(self.device, 2)
        self.assertNotIn("Present_Current", result["registers"])
        self.assertEqual(result["errors"], {"Present_Current": "no response"})
        self.assertEqual(result["registers"]["Status"], 7)
        self.assertEqual(self.device.bus.writes, [])

    def test_monitor_diagnostics_preserves_torque_and_uses_existing_hardware_owner(self):
        commands = queue.Queue()
        commands.put(self.command)
        stop = threading.Event()
        messages = []
        torque_before = dict(self.device.bus.torque)
        snapshot = {"side": "follower", "registers": {"Torque_Enable": 1}, "errors": {}}

        def publish(message):
            messages.append(message)
            if message.get("type") == "command_result":
                stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device), patch(
            "cf_robot.pose_monitor.inspect_calibration", return_value={
                "state": "warning", "warnings": ["unrelated calibration warning"], "joints": {},
            },
        ), patch("cf_robot.pose_monitor.read_servo_diagnostics", return_value=snapshot) as read:
            stream_follower_pose("/dev/fake", "follower", stop, publish, commands)
        read.assert_called_once_with(self.device, 2)
        result = next(m for m in messages if m.get("type") == "command_result")
        self.assertEqual(result["state"], "succeeded")
        self.assertEqual(result["servo_diagnostics"], snapshot)
        self.assertEqual(self.device.bus.torque, torque_before)
        self.assertEqual(self.device.bus.writes, [])
