from io import StringIO
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_angular_calibration import AngularBus

from cf_robot.angular_calibration import attach_angular_calibration
from cf_robot.angular_install import AngularInstall
from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.leader_limits import main
from cf_robot.servo_maintenance import read_servo_limits


class LeaderLimitsTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "so101_leader.json"
        self.bus = AngularBus()
        self.bus.protocol_version = 0
        self.bus.calibration["wrist_roll"].update(homing_offset=-1786, range_min=0, range_max=4095)
        self.bus.live["wrist_roll"].update(self.bus.calibration["wrist_roll"])
        self.bus.raw["wrist_roll"] = 2038
        self.bus.torque["gripper"] = True
        self.path.write_text(json.dumps(self.bus.calibration))
        self.profile_path = self.path.with_suffix(".cf-robot.json")
        self.profile_path.write_text(json.dumps({
            "version": 1, "side": "leader", "arm_id": "so101_leader",
            "joints": {"wrist_roll": {
                "id": 5, "model": "sts3215", "homing_offset": -1786,
                "range_min": 130, "range_max": 3936,
                "zero_ticks": 2038, "degrees_per_tick": 180 / 2047,
            }},
        }))
        self.device = SimpleNamespace(
            id="so101_leader", bus=self.bus, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.sdk_before = self.path.read_bytes()
        self.profile_before = self.profile_path.read_bytes()
        self.args = [
            "cf-robot-leader-limits", "--port", "/dev/fake", "--joint", "5",
            "--min", "130", "--max", "3936",
        ]

    def apply(self, manager, preview):
        return manager.apply(self.device, {
            "joint_id": 5, "token": preview["token"], "confirmation": "APPLY ANGULAR AND LIMITS J5",
        }, threading.Event())

    def test_preview_changes_nothing_and_names_exact_hardware_writes(self):
        preview = AngularInstall("leader").preview_limits(self.device, 5, 130, 3936)
        self.assertEqual(preview["hardware_writes"], {
            "Min_Position_Limit": 130, "Max_Position_Limit": 3936,
        })
        self.assertEqual(preview["live_homing_offset_preserved"], -1786)
        self.assertEqual(self.bus.writes, [])
        self.assertEqual(self.path.read_bytes(), self.sdk_before)

    def test_hardware_and_sdk_match_with_no_second_angular_range(self):
        before = self.bus.read_calibration()
        manager = AngularInstall("leader")
        self.apply(manager, manager.preview_limits(self.device, 5, 130, 3936))
        for name, record in before.items():
            if name != "wrist_roll":
                self.assertEqual(self.bus.live[name], record)
        self.assertEqual(self.bus.live["wrist_roll"]["homing_offset"], -1786)
        self.assertEqual(self.bus.live["wrist_roll"]["range_min"], 130)
        self.assertEqual(self.bus.live["wrist_roll"]["range_max"], 3936)
        self.assertTrue(self.bus.torque["gripper"])
        self.assertFalse(self.bus.torque["wrist_roll"])
        self.assertTrue(all(
            name == "wrist_roll" and register in ("Lock", "Min_Position_Limit", "Max_Position_Limit")
            for register, name, _ in self.bus.writes
        ))
        saved = json.loads(self.path.read_bytes())
        original_sdk = json.loads(self.sdk_before)
        for name in original_sdk:
            if name != "wrist_roll":
                self.assertEqual(saved[name], original_sdk[name])
        self.assertEqual(saved["wrist_roll"], self.bus.live["wrist_roll"])
        profile = json.loads(self.profile_path.read_bytes())["joints"]["wrist_roll"]
        self.assertNotIn("range_min", profile)
        self.assertNotIn("range_max", profile)
        fresh = AngularBus()
        fresh.calibration = saved
        fresh.live = self.bus.read_calibration()
        device = SimpleNamespace(
            id="so101_leader", bus=fresh, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        attach_angular_calibration(device, "leader")
        fresh.connect()
        item = inspect_calibration(device, "leader", read_servo_limits(fresh))["joints"]["wrist_roll.pos"]
        self.assertEqual(item["working_range"], item["servo_limits"])
        self.assertEqual(item["angular_calibration"]["limits_source"], "hardware")
        self.assertEqual(item["reference_ticks"], 2038)
        self.assertEqual(fresh._normalize({5: 2038}), {5: 0})

    def test_cancelled_cli_never_writes_eeprom_or_files(self):
        with patch("sys.argv", self.args), patch("sys.stdout", StringIO()), patch(
            "cf_robot.leader_limits.make_leader", return_value=self.device,
        ), patch("builtins.input", return_value=""):
            main()
        self.assertEqual(self.bus.writes, [])
        self.assertEqual(self.path.read_bytes(), self.sdk_before)
        self.assertEqual(self.profile_path.read_bytes(), self.profile_before)

    def test_cli_requires_explicit_limit_confirmation(self):
        with patch("sys.argv", self.args), patch("sys.stdout", StringIO()), patch(
            "cf_robot.leader_limits.make_leader", return_value=self.device,
        ), patch("builtins.input", return_value="APPLY J5 LIMITS"):
            main()
        self.assertEqual(self.bus.live["wrist_roll"]["range_min"], 130)
        self.assertEqual(self.bus.live["wrist_roll"]["range_max"], 3936)

    def test_lost_write_ack_is_not_retried_or_rolled_back(self):
        manager = AngularInstall("leader")
        preview = manager.preview_limits(self.device, 5, 130, 3936)
        original = self.bus.write
        attempted = []

        def write(register, *args, **kwargs):
            attempted.append(register)
            original(register, *args, **kwargs)
            if register == "Max_Position_Limit":
                raise OSError("simulated lost acknowledgement after EEPROM write")

        with patch.object(self.bus, "write", side_effect=write):
            with self.assertRaisesRegex(RuntimeError, "EEPROM may be partially changed"):
                self.apply(manager, preview)
        self.assertEqual(attempted.count("Max_Position_Limit"), 1)
        self.assertEqual(self.path.read_bytes(), self.sdk_before)
        self.assertEqual(self.profile_path.read_bytes(), self.profile_before)
        self.assertFalse(any(name in attempted for name in ("Homing_Offset", "Torque_Enable", "Goal_Position")))

    def test_already_matching_hardware_is_not_rewritten_during_file_migration(self):
        self.bus.live["wrist_roll"].update(range_min=130, range_max=3936)
        manager = AngularInstall("leader")
        preview = manager.preview_limits(self.device, 5, 130, 3936)
        self.assertEqual(preview["hardware_writes"], {})
        result = self.apply(manager, preview)
        self.assertEqual(result["servo_writes"], [])
        self.assertEqual(self.bus.writes, [])

    def test_near_endpoint_or_torque_on_refuses_preview(self):
        manager = AngularInstall("leader")
        self.bus.raw["wrist_roll"] = 130
        with self.assertRaisesRegex(ValueError, "20-tick margin"):
            manager.preview_limits(self.device, 5, 130, 3936)
        self.bus.raw["wrist_roll"] = 2038
        self.bus.torque["wrist_roll"] = True
        with self.assertRaisesRegex(ValueError, "must already be OFF"):
            manager.preview_limits(self.device, 5, 130, 3936)
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
