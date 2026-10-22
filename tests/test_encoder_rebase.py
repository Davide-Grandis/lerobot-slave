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
from cf_robot.encoder_rebase import EncoderRebase
from cf_robot.pose_monitor import _sample
from cf_robot.servo_maintenance import read_servo_limits


class RebaseBus(AngularBus):
    protocol_version = 0

    def write(self, register, motor, value, **kwargs):
        old_offset = self.live[motor]["homing_offset"]
        super().write(register, motor, value, **kwargs)
        if register == "Homing_Offset":
            self.raw[motor] = (self.raw[motor] + old_offset - value) % 4096


class EncoderRebaseTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "so101_leader.json"
        self.bus = RebaseBus()
        self.bus.calibration["elbow_flex"].update(homing_offset=965, range_min=3, range_max=4095)
        self.bus.live = json.loads(json.dumps(self.bus.calibration))
        self.bus.raw["elbow_flex"] = 1950
        self.path.write_text(json.dumps(self.bus.calibration))
        self.profile_path = self.path.with_suffix(".cf-robot.json")
        self.profile_path.write_text(json.dumps({
            "version": 1, "side": "leader", "arm_id": "so101_leader", "joints": {},
        }))
        self.device = SimpleNamespace(
            id="so101_leader", bus=self.bus, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.stop = threading.Event()

    def preview(self, manager):
        return manager.preview(self.device, 3, 3939, 2046, 500, 965)

    def test_wrap_projects_to_nonwrapping_range_without_writes(self):
        preview = self.preview(EncoderRebase())
        self.assertEqual(preview["tick_shift"], 657)
        self.assertEqual(preview["after"]["homing_offset"], 308)
        self.assertEqual((preview["after"]["range_min"], preview["after"]["range_max"]), (500, 2703))
        self.assertEqual(preview["projected_raw"], 2607)
        self.assertEqual(self.bus.writes, [])

    def test_apply_changes_only_selected_nv_and_preserves_sdk_other_joints(self):
        before = self.bus.read_calibration()
        torque = dict(self.bus.torque)
        manager = EncoderRebase()
        preview = self.preview(manager)
        result = manager.apply(self.device, preview["token"], self.stop)
        saved = json.loads(self.path.read_bytes())
        for name in before:
            if name != "elbow_flex":
                self.assertEqual(self.bus.live[name], before[name])
                self.assertEqual(saved[name], before[name])
        self.assertEqual(self.bus.torque, torque)
        self.assertEqual(self.bus.raw["elbow_flex"], 2607)
        self.assertEqual(saved["elbow_flex"], result["after"])
        self.assertTrue(all(
            name == "elbow_flex" and register in (
                "Lock", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
            ) for register, name, _ in self.bus.writes
        ))
        self.assertEqual(self.bus.locks["elbow_flex"], 1)
        self.assertTrue(all(Path(path).exists() for path in result["backups"]))
        record = json.loads(self.profile_path.read_bytes())["joints"]["elbow_flex"]
        self.assertTrue(record["pending_zero"])
        self.assertNotIn("zero_ticks", record)

    def test_endpoint_torque_on_or_wrong_expected_offset_blocks_preview(self):
        manager = EncoderRebase()
        self.bus.raw["elbow_flex"] = 2046
        with self.assertRaisesRegex(ValueError, "20-tick margin"):
            self.preview(manager)
        self.bus.raw["elbow_flex"] = 1950
        self.bus.torque["elbow_flex"] = True
        with self.assertRaisesRegex(ValueError, "OFF"):
            self.preview(manager)
        self.bus.torque["elbow_flex"] = False
        self.bus.live["elbow_flex"]["homing_offset"] = 308
        with self.assertRaisesRegex(ValueError, "already rebased"):
            self.preview(manager)
        self.assertEqual(self.bus.writes, [])

    def test_pending_zero_keeps_raw_visible_and_blocks_degree_and_goal_commands(self):
        manager = EncoderRebase()
        manager.apply(self.device, self.preview(manager)["token"], self.stop)
        self.bus.calibration = json.loads(self.path.read_bytes())
        self.bus.writes.clear()
        attach_angular_calibration(self.device, "leader")
        self.bus.connect()
        limits = read_servo_limits(self.bus)
        report = inspect_calibration(self.device, "leader", limits)
        sample = _sample(self.device, "leader", limits, report)
        self.assertEqual(sample["raw"]["elbow_flex.pos"], 2607)
        self.assertIsNone(sample["values"]["elbow_flex.pos"])
        self.assertIsNone(report["joints"]["elbow_flex.pos"]["reference_ticks"])
        with self.assertRaisesRegex(ValueError, "physical zero"):
            self.bus._unnormalize({3: 0})
        with self.assertRaisesRegex(ValueError, "physical zero"):
            self.bus.write("Goal_Position", "elbow_flex", 2607, normalize=False)
        self.assertEqual(self.bus.writes, [])

    def test_later_zero_can_be_saved_at_endpoint_without_more_nv_writes(self):
        manager = EncoderRebase()
        manager.apply(self.device, self.preview(manager)["token"], self.stop)
        self.bus.calibration = json.loads(self.path.read_bytes())
        self.bus.raw["elbow_flex"] = 2703
        self.bus.writes.clear()
        angular = AngularInstall("leader")
        preview = angular.preview_limits(self.device, 3, 500, 2703, zero_ticks=2703)
        result = angular.apply(self.device, {
            "joint_id": 3, "token": preview["token"], "confirmation": "APPLY ANGULAR AND LIMITS J3",
        }, self.stop)
        self.assertEqual(result["servo_writes"], [])
        self.assertEqual(self.bus.writes, [])
        record = json.loads(self.profile_path.read_bytes())["joints"]["elbow_flex"]
        self.assertNotIn("pending_zero", record)
        self.assertEqual(record["zero_ticks"], 2703)

    def test_uncertain_homing_write_has_no_automatic_retry_or_file_update(self):
        manager = EncoderRebase()
        preview = self.preview(manager)
        original = self.bus.write

        def write(register, *args, **kwargs):
            original(register, *args, **kwargs)
            if register == "Homing_Offset":
                raise OSError("lost acknowledgement")

        before = self.path.read_bytes()
        with patch.object(self.bus, "write", side_effect=write):
            with self.assertRaisesRegex(RuntimeError, "No automatic retry/rollback"):
                manager.apply(self.device, preview["token"], self.stop)
        self.assertEqual(sum(row[0] == "Homing_Offset" for row in self.bus.writes), 1)
        self.assertEqual(self.path.read_bytes(), before)
        with self.assertRaisesRegex(ValueError, "already rebased"):
            self.preview(manager)

    def test_j2_signed_shift_with_provisional_maximum(self):
        self.bus.calibration["shoulder_lift"].update(homing_offset=-806, range_min=0, range_max=4094)
        self.bus.live["shoulder_lift"].update(self.bus.calibration["shoulder_lift"])
        self.bus.raw["shoulder_lift"] = 2261
        self.path.write_text(json.dumps(self.bus.calibration))
        before = self.bus.read_calibration()
        manager = EncoderRebase()
        preview = manager.preview(
            self.device, 2, 2043, None, 500, -806, provisional_max=4095,
        )
        self.assertEqual(preview["tick_shift"], -1543)
        self.assertEqual(preview["projected_raw"], 718)
        self.assertEqual(preview["after"]["homing_offset"], 737)
        self.assertEqual((preview["after"]["range_min"], preview["after"]["range_max"]), (500, 4095))
        self.assertTrue(preview["maximum_is_provisional"])
        result = manager.apply(self.device, preview["token"], self.stop)
        self.assertEqual(result["raw"], 718)
        for name in before:
            if name != "shoulder_lift":
                self.assertEqual(self.bus.live[name], before[name])
        self.assertTrue(all(row[1] == "shoulder_lift" for row in self.bus.writes))
        self.bus.calibration = json.loads(self.path.read_bytes())
        self.bus.writes.clear()
        angular = AngularInstall("leader")
        zero = angular.preview_limits(self.device, 2, 500, 4095, zero_ticks=2200)
        angular.apply(self.device, {
            "joint_id": 2, "token": zero["token"], "confirmation": "APPLY ANGULAR AND LIMITS J2",
        }, self.stop)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(json.loads(self.profile_path.read_bytes())["joints"]["shoulder_lift"]["pending_range_max"])
        self.bus.calibration = json.loads(self.path.read_bytes())
        attach_angular_calibration(self.device, "leader")
        self.bus.connect()
        with self.assertRaisesRegex(ValueError, "maximum is provisional"):
            self.bus._normalize({2: 718})
        with self.assertRaisesRegex(ValueError, "maximum is provisional"):
            self.bus.write("Goal_Position", "shoulder_lift", 718, normalize=False)
        self.assertEqual(self.bus.writes, [])

    def test_nonwrapping_measured_j2_end_projects_consistently(self):
        self.bus.calibration["shoulder_lift"].update(homing_offset=-806, range_min=0, range_max=4094)
        self.bus.live["shoulder_lift"].update(self.bus.calibration["shoulder_lift"])
        self.bus.raw["shoulder_lift"] = 2261
        self.path.write_text(json.dumps(self.bus.calibration))
        preview = EncoderRebase().preview(self.device, 2, 2043, 4094, 500, -806)
        self.assertEqual(preview["after"]["homing_offset"], 737)
        self.assertEqual(preview["after"]["range_max"], 2551)
        self.assertFalse(preview["maximum_is_provisional"])
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
