import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from test_teleop_shared import FullBus

from cf_robot.angular_calibration import attach_angular_calibration
from cf_robot.calibration_inspection import inspect_calibration
from cf_robot.pose_monitor import _sample
from cf_robot.servo_maintenance import read_servo_limits
from cf_robot.teleop_hardware import TeleopLeader


class AngularBus(FullBus):
    def __init__(self):
        super().__init__()
        self.locks = dict.fromkeys(self.motors, 1)

    def _normalize(self, values):
        result = {}
        for name, motor in self.motors.items():
            if motor.id not in values:
                continue
            raw = values[motor.id]
            cal = self.calibration[name]
            if name == "gripper":
                value = (raw - cal["range_min"]) * 100 / (cal["range_max"] - cal["range_min"])
            else:
                value = (raw - (cal["range_min"] + cal["range_max"]) / 2) * 360 / 4095
            result[motor.id] = value
        return result

    def _unnormalize(self, values):
        result = {}
        for name, motor in self.motors.items():
            if motor.id not in values:
                continue
            value, cal = values[motor.id], self.calibration[name]
            if name == "gripper":
                raw = cal["range_min"] + value * (cal["range_max"] - cal["range_min"]) / 100
            else:
                raw = (cal["range_min"] + cal["range_max"]) / 2 + value * 4095 / 360
            result[motor.id] = int(raw)
        return result

    def read(self, register, motor, normalize=True, **kwargs):
        if register == "Homing_Offset":
            return self.live[motor]["homing_offset"]
        if register == "Lock":
            return self.locks[motor]
        raw = super().read(register, motor, **kwargs)
        if register == "Present_Position" and normalize:
            motor_id = self.motors[motor].id
            return self._normalize({motor_id: raw})[motor_id]
        return raw

    def sync_read(self, register, normalize=True, **kwargs):
        return {name: self.read(register, name, normalize=normalize) for name in self.motors}

    def write(self, register, motor, value, normalize=True, **kwargs):
        if register == "Lock":
            self.locks[motor] = value
            self.writes.append((register, motor, value))
            return
        if register in ("Min_Position_Limit", "Max_Position_Limit"):
            field = "range_min" if register == "Min_Position_Limit" else "range_max"
            self.live[motor][field] = value
            self.writes.append((register, motor, value))
            return
        if register == "Homing_Offset":
            self.live[motor]["homing_offset"] = value
            self.writes.append((register, motor, value))
            return
        if register == "Goal_Position" and normalize:
            motor_id = self.motors[motor].id
            value = self._unnormalize({motor_id: value})[motor_id]
            normalize = False
        super().write(register, motor, value, normalize=normalize, **kwargs)

    def sync_write(self, register, values, normalize=True, **kwargs):
        selected = values if isinstance(values, dict) else dict.fromkeys(self.motors, values)
        for name, value in selected.items():
            self.write(register, name, value, normalize=normalize, **kwargs)


class AngularCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "so101_leader.json"
        self.profile_path = self.path.with_suffix(".cf-robot.json")
        self.bus = AngularBus()
        self.bus.calibration["wrist_roll"].update(homing_offset=-1786, range_min=0, range_max=4095)
        self.bus.live["wrist_roll"].update(self.bus.calibration["wrist_roll"])
        self.bus.raw["wrist_roll"] = 2038
        self.path.write_text(json.dumps(self.bus.calibration))
        self.original_file = self.path.read_bytes()
        self.device = SimpleNamespace(
            id="so101_leader", bus=self.bus, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.data = {
            "version": 1, "side": "leader", "arm_id": "so101_leader",
            "joints": {"wrist_roll": {
                "id": 5, "model": "sts3215", "homing_offset": -1786,
                "range_min": 130, "range_max": 3936, "zero_ticks": 2038,
                "degrees_per_tick": 180 / 2047, "positive_direction": "clockwise",
            }},
        }

    def attach(self):
        self.profile_path.write_text(json.dumps(self.data))
        attach_angular_calibration(self.device, "leader")
        self.bus.connect()

    def test_verified_points_and_estimated_range_not_full_turn(self):
        self.attach()
        self.assertEqual(self.bus._normalize({5: 2038})[5], 0)
        self.assertAlmostEqual(self.bus._normalize({5: 1015})[5], -90, delta=0.05)
        self.assertAlmostEqual(self.bus._normalize({5: 3062})[5], 90, delta=0.05)
        self.assertAlmostEqual(self.bus._normalize({5: 130})[5], -167.78, places=2)
        self.assertAlmostEqual(self.bus._normalize({5: 3936})[5], 166.90, places=2)
        self.assertEqual(self.bus._unnormalize({5: -90})[5], 1015)
        self.assertEqual(self.bus._unnormalize({5: 90})[5], 3062)

    def test_other_joints_and_gripper_conversion_unchanged(self):
        raw = {index: 2500 for index in range(1, 7)}
        before = self.bus._normalize(raw)
        inverse_before = self.bus._unnormalize(before)
        self.attach()
        after = self.bus._normalize(raw)
        inverse_after = self.bus._unnormalize(after)
        for motor_id in (1, 2, 3, 4, 6):
            self.assertEqual(after[motor_id], before[motor_id])
            self.assertEqual(inverse_after[motor_id], inverse_before[motor_id])
        self.assertEqual(self.path.read_bytes(), self.original_file)
        self.assertEqual(self.bus.writes, [])

    def test_monitor_and_normal_teleop_use_same_effective_angle_and_limits(self):
        self.attach()
        self.bus.raw["wrist_roll"] = 3062
        limits = read_servo_limits(self.bus)
        report = inspect_calibration(self.device, "leader", limits)
        sample = _sample(self.device, "leader", limits, report)
        leader = TeleopLeader(self.device)
        leader.connect()
        self.assertEqual(leader.get_action()["wrist_roll.pos"], sample["values"]["wrist_roll.pos"])
        item = report["joints"]["wrist_roll.pos"]
        self.assertEqual(item["state"], "consistent")
        self.assertEqual(item["reference_ticks"], 2038)
        self.assertEqual(item["working_range"], {"min": 130, "max": 3936})
        self.assertEqual(leader.raw_limits["wrist_roll.pos"], item["working_range"])
        self.assertEqual(sample["limits"]["wrist_roll.pos"], {"min": 0, "max": 4095})
        self.assertEqual(item["angular_calibration"]["state"], "verified")
        self.assertEqual(report["joints"]["gripper.pos"]["servo_limits"], limits["gripper.pos"])
        self.bus.raw["wrist_roll"] = 125
        with self.assertRaises(ValueError):
            leader.get_action()
        leader.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_sdk_register_reads_and_writes_use_same_mapping(self):
        self.attach()
        self.bus.raw["wrist_roll"] = 3062
        self.assertAlmostEqual(self.bus.read("Present_Position", "wrist_roll"), 90, delta=0.05)
        self.bus.write("Goal_Position", "wrist_roll", -90)
        self.assertEqual(self.bus.raw["wrist_roll"], 1015)
        self.assertTrue(all(write[1] == "wrist_roll" for write in self.bus.writes))

    def test_live_offset_mismatch_keeps_raw_visible_but_blocks_angles(self):
        self.bus.live["wrist_roll"]["homing_offset"] = -1700
        self.attach()
        with self.assertRaisesRegex(ValueError, "live homing offset"):
            self.bus._normalize({5: 2038})
        limits = read_servo_limits(self.bus)
        report = inspect_calibration(self.device, "leader", limits)
        sample = _sample(self.device, "leader", limits, report)
        self.assertEqual(sample["raw"]["wrist_roll.pos"], 2038)
        self.assertIsNone(sample["values"]["wrist_roll.pos"])
        self.assertEqual(report["joints"]["wrist_roll.pos"]["state"], "warning")
        self.assertEqual(self.bus.writes, [])

    def test_raw_goal_limits_are_enforced_before_single_or_batch_writes(self):
        self.attach()
        with self.assertRaisesRegex(ValueError, "working range"):
            self.bus.write("Goal_Position", "wrist_roll", 0, normalize=False)
        with self.assertRaisesRegex(ValueError, "working range"):
            self.bus.sync_write(
                "Goal_Position", {"shoulder_pan": 2000, "wrist_roll": 0}, normalize=False,
            )
        self.assertEqual(self.bus.writes, [])

    def test_live_calibration_write_invalidates_active_mapping(self):
        self.attach()
        self.bus.write("Homing_Offset", "wrist_roll", -1700, normalize=False)
        with self.assertRaisesRegex(ValueError, "servo calibration changed"):
            self.bus._normalize({5: 2038})

    def test_saved_offset_changes_invalidate_the_profile(self):
        self.attach()
        self.bus.calibration["wrist_roll"]["homing_offset"] = -1700
        with self.assertRaisesRegex(ValueError, "saved homing offset"):
            self.bus._normalize({5: 2038})

    def test_profile_survives_reconstruction_and_does_not_change_sdk_file(self):
        self.attach()
        expected = self.bus._normalize({5: 3062})
        self.bus.disconnect(disable_torque=False)
        other = AngularBus()
        other.calibration = json.loads(self.path.read_text())
        other.live["wrist_roll"].update(other.calibration["wrist_roll"])
        device = SimpleNamespace(id="so101_leader", bus=other, calibration_fpath=self.path)
        attach_angular_calibration(device, "leader")
        other.connect()
        self.assertEqual(other._normalize({5: 3062}), expected)
        self.assertEqual(self.path.read_bytes(), self.original_file)
        self.assertEqual(other.writes, [])

    def test_profile_is_bound_to_arm_and_cannot_override_gripper(self):
        for change in ("wrong-arm", "gripper", "invalid-scale"):
            with self.subTest(change=change):
                data = json.loads(json.dumps(self.data))
                if change == "wrong-arm":
                    data["arm_id"] = "another-arm"
                elif change == "gripper":
                    data["joints"]["gripper"] = dict(data["joints"]["wrist_roll"])
                else:
                    data["joints"]["wrist_roll"]["degrees_per_tick"] = 0
                self.profile_path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):
                    attach_angular_calibration(self.device, "leader")
        self.assertEqual(self.bus.writes, [])

    def test_no_profile_preserves_sdk_behavior_and_validation_key_changes_on_install(self):
        self.assertIs(attach_angular_calibration(self.device, "leader"), self.device)
        self.bus.connect()
        limits = read_servo_limits(self.bus)
        before = inspect_calibration(self.device, "leader", limits)
        self.attach()
        after = inspect_calibration(self.device, "leader", limits)
        self.assertNotEqual(before["validation_key"], after["validation_key"])
        with self.assertRaisesRegex(ValueError, "working range"):
            self.bus._unnormalize({5: 180})


if __name__ == "__main__":
    unittest.main()
