from copy import deepcopy
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from cf_robot.calibration_restore import GripperCalibrationRestore, REGISTERS
from cf_robot.pose_compare_client import parse_pose_command
from cf_robot.pose_monitor import stream_follower_pose, validate_pose_command
from cf_robot.protocol import JOINTS


class RestoreBus:
    protocol_version = 0
    apply_drive_mode = True
    model_number_table = {"sts3215": 777}
    model_resolution_table = {"sts3215": 4096}

    def __init__(self):
        self.motors = {
            joint.removesuffix(".pos"): SimpleNamespace(
                id=index, model="sts3215",
                norm_mode="range_0_100" if index == 6 else "degrees",
            )
            for index, joint in enumerate(JOINTS, 1)
        }
        self.calibration = {
            name: {
                "id": motor.id, "drive_mode": 0, "homing_offset": 0,
                "range_min": 100, "range_max": 3900,
            } for name, motor in self.motors.items()
        }
        self.calibration["gripper"].update(
            homing_offset=-873, range_min=2038, range_max=3488,
        )
        self.live = deepcopy(self.calibration)
        self.live["gripper"].update(homing_offset=-208, range_min=0, range_max=4095)
        self.torque = {name: name != "gripper" for name in self.motors}
        self.lock = 1
        self.native_position = 1892  # Present 2100 now; 2765 under the saved offset.
        self.writes = []
        self.fail_register = None
        self.readback_wrong = False
        self.disconnected = False

    def connect(self):
        pass

    def disconnect(self, disable_torque=True):
        if disable_torque:
            raise AssertionError("must preserve other joints' torque")
        self.disconnected = True

    def ping(self, *args, **kwargs):
        return 777

    def read_calibration(self):
        return deepcopy(self.live)

    def read(self, register, motor, **kwargs):
        if register == "Torque_Enable":
            return int(self.torque[motor])
        if register == "Operating_Mode":
            return 0
        if register == "Present_Position":
            return self.native_position - self.live[motor]["homing_offset"]
        if register == "Lock":
            return self.lock
        field = next(key for key, value in REGISTERS.items() if value == register)
        value = self.live[motor][field]
        if self.readback_wrong and self.writes and register == "Max_Position_Limit":
            value -= 1
        return value

    def write(self, register, motor, value, *, normalize, num_retry):
        if motor != "gripper" or register not in (*REGISTERS.values(), "Lock"):
            raise AssertionError(f"forbidden write {register} {motor}")
        if normalize or num_retry:
            raise AssertionError("EEPROM writes must be raw and not retried")
        if self.torque[motor]:
            raise AssertionError("cannot write with torque enabled")
        if register == self.fail_register:
            raise OSError("simulated serial failure")
        self.writes.append((register, motor, value))
        if register == "Lock":
            self.lock = value
        else:
            field = next(key for key, value in REGISTERS.items() if value == register)
            self.live[motor][field] = value

    def sync_read(self, register, **kwargs):
        return {name: self.read(register, name) for name in self.motors}

    def _normalize(self, ids_values):
        return {motor_id: 0.0 for motor_id in ids_values}


class GripperRestoreTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.bus = RestoreBus()
        self.file = Path(self.directory.name) / "follower.json"
        self.file.write_text(json.dumps(self.bus.calibration))
        self.device = SimpleNamespace(
            id="so101_follower", bus=self.bus, calibration_fpath=self.file,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.restore = GripperCalibrationRestore()
        self.stop = threading.Event()

    def test_preview_is_read_only_and_predicts_offset_shift(self):
        preview = self.restore.preview(self.device)
        self.assertEqual((preview["current_raw"], preview["projected_raw"]), (2100, 2765))
        self.assertEqual(preview["proposed"]["homing_offset"], -873)
        self.assertEqual(self.bus.writes, [])

    def test_restore_changes_only_j6_and_preserves_file_and_torque(self):
        others = {key: deepcopy(value) for key, value in self.bus.live.items() if key != "gripper"}
        original_file, torque = self.file.read_bytes(), dict(self.bus.torque)
        preview = self.restore.preview(self.device)
        result = self.restore.apply(self.device, preview["token"], self.stop)
        self.assertEqual(result["after"], self.bus.calibration["gripper"])
        self.assertEqual(result["raw"], 2765)
        self.assertEqual(self.bus.torque, torque)
        self.assertEqual(self.file.read_bytes(), original_file)
        self.assertEqual({key: value for key, value in self.bus.live.items() if key != "gripper"}, others)
        self.assertEqual([write[0] for write in self.bus.writes], [
            "Lock", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit", "Lock",
        ])
        self.assertEqual(self.bus.lock, 1)
        self.assertFalse(self.restore.tainted)
        with self.assertRaisesRegex(ValueError, "unused"):
            self.restore.apply(self.device, preview["token"], self.stop)

    def test_preview_rejects_torque_on_or_projected_position_outside_limits(self):
        self.bus.torque["gripper"] = True
        with self.assertRaisesRegex(ValueError, "already be OFF"):
            self.restore.preview(self.device)
        self.bus.torque["gripper"] = False
        self.bus.native_position = 2863 - 208
        with self.assertRaisesRegex(ValueError, "3528"):
            self.restore.preview(self.device)
        self.assertEqual(self.bus.writes, [])

    def test_restoring_higher_range_writes_maximum_before_minimum(self):
        self.bus.live["gripper"]["range_max"] = 1800
        preview = self.restore.preview(self.device)
        self.restore.apply(self.device, preview["token"], self.stop)
        self.assertEqual([write[0] for write in self.bus.writes], [
            "Lock", "Homing_Offset", "Max_Position_Limit", "Min_Position_Limit", "Lock",
        ])

    def test_confirm_rejects_changed_position_file_registers_or_torque(self):
        mutations = (
            lambda: setattr(self.bus, "native_position", self.bus.native_position + 21),
            lambda: self.file.write_text("{}"),
            lambda: self.bus.live["gripper"].update(range_max=4094),
            lambda: self.bus.torque.update(gripper=True),
        )
        for mutation in mutations:
            with self.subTest(mutation=mutation):
                self.bus = RestoreBus()
                self.device.bus = self.bus
                self.file.write_text(json.dumps(self.bus.calibration))
                preview = self.restore.preview(self.device)
                mutation()
                with self.assertRaises(ValueError):
                    self.restore.apply(self.device, preview["token"], self.stop)
                self.assertEqual(self.bus.writes, [])

    def test_expired_wrong_token_and_stopped_plans_never_write(self):
        preview = self.restore.preview(self.device)
        with self.assertRaisesRegex(ValueError, "unused"):
            self.restore.apply(self.device, "0" * 32, self.stop)
        preview = self.restore.preview(self.device)
        self.restore.plan["created_at"] -= 61
        with self.assertRaisesRegex(ValueError, "expired"):
            self.restore.apply(self.device, preview["token"], self.stop)
        preview = self.restore.preview(self.device)
        self.stop.set()
        with self.assertRaisesRegex(ValueError, "cancelled"):
            self.restore.apply(self.device, preview["token"], self.stop)
        self.assertEqual(self.bus.writes, [])

    def test_partial_failure_is_not_rolled_back_or_retried(self):
        preview = self.restore.preview(self.device)
        self.bus.fail_register = "Min_Position_Limit"
        with self.assertRaisesRegex(RuntimeError, "partially changed"):
            self.restore.apply(self.device, preview["token"], self.stop)
        self.assertTrue(self.restore.tainted)
        self.assertEqual([write[0] for write in self.bus.writes], ["Lock", "Homing_Offset"])
        self.assertFalse(self.bus.torque["gripper"])

    def test_readback_failure_is_not_reported_as_success(self):
        preview = self.restore.preview(self.device)
        self.bus.readback_wrong = True
        with self.assertRaisesRegex(RuntimeError, "verification failed"):
            self.restore.apply(self.device, preview["token"], self.stop)
        self.assertTrue(self.restore.tainted)
        self.assertFalse(self.bus.torque["gripper"])

    def test_stop_between_writes_does_not_continue_or_enable_torque(self):
        preview = self.restore.preview(self.device)
        original_write = self.bus.write

        def write(register, *args, **kwargs):
            original_write(register, *args, **kwargs)
            if register == "Homing_Offset":
                self.stop.set()

        with patch.object(self.bus, "write", side_effect=write):
            with self.assertRaisesRegex(RuntimeError, "interrupted"):
                self.restore.apply(self.device, preview["token"], self.stop)
        self.assertEqual([write[0] for write in self.bus.writes], ["Lock", "Homing_Offset"])
        self.assertTrue(self.restore.tainted)
        self.assertFalse(self.bus.torque["gripper"])

    def test_cli_and_protocol_require_j6_and_explicit_token_confirmation(self):
        preview = parse_pose_command("J6 RESTORE PREVIEW", 1)["message"]
        self.assertEqual(validate_pose_command(preview)["mode"], "calibration_preview")
        apply = parse_pose_command("J6 RESTORE CONFIRM " + "a" * 32, 2)["message"]
        self.assertEqual(validate_pose_command(apply)["confirmation"], "RESTORE J6")
        for change in ({"joint_id": 5}, {"confirmation": ""}, {"token": "invalid"}, {"value": 123}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_pose_command({**apply, **change})

    def test_monitor_refreshes_inspector_after_restore(self):
        commands, messages = queue.Queue(), []
        commands.put(parse_pose_command("J6 RESTORE PREVIEW", 1)["message"])

        def publish(message):
            messages.append(message)
            if message.get("state") == "failed":
                self.stop.set()
            if message.get("calibration_preview"):
                token = message["calibration_preview"]["token"]
                commands.put(parse_pose_command(f"J6 RESTORE CONFIRM {token}", 2)["message"])
            if message["type"] == "pose_sample" and self.bus.writes:
                self.stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            stream_follower_pose("/dev/fake", "so101_follower", self.stop, publish, commands)
        report = messages[-1]["calibration"]["joints"]["gripper.pos"]
        self.assertEqual(report["state"], "consistent")
        self.assertEqual(report["live"], report["saved"])
        self.assertFalse(messages[-1]["torque"]["gripper.pos"])
        self.assertTrue(self.bus.disconnected)

    def test_partial_failure_blocks_subsequent_torque_enable_in_monitor(self):
        self.bus.fail_register = "Min_Position_Limit"
        commands, messages = queue.Queue(), []
        commands.put(parse_pose_command("J6 RESTORE PREVIEW", 1)["message"])

        def publish(message):
            messages.append(message)
            if message.get("calibration_preview"):
                token = message["calibration_preview"]["token"]
                commands.put(parse_pose_command(f"J6 RESTORE CONFIRM {token}", 2)["message"])
            if message.get("command_id") == 2 and message.get("state") == "failed":
                commands.put(parse_pose_command("J6 TE", 3)["message"])
            if message.get("command_id") == 3:
                self.stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            stream_follower_pose("/dev/fake", "so101_follower", self.stop, publish, commands)
        result = next(message for message in messages if message.get("command_id") == 3)
        self.assertIn("restore incomplete", result["message"])
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(all(write[1] == "gripper" for write in self.bus.writes))


if __name__ == "__main__":
    unittest.main()
