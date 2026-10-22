import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_encoder_rebase import RebaseBus

from cf_robot.angular_calibration import attach_angular_calibration
from cf_robot.encoder_rebase import EncoderRebase
from cf_robot.pose_monitor import stream_follower_pose, validate_pose_command


class FollowerRebaseTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "so101_follower.json"
        self.bus = RebaseBus()
        self.bus.live["shoulder_lift"].update(
            homing_offset=-1902, range_min=0, range_max=4095,
        )
        self.bus.raw["shoulder_lift"] = 787
        self.path.write_text(json.dumps(self.bus.calibration))
        self.profile_path = self.path.with_suffix(".cf-robot.json")
        self.profile_path.write_text(json.dumps({
            "version": 1, "side": "follower", "arm_id": "so101_follower",
            "joints": {},
        }))
        self.device = SimpleNamespace(
            id="so101_follower", bus=self.bus, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.command = {
            "version": 1, "type": "command", "command_id": 1,
            "joint_id": 2, "mode": "rebase_preview",
            "old_start": 3803, "new_start": 500,
            "provisional_max": 4095, "expected_offset": -1902,
        }

    def preview(self, manager):
        return manager.preview(
            self.device, 2, 3803, None, 500, -1902, provisional_max=4095,
        )

    def test_follower_offset_wrap_preserves_physical_coordinate(self):
        manager = EncoderRebase("follower")
        preview = self.preview(manager)
        self.assertEqual(preview["after"]["homing_offset"], 1401)
        self.assertEqual(preview["tick_shift"], 793)
        self.assertEqual(preview["projected_raw"], 1580)
        self.assertEqual(self.bus.writes, [])
        before = self.bus.read_calibration()
        saved_before = json.loads(self.path.read_bytes())
        result = manager.apply(self.device, preview["token"], threading.Event())
        self.assertEqual(result["raw"], 1580)
        self.assertEqual(result["after"]["range_min"], 500)
        self.assertEqual(result["after"]["range_max"], 4095)
        for name in before:
            if name != "shoulder_lift":
                self.assertEqual(self.bus.live[name], before[name])
                self.assertEqual(json.loads(self.path.read_bytes())[name], saved_before[name])
        self.assertTrue(all(row[1] == "shoulder_lift" for row in self.bus.writes))
        self.assertFalse(any(self.bus.torque.values()))
        self.bus.calibration = json.loads(self.path.read_bytes())
        attach_angular_calibration(self.device, "follower")
        self.bus.connect()
        with self.assertRaisesRegex(ValueError, "provisional"):
            self.bus._normalize({2: 1580})
        with self.assertRaisesRegex(ValueError, "provisional"):
            self.bus.write("Goal_Position", "shoulder_lift", 1580, normalize=False)

    def test_rebase_protocol_requires_exact_fields_and_confirmation(self):
        self.assertEqual(validate_pose_command(self.command), self.command)
        for update in (
            {"joint_id": 6}, {"old_start": True},
            {"new_start": -1}, {"provisional_max": 4096},
            {"expected_offset": -2048}, {"unexpected": 1},
        ):
            with self.subTest(update=update), self.assertRaises(ValueError):
                validate_pose_command({**self.command, **update})
        apply = {
            "version": 1, "type": "command", "command_id": 2,
            "joint_id": 2, "mode": "rebase_apply", "token": "a" * 32,
            "confirmation": "REBASE FOLLOWER J2",
        }
        self.assertEqual(validate_pose_command(apply), apply)
        with self.assertRaises(ValueError):
            validate_pose_command({**apply, "confirmation": "REBASE FOLLOWER J1"})

    def test_monitor_previews_confirms_then_disconnects_without_goals(self):
        commands = queue.Queue()
        commands.put(self.command)
        stop = threading.Event()
        messages = []

        def publish(message):
            messages.append(message)
            if message.get("rebase_preview"):
                commands.put(validate_pose_command({
                    "version": 1, "type": "command", "command_id": 2,
                    "joint_id": 2, "mode": "rebase_apply",
                    "token": message["rebase_preview"]["token"],
                    "confirmation": "REBASE FOLLOWER J2",
                }))
            if len(messages) > 10:
                stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            stream_follower_pose("/dev/fake", "so101_follower", stop, publish, commands)
        results = [m for m in messages if m.get("type") == "command_result"]
        self.assertEqual([m["state"] for m in results], ["preview", "succeeded"])
        self.assertEqual(results[1]["encoder_rebase"]["after"]["homing_offset"], 1401)
        self.assertTrue(stop.is_set())
        self.assertFalse(any(row[0] == "Goal_Position" for row in self.bus.writes))

    def test_torque_command_invalidates_rebase_preview(self):
        commands = queue.Queue()
        commands.put(self.command)
        stop = threading.Event()
        messages = []
        token = None

        def publish(message):
            nonlocal token
            messages.append(message)
            if message.get("rebase_preview"):
                token = message["rebase_preview"]["token"]
                commands.put({
                    "version": 1, "type": "command", "command_id": 2,
                    "joint_id": 2, "mode": "torque", "enabled": False,
                })
            elif message.get("command_id") == 2 and message.get("type") == "command_result":
                commands.put({
                    "version": 1, "type": "command", "command_id": 3,
                    "joint_id": 2, "mode": "rebase_apply", "token": token,
                    "confirmation": "REBASE FOLLOWER J2",
                })
            if len(messages) > 15:
                stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            stream_follower_pose("/dev/fake", "so101_follower", stop, publish, commands)
        result = next(m for m in messages if m.get("command_id") == 3)
        self.assertEqual(result["state"], "failed")
        self.assertFalse(any(row[0] == "Homing_Offset" for row in self.bus.writes))
