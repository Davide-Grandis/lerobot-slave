import json
from io import StringIO
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from test_angular_calibration import AngularBus

from cf_robot import angular_install
from cf_robot.angular_calibration import attach_angular_calibration
from cf_robot.angular_install import AngularInstall
from cf_robot.leader import main
from cf_robot.pose_compare_client import InspectionReports, parse_pose_command
from cf_robot.pose_monitor import stream_follower_pose, validate_pose_command
from cf_robot.teleop_hardware import TeleopFollower


class FollowerJ5Fixture:
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.bus = AngularBus()
        self.bus.protocol_version = 0
        self.bus.calibration["wrist_roll"].update(
            homing_offset=-1214, range_min=0, range_max=4095,
        )
        self.bus.calibration["gripper"].update(
            homing_offset=-873, range_min=2038, range_max=3488,
        )
        self.bus.live = json.loads(json.dumps(self.bus.calibration))
        self.bus.live["wrist_roll"]["homing_offset"] = -1164
        self.bus.live["shoulder_pan"]["homing_offset"] = 10
        self.bus.raw["wrist_roll"] = 2025
        self.bus.raw["gripper"] = 2600
        self.bus.torque["gripper"] = True
        self.path = Path(self.directory.name) / "so101_follower.json"
        self.path.write_text(json.dumps(self.bus.calibration))
        self.device = SimpleNamespace(
            id="so101_follower", bus=self.bus, calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )
        self.command = parse_pose_command("J5 ANGULAR PREVIEW 130 3946 1005 3044", 1)["message"]
        self.stop = threading.Event()

    def apply(self, manager, preview):
        return manager.apply(self.device, {
            "joint_id": 5, "token": preview["token"], "confirmation": "APPLY ANGULAR AND LIMITS J5",
        }, self.stop)

    def reload(self):
        self.bus.calibration = json.loads(self.path.read_bytes())
        self.bus.writes.clear()
        attach_angular_calibration(self.device, "follower")


class FollowerJ5InstallTests(FollowerJ5Fixture, unittest.TestCase):
    def test_preview_uses_current_live_offset_and_known_angle_references(self):
        manager = AngularInstall()
        before = self.path.read_bytes()
        preview = manager.preview(self.device, validate_pose_command(self.command))
        self.assertEqual(preview["angular_profile"]["zero_ticks"], 2024.5)
        self.assertAlmostEqual(preview["angular_profile"]["degrees_per_tick"], 180 / 2039)
        self.assertEqual(preview["saved_sdk_before"]["homing_offset"], -1214)
        self.assertEqual(preview["saved_sdk_after"]["homing_offset"], -1164)
        self.assertAlmostEqual(preview["angular_range"][0], -167.24, places=2)
        self.assertAlmostEqual(preview["angular_range"][1], 169.63, places=2)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(self.bus.writes, [])

    def test_apply_persists_only_j5_files_and_leaves_j6_and_live_motors_unchanged(self):
        manager = AngularInstall()
        before = json.loads(self.path.read_bytes())
        live = self.bus.read_calibration()
        torque = dict(self.bus.torque)
        preview = manager.preview(self.device, self.command)
        result = self.apply(manager, preview)
        after = json.loads(self.path.read_bytes())
        for name in before:
            if name != "wrist_roll":
                self.assertEqual(after[name], before[name])
        self.assertEqual(after["wrist_roll"]["homing_offset"], -1164)
        actual = self.bus.read_calibration()
        for name in live:
            if name != "wrist_roll":
                self.assertEqual(actual[name], live[name])
        self.assertEqual(actual["wrist_roll"]["homing_offset"], -1164)
        self.assertEqual(actual["wrist_roll"]["range_min"], 130)
        self.assertEqual(actual["wrist_roll"]["range_max"], 3946)
        self.assertEqual(self.bus.torque, torque)
        self.assertTrue(all(
            name == "wrist_roll" and register in ("Lock", "Min_Position_Limit", "Max_Position_Limit")
            for register, name, _ in self.bus.writes
        ))
        saved_profile = json.loads(self.path.with_suffix(".cf-robot.json").read_bytes())
        self.assertNotIn("range_min", saved_profile["joints"]["wrist_roll"])
        self.assertNotIn("range_max", saved_profile["joints"]["wrist_roll"])
        self.assertTrue(result["restart_required"])
        self.assertTrue(all(Path(path).is_file() for path in result["backups"]))
        self.reload()
        self.bus.connect()
        self.assertEqual(self.bus._normalize({5: 3044})[5], 90)
        self.assertEqual(self.bus._normalize({5: 1005})[5], -90)

    def test_existing_other_angular_joint_is_preserved(self):
        other = {
            "id": 1, "model": "sts3215", "homing_offset": 0,
            "range_min": 200, "range_max": 3800, "zero_ticks": 2000,
            "degrees_per_tick": 360 / 4095,
        }
        profile = {"version": 1, "side": "follower", "arm_id": "so101_follower",
                   "joints": {"shoulder_pan": other}}
        self.path.with_suffix(".cf-robot.json").write_text(json.dumps(profile))
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        saved = json.loads(self.path.with_suffix(".cf-robot.json").read_bytes())
        self.assertEqual(saved["joints"]["shoulder_pan"], other)

    def test_torque_on_or_changed_files_refuse_installation(self):
        manager = AngularInstall()
        self.bus.torque["wrist_roll"] = True
        with self.assertRaisesRegex(ValueError, "must already be OFF"):
            manager.preview(self.device, self.command)
        self.bus.torque["wrist_roll"] = False
        preview = manager.preview(self.device, self.command)
        self.path.write_text(self.path.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "changed since preview"):
            self.apply(manager, preview)
        self.assertFalse(self.path.with_suffix(".cf-robot.json").exists())
        self.assertEqual(self.bus.writes, [])

    def test_partial_file_write_fails_closed_after_verified_limit_writes(self):
        manager = AngularInstall()
        preview = manager.preview(self.device, self.command)
        original = angular_install._atomic_write

        def write(path, *args):
            if path == self.path:
                raise OSError("simulated SDK file failure")
            original(path, *args)

        with patch("cf_robot.angular_install._atomic_write", side_effect=write):
            with self.assertRaisesRegex(RuntimeError, "update incomplete"):
                self.apply(manager, preview)
        attach_angular_calibration(self.device, "follower")
        self.bus.connect()
        with self.assertRaisesRegex(ValueError, "saved homing offset"):
            self.bus._normalize({5: 2025})
        self.assertTrue(all(
            name == "wrist_roll" and register in ("Lock", "Min_Position_Limit", "Max_Position_Limit")
            for register, name, _ in self.bus.writes
        ))

    def test_cli_preview_confirmation_is_joint_bound_and_retained(self):
        manager = AngularInstall()
        preview = manager.preview(self.device, self.command)
        reports = InspectionReports()
        reports.retain_restore({
            "mode": "angular_preview", "joint_id": 5, "state": "preview",
            "angular_preview": preview,
        })
        message = validate_pose_command(reports.confirm_angular(2, 5))
        self.assertEqual(message["confirmation"], "APPLY ANGULAR AND LIMITS J5")
        with self.assertRaises(ValueError):
            reports.confirm_angular(3, 5)
        self.assertEqual(parse_pose_command("J5 ANGULAR CONFIRM", 3)["joint_id"], 5)

    def test_angular_protocol_never_accepts_j6_or_extra_fields(self):
        for change in (
            {"joint_id": 6}, {"positive_ticks": True}, {"range_max": "3946"},
            {"path": "/tmp/other"}, {"write_hardware_limits": False},
        ):
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_pose_command({**self.command, **change})
        file_only = dict(self.command)
        del file_only["write_hardware_limits"]
        with self.assertRaises(ValueError):
            validate_pose_command(file_only)

    def test_monitor_ends_after_install_so_old_angles_cannot_continue(self):
        commands, messages = queue.Queue(), []
        commands.put(self.command)

        def publish(message):
            messages.append(message)
            if message.get("angular_preview"):
                commands.put({
                    "version": 1, "type": "command", "command_id": 2,
                    "joint_id": 5, "mode": "angular_apply",
                    "token": message["angular_preview"]["token"], "confirmation": "APPLY ANGULAR AND LIMITS J5",
                })

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            stream_follower_pose("/dev/fake", "so101_follower", self.stop, publish, commands)
        self.assertTrue(self.stop.is_set())
        self.assertEqual(messages[-1]["state"], "succeeded")
        self.assertTrue(messages[-1]["angular_install"]["restart_required"])
        self.assertTrue(all(
            name == "wrist_roll" and register in ("Lock", "Min_Position_Limit", "Max_Position_Limit")
            for register, name, _ in self.bus.writes
        ))


class J5OnlyTests(FollowerJ5Fixture, unittest.TestCase):
    def test_cli_flag_and_mutual_exclusion(self):
        args = ["cf-robot-leader", "--url", "wss://robot.example/control",
                "--port", "/dev/fake", "--arm-id", "so101_leader"]
        with patch("sys.argv", [*args, "--J5-only"]), patch(
            "cf_robot.leader.run_teleop", new_callable=AsyncMock,
        ) as run:
            main()
        self.assertEqual(run.call_args.kwargs["joint_only"], 5)
        self.assertFalse(run.call_args.kwargs["gripper_only"])
        with patch("sys.argv", [*args, "--J5-only", "--J6-only"]), patch("sys.stderr", StringIO()):
            with self.assertRaises(SystemExit) as stopped:
                main()
        self.assertEqual(stopped.exception.code, 2)

    def test_j5_only_adapter_holds_wrist_on_stop_and_never_changes_j6(self):
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        self.reload()
        original_torque = dict(self.bus.torque)
        robot = TeleopFollower(self.device, ("wrist_roll.pos",))
        robot.connect()
        self.assertEqual(self.bus.writes, [])
        robot.send_action(robot.get_observation())
        robot.send_action({"wrist_roll.pos": 1})
        robot.disconnect()
        self.assertTrue(all(write[1] == "wrist_roll" for write in self.bus.writes))
        for name in original_torque:
            if name != "wrist_roll":
                self.assertEqual(self.bus.torque[name], original_torque[name])
        self.assertTrue(self.bus.torque["wrist_roll"])
        before_reconnect = len(self.bus.writes)
        again = TeleopFollower(self.device, ("wrist_roll.pos",))
        again.connect()
        self.assertEqual(len(self.bus.writes), before_reconnect)
        again.send_action(again.get_observation())
        again.disconnect()
        self.assertFalse(any(
            register == "Torque_Enable" and value == 0
            for register, _, value in self.bus.writes
        ))


try:
    from fastapi.testclient import TestClient

    from cf_robot.follower import create_app
    from cf_robot.protocol import action_message
    from test_control import wait_for
except ImportError:
    TestClient = None


@unittest.skipIf(TestClient is None, "install follower test dependencies")
class J5EndpointTests(FollowerJ5Fixture, unittest.TestCase):
    def test_j5_route_acknowledges_only_j5_and_preserves_other_motor_states(self):
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        self.reload()
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with TestClient(app) as client:
            with client.websocket_connect("/control/joint/5", headers={"x-cf-robot-key": "t" * 64}) as socket:
                ready = socket.receive_json()
                self.assertEqual(ready["mode"], "joint-only")
                self.assertEqual(ready["joints"], ["wrist_roll.pos"])
                self.assertEqual(self.bus.writes, [])
                socket.send_json(action_message(1, ready["observation"], ("wrist_roll.pos",)))
                self.assertTrue(wait_for(lambda: self.bus.torque["wrist_roll"]))
        self.assertTrue(all(write[1] == "wrist_roll" for write in self.bus.writes))
        self.assertTrue(self.bus.torque["gripper"])

    def test_angular_monitor_result_is_delivered_before_monitor_finished(self):
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with patch("cf_robot.pose_monitor.make_follower", return_value=self.device):
            with TestClient(app) as client:
                with client.websocket_connect("/monitor", headers={"x-cf-robot-key": "t" * 64}) as socket:
                    socket.send_json({"version": 1, "type": "start"})
                    self.assertEqual(socket.receive_json()["type"], "monitor_started")
                    socket.send_json(self.command)
                    while True:
                        preview = socket.receive_json()
                        if preview.get("angular_preview"):
                            break
                    socket.send_json({
                        "version": 1, "type": "command", "command_id": 2, "joint_id": 5,
                        "mode": "angular_apply", "token": preview["angular_preview"]["token"],
                        "confirmation": "APPLY ANGULAR AND LIMITS J5",
                    })
                    results = []
                    while True:
                        message = socket.receive_json()
                        results.append(message)
                        if message["type"] == "monitor_finished":
                            break
                    self.assertTrue(any(message.get("angular_install") for message in results))
        self.assertTrue(all(
            name == "wrist_roll" and register in ("Lock", "Min_Position_Limit", "Max_Position_Limit")
            for register, name, _ in self.bus.writes
        ))


if __name__ == "__main__":
    unittest.main()
