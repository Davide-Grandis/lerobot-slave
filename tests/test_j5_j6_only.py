from io import StringIO
import time
import unittest
from unittest.mock import AsyncMock, patch

from test_follower_j5 import FollowerJ5Fixture

from cf_robot.angular_install import AngularInstall
from cf_robot.gripper_control import wrist_gripper_control_url
from cf_robot.leader import main
from cf_robot.protocol import WRIST_GRIPPER_JOINTS, action_message, validate_action
from cf_robot.teleop_hardware import TeleopFollower

try:
    from fastapi.testclient import TestClient

    from cf_robot.follower import create_app
    from test_control import wait_for
except ImportError:
    TestClient = None


class PairedFixture(FollowerJ5Fixture):
    def setUp(self):
        super().setUp()
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        self.reload()
        for name in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"):
            self.bus.torque[name] = True
        self.bus.torque["gripper"] = False


class PairedHardwareTests(PairedFixture, unittest.TestCase):
    def test_only_pair_is_commanded_and_stop_holds_wrist_releases_gripper(self):
        robot = TeleopFollower(self.device, WRIST_GRIPPER_JOINTS)
        robot.connect()
        self.assertEqual(self.bus.writes, [])
        robot.send_action(robot.get_observation())
        writes = self.bus.writes
        first_enable = next(i for i, entry in enumerate(writes) if entry[0] == "Torque_Enable")
        self.assertEqual(
            {entry[1] for entry in writes[:first_enable] if entry[0] == "Goal_Position"},
            {"wrist_roll", "gripper"},
        )
        robot.send_action({"wrist_roll.pos": 1, "gripper.pos": 40})
        robot.hold_position()
        robot.disconnect()
        self.assertTrue(all(entry[1] in ("wrist_roll", "gripper") for entry in self.bus.writes))
        self.assertTrue(self.bus.torque["wrist_roll"])
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(all(self.bus.torque[name] for name in (
            "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex",
        )))

    def test_gripper_must_be_off_but_supporting_wrist_can_already_hold(self):
        self.bus.torque["gripper"] = True
        robot = TeleopFollower(self.device, WRIST_GRIPPER_JOINTS)
        try:
            with self.assertRaisesRegex(ValueError, "J6.*torque must already be OFF"):
                robot.connect()
        finally:
            robot.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_unselected_target_is_rejected_before_arming(self):
        robot = TeleopFollower(self.device, WRIST_GRIPPER_JOINTS)
        robot.connect()
        with self.assertRaisesRegex(ValueError, "exactly"):
            robot.send_action({**robot.get_observation(), "shoulder_pan.pos": 0})
        robot.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_wrist_read_failure_during_stop_still_releases_gripper(self):
        robot = TeleopFollower(self.device, WRIST_GRIPPER_JOINTS)
        robot.connect()
        robot.send_action(robot.get_observation())
        original = self.bus.read

        def read(register, motor, **kwargs):
            if register == "Present_Position" and motor == "wrist_roll":
                raise OSError("simulated wrist read failure")
            return original(register, motor, **kwargs)

        with patch.object(self.bus, "read", side_effect=read):
            with self.assertRaisesRegex(RuntimeError, "cleanup incomplete"):
                robot.disconnect()
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(self.bus.torque["wrist_roll"])


class PairedCliTests(unittest.TestCase):
    def test_flag_uses_shared_sync_and_wait_ready_reporting(self):
        async def run(*args, on_status, **kwargs):
            on_status({"type": "status", "state": "waiting for startup sync"})
            on_status({"type": "status", "state": "startup sync complete"})
            on_status({"type": "status", "state": "running"})

        args = [
            "cf-robot-leader", "--url", "wss://robot.example/control", "--port", "/dev/fake",
            "--arm-id", "so101_leader", "--J5-J6-only",
        ]
        output = StringIO()
        with patch("sys.argv", args), patch("sys.stdout", output), patch(
            "cf_robot.leader.run_teleop", new=AsyncMock(side_effect=run),
        ) as runner:
            main()
        self.assertTrue(runner.call_args.kwargs["wrist_gripper_only"])
        self.assertTrue(runner.call_args.kwargs["sync_at_start"])
        self.assertEqual(output.getvalue().splitlines(), [
            "WAIT - syncing J5-J6; hold the leader still.", "Ready for teleop",
        ])
        with patch("sys.argv", [*args, "--J5-only"]), patch("sys.stderr", StringIO()):
            with self.assertRaises(SystemExit):
                main()

    def test_pair_url_and_mixed_units(self):
        self.assertEqual(
            wrist_gripper_control_url("wss://robot.example/control"),
            "wss://robot.example/control/joints/5-6",
        )
        self.assertEqual(
            validate_action({"wrist_roll.pos": -90, "gripper.pos": 50}, WRIST_GRIPPER_JOINTS),
            {"wrist_roll.pos": -90, "gripper.pos": 50},
        )
        with self.assertRaises(ValueError):
            validate_action({"wrist_roll.pos": -90, "gripper.pos": 101}, WRIST_GRIPPER_JOINTS)


@unittest.skipIf(TestClient is None, "install follower test dependencies")
class PairedEndpointTests(PairedFixture, unittest.TestCase):
    def test_route_confirms_pair_and_never_changes_j1_through_j4(self):
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with TestClient(app) as client:
            with client.websocket_connect("/control/joints/5-6", headers={"x-cf-robot-key": "t" * 64}) as socket:
                ready = socket.receive_json()
                self.assertEqual(ready["mode"], "joint-only")
                self.assertEqual(ready["joints"], list(WRIST_GRIPPER_JOINTS))
                self.assertEqual(self.bus.writes, [])
                socket.send_json(action_message(1, ready["observation"], WRIST_GRIPPER_JOINTS))
                self.assertTrue(wait_for(lambda: self.bus.torque["gripper"]))
            self.assertTrue(wait_for(lambda: not self.bus.torque["gripper"]))
        self.assertTrue(all(entry[1] in ("wrist_roll", "gripper") for entry in self.bus.writes))
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(self.bus.torque["wrist_roll"])

    def test_pair_route_rejects_extra_joint_without_any_goal_or_torque_writes(self):
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with TestClient(app) as client:
            with client.websocket_connect("/control/joints/5-6", headers={"x-cf-robot-key": "t" * 64}) as socket:
                ready = socket.receive_json()
                socket.send_json({
                    "version": 1, "type": "action", "seq": 1, "sent_at": time.time(),
                    "action": {**ready["observation"], "shoulder_pan.pos": 0},
                })
                response = socket.receive_json()
                self.assertEqual(response["type"], "error")
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
