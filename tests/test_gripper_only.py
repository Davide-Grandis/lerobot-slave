import json
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

from test_calibration_restore import RestoreBus
from test_control import wait_for

from cf_robot.control import FollowerSession
from cf_robot.gripper_control import GripperFollower, GripperLeader, gripper_control_url
from cf_robot.leader import main
from cf_robot.protocol import GRIPPER_JOINTS, JOINTS, ProtocolError, action_message, validate_command


class GripperBus(RestoreBus):
    def __init__(self):
        super().__init__()
        self.live["gripper"] = deepcopy(self.calibration["gripper"])
        self.live["shoulder_lift"]["homing_offset"] = -1902
        self.speed = 300
        self.fail_enable = False

    def read(self, register, motor, **kwargs):
        if register == "Goal_Velocity":
            return self.speed
        return super().read(register, motor, **kwargs)

    def _normalize(self, values):
        calibration = self.calibration["gripper"]
        return {
            motor_id: (raw - calibration["range_min"]) * 100
            / (calibration["range_max"] - calibration["range_min"])
            for motor_id, raw in values.items()
        }

    def write(self, register, motor, value, *, normalize=True, num_retry=0):
        if motor != "gripper" or register not in ("Goal_Position", "Torque_Enable", "Goal_Velocity"):
            raise AssertionError(f"forbidden write: {register} {motor}")
        self.writes.append((register, motor, value))
        if register == "Goal_Position":
            if normalize:
                cal = self.calibration["gripper"]
                value = round(cal["range_min"] + value * (cal["range_max"] - cal["range_min"]) / 100)
            self.native_position = value + self.live["gripper"]["homing_offset"]
        elif register == "Goal_Velocity":
            self.speed = value
        else:
            self.torque[motor] = bool(value)
            if value and self.fail_enable:
                raise OSError("uncertain torque-enable acknowledgement")


class GripperFixture:
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.bus = GripperBus()
        self.file = Path(self.directory.name) / "arm.json"
        self.file.write_text(json.dumps(self.bus.calibration))
        self.device = SimpleNamespace(
            id="so101_follower", bus=self.bus, calibration_fpath=self.file,
            config=SimpleNamespace(port="/dev/fake"),
        )


class GripperOnlyTests(GripperFixture, unittest.TestCase):
    def test_j6_only_cli_flag_selects_isolated_mode(self):
        with patch("sys.argv", [
            "cf-robot-leader", "--url", "wss://robot.example/control",
            "--port", "/dev/fake", "--arm-id", "leader", "--J6-only",
        ]), patch("cf_robot.leader.run_teleop", new_callable=AsyncMock) as run:
            main()
        self.assertTrue(run.call_args.kwargs["gripper_only"])
        self.assertEqual(self.bus.writes, [])

    def test_connect_and_leader_reads_never_write_any_motor(self):
        leader = GripperLeader(self.device)
        leader.connect()
        self.assertEqual(set(leader.get_action()), set(GRIPPER_JOINTS))
        leader.disconnect()
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.torque["shoulder_lift"])

    def test_only_j6_is_enabled_commanded_and_released(self):
        original = deepcopy(self.bus.live)
        torque = dict(self.bus.torque)
        robot = GripperFollower(self.device)
        robot.connect()
        self.assertEqual(self.bus.writes, [])
        target = robot.get_observation()["gripper.pos"] + 1
        robot.send_action({"gripper.pos": target})
        self.assertTrue(self.bus.torque["gripper"])
        self.assertEqual(self.bus.speed, 102)
        robot.disconnect()
        self.assertEqual(self.bus.live, original)
        self.assertEqual(self.bus.torque, torque)
        self.assertEqual(self.bus.speed, 300)
        self.assertTrue(all(write[1] == "gripper" for write in self.bus.writes))
        self.assertEqual([write[0] for write in self.bus.writes[:3]], [
            "Goal_Position", "Goal_Velocity", "Torque_Enable",
        ])

    def test_uncertain_enable_is_cleaned_up_without_other_joint_writes(self):
        robot = GripperFollower(self.device)
        robot.connect()
        self.bus.fail_enable = True
        with self.assertRaises(OSError):
            robot.send_action(robot.get_observation())
        robot.disconnect()
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(self.bus.torque["shoulder_lift"])

    def test_mismatched_j6_or_enabled_torque_refuses_start_without_writes(self):
        for field, value in (("homing_offset", -208), ("range_min", 0)):
            self.bus.live["gripper"][field] = value
            robot = GripperFollower(self.device)
            with self.assertRaisesRegex(ValueError, "consistent"):
                robot.connect()
            robot.disconnect()
            self.bus.live["gripper"] = deepcopy(self.bus.calibration["gripper"])
        self.bus.torque["gripper"] = True
        leader = GripperLeader(self.device)
        with self.assertRaisesRegex(ValueError, "already be OFF"):
            leader.connect()
        leader.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_no_initial_arming_for_unaligned_or_non_j6_target(self):
        robot = GripperFollower(self.device)
        robot.connect()
        with self.assertRaises(ValueError):
            robot.send_action({"gripper.pos": 80})
        with self.assertRaises(ValueError):
            robot.send_action({joint: 50 for joint in JOINTS})
        robot.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_watchdog_releases_only_j6(self):
        robot = GripperFollower(self.device)
        session = FollowerSession(lambda: robot, joints=GRIPPER_JOINTS, watchdog=0.1)
        session.start()
        try:
            self.assertTrue(session.wait_ready(1))
            session.submit(dict(session.initial_observation))
            self.assertTrue(wait_for(lambda: self.bus.disconnected))
            self.assertIn("timed out", session.error)
            self.assertFalse(self.bus.torque["gripper"])
            self.assertTrue(self.bus.torque["shoulder_lift"])
            self.assertTrue(all(write[1] == "gripper" for write in self.bus.writes))
        finally:
            session.stop()

    def test_protocol_rejects_other_joints_and_out_of_range_percent(self):
        message = action_message(1, {"gripper.pos": 50}, GRIPPER_JOINTS)
        self.assertEqual(validate_command(message, 0, joints=GRIPPER_JOINTS)[1], {"gripper.pos": 50})
        with self.assertRaises(ProtocolError):
            validate_command(message, 0)
        for values in ({"gripper.pos": -1}, {"gripper.pos": 101}, {joint: 50 for joint in JOINTS}):
            with self.subTest(values=values), self.assertRaises(ProtocolError):
                action_message(1, values, GRIPPER_JOINTS)
        self.assertEqual(gripper_control_url("wss://robot.example/control"), "wss://robot.example/control/gripper")


try:
    from fastapi.testclient import TestClient
    from starlette.websockets import WebSocketDisconnect

    from cf_robot.follower import create_app
except ImportError:
    TestClient = None


@unittest.skipIf(TestClient is None, "install follower test dependencies")
class GripperRouteTests(GripperFixture, unittest.TestCase):
    def test_authentication_and_shared_control_lock(self):
        factory = Mock(return_value=self.device)
        app = create_app("/dev/fake", "so101_follower", factory=factory, origin_key="t" * 64)
        with TestClient(app) as client:
            with self.assertRaises(WebSocketDisconnect):
                with client.websocket_connect("/control/gripper"):
                    pass
            factory.assert_not_called()
            with client.websocket_connect("/control/gripper", headers={"x-cf-robot-key": "t" * 64}) as socket:
                self.assertEqual(socket.receive_json()["mode"], "gripper-only")
                with self.assertRaises(WebSocketDisconnect):
                    with client.websocket_connect("/control", headers={"x-cf-robot-key": "t" * 64}) as other:
                        other.receive_json()
            factory.assert_called_once()
        self.assertEqual(self.bus.writes, [])

    def test_route_acknowledges_isolation_and_rejects_all_arm_frames(self):
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with TestClient(app) as client:
            with client.websocket_connect("/control/gripper", headers={"x-cf-robot-key": "t" * 64}) as socket:
                ready = socket.receive_json()
                self.assertEqual(ready["joints"], ["gripper.pos"])
                self.assertEqual(ready["mode"], "gripper-only")
                self.assertEqual(self.bus.writes, [])
                socket.send_json(action_message(1, {joint: 0 for joint in JOINTS}))
                while True:
                    result = socket.receive_json()
                    if result["type"] == "error":
                        break
                self.assertIn("only gripper.pos", result["message"])
            self.assertEqual(self.bus.writes, [])

    def test_route_powers_only_j6_and_releases_it_on_close(self):
        app = create_app("/dev/fake", "so101_follower", factory=lambda: self.device, origin_key="t" * 64)
        with TestClient(app) as client:
            with client.websocket_connect("/control/gripper", headers={"x-cf-robot-key": "t" * 64}) as socket:
                ready = socket.receive_json()
                socket.send_json(action_message(1, ready["observation"], GRIPPER_JOINTS))
                self.assertTrue(wait_for(lambda: self.bus.torque["gripper"]))
            self.assertTrue(wait_for(lambda: self.bus.disconnected))
        self.assertFalse(self.bus.torque["gripper"])
        self.assertTrue(self.bus.torque["shoulder_lift"])
        self.assertTrue(all(write[1] == "gripper" for write in self.bus.writes))
