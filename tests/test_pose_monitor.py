import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from cf_robot.pose_compare_client import is_follower_sample, parse_pose_command
from cf_robot.protocol import JOINTS
from cf_robot.pose_monitor import (
    execute_pose_command,
    prepare_bus_torque,
    stream_follower_pose,
    stream_leader_pose,
)
from cf_robot.servo_maintenance import read_servo_limits


class FakeBus:
    def __init__(self):
        self.connected = False
        self.disconnect_torque_args = []
        self.writes = []
        self.limits = {
            joint.removesuffix(".pos"): {"min": 100, "max": 3900}
            for joint in JOINTS
        }
        self.present = {
            joint.removesuffix(".pos"): 2000
            for joint in JOINTS
        }
        self.torque = {
            joint.removesuffix(".pos"): True
            for joint in JOINTS
        }
        self.speeds = {
            joint.removesuffix(".pos"): 300
            for joint in JOINTS
        }
        self.motors = {
            joint.removesuffix(".pos"): SimpleNamespace(id=index + 1)
            for index, joint in enumerate(JOINTS)
        }

    def connect(self):
        self.connected = True

    def disconnect(self, disable_torque=True):
        self.disconnect_torque_args.append(disable_torque)
        self.connected = False

    def sync_read(self, register, *, normalize=True, num_retry=0):
        if register == "Torque_Enable":
            return dict(self.torque)
        if register != "Present_Position":
            raise AssertionError(f"unexpected register read: {register}")
        if normalize:
            return {
                motor: self._normalize_motor(motor, value)
                for motor, value in self.present.items()
            }
        return dict(self.present)

    def read(self, register, motor, *, normalize=True, num_retry=0):
        if register == "Min_Position_Limit":
            return self.limits[motor]["min"]
        if register == "Max_Position_Limit":
            return self.limits[motor]["max"]
        if register == "Present_Position":
            value = self.present[motor]
            return self._normalize_motor(motor, value) if normalize else value
        if register == "Torque_Enable":
            return int(self.torque[motor])
        if register == "Goal_Velocity":
            return self.speeds[motor]
        raise AssertionError(f"unexpected read: {register}")

    def write(self, *args, **kwargs):
        self.writes.append((args, kwargs))
        register, motor, value = args
        if register == "Torque_Enable":
            self.torque[motor] = bool(value)
        elif register == "Goal_Position":
            self.present[motor] = int(value)
        elif register == "Goal_Velocity":
            self.speeds[motor] = int(value)
        else:
            raise AssertionError(f"unexpected write: {register}")

    def _normalize_motor(self, motor, raw):
        limits = self.limits[motor]
        if motor == "gripper":
            return (raw - limits["min"]) * 100 / (limits["max"] - limits["min"])
        midpoint = (limits["min"] + limits["max"]) / 2
        return (raw - midpoint) * 360 / 4095

    def _normalize(self, ids_values):
        result = {}
        for motor, details in self.motors.items():
            if details.id in ids_values:
                result[details.id] = self._normalize_motor(motor, ids_values[details.id])
        return result

    def _unnormalize(self, ids_values):
        result = {}
        for motor, details in self.motors.items():
            if details.id not in ids_values:
                continue
            limits = self.limits[motor]
            value = ids_values[details.id]
            if motor == "gripper":
                raw = limits["min"] + value * (limits["max"] - limits["min"]) / 100
            else:
                midpoint = (limits["min"] + limits["max"]) / 2
                raw = value * 4095 / 360 + midpoint
            result[details.id] = int(raw)
        return result


class FakeArm:
    def __init__(self):
        self.bus = FakeBus()

    def get_action(self):
        return {
            joint: self.bus._normalize_motor(
                joint.removesuffix(".pos"),
                self.bus.present[joint.removesuffix(".pos")],
            )
            for joint in JOINTS
        }


class PoseMonitorTests(unittest.TestCase):
    def _check_stream(self, stream, factory_path):
        arm = FakeArm()
        stop = threading.Event()
        messages = []

        def publish(message):
            messages.append(message)
            if message["type"].endswith("_sample"):
                stop.set()

        if stream is stream_leader_pose:
            call_args = ("/dev/fake", "so101_leader", stop, publish)
        else:
            call_args = ("/dev/fake", "so101_follower", stop, publish)
        with patch(factory_path, return_value=arm):
            stream(*call_args)

        sample = next(message for message in messages if message["type"].endswith("_sample"))
        torque = next(message for message in messages if message["type"].endswith("_torque"))
        self.assertAlmostEqual(sample["values"]["shoulder_pan.pos"], 0.0)
        self.assertEqual(sample["raw"]["shoulder_pan.pos"], 2000)
        self.assertEqual(sample["limits"]["shoulder_pan.pos"], {"min": 100, "max": 3900})
        self.assertTrue(sample["torque"]["gripper.pos"])
        self.assertTrue(torque["before"]["gripper.pos"])
        self.assertEqual(torque["before"], torque["after"])
        self.assertEqual(arm.bus.disconnect_torque_args, [False])
        self.assertEqual(arm.bus.writes, [])

    def test_leader_stream_reads_action_without_configuring_or_writing(self):
        self._check_stream(stream_leader_pose, "cf_robot.pose_monitor.make_leader")

    def test_follower_stream_reads_observation_without_configuring_or_writing(self):
        self._check_stream(stream_follower_pose, "cf_robot.pose_monitor.make_follower")

    def test_release_mode_disables_only_torque_that_was_enabled(self):
        arm = FakeArm()
        arm.bus.torque["gripper"] = False
        messages = []

        prepare_bus_torque(arm.bus, "follower", True, messages.append)

        self.assertEqual(
            arm.bus.writes,
            [(("Torque_Enable", "shoulder_pan", 0), {"normalize": False, "num_retry": 2}),
             (("Torque_Enable", "shoulder_lift", 0), {"normalize": False, "num_retry": 2}),
             (("Torque_Enable", "elbow_flex", 0), {"normalize": False, "num_retry": 2}),
             (("Torque_Enable", "wrist_flex", 0), {"normalize": False, "num_retry": 2}),
             (("Torque_Enable", "wrist_roll", 0), {"normalize": False, "num_retry": 2})],
        )
        self.assertTrue(messages[0]["before"]["shoulder_pan.pos"])
        self.assertFalse(messages[0]["after"]["shoulder_pan.pos"])
        self.assertFalse(messages[0]["before"]["gripper.pos"])

    def test_client_accepts_only_follower_samples(self):
        self.assertTrue(
            is_follower_sample(
                {"type": "pose_sample", "side": "follower"}
            )
        )
        self.assertFalse(
            is_follower_sample(
                {"type": "pose_sample", "side": "leader"}
            )
        )

    def test_cli_parses_requested_tick_degree_gripper_and_torque_syntax(self):
        self.assertEqual(
            parse_pose_command("J1 T823", 1)["message"]["value"],
            823,
        )
        self.assertEqual(
            parse_pose_command("J3 D-120.34", 2)["message"]["value"],
            -120.34,
        )
        self.assertEqual(
            parse_pose_command("J6 %95.20", 3)["message"]["mode"],
            "percent",
        )
        self.assertTrue(
            parse_pose_command("J1 TE", 4)["message"]["enabled"]
        )
        self.assertFalse(
            parse_pose_command("J5 TD", 5)["message"]["enabled"]
        )
        with self.assertRaises(ValueError):
            parse_pose_command("J6 D10", 6)

    def test_manual_joint_command_uses_one_joint_and_restores_speed(self):
        robot = FakeArm()
        limits = read_servo_limits(robot.bus)
        results = []
        command = {
            "version": 1,
            "type": "command",
            "command_id": 1,
            "joint_id": 1,
            "mode": "ticks",
            "value": 2010,
        }
        with patch("cf_robot.pose_monitor.time.sleep", return_value=None):
            execute_pose_command(
                robot,
                limits,
                command,
                threading.Event(),
                results.append,
            )
        self.assertEqual(results[-1]["state"], "succeeded")
        self.assertEqual(robot.bus.present["shoulder_pan"], 2010)
        position_writes = [
            write for write in robot.bus.writes if write[0][0] == "Goal_Position"
        ]
        self.assertTrue(position_writes)
        self.assertTrue(all(write[0][1] == "shoulder_pan" for write in position_writes))

    def test_enable_torque_holds_present_position_before_enabling(self):
        robot = FakeArm()
        robot.bus.torque["wrist_roll"] = False
        results = []
        command = {
            "version": 1,
            "type": "command",
            "command_id": 6,
            "joint_id": 5,
            "mode": "torque",
            "enabled": True,
        }
        execute_pose_command(
            robot,
            read_servo_limits(robot.bus),
            command,
            threading.Event(),
            results.append,
        )
        writes = [entry[0] for entry in robot.bus.writes]
        self.assertEqual(
            writes,
            [
                ("Goal_Position", "wrist_roll", 2000),
                ("Torque_Enable", "wrist_roll", 1),
            ],
        )
        self.assertTrue(results[-1]["torque_enabled"])


if __name__ == "__main__":
    unittest.main()
