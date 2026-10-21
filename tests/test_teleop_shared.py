import json
import tempfile
import unittest
from copy import deepcopy
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, patch

from test_control import wait_for

from cf_robot.control import FollowerSession
from cf_robot.gripper_control import GripperFollower
from cf_robot.leader import main
from cf_robot.protocol import GRIPPER_JOINTS, JOINTS
from cf_robot.teleop_hardware import TeleopFollower, TeleopLeader
from cf_robot.teleop_motion import BoundedMotion
from cf_robot.teleop_runtime import Feedback, StartupSync


class FullBus:
    model_number_table = {"sts3215": 777}
    model_resolution_table = {"sts3215": 4096}
    apply_drive_mode = True

    def __init__(self):
        self.motors = {
            joint.removesuffix(".pos"): SimpleNamespace(
                id=index, model="sts3215",
                norm_mode="range_0_100" if index == 6 else "degrees",
            ) for index, joint in enumerate(JOINTS, 1)
        }
        self.calibration = {
            name: {"id": motor.id, "drive_mode": 0, "homing_offset": 0,
                   "range_min": 100, "range_max": 3900}
            for name, motor in self.motors.items()
        }
        self.live = deepcopy(self.calibration)
        self.raw = dict.fromkeys(self.motors, 2000)
        self.torque = dict.fromkeys(self.motors, False)
        self.speeds = dict.fromkeys(self.motors, 300)
        self.writes = []
        self.fail_hold = None

    def connect(self):
        pass

    def disconnect(self, disable_torque=True):
        if disable_torque:
            raise AssertionError("must not release the arm through SDK disconnect")

    def ping(self, *args, **kwargs):
        return 777

    def read_calibration(self):
        return deepcopy(self.live)

    def read(self, register, motor, **kwargs):
        if register == "Present_Position":
            if self.fail_hold == motor:
                raise OSError("read failed")
            return self.raw[motor]
        if register == "Torque_Enable":
            return self.torque[motor]
        if register == "Goal_Velocity":
            return self.speeds[motor]
        if register == "Operating_Mode":
            return 0
        field = {"Min_Position_Limit": "range_min", "Max_Position_Limit": "range_max"}[register]
        return self.live[motor][field]

    def sync_read(self, register, **kwargs):
        return {name: self.read(register, name) for name in self.motors}

    def _normalize(self, values):
        result = {}
        for name, motor in self.motors.items():
            if motor.id in values:
                raw = values[motor.id]
                result[motor.id] = (raw - 100) * 100 / 3800 if name == "gripper" else (raw - 2000) * 360 / 4095
        return result

    def write(self, register, motor, value, normalize=True, **kwargs):
        if register not in ("Goal_Position", "Goal_Velocity", "Torque_Enable"):
            raise AssertionError("teleoperation must not change calibration/EEPROM")
        self.writes.append((register, motor, value))
        if register == "Goal_Position":
            if normalize:
                value = 100 + value * 38 if motor == "gripper" else 2000 + value * 4095 / 360
            self.raw[motor] = round(value)
        elif register == "Goal_Velocity":
            self.speeds[motor] = value
        else:
            self.torque[motor] = bool(value)


class SharedHardwareTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.bus = FullBus()
        self.path = Path(self.directory.name) / "calibration.json"
        self.path.write_text(json.dumps(self.bus.calibration))
        self.device = SimpleNamespace(
            bus=self.bus, id="arm", calibration_fpath=self.path,
            config=SimpleNamespace(port="/dev/fake"),
        )

    def test_full_and_gripper_use_same_motion_policy(self):
        full = TeleopFollower(self.device)
        single = GripperFollower(self.device)
        self.assertIsInstance(full.motion, BoundedMotion)
        self.assertIsInstance(single.motion, BoundedMotion)

    def test_full_preflight_rejects_any_joint_mismatch_before_writes(self):
        for name in self.bus.motors:
            self.bus.live[name]["homing_offset"] = 20
            robot = TeleopFollower(self.device)
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, "calibration must be consistent"):
                robot.connect()
            robot.disconnect()
            self.bus.live = deepcopy(self.bus.calibration)
        self.assertEqual(self.bus.writes, [])

    def test_full_arm_goals_are_bounded_and_stop_holds_instead_of_dropping(self):
        robot = TeleopFollower(self.device)
        original_file = self.path.read_bytes()
        robot.connect()
        initial = robot.get_observation()
        self.assertEqual(self.bus.writes, [])
        robot.send_action(initial)
        self.assertTrue(all(self.bus.torque.values()))
        target = {joint: value + 20 for joint, value in initial.items()}
        measured = robot.get_observation()
        robot.send_action(target)
        for joint, applied in robot.motion.last_goals.items():
            self.assertLessEqual(abs(applied - measured[joint]), 5)
        self.assertTrue(all(register == "Goal_Position" for register, _, _ in self.bus.writes[:6]))
        robot.disconnect()
        self.assertTrue(all(self.bus.torque.values()))
        self.assertFalse(any(register == "Torque_Enable" and value == 0 for register, _, value in self.bus.writes))
        self.assertTrue(all(speed == 300 for speed in self.bus.speeds.values()))
        self.assertEqual(self.path.read_bytes(), original_file)

    def test_full_leader_reads_without_any_writes(self):
        leader = TeleopLeader(self.device)
        leader.connect()
        self.assertEqual(set(leader.get_action()), set(JOINTS))
        leader.disconnect()
        self.assertEqual(self.bus.writes, [])

    def test_full_follower_rejects_target_outside_saved_degree_range_before_writes(self):
        robot = TeleopFollower(self.device)
        robot.connect()
        target = robot.get_observation()
        target["shoulder_pan.pos"] = 200
        with self.assertRaisesRegex(ValueError, "outside saved range"):
            robot.send_action(target)
        self.assertEqual(self.bus.writes, [])
        robot.disconnect()

    def test_failed_hold_does_not_restore_fast_speed_on_uncertain_joint(self):
        robot = TeleopFollower(self.device)
        robot.connect()
        robot.send_action(robot.get_observation())
        self.bus.fail_hold = "elbow_flex"
        with self.assertRaisesRegex(RuntimeError, "cleanup incomplete"):
            robot.disconnect()
        self.assertEqual(self.bus.speeds["elbow_flex"], 102)
        self.assertTrue(self.bus.torque["elbow_flex"])

    def test_startup_seed_uses_current_measurement_not_old_ready_snapshot(self):
        robot = TeleopFollower(self.device)
        session = FollowerSession(lambda: robot, watchdog=1)
        session.start()
        try:
            self.assertTrue(session.wait_ready(1))
            current = dict(session.initial_observation)
            self.bus.raw["shoulder_pan"] += 228
            current["shoulder_pan.pos"] = 228 * 360 / 4095
            session.submit(current)
            self.assertTrue(wait_for(lambda: robot.motion.last_goals is not None))
            self.assertIsNone(session.error)
        finally:
            session.stop()


class SharedRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0

    def feedback(self, joints, values):
        feedback = Feedback(joints, clock=lambda: self.now, wall_clock=lambda: 100 + self.now)
        self.update(feedback, values)
        return feedback

    def update(self, feedback, values):
        return feedback.accept({"observation": dict(values), "observed_at": 100 + self.now})

    def test_full_and_single_sync_use_same_seed_and_fresh_settling_policy(self):
        for joints in (JOINTS, GRIPPER_JOINTS):
            self.now = 0.0
            actual = dict.fromkeys(joints, 20.0)
            target = dict.fromkeys(joints, 27.0)
            feedback = self.feedback(joints, actual)
            plan = StartupSync(target, joints, clock=lambda: self.now)
            self.assertEqual(plan.advance(feedback), actual)
            for _ in range(100):
                self.now += 0.1
                goal = plan.advance(feedback)
                if plan.complete:
                    break
                changed = [joint for joint in joints if goal[joint] != actual[joint]]
                self.assertLessEqual(len(changed), 1)
                self.assertTrue(all(abs(goal[joint] - actual[joint]) <= 2.5 for joint in joints))
                actual.update(goal)
                self.update(feedback, actual)
            self.assertTrue(plan.complete)
            self.assertEqual(actual, target)

    def test_full_sync_holds_bounded_goal_on_feedback_gap_then_stops(self):
        actual = dict.fromkeys(JOINTS, 20.0)
        feedback = self.feedback(JOINTS, actual)
        plan = StartupSync(dict.fromkeys(JOINTS, 50), clock=lambda: self.now)
        seed = plan.advance(feedback)
        self.now = 0.6
        self.assertEqual(plan.advance(feedback), seed)
        self.assertEqual(plan.phase, "waiting_feedback")
        self.now = 1.1
        with self.assertRaisesRegex(RuntimeError, "last status packet="):
            plan.advance(feedback)

    def test_ready_requires_every_joint_to_remain_aligned_at_completion(self):
        actual = dict.fromkeys(JOINTS, 20.0)
        feedback = self.feedback(JOINTS, actual)
        plan = StartupSync(actual, clock=lambda: self.now)
        plan.advance(feedback)
        for _ in range(30):
            self.now += 0.1
            self.update(feedback, actual)
            plan.advance(feedback)
            if plan.index == 5 and plan.final_generation is not None:
                break
        actual["shoulder_pan.pos"] = 15
        self.now += 0.1
        self.update(feedback, actual)
        goal = plan.advance(feedback)
        self.assertFalse(plan.complete)
        self.assertEqual(plan.active_joint, "shoulder_pan.pos")
        self.assertEqual(goal["shoulder_pan.pos"], 17.5)

    def test_duplicate_timestamp_does_not_refresh_full_arm_feedback(self):
        actual = dict.fromkeys(JOINTS, 20)
        feedback = self.feedback(JOINTS, actual)
        self.now = 0.4
        self.assertFalse(feedback.accept({"observation": actual, "observed_at": 100.0}))
        self.assertEqual(feedback["generation"], 1)
        self.now = 0.6
        self.assertFalse(feedback.fresh)

    def test_full_progress_failure_identifies_degree_joint(self):
        policy = BoundedMotion(clock=lambda: self.now)
        measured = dict.fromkeys(JOINTS, 20)
        policy.committed(policy.plan(measured, measured))
        desired = {**measured, "elbow_flex.pos": 40}
        self.now = 0.1
        policy.committed(policy.plan(desired, measured))
        self.now = 1.2
        with self.assertRaisesRegex(RuntimeError, "J3.*degrees.*no measured progress"):
            policy.plan(desired, measured)

    def test_impulse_then_stationary_leader_allows_slow_catch_up_in_all_modes(self):
        for joints in (JOINTS, GRIPPER_JOINTS):
            with self.subTest(joints=joints):
                self.now = 0.0
                motion = BoundedMotion(joints, clock=lambda: self.now)
                measured = dict.fromkeys(joints, 20.0)
                motion.committed(motion.plan(measured, measured))
                # A manual impulse changes the leader by 60 units at once.
                # It then stays still while the much slower follower catches up.
                held_leader = dict.fromkeys(joints, 80.0)
                for _ in range(240):
                    self.now += 0.1
                    applied = motion.plan(held_leader, measured)
                    self.assertTrue(all(
                        abs(applied[joint] - measured[joint]) <= 5.0
                        for joint in joints
                    ))
                    motion.committed(applied)
                    for joint in joints:
                        measured[joint] += max(-0.5, min(0.5, applied[joint] - measured[joint]))
                self.assertGreater(self.now, 10)
                self.assertEqual(measured, held_leader)
                self.assertEqual(motion.last_goals, held_leader)

    def test_new_leader_position_replaces_impulse_without_replaying_backlog(self):
        for joints in (JOINTS, GRIPPER_JOINTS):
            with self.subTest(joints=joints):
                self.now = 0.0
                motion = BoundedMotion(joints, clock=lambda: self.now)
                measured = dict.fromkeys(joints, 40.0)
                motion.committed(motion.plan(measured, measured))
                motion.committed(motion.plan(dict.fromkeys(joints, 90.0), measured))
                self.now += 0.1
                latest_leader = dict.fromkeys(joints, 25.0)
                applied = motion.plan(latest_leader, measured)
                self.assertEqual(applied, dict.fromkeys(joints, 35.0))

    def test_normal_cli_uses_wait_and_ready_messages_without_joint_only_flag(self):
        async def run(*args, on_status, **kwargs):
            for state in (
                "waiting for startup sync", "ready", "synchronizing shoulder_pan.pos",
                "waiting for follower feedback", "startup sync complete", "running", "ready",
            ):
                on_status({"type": "status", "state": state})

        output = StringIO()
        with patch("sys.argv", [
            "cf-robot-leader", "--url", "wss://robot.example/control",
            "--port", "/dev/fake", "--arm-id", "leader",
        ]), patch("cf_robot.leader.run_teleop", new=AsyncMock(side_effect=run)), patch(
            "sys.stdout", output,
        ):
            main()
        lines = output.getvalue().splitlines()
        self.assertTrue(lines[0].startswith("WAIT"))
        self.assertTrue(lines[1].startswith("WAIT"))
        self.assertIn("holding the last selected-joint sync goal", lines[2])
        self.assertEqual(lines[3], "Ready for teleop")
        self.assertEqual(len(lines), 4)


if __name__ == "__main__":
    unittest.main()
