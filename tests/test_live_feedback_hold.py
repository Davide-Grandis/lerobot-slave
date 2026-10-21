import asyncio
import contextlib
import json
import time
import unittest

from test_control import FakeFollower, wait_for
from test_gripper_only import GripperFixture

from cf_robot.control import FollowerSession
from cf_robot.gripper_control import GripperFollower
from cf_robot.protocol import (
    GRIPPER_JOINTS, JOINTS, ProtocolError, hold_message, validate_command,
)
from cf_robot.teleop_runtime import Feedback, recover_live_feedback


class HoldProtocolTests(unittest.TestCase):
    def test_hold_is_explicit_capability_and_has_no_remote_target(self):
        message = hold_message(2)
        with self.assertRaises(ProtocolError):
            validate_command(message, 0, now=message["sent_at"])
        self.assertEqual(
            validate_command(message, 0, now=message["sent_at"], allow_hold=True), (2, None),
        )
        for change in ({"action": {"gripper.pos": 100}}, {"joints": list(JOINTS)}):
            with self.subTest(change=change), self.assertRaises(ProtocolError):
                validate_command(
                    {**message, **change}, 0, now=message["sent_at"], allow_hold=True,
                )
        for last_seq, age in ((2, 0), (0, 0.6), (0, -0.3)):
            with self.subTest(last_seq=last_seq, age=age), self.assertRaises(ProtocolError):
                validate_command(
                    message, last_seq, now=message["sent_at"] + age, allow_hold=True,
                )

    def test_unarmed_hold_reads_but_does_not_arm_or_send_goals(self):
        robot = FakeFollower()
        session = FollowerSession(lambda: robot)
        session.start()
        try:
            self.assertTrue(session.wait_ready(1))
            robot.observation = dict.fromkeys(JOINTS, 2)
            session.submit_hold()
            self.assertTrue(wait_for(lambda: session.observation == robot.observation))
            self.assertEqual(robot.actions, [])
        finally:
            session.stop()

    def test_armed_hold_uses_pi_measurement_and_watchdog_stays_active(self):
        robot = FakeFollower()
        session = FollowerSession(lambda: robot, watchdog=0.1)
        session.start()
        try:
            self.assertTrue(session.wait_ready(1))
            session.submit(dict(session.initial_observation))
            self.assertTrue(wait_for(lambda: len(robot.actions) == 1))
            measured = dict.fromkeys(JOINTS, 2)
            robot.observation = dict(measured)
            session.submit_hold()
            self.assertTrue(wait_for(lambda: len(robot.actions) == 2))
            self.assertEqual(robot.actions[-1], measured)
            self.assertTrue(wait_for(lambda: session.error is not None))
            self.assertIn("timed out", session.error)
        finally:
            session.stop()


class GripperLocalHoldTests(GripperFixture, unittest.TestCase):
    def test_local_hold_only_commands_selected_gripper(self):
        robot = GripperFollower(self.device)
        session = FollowerSession(lambda: robot, joints=GRIPPER_JOINTS, watchdog=1)
        session.start()
        try:
            self.assertTrue(session.wait_ready(1))
            session.submit(dict(session.initial_observation))
            self.assertTrue(wait_for(lambda: robot.last_goal is not None))
            previous_goal = robot.last_goal
            self.bus.native_position += 10
            expected = self.bus._normalize({
                6: self.bus.native_position - self.bus.live["gripper"]["homing_offset"],
            })[6]
            before = len(self.bus.writes)
            session.submit_hold()
            self.assertTrue(wait_for(lambda: robot.last_goal != previous_goal))
            self.assertAlmostEqual(robot.last_goal, expected)
            self.assertTrue(all(write[1] == "gripper" for write in self.bus.writes))
            self.assertTrue(self.bus.torque["shoulder_lift"])
            self.assertTrue(all(write[0] == "Goal_Position" for write in self.bus.writes[before:]))
        finally:
            session.stop()


class LiveFeedbackRecoveryTests(unittest.IsolatedAsyncioTestCase):
    async def exercise(self, joints, age=0.513, recover_after=2, supports_hold=True, cancel=False):
        feedback = Feedback(joints)
        feedback.accept({"observation": dict.fromkeys(joints, 20), "observed_at": time.time()})
        feedback["received_at"] = time.monotonic() - age
        reader = asyncio.create_task(asyncio.Event().wait())
        stop = asyncio.Event()
        sent, states = [], []
        self.sent, self.states = sent, states

        class Socket:
            async def send(self, raw):
                message = json.loads(raw)
                sent.append(message)
                if cancel:
                    stop.set()
                if recover_after is not None and len(sent) >= recover_after:
                    feedback.accept({
                        "observation": dict.fromkeys(joints, 21),
                        "observed_at": time.time(),
                    })

        try:
            result = await asyncio.wait_for(recover_live_feedback(
                Socket(), reader, feedback, 10, stop, states.append, supports_hold,
            ), 2)
            return result, sent, states
        finally:
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader

    async def test_brief_gap_uses_local_hold_in_full_and_joint_only_modes(self):
        for joints in (JOINTS, GRIPPER_JOINTS):
            result, sent, states = await self.exercise(joints)
            self.assertEqual(result, (12, True))
            self.assertTrue(all(message["type"] == "hold" for message in sent))
            self.assertTrue(all("action" not in message for message in sent))
            self.assertEqual([message["seq"] for message in sent], [11, 12])
            self.assertEqual(states[0]["state"], "waiting for live feedback")
            self.assertEqual(states[-1]["state"], "running")

    async def test_persistent_gap_still_stops_at_one_second(self):
        with self.assertRaisesRegex(RuntimeError, "during live feedback hold"):
            await self.exercise(JOINTS, age=0.95, recover_after=None)
        self.assertTrue(self.sent)
        self.assertFalse(any(state.get("state") == "running" for state in self.states))

    async def test_old_pi_fails_closed_instead_of_replaying_a_far_target(self):
        with self.assertRaisesRegex(RuntimeError, "does not advertise safe hold support"):
            await self.exercise(GRIPPER_JOINTS, supports_hold=False)
        self.assertEqual(self.sent, [])

    async def test_cancelled_hold_does_not_resume(self):
        result, sent, states = await self.exercise(JOINTS, recover_after=None, cancel=True)
        self.assertEqual(result, (11, True))
        self.assertEqual(len(sent), 1)
        self.assertFalse(any(state.get("state") == "running" for state in states))

    async def test_already_arrived_status_can_be_processed_before_timeout(self):
        feedback = Feedback()
        feedback["received_at"] = time.monotonic() - 0.513

        async def queued_reader():
            feedback.accept({
                "observation": dict.fromkeys(JOINTS, 20), "observed_at": time.time(),
            })
            await asyncio.Event().wait()

        reader = asyncio.create_task(queued_reader())
        try:
            result = await recover_live_feedback(
                None, reader, feedback, 10, asyncio.Event(), lambda message: None,
                supports_hold=False,
            )
            self.assertEqual(result, (10, False))
        finally:
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader


if __name__ == "__main__":
    unittest.main()
