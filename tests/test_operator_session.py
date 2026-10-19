import asyncio
import time
import unittest

from cf_robot.protocol import JOINTS

try:
    from cf_robot.operator_session import OperatorSession
except ImportError:
    OperatorSession = None


def pose(value=0.0):
    return {joint: value for joint in JOINTS}


@unittest.skipIf(OperatorSession is None, "install project dependencies including websockets")
class OperatorSessionTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.session = OperatorSession("wss://robot.example/control", None, "leader")
        self.session.socket = object()
        self.session.last_target = pose()
        self.session.status.update(
            state="ready",
            follower_positions=pose(),
            follower_observed_at=time.time(),
        )

    async def test_manual_targets_are_limited_from_the_previous_target(self):
        target = pose()
        target["shoulder_pan.pos"] = 5.01
        with self.assertRaisesRegex(ValueError, "5-unit"):
            await self.session.submit_manual(target)
        self.assertTrue(self.session.manual_queue.empty())

    async def test_manual_targets_use_urdf_joint_bounds(self):
        target = pose()
        target["shoulder_pan.pos"] = 4
        target["gripper.pos"] = 101
        with self.assertRaisesRegex(ValueError, "outside model limits"):
            await self.session.submit_manual(target)
        self.assertTrue(self.session.manual_queue.empty())

    async def test_manual_target_waits_for_socket_send_acknowledgement(self):
        target = pose()
        target["shoulder_pan.pos"] = 1
        submit = asyncio.create_task(self.session.submit_manual(target))
        await asyncio.sleep(0)
        queued_target, acknowledgement = self.session.manual_queue.get_nowait()
        self.assertEqual(queued_target, target)
        self.assertFalse(submit.done())
        acknowledgement.set_result(True)
        await submit

    async def test_leader_mode_requires_connected_and_aligned_arms(self):
        with self.assertRaisesRegex(RuntimeError, "connect the leader"):
            self.session.set_mode("leader")

        self.session.leader = object()
        self.session.status.update(
            leader_positions=pose(),
            aligned=True,
        )
        self.assertEqual(self.session.set_mode("leader")["mode"], "leader")

        self.session.status["follower_observed_at"] = time.time() - 1
        with self.assertRaisesRegex(RuntimeError, "feedback is stale"):
            self.session.set_mode("leader")


if __name__ == "__main__":
    unittest.main()
