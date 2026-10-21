import asyncio
import contextlib
import json
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
        self.session.feedback.accept({
            "observation": pose(), "observed_at": time.time(),
        })

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
        with self.assertRaisesRegex(ValueError, "gripper target"):
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
        self.assertTrue(self.session.sync_pending)

        self.session.sync_pending = False
        self.session.mode = "manual"
        self.session.status["follower_observed_at"] = time.time() - 1
        with self.assertRaisesRegex(RuntimeError, "feedback is stale"):
            self.session.set_mode("leader")

    async def test_unaligned_leader_mode_requires_explicit_sync_confirmation(self):
        self.session.leader = object()
        self.session.status["leader_positions"] = pose(6)
        with self.assertRaisesRegex(RuntimeError, "confirm startup sync"):
            self.session.set_mode("leader")
        result = self.session.set_mode("leader", confirm_sync=True)
        self.assertEqual(result["state"], "synchronizing")
        self.assertTrue(result["teleop_message"].startswith("WAIT"))
        with self.assertRaisesRegex(RuntimeError, "stop the control session"):
            self.session.set_mode("manual")

    async def test_browser_leader_sync_uses_shared_sequential_ramp_before_ready(self):
        session = self.session
        desired = pose(6)
        session.leader = type("Leader", (), {"get_action": lambda self: dict(desired)})()
        session.status["leader_positions"] = dict(desired)
        messages, targets = [], []

        def publish(status):
            messages.append(dict(status))
            if status.get("teleop_message") == "Ready for teleop":
                session.stop_event.set()

        class Socket:
            async def send(self, raw):
                target = json.loads(raw)["action"]
                targets.append(target)
                session.feedback.accept({"observation": target, "observed_at": time.time()})

        session.on_status = publish
        session.set_mode("leader", confirm_sync=True)
        reader = asyncio.create_task(asyncio.Event().wait())
        try:
            await asyncio.wait_for(session._send_loop(Socket(), reader), 4)
        finally:
            reader.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await reader
        self.assertTrue(targets)
        self.assertEqual(targets[0], pose())
        self.assertEqual(targets[-1], desired)
        self.assertTrue(any(status.get("state") == "synchronizing" for status in messages))
        self.assertEqual(messages[-1]["teleop_message"], "Ready for teleop")
        for previous, current in zip(targets, targets[1:]):
            changed = [joint for joint in JOINTS if previous[joint] != current[joint]]
            self.assertLessEqual(len(changed), 1)
            self.assertTrue(all(abs(current[joint] - previous[joint]) <= 2.5 for joint in JOINTS))

    async def test_remote_hardware_ready_does_not_override_browser_wait_phase(self):
        self.session.status.update(state="synchronizing", teleop_message="WAIT - syncing")

        class Socket:
            def __aiter__(self):
                return self.messages()

            async def messages(self):
                yield json.dumps({
                    "type": "status", "state": "ready",
                    "observation": pose(), "observed_at": time.time(),
                })

        with self.assertRaises(ConnectionError):
            await self.session._read_status(Socket())
        self.assertEqual(self.session.status["state"], "synchronizing")
        self.assertEqual(self.session.status["teleop_message"], "WAIT - syncing")


if __name__ == "__main__":
    unittest.main()
