import asyncio
import contextlib
import json
import time
import unittest
from unittest.mock import Mock, patch

from test_gripper_only import GripperFixture

from cf_robot.leader import run_teleop
from cf_robot.protocol import GRIPPER_JOINTS, VERSION, validate_command

try:
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed
except ImportError:
    serve = None


@unittest.skipIf(serve is None, "install websockets")
class GripperClientTests(GripperFixture, unittest.IsolatedAsyncioTestCase):
    async def exercise(
        self, follower_percent=50, acknowledge=True, confirm="GRIPPER",
        stop_after=2, stale_after_one=False,
    ):
        stop = asyncio.Event()
        accepted, paths = [], []

        async def follower(socket):
            paths.append(socket.request.path)
            await socket.send(json.dumps({
                "type": "ready", "version": VERSION,
                "mode": "gripper-only" if acknowledge else "normal",
                "joints": ["gripper.pos"],
                "observation": {"gripper.pos": follower_percent},
            }))

            async def statuses():
                while True:
                    await socket.send(json.dumps({
                        "type": "status", "state": "ready",
                        "observation": {"gripper.pos": follower_percent},
                        "observed_at": time.time() - (5 if stale_after_one and accepted else 0),
                    }))
                    await asyncio.sleep(0.02)

            task = asyncio.create_task(statuses())
            seq = 0
            try:
                async for raw in socket:
                    seq, action = validate_command(json.loads(raw), seq, joints=GRIPPER_JOINTS)
                    accepted.append(action)
                    if stop_after is not None and len(accepted) >= stop_after:
                        stop.set()
            except ConnectionClosed:
                pass
            finally:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):
                    await task

        factory = Mock(return_value=self.device)
        with patch("cf_robot.leader.access_headers", return_value={}), patch(
            "builtins.input", return_value=confirm,
        ):
            async with serve(follower, "127.0.0.1", 0) as server:
                port = server.sockets[0].getsockname()[1]
                await asyncio.wait_for(run_teleop(
                    f"ws://127.0.0.1:{port}/control", "/dev/fake", "leader",
                    stop=stop, leader_factory=factory,
                    sync_at_start=True, gripper_only=True,
                ), 3)
        return accepted, paths, factory

    async def test_client_sends_only_gripper_and_never_configures_leader(self):
        accepted, paths, factory = await self.exercise()
        self.assertGreaterEqual(len(accepted), 2)
        self.assertEqual(paths, ["/control/gripper"])
        self.assertTrue(all(set(action) == {"gripper.pos"} for action in accepted))
        factory.assert_called_once()
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.disconnected)

    async def test_missing_isolation_acknowledgement_refuses_stream(self):
        with self.assertRaisesRegex(RuntimeError, "confirm J6-only isolation"):
            await self.exercise(acknowledge=False)
        self.assertEqual(self.bus.writes, [])
        self.assertFalse(self.bus.disconnected)  # Local bus was never opened.

    async def test_unaligned_grippers_never_trigger_startup_sync(self):
        with self.assertRaisesRegex(RuntimeError, "no startup movement"):
            await self.exercise(follower_percent=20)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.disconnected)

    async def test_cancelled_confirmation_never_streams(self):
        with self.assertRaisesRegex(RuntimeError, "cancelled without arming"):
            await self.exercise(confirm="")
        self.assertEqual(self.bus.writes, [])

    async def test_stale_measured_feedback_stops_an_active_stream(self):
        with self.assertRaisesRegex(RuntimeError, "readback lost"):
            await self.exercise(stop_after=None, stale_after_one=True)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.disconnected)


if __name__ == "__main__":
    unittest.main()
