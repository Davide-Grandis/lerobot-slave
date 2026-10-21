import asyncio
import contextlib
import json
import time
import unittest
from io import StringIO
from unittest.mock import AsyncMock, Mock, patch

from test_gripper_only import GripperFixture

from cf_robot.leader import main, run_teleop
from cf_robot.protocol import GRIPPER_JOINTS, VERSION, validate_command

try:
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed
except ImportError:
    serve = None


@unittest.skipIf(serve is None, "install websockets")
class GripperClientTests(GripperFixture, unittest.IsolatedAsyncioTestCase):
    async def exercise(
        self, follower_percent=50, acknowledge=True, confirm="",
        stop_after=2, stale_after_one=False, sync=True, stalled=False,
        stale_during_sync=False, cancel_sync=False, change_leader=False, cancel_after=None,
    ):
        stop = asyncio.Event()
        accepted, paths = [], []
        self.accepted = accepted
        self.states = []
        self.live_index = None
        state = {"measured": follower_percent, "goal": follower_percent}
        self.feedback_at_running = None

        def on_status(message):
            name = message.get("state")
            if name:
                self.states.append(name)
            if name == "running":
                self.live_index = len(accepted)
                self.feedback_at_running = state["measured"]
            if name == "synchronizing gripper.pos" and cancel_sync:
                stop.set()

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
                    if not stalled:
                        state["measured"] += max(-1, min(1, state["goal"] - state["measured"]))
                    stale = accepted and (
                        stale_during_sync or (stale_after_one and self.live_index is not None)
                    )
                    await socket.send(json.dumps({
                        "type": "status", "state": "ready",
                        "observation": {"gripper.pos": state["measured"]},
                        "observed_at": time.time() - (5 if stale else 0),
                    }))
                    await asyncio.sleep(0.02)

            task = asyncio.create_task(statuses())
            seq = 0
            try:
                async for raw in socket:
                    seq, action = validate_command(json.loads(raw), seq, joints=GRIPPER_JOINTS)
                    accepted.append(action)
                    state["goal"] = action["gripper.pos"]
                    if change_leader and len(accepted) == 1:
                        self.bus.native_position += 145
                    if cancel_after is not None and len(accepted) >= cancel_after:
                        stop.set()
                    if (
                        stop_after is not None and self.live_index is not None
                        and len(accepted) - self.live_index >= stop_after
                    ):
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
                    sync_at_start=sync, gripper_only=True, on_status=on_status,
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

    async def test_disabled_startup_sync_still_requires_manual_alignment(self):
        with self.assertRaisesRegex(RuntimeError, "no startup movement"):
            await self.exercise(follower_percent=20, sync=False)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.disconnected)

    async def test_unaligned_gripper_syncs_before_live_following(self):
        accepted, _, _ = await self.exercise(follower_percent=20)
        self.assertEqual(accepted[0], {"gripper.pos": 20.0})
        self.assertIn("J6 startup sync complete", self.states)
        self.assertLessEqual(abs(self.feedback_at_running - 50.14), 1)
        self.assertGreater(self.live_index, 2)
        startup = accepted[:self.live_index]
        self.assertTrue(all(set(action) == {"gripper.pos"} for action in startup))
        for previous, current in zip(startup, startup[1:]):
            self.assertLessEqual(abs(current["gripper.pos"] - previous["gripper.pos"]), 2.5 + 1e-9)
        self.assertEqual(self.bus.writes, [])

    async def test_sync_holds_leader_snapshot_until_settled(self):
        accepted, _, _ = await self.exercise(follower_percent=20, change_leader=True)
        self.assertLessEqual(abs(self.feedback_at_running - 50.14), 1)
        self.assertLessEqual(max(action["gripper.pos"] for action in accepted[:self.live_index]), 50.14)
        self.assertGreater(accepted[self.live_index]["gripper.pos"], 60)

    async def test_startup_sync_can_close_toward_leader(self):
        accepted, _, _ = await self.exercise(follower_percent=80)
        self.assertEqual(accepted[0], {"gripper.pos": 80.0})
        self.assertLessEqual(abs(self.feedback_at_running - 50.14), 1)
        startup = accepted[:self.live_index]
        self.assertTrue(all(
            -2.5 - 1e-9 <= current["gripper.pos"] - previous["gripper.pos"] <= 0
            for previous, current in zip(startup, startup[1:])
        ))

    async def test_stalled_startup_sync_never_enters_live_following(self):
        with patch("cf_robot.leader.STARTUP_SYNC_SETTLE_TIMEOUT_S", 0.2):
            with self.assertRaisesRegex(RuntimeError, "startup sync timed out"):
                await self.exercise(follower_percent=20, stalled=True)
        self.assertNotIn("running", self.states)
        self.assertEqual(self.bus.writes, [])

    async def test_cancelled_sync_sends_no_goals(self):
        await self.exercise(follower_percent=20, cancel_sync=True)
        self.assertEqual(self.accepted, [])
        self.assertNotIn("running", self.states)

    async def test_cancel_during_ramp_never_enters_live_following(self):
        await self.exercise(follower_percent=20, cancel_after=3)
        self.assertEqual(len(self.accepted), 3)
        self.assertNotIn("running", self.states)
        self.assertTrue(self.bus.disconnected)

    async def test_stale_feedback_during_sync_stops_before_live_following(self):
        with self.assertRaisesRegex(RuntimeError, "readback lost during startup sync"):
            await self.exercise(follower_percent=20, stale_during_sync=True)
        self.assertNotIn("running", self.states)

    async def test_cancelled_confirmation_never_streams(self):
        with self.assertRaisesRegex(RuntimeError, "cancelled without arming"):
            await self.exercise(confirm="cancel")
        self.assertEqual(self.bus.writes, [])

    async def test_stale_measured_feedback_stops_an_active_stream(self):
        with self.assertRaisesRegex(RuntimeError, "readback lost"):
            await self.exercise(stop_after=None, stale_after_one=True)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.disconnected)


class GripperCliStatusTests(unittest.TestCase):
    def test_wait_and_ready_messages_distinguish_hardware_ready_from_live_following(self):
        async def run(*args, on_status, **kwargs):
            on_status({"type": "status", "state": "ready"})
            on_status({"type": "status", "state": "waiting for J6 startup sync"})
            on_status({"type": "status", "state": "ready"})
            on_status({"type": "status", "state": "synchronizing gripper.pos"})
            on_status({"type": "gripper_sample", "leader_percent": 50, "follower_percent": 20})
            on_status({"type": "status", "state": "J6 startup sync complete"})
            on_status({"type": "status", "state": "ready"})
            on_status({"type": "status", "state": "running"})
            on_status({"type": "gripper_sample", "leader_percent": 50, "follower_percent": 50})
            on_status({"type": "status", "state": "ready"})

        output = StringIO()
        with patch("sys.argv", [
            "cf-robot-leader", "--url", "wss://robot.example/control",
            "--port", "/dev/fake", "--arm-id", "leader", "--J6-only",
        ]), patch("cf_robot.leader.run_teleop", new=AsyncMock(side_effect=run)), patch(
            "sys.stdout", output,
        ):
            main()
        lines = output.getvalue().splitlines()
        self.assertEqual(lines[0], "WAIT - syncing J6; hold the leader still.")
        self.assertTrue(lines[1].startswith("WAIT - J6 leader="))
        self.assertEqual(lines[2], "Ready for teleop")
        self.assertTrue(lines[3].startswith("J6 leader="))
        self.assertEqual(len(lines), 4)


if __name__ == "__main__":
    unittest.main()
