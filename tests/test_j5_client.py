import asyncio
import contextlib
import json
import time
import unittest
from unittest.mock import patch

from test_follower_j5 import FollowerJ5Fixture

from cf_robot.angular_install import AngularInstall
from cf_robot.leader import run_teleop
from cf_robot.protocol import HOLD_CAPABILITY, VERSION, validate_command

try:
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed
except ImportError:
    serve = None


@unittest.skipIf(serve is None, "install websockets")
class J5ClientTests(FollowerJ5Fixture, unittest.IsolatedAsyncioTestCase):
    async def exercise(self, wrong_joint=False):
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        self.reload()
        self.bus.raw["wrist_roll"] = 1005
        stop = asyncio.Event()
        frames, paths, states = [], [], []
        self.frames = frames
        live = False
        live_frames = 0
        position = {"actual": -88.0, "goal": -88.0}

        def publish(message):
            nonlocal live
            state = message.get("state")
            if state:
                states.append(state)
            if state == "running":
                live = True

        async def peer(socket):
            nonlocal live_frames
            paths.append(socket.request.path)
            await socket.send(json.dumps({
                "version": VERSION, "type": "ready", "mode": "joint-only",
                "joints": ["gripper.pos"] if wrong_joint else ["wrist_roll.pos"],
                "observation": {"wrist_roll.pos": position["actual"]},
                "capabilities": [HOLD_CAPABILITY],
            }))

            async def feedback():
                while True:
                    position["actual"] += max(-0.5, min(0.5, position["goal"] - position["actual"]))
                    await socket.send(json.dumps({
                        "type": "status", "state": "ready",
                        "observation": {"wrist_roll.pos": position["actual"]},
                        "observed_at": time.time(),
                    }))
                    await asyncio.sleep(0.02)

            reader = asyncio.create_task(feedback())
            seq = 0
            try:
                async for raw in socket:
                    seq, action = validate_command(
                        json.loads(raw), seq, joints=("wrist_roll.pos",), allow_hold=True,
                    )
                    if action is None:
                        position["goal"] = position["actual"]
                    else:
                        frames.append(action)
                        position["goal"] = action["wrist_roll.pos"]
                        if live:
                            live_frames += 1
                            if live_frames >= 2:
                                stop.set()
            except ConnectionClosed:
                pass
            finally:
                reader.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):
                    await reader

        with patch("cf_robot.leader.access_headers", return_value={}), patch("builtins.input", return_value=""):
            async with serve(peer, "127.0.0.1", 0) as server:
                port = server.sockets[0].getsockname()[1]
                await asyncio.wait_for(run_teleop(
                    f"ws://127.0.0.1:{port}/control", "/dev/fake", "leader",
                    leader_factory=lambda: self.device, joint_only=5,
                    sync_at_start=True, stop=stop, on_status=publish,
                ), 3)
        return frames, paths, states

    async def test_j5_client_syncs_and_streams_only_degree_joint(self):
        frames, paths, states = await self.exercise()
        self.assertEqual(paths, ["/control/joint/5"])
        self.assertIn("startup sync complete", states)
        self.assertIn("running", states)
        self.assertTrue(all(set(frame) == {"wrist_roll.pos"} for frame in frames))
        self.assertEqual(frames[-1]["wrist_roll.pos"], -90)
        self.assertEqual(self.bus.writes, [])
        self.assertTrue(self.bus.torque["gripper"])

    async def test_wrong_isolation_acknowledgement_never_sends_a_target(self):
        with self.assertRaisesRegex(RuntimeError, "confirm J5-only isolation"):
            await self.exercise(wrong_joint=True)
        self.assertEqual(self.frames, [])
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
