import asyncio
import contextlib
import json
import time
import unittest
from unittest.mock import patch

from test_follower_j5 import FollowerJ5Fixture

from cf_robot.angular_install import AngularInstall
from cf_robot.leader import run_teleop
from cf_robot.protocol import HOLD_CAPABILITY, VERSION, WRIST_GRIPPER_JOINTS, validate_command

try:
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed
except ImportError:
    serve = None


@unittest.skipIf(serve is None, "install websockets")
class J5ClientTests(FollowerJ5Fixture, unittest.IsolatedAsyncioTestCase):
    async def exercise(self, wrong_joint=False, paired=False):
        manager = AngularInstall()
        self.apply(manager, manager.preview(self.device, self.command))
        self.reload()
        self.bus.raw["wrist_roll"] = 1005
        joints = WRIST_GRIPPER_JOINTS if paired else ("wrist_roll.pos",)
        if paired:
            self.bus.torque["gripper"] = False
        stop = asyncio.Event()
        frames, paths, states = [], [], []
        self.frames = frames
        self.events = []
        self.position_at_running = None
        live = False
        live_frames = 0
        initial = {"wrist_roll.pos": -88.0, **({"gripper.pos": 35.0} if paired else {})}
        position = {"actual": dict(initial), "goal": dict(initial)}

        def publish(message):
            nonlocal live
            state = message.get("state")
            if state:
                states.append(state)
                self.events.append(state)
            if state == "running":
                live = True
                self.position_at_running = dict(position["actual"])

        async def peer(socket):
            nonlocal live_frames
            paths.append(socket.request.path)
            await socket.send(json.dumps({
                "version": VERSION, "type": "ready", "mode": "joint-only",
                "joints": ["gripper.pos"] if wrong_joint else list(joints),
                "observation": dict(position["actual"]),
                "capabilities": [HOLD_CAPABILITY],
            }))

            async def feedback():
                while True:
                    for joint in joints:
                        position["actual"][joint] += max(
                            -0.5, min(0.5, position["goal"][joint] - position["actual"][joint]),
                        )
                    await socket.send(json.dumps({
                        "type": "status", "state": "ready",
                        "observation": dict(position["actual"]),
                        "observed_at": time.time(),
                    }))
                    await asyncio.sleep(0.02)

            reader = asyncio.create_task(feedback())
            seq = 0
            try:
                async for raw in socket:
                    seq, action = validate_command(
                        json.loads(raw), seq, joints=joints, allow_hold=True,
                    )
                    if action is None:
                        position["goal"] = dict(position["actual"])
                    else:
                        frames.append(action)
                        self.events.append("action")
                        position["goal"] = dict(action)
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
                    leader_factory=lambda: self.device, joint_only=None if paired else 5,
                    wrist_gripper_only=paired,
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

    async def test_paired_client_syncs_both_then_streams_only_j5_j6(self):
        frames, paths, states = await self.exercise(paired=True)
        self.assertEqual(paths, ["/control/joints/5-6"])
        self.assertIn("startup sync complete", states)
        self.assertIn("running", states)
        self.assertTrue(all(set(frame) == set(WRIST_GRIPPER_JOINTS) for frame in frames))
        self.assertEqual(frames[0], {"wrist_roll.pos": -88.0, "gripper.pos": 35.0})
        self.assertLess(self.events.index("waiting for startup sync"), self.events.index("action"))
        self.assertLess(self.events.index("startup sync complete"), self.events.index("running"))
        self.assertTrue(all(
            abs(self.position_at_running[joint] - frames[-1][joint]) <= 1
            for joint in WRIST_GRIPPER_JOINTS
        ))
        self.assertEqual(frames[-1]["wrist_roll.pos"], -90)
        self.assertTrue(0 <= frames[-1]["gripper.pos"] <= 100)
        self.assertEqual(self.bus.writes, [])

    async def test_paired_client_rejects_partial_isolation_acknowledgement(self):
        with self.assertRaisesRegex(RuntimeError, "confirm J5-J6-only isolation"):
            await self.exercise(wrong_joint=True, paired=True)
        self.assertEqual(self.frames, [])
        self.assertEqual(self.bus.writes, [])


if __name__ == "__main__":
    unittest.main()
