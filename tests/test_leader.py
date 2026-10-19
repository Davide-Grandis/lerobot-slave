import asyncio
import contextlib
import json
import time
import unittest
from unittest.mock import patch

try:
    from websockets.asyncio.server import serve
    from websockets.exceptions import ConnectionClosed
    from cf_robot.leader import STARTUP_SYNC_STEP_UNITS, run_teleop
except ImportError:
    serve = None
    STARTUP_SYNC_STEP_UNITS = None
    run_teleop = None
from cf_robot.protocol import JOINTS, VERSION, validate_command


@unittest.skipIf(serve is None, "install project dependencies including websockets")
class LeaderSyncTests(unittest.IsolatedAsyncioTestCase):
    async def test_startup_sync_streams_small_single_joint_targets_before_running(self):
        follower_pose = {joint: 0.0 for joint in JOINTS}
        leader_pose = {joint: 0.0 for joint in JOINTS}
        leader_pose["shoulder_lift.pos"] = -7.2
        accepted = []
        follower_state = {
            "measured": dict(follower_pose),
            "target": dict(follower_pose),
            "last_target_change": time.monotonic(),
            "last_command": time.monotonic(),
        }

        class FakeLeader:
            is_connected = False

            def connect(self, calibrate=True):
                self.is_connected = True

            def get_action(self):
                return dict(leader_pose)

            def disconnect(self):
                self.is_connected = False

        async def follower_socket(socket):
            await socket.send(json.dumps({
                "type": "ready",
                "version": VERSION,
                "observation": follower_pose,
            }))

            async def send_status():
                while True:
                    now = time.monotonic()
                    if now - follower_state["last_command"] > 0.5:
                        await socket.send(json.dumps({
                            "type": "error",
                            "message": "control stream timed out",
                        }))
                        return
                    if now - follower_state["last_target_change"] > 0.75:
                        follower_state["measured"] = dict(follower_state["target"])
                    await socket.send(json.dumps({
                        "type": "status",
                        "state": "ready",
                        "observation": dict(follower_state["measured"]),
                        "observed_at": time.time() - 5,
                    }))
                    await asyncio.sleep(0.05)

            status_task = asyncio.create_task(send_status())
            last_seq = 0
            try:
                async for raw in socket:
                    seq, action = validate_command(json.loads(raw), last_seq)
                    last_seq = seq
                    accepted.append(action)
                    if action != follower_state["target"]:
                        follower_state["target"] = dict(action)
                        follower_state["last_target_change"] = time.monotonic()
                    follower_state["last_command"] = time.monotonic()
            finally:
                status_task.cancel()
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed):
                    await status_task

        stop = asyncio.Event()

        def on_status(message):
            if message.get("state") == "running":
                stop.set()

        with patch("cf_robot.leader.access_headers", return_value={}), patch(
            "builtins.input", return_value=""
        ):
            async with serve(follower_socket, "127.0.0.1", 0) as server:
                port = server.sockets[0].getsockname()[1]
                await run_teleop(
                    f"ws://127.0.0.1:{port}/control",
                    "/dev/null",
                    "test-leader",
                    stop=stop,
                    on_status=on_status,
                    leader_factory=FakeLeader,
                    sync_at_start=True,
                )

        self.assertTrue(accepted)
        self.assertGreater(len(accepted), 3)
        self.assertLess(STARTUP_SYNC_STEP_UNITS, 5.0)
        self.assertEqual(accepted[-1], leader_pose)
        previous = follower_pose
        for target in accepted:
            changed = [
                joint for joint in JOINTS
                if abs(target[joint] - previous[joint]) > 1e-9
            ]
            self.assertLessEqual(len(changed), 1)
            if changed:
                self.assertLessEqual(
                    abs(target[changed[0]] - previous[changed[0]]),
                    STARTUP_SYNC_STEP_UNITS + 1e-9,
                )
            previous = target
        self.assertTrue(all(
            abs(previous[joint] - leader_pose[joint]) <= 1e-9
            for joint in JOINTS
        ))


if __name__ == "__main__":
    unittest.main()
