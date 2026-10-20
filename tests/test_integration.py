"""Origin WebSocket smoke tests; run when the optional HTTP test stack is installed."""

import asyncio
import os
from pathlib import Path
import queue
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import AsyncMock, patch

try:
    import cv2
    from fastapi.testclient import TestClient
    import httpx  # noqa: F401 - TestClient's transport
    import numpy as np
    from starlette.websockets import WebSocketDisconnect

    from cf_robot.follower import create_app
    from cf_robot.web import create_app as create_web_app
except ImportError:
    TestClient = None

from cf_robot.protocol import JOINTS, action_message
from test_control import FakeFollower, wait_for


@unittest.skipIf(TestClient is None, "install project dependencies and httpx for WebSocket smoke tests")
class OriginTests(unittest.TestCase):
    key = "t" * 64
    headers = {"x-cf-robot-key": key}

    def test_control_session_and_disconnect(self):
        robot = FakeFollower()
        app = create_app("unused", "test", factory=lambda: robot, origin_key=self.key)
        with TestClient(app) as client:
            self.assertFalse(client.get("/health").json()["control_busy"])
            with client.websocket_connect("/control", headers=self.headers) as socket:
                ready = socket.receive_json()
                self.assertEqual(ready["type"], "ready")
                self.assertEqual(ready["observation"], {joint: 0.0 for joint in JOINTS})
                self.assertTrue(client.get("/health").json()["control_busy"])
                socket.send_json(action_message(1, {joint: 1.0 for joint in JOINTS}))
                self.assertTrue(wait_for(lambda: len(robot.actions) == 1))
            self.assertTrue(wait_for(lambda: robot.disconnected))
            self.assertTrue(wait_for(lambda: not client.get("/health").json()["control_busy"]))

    def test_video_jpeg(self):
        class Camera:
            disconnected = False

            def connect(self):
                pass

            def read(self):
                return np.zeros((48, 64, 3), dtype=np.uint8)

            def disconnect(self):
                self.disconnected = True

        camera = Camera()
        app = create_app("unused", "test", camera_device="fake", camera_factory=lambda: camera,
                         origin_key=self.key)
        with TestClient(app) as client, client.websocket_connect("/video", headers=self.headers) as socket:
            self.assertTrue(socket.receive_bytes().startswith(b"\xff\xd8"))

    def test_control_rejects_missing_origin_key(self):
        robot = FakeFollower()
        app = create_app("unused", "test", factory=lambda: robot, origin_key=self.key)
        with TestClient(app) as client:
            with self.assertRaises(WebSocketDisconnect):
                with client.websocket_connect("/control"):
                    pass
        self.assertFalse(robot.connected)

    def test_update_websocket_queues_only_fixed_update_request(self):
        with tempfile.TemporaryDirectory() as temp:
            request_file = Path(temp) / "update-request"
            with (
                patch("cf_robot.follower.UPDATE_REQUEST", request_file),
                patch(
                    "cf_robot.follower.subprocess.run",
                    return_value=subprocess.CompletedProcess([], 0),
                ),
            ):
                app = create_app("unused", "test", factory=FakeFollower, origin_key=self.key)
                with TestClient(app) as client:
                    with client.websocket_connect("/update", headers=self.headers) as socket:
                        socket.send_json({"version": 1, "type": "update"})
                        response = socket.receive_json()
                    self.assertEqual(response["type"], "update_queued")
                    self.assertEqual(request_file.read_text(), response["request_id"])
                    with client.websocket_connect("/update", headers=self.headers) as socket:
                        socket.send_json({
                            "version": 1,
                            "type": "update_status",
                            "request_id": response["request_id"],
                        })
                        status = socket.receive_json()
                    self.assertEqual(status["type"], "update_status")
                    self.assertEqual(status["state"], "queued")

    def test_calibration_websocket_relays_prompts_and_input(self):
        processes = []

        class FakeStdin:
            def __init__(self, process):
                self.process = process
                self.inputs = []

            def write(self, value):
                self.inputs.append(value)
                self.process.stdout.feed_data(b"\nCalibration saved.\n")
                self.process.stdout.feed_eof()
                self.process.returncode = 0
                self.process.finished.set()

            async def drain(self):
                pass

        class FakeProcess:
            def __init__(self):
                self.stdout = asyncio.StreamReader()
                self.stdout.feed_data(b"Press c to recalibrate: ")
                self.returncode = None
                self.finished = asyncio.Event()
                self.stdin = FakeStdin(self)
                processes.append(self)

            async def wait(self):
                await self.finished.wait()
                return self.returncode

            def terminate(self):
                self.returncode = -15
                self.finished.set()

            def kill(self):
                self.terminate()

        async def create_process(*args, **kwargs):
            return FakeProcess()

        with patch("cf_robot.follower.CALIBRATOR", Path(sys.executable)), patch(
            "cf_robot.follower.asyncio.create_subprocess_exec",
            new=AsyncMock(side_effect=create_process),
        ):
            app = create_app(
                "/dev/null",
                "so101_follower",
                factory=FakeFollower,
                origin_key=self.key,
            )
            with TestClient(app) as client:
                with client.websocket_connect("/calibrate", headers=self.headers) as socket:
                    socket.send_json({"version": 1, "type": "start"})
                    self.assertEqual(socket.receive_json()["type"], "calibration_started")
                    prompt = socket.receive_json()
                    self.assertEqual(prompt["type"], "calibration_output")
                    self.assertIn("Press c", prompt["text"])
                    socket.send_json({
                        "version": 1,
                        "type": "input",
                        "text": "c",
                    })
                    output = socket.receive_json()
                    self.assertEqual(output["type"], "calibration_output")
                    self.assertIn("Calibration saved", output["text"])
                    finished = socket.receive_json()
                    self.assertEqual(finished["type"], "calibration_finished")
                    self.assertEqual(finished["exit_code"], 0)
                self.assertEqual(processes[0].stdin.inputs, [b"c\n"])
                self.assertFalse(client.get("/health").json()["calibration_busy"])

    def test_servo_maintenance_wss_reads_limits_and_confirms_home(self):
        limits = {
            joint: {"min": 100, "max": 3900}
            for joint in JOINTS
        }
        current = {joint: 2000 for joint in JOINTS}
        record = {
            "version": 1,
            "arm_id": "so101_follower",
            "units": "raw_servo_ticks",
            "read_at": 1.0,
            "limits": limits,
        }

        def fake_move(port, arm_id, planned_limits, planned_current, home, speed, cancelled, report):
            report({"type": "home_progress", "state": "moving", "target": home})
            return {"state": "succeeded", "target": home, "present": home}

        with patch(
            "cf_robot.follower.read_limits_and_pose",
            return_value={"record": record, "current": current},
        ), patch("cf_robot.follower.move_to_home", side_effect=fake_move):
            app = create_app(
                "/dev/fake",
                "so101_follower",
                factory=FakeFollower,
                origin_key=self.key,
            )
            with TestClient(app) as client:
                with client.websocket_connect("/maintenance", headers=self.headers) as socket:
                    socket.send_json({"version": 1, "type": "read_limits"})
                    limits_message = socket.receive_json()
                self.assertEqual(limits_message["type"], "servo_limits")
                self.assertEqual(limits_message["record"]["limits"], limits)

                with client.websocket_connect("/maintenance", headers=self.headers) as socket:
                    socket.send_json(
                        {"version": 1, "type": "go_home", "speed_percent": 75}
                    )
                    plan = socket.receive_json()
                    self.assertEqual(plan["type"], "home_plan")
                    self.assertEqual(plan["current"], current)
                    self.assertEqual(plan["target"]["shoulder_pan.pos"], 2000)
                    socket.send_json({"version": 1, "type": "confirm_home"})
                    messages = []
                    while not messages or messages[-1]["type"] != "home_finished":
                        messages.append(socket.receive_json())
                self.assertEqual(messages[-1]["state"], "succeeded")
                self.assertFalse(client.get("/health").json()["maintenance_busy"])

    def test_pose_monitor_wss_streams_read_only_follower_samples(self):
        def fake_monitor(
            port, arm_id, stop, publish, commands, command_busy
        ):
            publish(
                {
                    "type": "pose_sample",
                    "side": "follower",
                    "values": {joint: 1.0 for joint in JOINTS},
                    "raw": {joint: 2000 for joint in JOINTS},
                    "torque": {
                        joint: index % 2 == 0 for index, joint in enumerate(JOINTS)
                    },
                    "sampled_at": 1.0,
                }
            )
            while not stop.is_set():
                try:
                    command = commands.get(timeout=0.05)
                except queue.Empty:
                    continue
                command_busy.set()
                if command.get("mode") == "torque" and command.get("joint_id") == "*":
                    joint_results = [
                        {
                            "joint_id": index,
                            "joint_name": joint.removesuffix(".pos").replace("_", " ").title(),
                            "torque_enabled": command["enabled"],
                            "message": (
                                "torque enabled" if command["enabled"] else "torque released"
                            ),
                        }
                        for index, joint in enumerate(JOINTS, start=1)
                    ]
                    publish(
                        {
                            "type": "command_result",
                            "command_id": command["command_id"],
                            "joint_id": "*",
                            "state": "succeeded",
                            "joint_results": joint_results,
                        }
                    )
                else:
                    publish(
                        {
                            "type": "command_result",
                            "command_id": command["command_id"],
                            "joint_id": command["joint_id"],
                            "state": "succeeded",
                        }
                    )
                command_busy.clear()

        with patch("cf_robot.follower.stream_follower_pose", side_effect=fake_monitor):
            app = create_app(
                "/dev/fake",
                "so101_follower",
                factory=FakeFollower,
                origin_key=self.key,
            )
            with TestClient(app) as client:
                with client.websocket_connect("/monitor", headers=self.headers) as socket:
                    socket.send_json(
                        {"version": 1, "type": "start"}
                    )
                    started = socket.receive_json()
                    self.assertEqual(started["type"], "monitor_started")
                    sample = socket.receive_json()
                    self.assertEqual(sample["type"], "pose_sample")
                    self.assertEqual(sample["side"], "follower")
                    self.assertEqual(sample["values"]["wrist_flex.pos"], 1.0)
                    self.assertTrue(sample["torque"]["shoulder_pan.pos"])
                    self.assertTrue(client.get("/health").json()["monitor_busy"])
                    socket.send_json(
                        {
                            "version": 1,
                            "type": "command",
                            "command_id": 1,
                            "joint_id": 3,
                            "mode": "degrees",
                            "value": -120.34,
                        }
                    )
                    self.assertEqual(socket.receive_json()["type"], "command_queued")
                    result = socket.receive_json()
                    self.assertEqual(result["type"], "command_result")
                    self.assertEqual(result["joint_id"], 3)
                    socket.send_json(
                        {
                            "version": 1,
                            "type": "command",
                            "command_id": 2,
                            "joint_id": "*",
                            "mode": "torque",
                            "enabled": False,
                        }
                    )
                    self.assertEqual(socket.receive_json()["type"], "command_queued")
                    all_torque_result = socket.receive_json()
                    self.assertEqual(len(all_torque_result["joint_results"]), 6)
                    self.assertEqual(
                        all_torque_result["joint_results"][-1]["joint_name"],
                        "Gripper",
                    )
                    socket.send_json({"version": 1, "type": "stop"})
                    self.assertEqual(socket.receive_json()["type"], "monitor_finished")
                    with self.assertRaises(WebSocketDisconnect):
                        socket.receive_text()
                self.assertFalse(client.get("/health").json()["monitor_busy"])

    def test_web_controls_require_same_origin(self):
        async def fake_runner(url, port, arm_id, fps, stop, on_status):
            on_status({"state": "running"})
            await stop.wait()

        with patch.dict(os.environ, {"CF_ROBOT_ORIGIN_KEY": self.key}):
            app = create_web_app("ws://127.0.0.1:8765", "unused", "test", runner=fake_runner)
            with TestClient(app, base_url="http://127.0.0.1:8080") as client:
                self.assertEqual(client.post("/api/start").status_code, 403)
                headers = {"origin": "http://127.0.0.1:8080"}
                self.assertEqual(client.post("/api/start", headers=headers).status_code, 200)
                self.assertEqual(client.post("/api/start", headers=headers).status_code, 409)
                self.assertEqual(client.post("/api/stop", headers=headers).status_code, 200)

    def test_browser_heartbeat_loss_stops_control(self):
        async def fake_runner(url, port, arm_id, fps, stop, on_status):
            await stop.wait()

        with patch.dict(os.environ, {"CF_ROBOT_ORIGIN_KEY": self.key}):
            app = create_web_app("ws://127.0.0.1:8765", "unused", "test",
                                 runner=fake_runner, heartbeat_timeout=0.2)
            with TestClient(app, base_url="http://127.0.0.1:8080") as client:
                headers = {"origin": "http://127.0.0.1:8080"}
                client.post("/api/start", headers=headers)
                self.assertTrue(wait_for(
                    lambda: client.get("/api/status", headers=headers).json()["error"]
                    == "Browser heartbeat lost; control stopped."
                ))


if __name__ == "__main__":
    unittest.main()
