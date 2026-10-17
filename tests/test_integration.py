"""Origin WebSocket smoke tests; run when the optional HTTP test stack is installed."""

import os
import unittest
from unittest.mock import patch

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
                self.assertEqual(socket.receive_json()["type"], "ready")
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
                client.post("/api/start", headers={"origin": "http://127.0.0.1:8080"})
                self.assertTrue(wait_for(
                    lambda: client.get("/api/status").json()["error"] == "Browser heartbeat lost; control stopped."
                ))


if __name__ == "__main__":
    unittest.main()
