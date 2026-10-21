"""Video relay and teleoperation interlocks without camera or arm hardware."""

import asyncio
import os
import unittest
from unittest.mock import patch

try:
    import httpx

    from cf_robot.web import create_app
except ImportError:
    create_app = None


class VideoSocket:
    def __init__(self):
        self.frames = asyncio.Queue()

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        pass

    def __aiter__(self):
        return self

    async def __anext__(self):
        return await self.frames.get()


class IdleController:
    def __init__(self, *args, on_status, **kwargs):
        self.stop_event = asyncio.Event()
        self.on_status = on_status

    async def run(self):
        self.on_status({"state": "ready"})
        await self.stop_event.wait()


@unittest.skipIf(create_app is None, "install the web bridge dependencies and httpx")
class WebVideoTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.socket = VideoSocket()
        self.controllers = []
        self.environment = patch.dict(os.environ, {"CF_ROBOT_ORIGIN_KEY": "t" * 64})
        self.environment.start()
        self.connection = patch("cf_robot.web.connect", return_value=self.socket)
        self.connect = self.connection.start()

        def controller_factory(*args, **kwargs):
            controller = IdleController(*args, **kwargs)
            self.controllers.append(controller)
            return controller

        self.app = create_app(
            "ws://127.0.0.1:8765", None, "leader",
            operator_origin="https://cf-robot.davideslab.eu", bridge_token="quiet-otter",
            controller_factory=controller_factory,
        )
        self.lifespan = self.app.router.lifespan_context(self.app)
        await self.lifespan.__aenter__()
        self.client = httpx.AsyncClient(
            transport=httpx.ASGITransport(app=self.app), base_url="http://127.0.0.1:8080",
            headers={
                "Origin": "https://cf-robot.davideslab.eu",
                "X-CF-Robot-Bridge": "quiet-otter",
            },
        )

    async def asyncTearDown(self):
        await self.lifespan.__aexit__(None, None, None)
        await self.client.aclose()
        self.connection.stop()
        self.environment.stop()

    async def wait_for_status(self, predicate):
        async with asyncio.timeout(2):
            while True:
                status = (await self.client.get("/api/status")).json()
                if predicate(status):
                    return status
                await asyncio.sleep(0.01)

    async def prime_camera(self):
        response = await self.client.get("/api/video/frame")
        self.assertEqual(response.status_code, 503)
        await self.socket.frames.put(b"\xff\xd8first-frame\xff\xd9")
        await self.wait_for_status(lambda status: status["camera_live"])

    async def start_control(self):
        response = await self.client.post("/api/start", json={"camera_live": True})
        self.assertEqual(response.status_code, 200)
        await self.wait_for_status(lambda status: status["state"] == "ready")

    async def test_authenticated_frames_continue_during_teleoperation(self):
        await self.prime_camera()
        self.assertEqual(self.connect.call_args.args[0], "ws://127.0.0.1:8765/video")
        self.assertEqual(
            self.connect.call_args.kwargs["additional_headers"]["X-CF-Robot-Key"], "t" * 64,
        )
        await self.start_control()
        for frame in (b"\xff\xd8second-frame\xff\xd9", b"\xff\xd8third-frame\xff\xd9"):
            await self.socket.frames.put(frame)
            async with asyncio.timeout(2):
                while True:
                    response = await self.client.get("/api/video/frame")
                    if response.content == frame:
                        break
                    await asyncio.sleep(0.01)
            self.assertEqual(response.headers["content-type"], "image/jpeg")
            self.assertIn("no-store", response.headers["cache-control"])
            await self.client.post("/api/heartbeat", json={"camera_live": True})
            self.assertFalse(self.controllers[0].stop_event.is_set())
        self.assertEqual(self.connect.call_count, 1)

    async def test_start_requires_camera_and_browser_confirmation(self):
        response = await self.client.post("/api/start", json={"camera_live": True})
        self.assertEqual(response.status_code, 409)
        await self.prime_camera()
        response = await self.client.post("/api/start", json={"camera_live": False})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.controllers, [])

    async def test_stale_follower_video_stops_actual_controller(self):
        await self.prime_camera()
        await self.start_control()
        with patch("cf_robot.web.VIDEO_STALE_S", 0.05):
            status = await self.wait_for_status(
                lambda status: status["error"] == "Follower camera feed lost; control stopped.",
            )
            self.assertTrue(status["estop_latched"])
            self.assertTrue(self.controllers[0].stop_event.is_set())
            self.assertEqual((await self.client.get("/api/video/frame")).status_code, 503)

    async def test_browser_video_loss_stops_actual_controller(self):
        await self.prime_camera()
        await self.start_control()
        await self.client.post("/api/heartbeat", json={"camera_live": False})
        status = await self.wait_for_status(
            lambda status: status["error"] == "Browser stopped receiving follower video; control stopped.",
        )
        self.assertTrue(status["camera_live"])
        self.assertTrue(status["estop_latched"])
        self.assertTrue(self.controllers[0].stop_event.is_set())
