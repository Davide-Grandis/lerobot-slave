import asyncio
import json
import os
import time
import unittest
from unittest.mock import patch

from cf_robot.protocol import JOINTS

try:
    import httpx

    from cf_robot.web import create_app
    from cf_robot.web_pose import BrowserPoseMonitor
except ImportError:
    BrowserPoseMonitor = None


class MonitorSocket:
    def __init__(self):
        self.messages = asyncio.Queue()
        self.sent = []

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        pass

    async def send(self, raw):
        message = json.loads(raw)
        self.sent.append(message)
        if message["type"] == "start":
            await self.messages.put(self.started())
        elif message["type"] == "stop":
            await self.messages.put({"type": "monitor_finished"})

    def started(self):
        return {"type": "monitor_started", "version": 1}

    async def recv(self):
        return json.dumps(await self.messages.get())


async def wait_until(predicate):
    async with asyncio.timeout(2):
        while not predicate():
            await asyncio.sleep(0.01)


@unittest.skipIf(BrowserPoseMonitor is None, "install the web bridge dependencies and httpx")
class BrowserPoseTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.socket = MonitorSocket()
        self.auth = patch.dict(os.environ, {"CF_ROBOT_ORIGIN_KEY": "t" * 64})
        self.auth.start()
        self.connection = patch("cf_robot.web_pose.connect", return_value=self.socket)
        self.connection.start()
        self.monitor = BrowserPoseMonitor("ws://127.0.0.1:8765", None, "leader")

    async def asyncTearDown(self):
        self.monitor.stop()
        if self.monitor.task is not None:
            await self.monitor.task
        self.connection.stop()
        self.auth.stop()

    async def start_monitor(self):
        self.monitor.start()
        await wait_until(lambda: self.monitor.status["state"] == "ready")
        self.monitor.status["follower"] = {
            "sampled_at": time.time(), "calibration": {"state": "consistent"},
        }

    async def test_start_preserves_torque_samples_and_explicit_shutdown(self):
        await self.start_monitor()
        self.assertEqual(self.socket.sent, [
            {"version": 1, "type": "start"},
        ])
        sample = {
            "type": "pose_sample", "side": "follower",
            "values": {joint: 1.0 for joint in JOINTS}, "sampled_at": time.time(),
        }
        await self.socket.messages.put(sample)
        await wait_until(lambda: self.monitor.status["follower"].get("type") == "pose_sample")
        self.assertEqual(self.monitor.snapshot()["follower"], sample)
        self.monitor.stop()
        await self.monitor.task
        self.assertFalse(self.monitor.running)
        self.assertEqual(self.monitor.status["state"], "idle")
        self.assertEqual(self.socket.sent[-1], {"version": 1, "type": "stop"})

    async def test_leader_reader_starts_without_torque_release_argument(self):
        disconnected = []

        def leader_reader(port, arm_id, stop, publish):
            publish({"type": "pose_sample", "side": "leader", "sampled_at": time.time()})
            stop.wait()
            disconnected.append(True)

        self.monitor.port = "fake-usb"
        self.monitor.leader_reader = leader_reader
        await self.start_monitor()
        await wait_until(lambda: self.monitor.status["leader"] is not None)
        self.assertEqual(self.socket.sent[0], {"version": 1, "type": "start"})
        self.monitor.stop()
        await self.monitor.task
        self.assertEqual(disconnected, [True])

    async def test_commands_are_validated_serialized_and_track_results(self):
        with self.assertRaisesRegex(RuntimeError, "start the pose monitor"):
            await self.monitor.command({"joint_id": 1, "mode": "ticks", "value": 50})
        await self.start_monitor()
        with self.assertRaises(ValueError):
            await self.monitor.command({"joint_id": 6, "mode": "degrees", "value": 50})
        with self.assertRaises(ValueError):
            await self.monitor.command({"joint_id": 1, "mode": "ticks", "value": 1.5})
        result = await self.monitor.command({"joint_id": 6, "mode": "percent", "value": 50})
        self.assertEqual(result, {"command_id": 1})
        with self.assertRaisesRegex(RuntimeError, "current command"):
            await self.monitor.command({"joint_id": 1, "mode": "torque", "enabled": True})
        await self.socket.messages.put({
            "type": "command_progress", "command_id": 1, "joint_id": 6,
            "current": 20, "target": 50,
        })
        await wait_until(lambda: self.monitor.status["command"]["type"] == "command_progress")
        self.assertTrue(self.monitor.status["command_pending"])
        await self.socket.messages.put({
            "type": "command_result", "command_id": 1, "joint_id": 6, "state": "succeeded",
        })
        await wait_until(lambda: not self.monitor.status["command_pending"])
        await self.monitor.command({"joint_id": 1, "mode": "torque", "enabled": False})
        self.assertEqual(self.socket.sent[-1]["command_id"], 2)
        self.assertIs(self.socket.sent[-1]["enabled"], False)

    async def test_restore_commands_allow_calibration_mismatches_but_require_fresh_sample(self):
        await self.start_monitor()
        self.monitor.status["follower"]["calibration"]["state"] = "warning"
        await self.monitor.command({"joint_id": 6, "mode": "calibration_preview"})
        self.monitor.publish({"type": "command_result", "command_id": 1, "state": "preview"})
        await self.monitor.command({
            "joint_id": 6, "mode": "calibration_restore",
            "token": "a" * 32, "confirmation": "RESTORE J6",
        })
        self.assertEqual(self.socket.sent[-1]["mode"], "calibration_restore")
        self.monitor.publish({"type": "command_result", "command_id": 2, "state": "succeeded"})
        self.monitor.status["follower"]["sampled_at"] -= 10
        with self.assertRaisesRegex(ValueError, "fresh follower sample"):
            await self.monitor.command({"joint_id": 6, "mode": "calibration_preview"})

    async def test_browser_loss_stops_monitor_without_restoring_torque(self):
        await self.start_monitor()
        self.monitor.last_heartbeat -= 10
        await asyncio.wait_for(self.monitor.task, 2)
        self.assertIn("heartbeat lost", self.monitor.status["error"])
        self.assertEqual([message["type"] for message in self.socket.sent], ["start", "stop"])

    async def test_command_timeout_stops_monitor(self):
        await self.start_monitor()
        await self.monitor.command({"joint_id": 1, "mode": "ticks", "value": 500})
        self.monitor.pending_since -= 20
        await asyncio.wait_for(self.monitor.task, 2)
        self.assertIn("timed out", self.monitor.status["error"])

    async def test_leader_failure_stops_remote_session(self):
        def broken_reader(*args):
            raise RuntimeError("USB disconnected")

        self.monitor.port = "fake-usb"
        self.monitor.leader_reader = broken_reader
        self.monitor.start()
        await asyncio.wait_for(self.monitor.task, 2)
        self.assertEqual(self.monitor.status["state"], "error")
        self.assertIn("USB disconnected", self.monitor.status["error"])

    async def test_invalid_monitor_start_never_opens_local_leader(self):
        self.socket.started = lambda: {"type": "monitor_started", "version": 2}
        self.monitor.port = "fake-usb"
        with patch.object(self.monitor, "leader_reader") as reader:
            self.monitor.start()
            await asyncio.wait_for(self.monitor.task, 2)
            reader.assert_not_called()
        self.assertEqual(self.monitor.status["state"], "error")

    async def test_stop_during_command_prevents_further_commands(self):
        await self.start_monitor()
        await self.monitor.command({"joint_id": 1, "mode": "ticks", "value": 500})
        self.monitor.stop()
        with self.assertRaisesRegex(RuntimeError, "start the pose monitor"):
            await self.monitor.command({"joint_id": 2, "mode": "ticks", "value": 500})
        await self.monitor.task
        self.assertFalse(self.monitor.status["command_pending"])
        self.assertEqual([message["type"] for message in self.socket.sent], [
            "start", "command", "stop",
        ])

    async def test_paired_routes_and_mutual_exclusion(self):
        async def runner(url, port, arm_id, fps, stop, on_status):
            await stop.wait()

        app = create_app(
            "ws://127.0.0.1:8765", None, "leader", runner=runner,
            operator_origin="https://cf-robot.davideslab.eu", bridge_token="pairing-test",
        )
        headers = {
            "Origin": "https://cf-robot.davideslab.eu",
            "X-CF-Robot-Bridge": "pairing-test",
        }
        async with httpx.AsyncClient(
            transport=httpx.ASGITransport(app=app), base_url="http://127.0.0.1:8080",
        ) as client:
            for path in ("status", "start", "stop", "heartbeat", "command"):
                method = client.get if path == "status" else client.post
                self.assertEqual((await method(f"/api/pose/{path}")).status_code, 403)
                self.assertEqual((await method(
                    f"/api/pose/{path}", headers={"Origin": headers["Origin"]},
                )).status_code, 403)
            for invalid in ([], {"release_torque": False}, {"release_torque": True}):
                response = await client.post("/api/pose/start", headers=headers, json=invalid)
                self.assertEqual(response.status_code, 409)
            self.assertEqual(self.socket.sent, [])
            response = await client.post(
                "/api/pose/start", headers=headers,
                json={},
            )
            self.assertEqual(response.status_code, 200)
            await wait_until(lambda: len(self.socket.sent) >= 1)
            self.assertEqual((await client.post("/api/start", headers=headers)).status_code, 409)
            self.assertEqual((await client.post(
                "/api/pose/start", headers=headers, json={},
            )).status_code, 409)
            await client.post("/api/pose/stop", headers=headers)
            await wait_until(lambda: self.socket.sent[-1]["type"] == "stop")
            await asyncio.sleep(0.05)
            self.assertEqual((await client.post("/api/start", headers=headers)).status_code, 200)
            self.assertEqual((await client.post(
                "/api/pose/start", headers=headers, json={},
            )).status_code, 409)
            await client.post("/api/stop", headers=headers)
            await asyncio.sleep(0.05)
            self.assertEqual((await client.post(
                "/api/pose/start", headers=headers, json={},
            )).status_code, 200)
            await asyncio.sleep(0.05)
            await client.post("/api/estop", headers=headers)
            await asyncio.sleep(0.25)
            self.assertFalse((await client.get("/api/pose/status", headers=headers)).json()["running"])
            self.assertEqual((await client.post(
                "/api/pose/start", headers=headers, json={},
            )).status_code, 409)
            self.assertEqual((await client.get("/pose-monitor.js")).status_code, 200)


if __name__ == "__main__":
    unittest.main()
