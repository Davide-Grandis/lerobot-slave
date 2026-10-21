import asyncio
import json
import os
import queue
import tempfile
import threading
import time
import unittest
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from test_pose_monitor import FakeArm

from cf_robot.calibration_inspection import (
    capture_validation,
    inspect_calibration,
    validation_analysis,
)
from cf_robot.pose_monitor import _sample, stream_follower_pose
from cf_robot.protocol import JOINTS
from cf_robot.servo_maintenance import read_servo_limits


class CalibrationInspectionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.arm = FakeArm()
        self.arm.id = "test-arm"
        self.arm.config = SimpleNamespace(port="/dev/fake")
        self.arm.calibration_fpath = Path(self.directory.name) / "arm.json"
        bus = self.arm.bus
        bus.calibration = {}
        bus.model_resolution_table = {"sts3215": 4096}
        bus.model_number_table = {"sts3215": 777}
        bus.apply_drive_mode = True
        for name, motor in bus.motors.items():
            motor.model = "sts3215"
            motor.norm_mode = "range_0_100" if name == "gripper" else "degrees"
            bus.calibration[name] = {
                "id": motor.id, "drive_mode": 0, "homing_offset": 50,
                "range_min": 100, "range_max": 3900,
            }
        self.arm.calibration_fpath.write_text(json.dumps(bus.calibration))
        self.live = deepcopy(bus.calibration)
        bus.read_calibration = lambda: self.live
        bus.ping = lambda *args, **kwargs: 777
        original_read = bus.read
        bus.read = lambda register, *args, **kwargs: (
            0 if register == "Operating_Mode" else original_read(register, *args, **kwargs)
        )

    def sample(self, side="leader"):
        limits = read_servo_limits(self.arm.bus, strict=False)
        report = inspect_calibration(self.arm, side, limits)
        return _sample(self.arm, side, limits, report)

    def test_inspection_is_read_only_and_exposes_actual_conversion(self):
        sample = self.sample()
        report = sample["calibration"]
        self.assertEqual(report["state"], "consistent")
        self.assertTrue(report["file"]["matches_loaded"])
        self.assertEqual(report["joints"][JOINTS[0]]["reference_value"], 0)
        self.assertEqual(report["joints"][JOINTS[0]]["resolution"], 4096)
        self.assertEqual(report["joints"][JOINTS[5]]["unit"], "%")
        self.assertEqual(self.arm.bus.writes, [])

    def test_saved_live_offset_and_mode_mismatches_are_visible(self):
        self.live["shoulder_pan"]["homing_offset"] = 51
        self.arm.bus.motors["elbow_flex"].norm_mode = "range_m100_100"
        report = self.sample()["calibration"]
        self.assertEqual(report["state"], "warning")
        self.assertIn("homing_offset", " ".join(report["joints"][JOINTS[0]]["warnings"]))
        self.assertIn("normalization", " ".join(report["joints"][JOINTS[2]]["warnings"]))

    def test_bad_calibration_still_allows_raw_inspection(self):
        self.arm.bus.calibration = {}
        self.arm.bus._normalize = lambda values: (_ for _ in ()).throw(RuntimeError("no calibration"))
        sample = self.sample()
        self.assertEqual(sample["raw"][JOINTS[0]], 2000)
        self.assertIsNone(sample["values"][JOINTS[0]])
        self.assertIn("no calibration", sample["conversion_errors"][JOINTS[0]])
        self.assertEqual(self.arm.bus.writes, [])

    def test_reversed_limits_are_not_silently_corrected_in_inspector(self):
        self.arm.bus.limits["shoulder_pan"] = {"min": 3900, "max": 100}
        self.live["shoulder_pan"].update(range_min=3900, range_max=100)
        sample = self.sample()
        self.assertEqual(sample["limits"][JOINTS[0]], {"min": 3900, "max": 100})
        self.assertEqual(sample["calibration"]["state"], "warning")

    def test_independent_capture_needs_no_other_arm(self):
        sample = self.sample()
        capture = capture_validation({"leader": sample}, {
            "side": "leader", "label": "Measured reference", "joint": JOINTS[0],
            "expected": 0, "tolerance": 2, "confirmed": True,
        })
        self.assertTrue(capture["passed"])
        self.assertEqual(set(capture["evidence"]), {"leader"})
        self.assertEqual(self.arm.bus.writes, [])

    def test_stale_unconfirmed_or_mismatched_units_are_rejected(self):
        sample = self.sample()
        body = {
            "side": "leader", "label": "reference", "joint": JOINTS[0],
            "expected": 0, "tolerance": 2, "confirmed": True,
        }
        for change in ({"confirmed": False}, {"expected": float("nan")}, {"tolerance": 0}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                capture_validation({"leader": sample}, {**body, **change})
        sample["sampled_at"] -= 10
        with self.assertRaisesRegex(ValueError, "stale"):
            capture_validation({"leader": sample}, body)

    def test_fit_flags_scale_direction_offset_and_ignores_old_calibration(self):
        for slope, offset, expected_note in (
            (2, 0, "scale"), (-1, 0, "reversed"), (1, 8, "offset"),
        ):
            sample = self.sample()
            captures = []
            for x in (-20, 0, 20):
                sample["values"][JOINTS[0]] = slope * x + offset
                captures.append(capture_validation({"leader": sample}, {
                    "side": "leader", "label": str(x), "joint": JOINTS[0],
                    "expected": x, "tolerance": 2, "confirmed": True,
                }))
            results = validation_analysis(captures, {"leader": sample})
            self.assertIn(expected_note, " ".join(results[0]["notes"]).lower())
            sample["calibration"]["validation_key"] = "changed"
            self.assertEqual(validation_analysis(captures, {"leader": sample}), [])

    def test_matched_capture_is_optional_and_requires_time_and_unit_alignment(self):
        leader, follower = self.sample(), self.sample("follower")
        body = {"side": "both", "label": "matched reference", "confirmed": True}
        result = capture_validation({"leader": leader, "follower": follower}, body)
        self.assertEqual(result["deltas"][JOINTS[0]], 0)
        follower["calibration"]["joints"][JOINTS[0]]["mode"] = "range_m100_100"
        with self.assertRaisesRegex(ValueError, "matching units"):
            capture_validation({"leader": leader, "follower": follower}, body)

    def test_sample_flags_wrap_and_range_clipping(self):
        previous = self.sample()
        self.arm.bus.present["shoulder_pan"] = 4090
        sample = _sample(self.arm, "leader", previous["limits"], previous["calibration"], previous)
        self.assertTrue(any("discontinuity" in warning for warning in sample["motion_warnings"]))
        self.assertTrue(any("outside saved range" in warning for warning in sample["motion_warnings"]))

    def test_bad_calibration_blocks_follower_position_commands_without_writes(self):
        commands = queue.Queue()
        commands.put({
            "version": 1, "type": "command", "command_id": 1,
            "joint_id": 1, "mode": "ticks", "value": 2010,
        })
        self.live["shoulder_pan"]["homing_offset"] = 99
        stop, messages = threading.Event(), []

        def publish(message):
            messages.append(message)
            if message["type"] == "pose_sample":
                stop.set()

        with patch("cf_robot.pose_monitor.make_follower", return_value=self.arm):
            stream_follower_pose("/dev/fake", "test", stop, publish, commands)
        self.assertIn("inspector warnings", messages[0]["message"])
        self.assertEqual(self.arm.bus.writes, [])


try:
    import httpx

    from cf_robot.web import create_app
    from cf_robot.web_calibration import BrowserCalibration
    from cf_robot.web_pose import BrowserPoseMonitor
except ImportError:
    BrowserCalibration = None


@unittest.skipIf(BrowserCalibration is None, "install web dependencies")
class IndependentMonitorTests(unittest.IsolatedAsyncioTestCase):
    async def test_leader_only_never_connects_to_follower(self):
        def reader(port, arm_id, stop, publish):
            publish({"type": "pose_sample", "side": "leader", "sampled_at": time.time()})
            stop.wait()

        monitor = BrowserPoseMonitor("ws://unused", "/dev/fake", "leader", leader_reader=reader)
        with patch("cf_robot.web_pose.connect") as connection:
            monitor.start("leader")
            try:
                async with asyncio.timeout(2):
                    while not monitor.status.get("leader"):
                        await asyncio.sleep(0.01)
            finally:
                monitor.stop()
                await monitor.task
            connection.assert_not_called()
        self.assertEqual(monitor.status["state"], "idle")

    async def test_calibration_watchdog_cancels_and_waits_for_cleanup(self):
        calibration = BrowserCalibration("ws://unused", "/dev/fake", "leader", heartbeat_timeout=0.05)
        cleaned = []

        async def local():
            try:
                await asyncio.Event().wait()
            finally:
                await asyncio.sleep(0.03)
                cleaned.append(True)

        with patch.object(calibration, "local", side_effect=local):
            calibration.start("leader")
            await calibration.task
        self.assertFalse(calibration.running)
        self.assertEqual(cleaned, [True])
        self.assertIn("heartbeat lost", calibration.status["error"])

    async def test_calibration_cancel_is_not_reported_as_success(self):
        calibration = BrowserCalibration("ws://unused", "/dev/fake", "leader")
        with patch.object(calibration, "local", side_effect=lambda: asyncio.sleep(10)):
            calibration.start("leader")
            calibration.stop()
            await calibration.task
        self.assertEqual(calibration.status["state"], "cancelled")

    async def test_calibration_routes_require_pairing_confirmation_and_exclusive_ownership(self):
        async def local(calibration):
            calibration.status["state"] = "running"
            await asyncio.Event().wait()

        with patch.dict(os.environ, {"CF_ROBOT_ORIGIN_KEY": "t" * 64}):
            app = create_app(
                "ws://127.0.0.1:8765", "/dev/fake", "leader",
                operator_origin="https://console.example", bridge_token="pair",
            )
        headers = {"Origin": "https://console.example", "X-CF-Robot-Bridge": "pair"}
        with patch.object(BrowserCalibration, "local", local):
            async with httpx.AsyncClient(
                transport=httpx.ASGITransport(app=app), base_url="http://127.0.0.1:8080",
            ) as client:
                for path in ("status", "start", "input", "stop", "heartbeat"):
                    method = client.get if path == "status" else client.post
                    response = await method(f"/api/calibration/{path}")
                    self.assertEqual(response.status_code, 403)
                response = await client.post("/api/calibration/start", headers=headers, json={
                    "side": "leader", "confirmation": "no",
                })
                self.assertEqual(response.status_code, 409)
                response = await client.post("/api/calibration/start", headers=headers, json={
                    "side": "leader", "confirmation": "CALIBRATE",
                })
                self.assertEqual(response.status_code, 200)
                await asyncio.sleep(0.02)
                try:
                    for path, body in (
                        ("/api/pose/start", {"sides": "leader"}),
                        ("/api/start", {}),
                        ("/api/calibration/start", {"side": "follower", "confirmation": "CALIBRATE"}),
                    ):
                        response = await client.post(path, headers=headers, json=body)
                        self.assertEqual(response.status_code, 409)
                    response = await client.post("/api/calibration/input", headers=headers, json={"text": "c\n"})
                    self.assertEqual(response.status_code, 409)
                    await client.post("/api/estop", headers=headers)
                    async with asyncio.timeout(2):
                        while (await client.get("/api/calibration/status", headers=headers)).json()["running"]:
                            await asyncio.sleep(0.02)
                    response = await client.post("/api/calibration/start", headers=headers, json={
                        "side": "leader", "confirmation": "CALIBRATE",
                    })
                    self.assertEqual(response.status_code, 409)
                finally:
                    await client.post("/api/calibration/stop", headers=headers)
                    await asyncio.sleep(0.2)


if __name__ == "__main__":
    unittest.main()
