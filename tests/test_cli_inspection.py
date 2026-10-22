import asyncio
import json
import tempfile
import unittest
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from cf_robot.pose_compare_client import (
    InspectionReports,
    monitor_leader_only,
    monitor_poses,
    parse_pose_command,
)


class InspectionReportTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.enterContext(patch("cf_robot.pose_compare_client.tempfile.tempdir", directory.name))
        self.report = {"joints": {"gripper.pos": {"saved": {"homing_offset": -873}}}}
        self.samples = {"follower": {"calibration": self.report}}
        self.inspections = InspectionReports()

    def test_inspect_snapshot_remains_after_sample_changes(self):
        message = self.inspections.command(parse_pose_command("INSPECT F", 1), self.samples)
        self.report["joints"].clear()
        output = StringIO()
        with patch("cf_robot.pose_compare_client.sys.stdout", output):
            self.inspections.print_on_exit()
        self.assertIn(message, output.getvalue())
        self.assertIn("-873", output.getvalue())
        self.assertIn("FOLLOWER INSPECTION", output.getvalue())

    def test_save_handles_quoted_paths_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "follower inspection.json"
            command = parse_pose_command(f'SAVE F "{path}"', 1)
            result = self.inspections.command(command, self.samples)
            self.assertIn("Saved follower inspection", result)
            self.assertEqual(json.loads(path.read_text()), self.report)
            original = path.read_bytes()
            self.report["new"] = True
            result = self.inspections.command(command, self.samples)
            self.assertIn("Could not save", result)
            self.assertEqual(path.read_bytes(), original)

    def test_unavailable_arm_does_not_export_or_print_null(self):
        result = self.inspections.command(parse_pose_command("INSPECT L", 1), self.samples)
        self.assertIn("unavailable", result)
        self.assertEqual(self.inspections.reports, {})

    def test_status_reports_are_retained_separately_for_each_joint(self):
        for joint_id in (2, 3):
            self.inspections.retain_diagnostics({
                "mode": "diagnostics", "joint_id": joint_id,
                "servo_diagnostics": {"registers": {"Present_Position": 800 + joint_id}, "errors": {}},
            })
        output = StringIO()
        with patch("cf_robot.pose_compare_client.sys.stdout", output):
            self.inspections.print_on_exit()
        self.assertIn("FOLLOWER J2 STATUS", output.getvalue())
        self.assertIn("FOLLOWER J3 STATUS", output.getvalue())
        self.assertIn('"Present_Position": 802', output.getvalue())
        self.assertIn('"Present_Position": 803', output.getvalue())

    def test_status_automatically_saves_each_snapshot_to_a_unique_file(self):
        report = {
            "mode": "diagnostics", "joint_id": 3,
            "servo_diagnostics": {"registers": {"Present_Position": 2182}, "errors": {}},
        }
        first_status = self.inspections.retain_diagnostics(report)
        first_path = self.inspections.diagnostic_paths[3]
        self.assertIn(str(first_path), first_status)
        self.assertEqual(json.loads(first_path.read_text()), report)
        report["servo_diagnostics"]["registers"]["Present_Position"] = 2183
        self.inspections.retain_diagnostics(report)
        second_path = self.inspections.diagnostic_paths[3]
        self.assertNotEqual(first_path, second_path)
        self.assertEqual(json.loads(first_path.read_text())["servo_diagnostics"]["registers"]["Present_Position"], 2182)
        self.assertEqual(json.loads(second_path.read_text()), report)

    def test_status_save_failure_keeps_copyable_snapshot(self):
        report = {
            "mode": "diagnostics", "joint_id": 3,
            "servo_diagnostics": {"registers": {}, "errors": {}},
        }
        with patch("cf_robot.pose_compare_client.tempfile.NamedTemporaryFile", side_effect=OSError("disk full")):
            status = self.inspections.retain_diagnostics(report)
        self.assertIn("Could not auto-save", status)
        self.assertEqual(json.loads(self.inspections.diagnostics[3]), report)
        self.assertNotIn(3, self.inspections.diagnostic_paths)

    def test_save_status_exports_cached_report_without_overwriting(self):
        report = {
            "mode": "diagnostics", "joint_id": 3,
            "servo_diagnostics": {"registers": {"Torque_Enable": 1}, "errors": {}},
        }
        self.inspections.retain_diagnostics(report)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "J3 status.json"
            command = parse_pose_command(f'SAVE J3 "{path}"', 1)
            self.assertEqual(command["kind"], "save_diagnostics")
            self.assertIn("Saved follower J3 STATUS", self.inspections.command(command, {}))
            self.assertEqual(json.loads(path.read_text()), report)
            self.assertIn("Could not save", self.inspections.command(command, {}))
            self.assertEqual(json.loads(path.read_text()), report)

    def test_save_status_without_capture_does_not_create_a_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            command = parse_pose_command(f'SAVE J3 "{path}"', 1)
            self.assertIn("request J3 STATUS first", self.inspections.command(command, {}))
            self.assertFalse(path.exists())

    def test_invalid_save_syntax_is_rejected(self):
        for line in ("SAVE F", "SAVE X file.json", "SAVE F file with spaces.json"):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_pose_command(line, 1)

    def test_restore_confirmation_uses_preview_token_once_without_copying(self):
        self.assertEqual(parse_pose_command("J6 RESTORE CONFIRM", 1)["kind"], "confirm_restore")
        with self.assertRaisesRegex(ValueError, "PREVIEW"):
            self.inspections.confirm_restore(1)
        self.inspections.retain_restore({
            "mode": "calibration_preview", "state": "preview",
            "calibration_preview": {"token": "a" * 32},
        })
        command = self.inspections.confirm_restore(2)
        self.assertEqual(command["token"], "a" * 32)
        self.assertEqual(command["confirmation"], "RESTORE J6")
        with self.assertRaises(ValueError):
            self.inspections.confirm_restore(3)
        output = StringIO()
        with patch("cf_robot.pose_compare_client.sys.stdout", output):
            self.inspections.print_on_exit()
        self.assertIn("CALIBRATION_PREVIEW", output.getvalue())


class InspectionExitTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.enterContext(patch("cf_robot.pose_compare_client.tempfile.tempdir", directory.name))

    async def test_status_display_is_compact_and_report_survives_exit(self):
        messages = asyncio.Queue()
        terminal = SimpleNamespace(
            lines=asyncio.Queue(), changed=asyncio.Event(),
            stop_requested=asyncio.Event(), buffer="", close=lambda: print("TERMINAL RESTORED"),
        )
        report = {
            "type": "command_result", "mode": "diagnostics", "joint_id": 3,
            "command_id": 1, "state": "succeeded",
            "servo_diagnostics": {
                "registers": {"Present_Position": 2182, "Goal_Position": 2200, "Torque_Enable": 1},
                "errors": {},
            },
        }
        statuses, sent = [], []
        stage = 0

        class Socket:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def recv(self):
                return json.dumps(await messages.get())

            async def send(self, raw):
                message = json.loads(raw)
                sent.append(message)
                if message["type"] == "start":
                    messages.put_nowait({"type": "monitor_started"})
                    messages.put_nowait({"type": "pose_sample", "side": "follower"})
                elif message["type"] == "command":
                    messages.put_nowait(report)
                elif message["type"] == "stop":
                    messages.put_nowait({"type": "monitor_finished"})

        def render(build, leader, follower, buffer, status):
            nonlocal stage
            statuses.append(status)
            if stage == 0:
                stage = 1
                terminal.lines.put_nowait("J3 STATUS")
            elif stage == 1 and "SAVE J3" in status:
                stage = 2
                terminal.lines.put_nowait("q")

        output = StringIO()
        with patch("cf_robot.pose_compare_client.access_headers", return_value={}), patch(
            "cf_robot.pose_compare_client.connect", return_value=Socket(),
        ), patch("cf_robot.pose_compare_client.TerminalInput", return_value=terminal), patch(
            "cf_robot.pose_compare_client._render", side_effect=render,
        ), patch("cf_robot.pose_compare_client.sys.stdout", output):
            await asyncio.wait_for(monitor_poses("ws://unused", None, "leader", "test"), 2)
        self.assertFalse(any('"servo_diagnostics"' in status for status in statuses))
        self.assertIn(json.dumps(report, indent=2), output.getvalue())
        self.assertLess(output.getvalue().index("TERMINAL RESTORED"), output.getvalue().index("FOLLOWER J3 STATUS"))
        self.assertEqual([m["type"] for m in sent], ["start", "command", "stop"])

    async def test_follower_exit_retains_report_after_stop_redraw(self):
        messages = asyncio.Queue()
        report = {"joints": {"gripper.pos": {"live": {"homing_offset": -873}}}}
        terminal = SimpleNamespace(
            lines=asyncio.Queue(), changed=asyncio.Event(),
            stop_requested=asyncio.Event(), buffer="", close=lambda: None,
        )
        statuses = []

        class Socket:
            async def __aenter__(self):
                return self

            async def __aexit__(self, *args):
                pass

            async def recv(self):
                return json.dumps(await messages.get())

            async def send(self, raw):
                kind = json.loads(raw)["type"]
                if kind == "start":
                    messages.put_nowait({"type": "monitor_started"})
                    messages.put_nowait({
                        "type": "pose_sample", "side": "follower", "calibration": report,
                    })
                elif kind == "stop":
                    messages.put_nowait({"type": "monitor_finished"})

        def render(build, leader, follower, buffer, status):
            statuses.append(status)
            if len(statuses) == 1:
                terminal.lines.put_nowait("INSPECT F")
            elif status.startswith("{"):
                terminal.lines.put_nowait("q")

        output = StringIO()
        with patch("cf_robot.pose_compare_client.access_headers", return_value={}), patch(
            "cf_robot.pose_compare_client.connect", return_value=Socket(),
        ), patch("cf_robot.pose_compare_client.TerminalInput", return_value=terminal), patch(
            "cf_robot.pose_compare_client._render", side_effect=render,
        ), patch("cf_robot.pose_compare_client.sys.stdout", output):
            await asyncio.wait_for(monitor_poses("ws://unused", None, "leader", "test"), 2)
        self.assertIn("Stopping monitor...", statuses)
        self.assertIn("FOLLOWER INSPECTION", output.getvalue())
        self.assertIn(json.dumps(report, indent=2), output.getvalue())

    async def test_leader_exit_prints_inspection_after_terminal_restore(self):
        terminal = SimpleNamespace(
            lines=asyncio.Queue(), changed=asyncio.Event(),
            stop_requested=asyncio.Event(), buffer="",
            close=lambda: print("TERMINAL RESTORED"),
        )
        report = {"joints": {"gripper.pos": {"live": {"homing_offset": -873}}}}
        requested = False

        def reader(leader, loop, samples, stop):
            loop.call_soon_threadsafe(samples.put_nowait, {
                "type": "leader_sample", "calibration": report,
            })
            stop.wait()

        def render(build, leader, follower, buffer, status):
            nonlocal requested
            if not requested:
                terminal.lines.put_nowait("INSPECT L")
                requested = True
            elif status.startswith("{"):
                terminal.lines.put_nowait("q")

        output = StringIO()
        with patch("cf_robot.pose_compare_client.make_leader", return_value=object()), patch(
            "cf_robot.pose_compare_client.TerminalInput", return_value=terminal,
        ), patch("cf_robot.pose_compare_client._read_leader", side_effect=reader), patch(
            "cf_robot.pose_compare_client._render", side_effect=render,
        ), patch("cf_robot.pose_compare_client.sys.stdout", output):
            await asyncio.wait_for(monitor_leader_only("/dev/fake", "leader", "test"), 2)
        content = output.getvalue()
        self.assertLess(content.index("TERMINAL RESTORED"), content.index("LEADER INSPECTION"))
        self.assertIn(json.dumps(report, indent=2), content)


if __name__ == "__main__":
    unittest.main()
