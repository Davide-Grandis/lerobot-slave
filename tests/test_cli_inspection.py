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

    def test_invalid_save_syntax_is_rejected(self):
        for line in ("SAVE F", "SAVE X file.json", "SAVE F file with spaces.json"):
            with self.subTest(line=line), self.assertRaises(ValueError):
                parse_pose_command(line, 1)


class InspectionExitTests(unittest.IsolatedAsyncioTestCase):
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
