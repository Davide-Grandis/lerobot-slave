import asyncio
import contextlib
import importlib.util
import io
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import AsyncMock, patch

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect
from test_control import FakeFollower

from cf_robot.follower import create_app
from cf_robot.system_client import main, request_command, system_url
from cf_robot.system_commands import COMMANDS, power_result, queue_power, run_command


class SystemCommandTests(unittest.TestCase):
    key = "k" * 64
    def setUp(self):
        self.headers = {"X-CF-Robot-Key": self.key}
        self.app = create_app("unused", "test", origin_key=self.key)

    def request(self, command=COMMANDS[0], confirmed=False):
        return {
            "version": 1, "type": "system_command",
            "command": command, "confirmed": confirmed,
        }

    def test_authentication_required(self):
        with (
            TestClient(self.app) as client,
            self.assertRaises(WebSocketDisconnect),
            client.websocket_connect("/system"),
        ):
            pass

    def test_interface_response_contains_stdout_stderr_and_exit_status(self):
        result = {"stdout": "2: eth0 inet 192.0.2.1/24\n", "stderr": "warning\n", "exit_code": 3}
        with (
            patch("cf_robot.system_commands.run_command", new=AsyncMock(return_value=result)) as run,
            TestClient(self.app) as client,
            client.websocket_connect("/system", headers=self.headers) as socket,
        ):
            socket.send_json(self.request())
            response = socket.receive_json()
        self.assertEqual(response["type"], "system_result")
        self.assertEqual({key: response[key] for key in result}, result)
        run.assert_awaited_once_with("/usr/sbin/ip", "-details", "address", "show")

    def test_rejects_shell_injection_malformed_and_unconfirmed_requests(self):
        requests = [
            self.request("sudo reboot; touch /tmp/injected", True),
            self.request(COMMANDS[1]),
            self.request(COMMANDS[2], "yes"),
            {**self.request(), "extra": "argument"},
            ["sudo reboot"],
            self.request(["sudo reboot"]),
            self.request("cat /etc/shadow"),
        ]
        with (
            patch("cf_robot.system_commands.run_command", new=AsyncMock()) as run,
            TestClient(self.app) as client,
        ):
            for request in requests:
                with (
                    self.subTest(request=request),
                    client.websocket_connect("/system", headers=self.headers) as socket,
                ):
                    socket.send_json(request)
                    self.assertEqual(socket.receive_json()["type"], "error")
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_text("{")
                self.assertEqual(socket.receive_json()["type"], "error")
        run.assert_not_awaited()

    def test_power_result_blocks_new_sessions_and_rejects_second_request(self):
        result = {"exit_code": 0, "stdout": "Reboot scheduled\n", "stderr": ""}
        with (
            patch("cf_robot.system_commands.queue_power", new=AsyncMock()) as queue,
            patch("cf_robot.system_commands.power_result", new=AsyncMock(return_value=result)),
            TestClient(self.app) as client,
        ):
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_json(self.request(COMMANDS[1], True))
                self.assertEqual(socket.receive_json()["type"], "system_queued")
                self.assertEqual(socket.receive_json()["stdout"], "Reboot scheduled\n")
            with (
                self.assertRaises(WebSocketDisconnect),
                client.websocket_connect("/control", headers=self.headers),
            ):
                pass
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_json(self.request(COMMANDS[2], True))
                self.assertEqual(socket.receive_json()["type"], "error")
        self.assertEqual(queue.await_count, 1)

    def test_failed_power_queue_releases_interlock(self):
        with (
            patch("cf_robot.system_commands.queue_power", new=AsyncMock(side_effect=RuntimeError("not installed"))),
            TestClient(self.app) as client,
        ):
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_json(self.request(COMMANDS[1], True))
                self.assertIn("not installed", socket.receive_json()["message"])
            self.assertFalse(client.get("/health").json()["control_busy"])
            # The interlock must permit a new request (not report busy).
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_json(self.request(COMMANDS[1], True))
                self.assertIn("not installed", socket.receive_json()["message"])

    def test_power_rejected_while_update_active(self):
        with (
            patch("cf_robot.system_commands.queue_power", new=AsyncMock()) as queue,
            patch("cf_robot.follower.UPDATE_REQUEST") as update,
        ):
            update.exists.return_value = True
            app = create_app("unused", "test", origin_key=self.key)
            with (
                TestClient(app) as client,
                client.websocket_connect("/system", headers=self.headers) as socket,
            ):
                socket.send_json(self.request(COMMANDS[2], True))
                self.assertEqual(socket.receive_json()["type"], "error")
            queue.assert_not_awaited()

    def test_power_rejected_while_control_active(self):
        app = create_app("unused", "test", factory=FakeFollower, origin_key=self.key)
        with (
            patch("cf_robot.system_commands.queue_power", new=AsyncMock()) as queue,
            TestClient(app) as client,
            client.websocket_connect("/control", headers=self.headers) as control,
        ):
            self.assertEqual(control.receive_json()["type"], "ready")
            with client.websocket_connect("/system", headers=self.headers) as socket:
                socket.send_json(self.request(COMMANDS[1], True))
                self.assertIn("stop all", socket.receive_json()["message"])
            queue.assert_not_awaited()

    def test_failed_schedule_releases_interlock_but_unknown_outcome_does_not(self):
        for unknown in (False, True):
            with self.subTest(unknown=unknown):
                app = create_app("unused", "test", origin_key=self.key)
                with (
                    patch("cf_robot.system_commands.queue_power", new=AsyncMock()) as queue,
                    patch("cf_robot.system_commands.power_result", new=AsyncMock(return_value={
                        "exit_code": 1, "stdout": "", "stderr": "failure",
                        "outcome_unknown": unknown,
                    })),
                    TestClient(app) as client,
                ):
                    for _ in range(2):
                        with client.websocket_connect("/system", headers=self.headers) as socket:
                            socket.send_json(self.request(COMMANDS[1], True))
                            response = socket.receive_json()
                            if response["type"] == "system_queued":
                                self.assertEqual(socket.receive_json()["exit_code"], 1)
                    self.assertEqual(queue.await_count, 1 if unknown else 2)

    def test_queue_and_status_use_matching_request_id(self):
        with tempfile.TemporaryDirectory() as directory:
            request = Path(directory) / "request"
            status = Path(directory) / "status"
            result = {"request_id": "a" * 32, "exit_code": 0, "stdout": "", "stderr": ""}
            with (
                patch("cf_robot.system_commands.POWER_REQUEST", request),
                patch("cf_robot.system_commands.POWER_STATUS", status),
                patch("cf_robot.system_commands.run_command", new=AsyncMock(return_value={"exit_code": 0})),
            ):
                asyncio.run(queue_power(COMMANDS[1], result["request_id"]))
                self.assertEqual(json.loads(request.read_text())["command"], COMMANDS[1])
                self.assertEqual(request.stat().st_mode & 0o777, 0o600)
                status.write_text(json.dumps(result))
                self.assertEqual(asyncio.run(power_result(result["request_id"])), result)
                with self.assertRaises(RuntimeError):
                    asyncio.run(queue_power(COMMANDS[2], "b" * 32))

    def test_run_command_collects_actual_output(self):
        # This test only runs a harmless local interpreter, never a power command.
        result = asyncio.run(run_command(
            sys.executable, "-c",
            "import sys; print('interfaces'); print('error', file=sys.stderr); sys.exit(4)",
        ))
        self.assertEqual(result, {"exit_code": 4, "stdout": "interfaces\n", "stderr": "error\n"})

    def test_timed_out_subprocess_is_killed_and_reaped(self):
        process = AsyncMock()
        process.returncode = None
        process.kill = unittest.mock.Mock()

        async def timeout(awaitable, timeout):
            await awaitable
            raise TimeoutError

        with (
            patch("cf_robot.system_commands.asyncio.create_subprocess_exec", return_value=process),
            patch("cf_robot.system_commands.asyncio.wait_for", side_effect=timeout),
            self.assertRaises(TimeoutError),
        ):
            asyncio.run(run_command("unused"))
        process.kill.assert_called_once()
        process.wait.assert_awaited_once()

    def test_stale_power_status_does_not_count_as_completion(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch("cf_robot.system_commands.asyncio.sleep", new=AsyncMock()),
        ):
            status = Path(directory) / "status"
            status.write_text(json.dumps({"request_id": "old", "exit_code": 0}))
            with (
                patch("cf_robot.system_commands.POWER_STATUS", status),
                self.assertRaisesRegex(TimeoutError, "outcome is unknown"),
            ):
                asyncio.run(power_result("new"))


class SystemClientTests(unittest.TestCase):
    def test_url_requires_wss_and_preserves_control_hostname(self):
        self.assertEqual(system_url("wss://robot.example.com/control"), "wss://robot.example.com/system")
        for url in (
            "ws://robot.example.com/control", "wss://id:secret@robot.example.com/control",
            "wss://robot.example.com/video", "wss://robot.example.com/control?key=secret",
        ):
            with self.assertRaises(ValueError):
                system_url(url)

    def test_cli_prints_response_and_propagates_exit_status(self):
        with (
            patch.dict(os.environ, {"CF_ROBOT_CONTROL_URL": "wss://robot.example.com/control"}),
            patch("sys.argv", ["cf-robot-system", "interfaces"]),
            patch("cf_robot.system_client.request_command", new=AsyncMock(return_value={
                "stdout": "eth0\n", "stderr": "failed\n", "exit_code": 4,
            })),
            contextlib.redirect_stdout(io.StringIO()) as stdout,
            contextlib.redirect_stderr(io.StringIO()) as stderr,
            self.assertRaises(SystemExit) as exited,
        ):
            main()
        self.assertEqual(exited.exception.code, 4)
        self.assertIn("eth0\n", stdout.getvalue())
        self.assertIn("Pi exit status: 4", stdout.getvalue())
        self.assertEqual(stderr.getvalue(), "failed\n")

    def test_cli_does_not_send_power_without_confirmation(self):
        with (
            patch("sys.argv", ["cf-robot-system", "sudo reboot", "--url", "wss://robot.example.com/control"]),
            patch("builtins.input", return_value="no"),
            patch("cf_robot.system_client.request_command", new=AsyncMock()) as request,
            contextlib.redirect_stdout(io.StringIO()),
        ):
            main()
        request.assert_not_awaited()

    def test_wss_client_sends_headers_and_receives_result(self):
        socket = AsyncMock()
        socket.recv.side_effect = [
            json.dumps({"type": "system_queued", "request_id": "a", "message": "queued"}),
            json.dumps({
                "version": 1, "type": "system_result", "request_id": "a",
                "command": COMMANDS[1], "stdout": "scheduled", "stderr": "", "exit_code": 0,
            }),
        ]
        connection = AsyncMock()
        connection.__aenter__.return_value = socket
        with (
            patch("cf_robot.system_client.connect", return_value=connection) as connect,
            patch("cf_robot.system_client.access_headers", return_value={"X-CF-Robot-Key": "key"}),
            contextlib.redirect_stdout(io.StringIO()),
        ):
            result = asyncio.run(request_command("wss://robot.example.com/control", COMMANDS[1], True))
        self.assertEqual(result["stdout"], "scheduled")
        self.assertEqual(connect.call_args.kwargs["additional_headers"], {"X-CF-Robot-Key": "key"})
        self.assertTrue(json.loads(socket.send.call_args.args[0])["confirmed"])


class PowerHelperTests(unittest.TestCase):
    def test_root_helper_only_executes_fixed_commands_and_records_response(self):
        spec = importlib.util.spec_from_file_location(
            "power_helper", Path(__file__).resolve().parents[1] / "deploy/cf-robot-power.py"
        )
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        with tempfile.TemporaryDirectory() as directory:
            helper.DIRECTORY = Path(directory)
            for command, arguments in helper.OPERATIONS.items():
                request = helper.DIRECTORY / "power-request"
                request.write_text(json.dumps({"command": command, "request_id": "a" * 32}))
                with patch.object(helper.subprocess, "run", return_value=subprocess.CompletedProcess(
                    [], 0, stdout="scheduled\n", stderr="",
                )) as run:
                    helper.main()
                self.assertEqual(run.call_args.args[0], arguments)
                self.assertEqual(json.loads((helper.DIRECTORY / "power-status.json").read_text())["stdout"], "scheduled\n")
            request.write_text(json.dumps({"command": "sudo reboot; whoami", "request_id": "a" * 32}))
            with patch.object(helper.subprocess, "run") as run:
                with self.assertRaises(ValueError):
                    helper.main()
                run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
