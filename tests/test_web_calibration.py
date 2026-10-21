import asyncio
import json
import unittest
from unittest.mock import AsyncMock, patch

try:
    from cf_robot.web_calibration import BrowserCalibration
except ImportError:
    BrowserCalibration = None


class CalibrationSocket:
    def __init__(self, exit_code=0):
        self.messages = asyncio.Queue()
        self.sent = []
        self.exit_code = exit_code

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        pass

    async def send(self, data):
        message = json.loads(data)
        self.sent.append(message)
        if message["type"] == "start":
            await self.messages.put({"type": "calibration_started", "version": 1})
            await self.messages.put({"type": "calibration_output", "text": "Enter or c: "})
        elif message["type"] == "input":
            await self.messages.put({"type": "calibration_finished", "exit_code": self.exit_code})

    async def recv(self):
        return json.dumps(await self.messages.get())


@unittest.skipIf(BrowserCalibration is None, "install web dependencies")
class GuidedCalibrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_remote_guided_input_and_exit_status(self):
        for code in (0, 2):
            with self.subTest(code=code):
                socket = CalibrationSocket(code)
                calibration = BrowserCalibration("ws://127.0.0.1", None, "leader")
                with patch("cf_robot.web_calibration.access_headers", return_value={}), patch(
                    "cf_robot.web_calibration.connect", return_value=socket,
                ):
                    calibration.start("follower")
                    async with asyncio.timeout(2):
                        while "Enter or c" not in calibration.status["output"]:
                            await asyncio.sleep(0.01)
                        calibration.inputs.put_nowait("c")
                        await calibration.task
                self.assertEqual(calibration.status["state"], "finished" if code == 0 else "error")
                self.assertEqual(socket.sent[1], {"version": 1, "type": "input", "text": "c"})

    async def test_remote_cancel_sends_cancel_and_never_starts_local_calibrator(self):
        socket = CalibrationSocket()
        calibration = BrowserCalibration("ws://127.0.0.1", None, "leader")
        with patch("cf_robot.web_calibration.access_headers", return_value={}), patch(
            "cf_robot.web_calibration.connect", return_value=socket,
        ), patch.object(calibration, "local") as local:
            calibration.start("follower")
            async with asyncio.timeout(2):
                while calibration.status["state"] != "running":
                    await asyncio.sleep(0.01)
                calibration.stop()
                await calibration.task
            local.assert_not_called()
        self.assertEqual(calibration.status["state"], "cancelled")
        self.assertEqual(socket.sent[-1]["type"], "cancel")

    async def test_local_cancel_terminates_and_reaps_fixed_subprocess(self):
        calibration = BrowserCalibration("ws://127.0.0.1", "/dev/fake", "my-leader")
        exited = asyncio.Event()
        output = asyncio.StreamReader()

        class Process:
            returncode = None
            stdout = output
            stdin = None
            terminated = False

            async def wait(self):
                await exited.wait()
                return self.returncode

            def terminate(self):
                self.terminated = True
                self.returncode = -15
                output.feed_eof()
                exited.set()

        process = Process()
        spawn = AsyncMock(return_value=process)
        with patch("cf_robot.web_calibration.Path.is_file", return_value=True), patch(
            "cf_robot.web_calibration.asyncio.create_subprocess_exec", spawn,
        ):
            calibration.start("leader")
            async with asyncio.timeout(2):
                while calibration.status["state"] != "running":
                    await asyncio.sleep(0.01)
                calibration.stop()
                await calibration.task
        self.assertTrue(process.terminated)
        self.assertEqual(calibration.status["state"], "cancelled")
        self.assertEqual(spawn.call_args.args[1:], (
            "--teleop.type=so101_leader", "--teleop.port=/dev/fake", "--teleop.id=my-leader",
        ))
        self.assertNotIn("shell", spawn.call_args.kwargs)


if __name__ == "__main__":
    unittest.main()
