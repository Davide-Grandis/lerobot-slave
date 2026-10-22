import asyncio
from copy import deepcopy
from io import StringIO
import json
import time
import unittest
from unittest.mock import AsyncMock, patch

from cf_robot.protocol import JOINTS, SAFE_MONITOR_MOVES_CAPABILITY
from cf_robot.sweep_client import main, run_sweep, sweep_plan


def sample():
    return {
        "type": "pose_sample", "side": "follower", "sampled_at": time.time(),
        "raw": dict.fromkeys(JOINTS, 2000), "torque": dict.fromkeys(JOINTS, True),
        "limits": {joint: {"min": 100, "max": 3900} for joint in JOINTS},
        "calibration": {
            "state": "consistent",
            "joints": {
                joint: {
                    "state": "consistent",
                    "mode": "range_0_100" if index == 6 else "degrees",
                    "saved": {"range_min": 100, "range_max": 3900},
                    "live": {"range_min": 100, "range_max": 3900},
                    "angular_calibration": {"state": "verified", "zero_ticks": 2000},
                } for index, joint in enumerate(JOINTS, 1)
            },
        },
    }


class SweepPlanTests(unittest.TestCase):
    def test_zero_comes_from_independent_reference_and_gripper_uses_min(self):
        pose = sample()
        pose["calibration"]["joints"][JOINTS[4]]["angular_calibration"]["zero_ticks"] = 2024.5
        plan = sweep_plan(pose, (1, 2, 3, 4, 5, 6))
        self.assertEqual([item["id"] for item in plan], list(range(1, 7)))
        self.assertEqual(plan[4]["zero"], 2025)
        self.assertEqual(plan[5]["zero"], 100)
        self.assertIn("not degrees", plan[5]["zero_label"])

    def test_missing_zero_provisional_range_or_mismatched_limits_fail_closed(self):
        for change in ("zero", "pending", "limits", "warnings"):
            pose = sample()
            item = pose["calibration"]["joints"][JOINTS[2]]
            if change == "zero":
                item["angular_calibration"].pop("zero_ticks")
            elif change == "pending":
                item["angular_calibration"]["pending_range_max"] = True
            elif change == "limits":
                item["live"]["range_max"] = 4095
            else:
                pose["calibration"]["state"] = "warning"
            with self.subTest(change=change), self.assertRaises(ValueError):
                sweep_plan(pose, (3,))

    def test_noninteractive_cli_is_rejected_before_connecting(self):
        with patch("sys.argv", ["sweep", "--url", "wss://robot.example/control"]), patch(
            "cf_robot.sweep_client.sys.stdin.isatty", return_value=False,
        ), patch("cf_robot.sweep_client.connect") as connect, patch("sys.stderr", StringIO()):
            with self.assertRaises(SystemExit):
                main()
        connect.assert_not_called()


class FakeSocket:
    def __init__(self):
        self.messages = asyncio.Queue()
        self.pose = sample()
        self.sent = []
        self.capabilities = [SAFE_MONITOR_MOVES_CAPABILITY]
        self.producer = None
        self.fail_joint = None
        self.hang_move = False
        self.move_started = asyncio.Event()

    async def __aenter__(self):
        return self

    async def __aexit__(self, *args):
        if self.producer:
            self.producer.cancel()
            try:
                await self.producer
            except asyncio.CancelledError:
                pass

    def __aiter__(self):
        return self

    async def __anext__(self):
        return await self.recv()

    async def recv(self):
        return json.dumps(await self.messages.get())

    async def publish(self):
        while True:
            self.pose["sampled_at"] = time.time()
            self.messages.put_nowait(deepcopy(self.pose))
            await asyncio.sleep(0.005)

    async def send(self, raw):
        message = json.loads(raw)
        self.sent.append(message)
        if message["type"] == "start":
            self.messages.put_nowait({
                "type": "monitor_started", "version": 1,
                "capabilities": self.capabilities,
            })
            self.producer = asyncio.create_task(self.publish())
        elif message["type"] == "stop":
            if self.hang_move:
                self.messages.put_nowait({
                    "type": "command_result", "command_id": 3,
                    "joint_id": 3, "state": "cancelled",
                })
            self.messages.put_nowait({"type": "monitor_finished"})
        elif message["type"] == "command":
            joint_id = message["joint_id"]
            state = "succeeded"
            if message["mode"] == "torque":
                for index, joint in enumerate(JOINTS, 1):
                    if joint_id == "*" or joint_id == index:
                        self.pose["torque"][joint] = message["enabled"]
            else:
                joint = JOINTS[joint_id - 1]
                assert self.pose["torque"] == {name: name == joint for name in JOINTS}
                if self.hang_move:
                    self.move_started.set()
                    return
                if joint_id == self.fail_joint:
                    state = "failed"
                else:
                    self.pose["raw"][joint] = message["value"]
            self.messages.put_nowait({
                "type": "command_result", "command_id": message["command_id"],
                "joint_id": joint_id, "state": state, "message": "test result",
            })


class SweepClientTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.socket = FakeSocket()
        self.confirm = AsyncMock()
        self.enterContext(patch("cf_robot.sweep_client.access_headers", return_value={}))
        self.enterContext(patch("cf_robot.sweep_client.connect", return_value=self.socket))
        self.enterContext(patch("sys.stdout", StringIO()))

    async def run_sweep(self, joints=(1, 2, 3, 4, 5, 6), confirm=None):
        return await asyncio.wait_for(
            run_sweep("wss://robot.example/control", joints, confirm or self.confirm), 2,
        )

    def commands(self):
        return [m for m in self.socket.sent if m["type"] == "command"]

    async def test_full_sequence_confirms_release_sweep_zero_and_final_release(self):
        await self.run_sweep()
        moves = [m for m in self.commands() if m["mode"] == "ticks"]
        self.assertEqual([m["joint_id"] for m in moves], [j for j in range(1, 7) for _ in range(3)])
        self.assertEqual([m["value"] for m in moves], [v for _ in range(5) for v in (100, 3900, 2000)] + [100, 3900, 100])
        self.assertEqual(self.confirm.await_count, 19)
        prompts = [call.args[0] for call in self.confirm.await_args_list]
        self.assertIn("RELEASE ALL", prompts[0])
        self.assertIn("enable ONLY J1", prompts[1])
        self.assertIn("0 degrees", prompts[2])
        self.assertIn("0% opening", prompts[-2])
        self.assertFalse(any(self.socket.pose["torque"].values()))
        self.assertEqual(self.socket.sent[-1]["type"], "stop")

    async def test_old_pi_rejected_before_any_confirmation_or_torque_write(self):
        self.socket.capabilities = []
        with self.assertRaisesRegex(RuntimeError, "safe-monitor-moves update"):
            await self.run_sweep()
        self.assertEqual(self.commands(), [])
        self.confirm.assert_not_awaited()

    async def test_bad_calibration_rejected_without_torque_changes(self):
        self.socket.pose["calibration"]["state"] = "warning"
        with self.assertRaisesRegex(ValueError, "consistent"):
            await self.run_sweep()
        self.assertEqual(self.commands(), [])

    async def test_failed_joint_aborts_without_max_zero_next_joint_or_auto_release(self):
        self.socket.fail_joint = 3
        with self.assertRaisesRegex(RuntimeError, "without retry"):
            await self.run_sweep()
        moves = [m for m in self.commands() if m["mode"] == "ticks"]
        self.assertEqual([m["joint_id"] for m in moves], [1, 1, 1, 2, 2, 2, 3])
        self.assertTrue(self.socket.pose["torque"][JOINTS[2]])
        self.assertEqual(self.commands()[-1]["mode"], "ticks")
        self.assertEqual(self.socket.sent[-1]["type"], "stop")

    async def test_single_joint_mode_does_not_send_other_position_goals(self):
        await self.run_sweep((3,))
        moves = [m for m in self.commands() if m["mode"] == "ticks"]
        self.assertEqual([m["joint_id"] for m in moves], [3, 3, 3])

    async def test_cancel_before_first_release_sends_no_commands(self):
        self.confirm.side_effect = RuntimeError("cancelled")
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            await self.run_sweep()
        self.assertEqual(self.commands(), [])
        self.assertTrue(all(self.socket.pose["torque"].values()))

    async def test_cancel_at_zero_prompt_leaves_selected_holding_and_others_off(self):
        self.confirm.side_effect = [None, None, RuntimeError("cancelled")]
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            await self.run_sweep((3,))
        moves = [m for m in self.commands() if m["mode"] == "ticks"]
        self.assertEqual([m["value"] for m in moves], [100, 3900])
        self.assertEqual(self.socket.pose["torque"], {name: name == JOINTS[2] for name in JOINTS})

    async def test_reader_failure_interrupts_confirmation_without_waiting_for_enter(self):
        cancelled = asyncio.Event()

        async def confirm(prompt):
            self.socket.messages.put_nowait({"type": "error", "message": "bus lost"})
            try:
                await asyncio.Event().wait()
            finally:
                cancelled.set()

        with patch("sys.stderr", StringIO()), self.assertRaisesRegex(RuntimeError, "bus lost"):
            await self.run_sweep(confirm=confirm)
        self.assertTrue(cancelled.is_set())
        self.assertEqual(self.commands(), [])

    async def test_cancellation_during_motion_accepts_terminal_result_and_stops_session(self):
        self.socket.hang_move = True
        task = asyncio.create_task(self.run_sweep((3,)))
        await asyncio.wait_for(self.socket.move_started.wait(), 2)
        task.cancel()
        errors = StringIO()
        with patch("sys.stderr", errors), self.assertRaises(asyncio.CancelledError):
            await task
        self.assertEqual(errors.getvalue(), "")
        self.assertEqual(self.socket.sent[-1]["type"], "stop")
        self.assertTrue(self.socket.pose["torque"][JOINTS[2]])
