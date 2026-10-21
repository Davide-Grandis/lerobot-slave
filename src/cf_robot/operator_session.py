"""One browser-owned session with mutually exclusive leader and manual control."""

import asyncio
import json
import time

from websockets.asyncio.client import connect

from cf_robot.auth import access_headers
from cf_robot.hardware import joint_positions, make_leader
from cf_robot.protocol import VERSION, action_message, validate_action
from cf_robot.teleop_hardware import TeleopLeader
from cf_robot.teleop_runtime import Feedback, synchronize

ALIGNMENT_TOLERANCE = 5.0
OBSERVATION_MAX_AGE_S = 0.5
MANUAL_LIMITS = {
    "shoulder_pan.pos": (-110.0, 110.0),
    "shoulder_lift.pos": (-100.0, 100.0),
    "elbow_flex.pos": (-100.0, 90.0),
    "wrist_flex.pos": (-95.0, 95.0),
    "wrist_roll.pos": (-160.0, 160.0),
    "gripper.pos": (0.0, 100.0),
}


class OperatorSession:
    """Own one Pi control socket and route exactly one input source to it."""

    def __init__(self, url, port, arm_id, fps=30, on_status=None, leader_factory=None):
        if not 1 <= fps <= 60:
            raise ValueError("fps must be between 1 and 60")
        self.url = url
        self.port = port
        self.arm_id = arm_id
        self.fps = fps
        self.on_status = on_status or (lambda status: None)
        self.leader_factory = leader_factory or (lambda: make_leader(port, arm_id))
        self.stop_event = asyncio.Event()
        self.manual_queue = asyncio.Queue(maxsize=1)
        self.leader_io_lock = asyncio.Lock()
        self.mode = "manual"
        self.leader = None
        self.socket = None
        self.seq = 0
        self.last_target = None
        self.feedback = Feedback()
        self.sync_pending = False
        self.status = {
            "state": "connecting",
            "mode": self.mode,
            "leader_connected": False,
            "leader_positions": None,
            "follower_positions": None,
            "follower_observed_at": None,
            "aligned": False,
            "alignment_errors": {},
            "commands_sent": 0,
            "error": None,
            "teleop_message": "",
        }

    def _publish(self, **updates):
        self.status.update(updates)
        self.status["mode"] = self.mode
        self.on_status(dict(self.status))

    async def run(self):
        headers = access_headers(self.url)
        try:
            async with connect(
                self.url,
                additional_headers=headers,
                open_timeout=10,
                ping_interval=10,
                ping_timeout=5,
                max_size=2 * 1024 * 1024,
            ) as socket:
                self.socket = socket
                raw = await asyncio.wait_for(socket.recv(), timeout=25)
                ready = json.loads(raw)
                if ready.get("type") == "error":
                    raise RuntimeError(ready.get("message", "follower failed to connect"))
                if ready.get("type") != "ready" or ready.get("version") != VERSION:
                    raise RuntimeError("follower did not send a compatible ready message")
                self.last_target = validate_action(ready["observation"])
                self.feedback.accept(ready)
                self._publish(
                    state="ready",
                    follower_positions=self.last_target,
                    follower_observed_at=time.time(),
                    error=None,
                )
                reader = asyncio.create_task(self._read_status(socket))
                leader_monitor = asyncio.create_task(self._monitor_leader())
                try:
                    await self._send_loop(socket, reader)
                finally:
                    self.stop_event.set()
                    await leader_monitor
                    reader.cancel()
                    try:
                        await reader
                    except asyncio.CancelledError:
                        pass
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._publish(state="error", error=str(exc))
            raise
        finally:
            self.socket = None
            self._clear_manual_queue("control session stopped before command was sent")
            await self.disconnect_leader()
            if self.status["state"] != "error":
                self._publish(state="idle", leader_connected=False)

    async def _read_status(self, socket):
        async for raw in socket:
            message = json.loads(raw)
            if message.get("type") == "error":
                raise RuntimeError(message.get("message", "follower control failed"))
            if message.get("type") != "status":
                continue
            updates = {}
            if self.feedback.accept(message):
                updates["follower_positions"] = dict(self.feedback["observation"])
                updates["follower_observed_at"] = self.feedback["sampled_at"]
                self._refresh_alignment(updates)
            self._publish(**updates)
        raise ConnectionError("follower control WebSocket closed")

    async def _send_loop(self, socket, reader):
        period = 1 / self.fps
        deadline = time.monotonic() + 2
        while not self.feedback.fresh and not self.stop_event.is_set():
            if reader.done():
                reader.result()
            if time.monotonic() >= deadline:
                raise self.feedback.error("before arming")
            await asyncio.sleep(0.05)
        while not self.stop_event.is_set():
            if reader.done():
                reader.result()
            started = time.monotonic()
            target = None
            acknowledged = None
            mode_at_start = self.mode
            if self.mode == "leader":
                if self.leader is None:
                    raise RuntimeError("leader arm disconnected while streaming")
                target = await self._read_leader()
                if self.mode != "leader":
                    continue
                self._publish(leader_positions=target)
                if self.sync_pending:
                    self.seq = await synchronize(
                        socket, reader, self.feedback, dict(target), self.seq,
                        self.stop_event, self._sync_status,
                    )
                    if self.stop_event.is_set():
                        break
                    self.sync_pending = False
                    self.last_target = dict(target)
                    self._publish(state="ready", teleop_message="Ready for teleop")
            else:
                try:
                    target, acknowledged = await asyncio.wait_for(
                        self.manual_queue.get(), timeout=period
                    )
                except asyncio.TimeoutError:
                    if self.last_target is None:
                        continue
                    target = self.last_target
            if self.stop_event.is_set():
                break
            if self.mode != mode_at_start:
                if acknowledged is not None and not acknowledged.done():
                    acknowledged.set_exception(RuntimeError("control mode changed before send"))
                continue
            if not self.feedback.fresh:
                raise self.feedback.error("during live following")
            self.seq += 1
            await socket.send(
                json.dumps(action_message(self.seq, target), separators=(",", ":"))
            )
            self.last_target = target
            self._publish(commands_sent=self.status["commands_sent"] + 1)
            if self.mode == "manual" and acknowledged is not None and not acknowledged.done():
                acknowledged.set_result(True)
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                try:
                    await asyncio.wait_for(self.stop_event.wait(), timeout=remaining)
                except asyncio.TimeoutError:
                    pass

    def _sync_status(self, message):
        if message.get("type") == "sync_target":
            self.seq = message["seq"]
            self.last_target = dict(message["target"])
            self._publish(commands_sent=self.status["commands_sent"] + 1)
        elif message.get("state") == "waiting for follower feedback":
            self._publish(
                state="synchronizing",
                teleop_message="WAIT - feedback delayed; holding the last sync goal.",
            )
        elif str(message.get("state", "")).startswith("synchronizing "):
            self._publish(
                state="synchronizing",
                teleop_message=f"WAIT - {message['state']}; hold the leader still.",
            )

    async def _read_leader(self):
        async with self.leader_io_lock:
            if self.leader is None:
                raise RuntimeError("leader arm is disconnected")
            raw_action = await asyncio.to_thread(self.leader.get_action)
        return validate_action(joint_positions(raw_action))

    async def _monitor_leader(self):
        while not self.stop_event.is_set():
            if self.leader is not None and self.mode == "manual":
                try:
                    positions = await self._read_leader()
                    updates = {"leader_positions": positions}
                    self._refresh_alignment(updates)
                    self._publish(**updates)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    self._publish(error=f"Leader feedback failed: {exc}")
                    await self.disconnect_leader()
            await asyncio.sleep(0.1)

    async def connect_leader(self):
        if self.socket is None or self.status["state"] != "ready":
            raise RuntimeError("connect the follower before connecting the leader")
        if not self.port:
            raise RuntimeError("configure the leader USB port before connecting the leader")
        if self.leader is not None:
            return dict(self.status)
        leader = self.leader_factory()
        if hasattr(leader, "bus"):
            leader = TeleopLeader(leader)
        try:
            await asyncio.to_thread(leader.connect, calibrate=False)
            positions = validate_action(joint_positions(await asyncio.to_thread(leader.get_action)))
        except Exception:
            try:
                await asyncio.to_thread(leader.disconnect)
            except Exception:
                pass
            raise
        self.leader = leader
        updates = {"leader_connected": True, "leader_positions": positions, "error": None}
        self._refresh_alignment(updates)
        self._publish(**updates)
        return dict(self.status)

    async def disconnect_leader(self):
        if self.sync_pending:
            self.stop_event.set()
        self.sync_pending = False
        self.mode = "manual"
        if self.feedback.fresh:
            self.last_target = dict(self.feedback["observation"])
        async with self.leader_io_lock:
            leader, self.leader = self.leader, None
            if leader is not None:
                await asyncio.to_thread(leader.disconnect)
        self._publish(leader_connected=False, leader_positions=None, aligned=False)

    def set_mode(self, mode, confirm_sync=False):
        if mode not in {"manual", "leader"}:
            raise ValueError("mode must be 'manual' or 'leader'")
        if self.sync_pending:
            if mode == "leader":
                return dict(self.status)
            raise RuntimeError("stop the control session to cancel startup sync")
        if mode == "leader":
            if self.leader is None:
                raise RuntimeError("connect the leader arm before selecting leader mode")
            observed_at = self.status["follower_observed_at"]
            if not self.feedback.fresh or observed_at is None or time.time() - observed_at > OBSERVATION_MAX_AGE_S:
                raise RuntimeError("follower servo feedback is stale; control remains disarmed")
            if not self.status["aligned"] and confirm_sync is not True:
                raise RuntimeError("confirm startup sync before the follower moves toward the leader")
            self.sync_pending = True
        elif self.feedback.fresh:
            self.last_target = dict(self.feedback["observation"])
        self.mode = mode
        self._clear_manual_queue("control mode changed before command was sent")
        self._publish(
            mode=mode, state="synchronizing" if self.sync_pending else "ready",
            teleop_message="WAIT - preparing startup sync" if self.sync_pending else "Manual mode ready",
        )
        return dict(self.status)

    async def submit_manual(self, action):
        if self.socket is None or self.status["state"] != "ready":
            raise RuntimeError("connect the follower before sending manual targets")
        if self.mode != "manual":
            raise RuntimeError("switch to manual mode before using on-screen controls")
        if not self.feedback.fresh:
            raise self.feedback.error("before manual command")
        target = validate_action(action)
        out_of_range = [
            joint
            for joint, (minimum, maximum) in MANUAL_LIMITS.items()
            if not minimum <= target[joint] <= maximum
        ]
        if out_of_range:
            raise ValueError(f"manual target is outside model limits for: {', '.join(out_of_range)}")
        baseline = self.last_target or self.status["follower_positions"]
        if baseline is None:
            raise RuntimeError("follower pose is unavailable")
        oversized = {
            joint: target[joint] - baseline[joint]
            for joint in target
            if abs(target[joint] - baseline[joint]) > ALIGNMENT_TOLERANCE
        }
        if oversized:
            raise ValueError("manual target exceeds the 5-unit per-command movement limit")
        acknowledged = asyncio.get_running_loop().create_future()
        try:
            self.manual_queue.put_nowait((target, acknowledged))
        except asyncio.QueueFull as exc:
            raise RuntimeError("wait for the previous manual target to send") from exc
        try:
            await asyncio.wait_for(acknowledged, timeout=1)
        except asyncio.TimeoutError as exc:
            raise RuntimeError("manual target did not reach the follower before timeout") from exc

    def _refresh_alignment(self, updates):
        follower = updates.get("follower_positions", self.status["follower_positions"])
        leader = updates.get("leader_positions", self.status["leader_positions"])
        if follower is None or leader is None:
            updates.update(aligned=False, alignment_errors={})
            return
        errors = {joint: leader[joint] - follower[joint] for joint in follower}
        updates.update(
            aligned=all(abs(value) <= ALIGNMENT_TOLERANCE for value in errors.values()),
            alignment_errors=errors,
        )

    def _clear_manual_queue(self, reason="manual command queue cleared"):
        while not self.manual_queue.empty():
            try:
                _, acknowledged = self.manual_queue.get_nowait()
                if not acknowledged.done():
                    acknowledged.set_exception(RuntimeError(reason))
            except asyncio.QueueEmpty:
                break
