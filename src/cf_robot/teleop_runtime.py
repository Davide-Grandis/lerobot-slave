"""Shared feedback validation and sequential startup synchronization."""

import asyncio
import json
import math
import time

from cf_robot.parking import PARKING_JOINT_ORDER
from cf_robot.protocol import GRIPPER_JOINTS, JOINTS, action_message, hold_message, validate_action
from cf_robot.teleop_motion import joint_label

FEEDBACK_FRESH_S = 0.5
SYNC_FEEDBACK_STOP_S = 1.0
LIVE_FEEDBACK_STOP_S = 1.0
HOLD_INTERVAL_S = 0.1
SYNC_TOLERANCE = 1.0
SYNC_STEP = 2.5
SYNC_INTERVAL_S = 0.1
SYNC_TIMEOUT_S = 60.0


class Feedback(dict):
    def __init__(self, joints=JOINTS, clock=None, wall_clock=None):
        super().__init__(
            observation=None, received_at=None, generation=0,
            last_status_at=None, sampled_at=None, sample_check="no sample received",
        )
        self.joints = joints
        self.clock = clock or time.monotonic
        self.wall_clock = wall_clock or time.time

    @property
    def age(self):
        return float("inf") if self["received_at"] is None else self.clock() - self["received_at"]

    @property
    def fresh(self):
        return self.age <= FEEDBACK_FRESH_S

    def accept(self, message):
        self["last_status_at"] = self.clock()
        observed_at = message.get("observed_at")
        if type(observed_at) not in (int, float) or not math.isfinite(observed_at):
            self["sample_check"] = "missing/invalid observed_at timestamp"
            return False
        age = self.wall_clock() - observed_at
        self["sample_check"] = f"timestamp age={age:.3f}s at receipt (includes network delay and clock offset)"
        if observed_at == self["sampled_at"]:
            self["sample_check"] += "; Pi repeated its previous hardware sample"
            return False
        if not -0.25 <= age <= FEEDBACK_FRESH_S:
            self["sample_check"] += "; rejected as stale/future"
            return False
        try:
            observation = validate_action(message.get("observation"), self.joints)
        except ValueError:
            self["sample_check"] = "invalid joint observation"
            return False
        self.update(
            observation=observation, received_at=self.clock(),
            sampled_at=observed_at, generation=self["generation"] + 1,
        )
        return True

    def error(self, phase):
        def age(key):
            return "never received" if self[key] is None else f"{self.clock() - self[key]:.3f}s ago"

        label = f"J{JOINTS.index(self.joints[0]) + 1} " if len(self.joints) == 1 else ""
        return RuntimeError(
            f"fresh {label}follower readback lost {phase}: "
            f"last accepted sample={age('received_at')}; "
            f"last status packet={age('last_status_at')}; last sample check={self['sample_check']}. "
            "This is a feedback-timing failure, not a position-gap limit. "
            "Check Pi serial/service logs, WSS latency and clock synchronization."
        )


async def recover_live_feedback(socket, reader, feedback, seq, stop, on_status, supports_hold=False):
    """Pause advancement at Pi-local measured positions during a short return-path gap."""
    if stop.is_set():
        return seq, True
    if feedback.fresh:
        return seq, False
    # Give the status reader a chance to drain already-arrived frames after
    # a local scheduling stall before declaring a network feedback gap.
    await asyncio.sleep(0)
    if reader.done():
        reader.result()
    if feedback.fresh:
        return seq, False
    if not supports_hold:
        raise RuntimeError(
            f"{feedback.error('during live following')} "
            "Pi does not advertise safe hold support; update its follower package/service."
        )
    on_status({"type": "status", "state": "waiting for live feedback"})
    while not stop.is_set():
        if reader.done():
            reader.result()
        if feedback.fresh:
            on_status({"type": "status", "state": "running"})
            return seq, True
        if feedback.age >= LIVE_FEEDBACK_STOP_S:
            raise feedback.error("during live feedback hold")
        seq += 1
        await socket.send(json.dumps(hold_message(seq), separators=(",", ":")))
        on_status({"type": "hold_command", "seq": seq})
        try:
            await asyncio.wait_for(stop.wait(), timeout=HOLD_INTERVAL_S)
        except TimeoutError:
            pass
    return seq, True


class StartupSync:
    """Freeze the leader snapshot and move one selected joint at a time."""

    def __init__(self, target, joints=JOINTS, timeout=SYNC_TIMEOUT_S, clock=None):
        self.joints = joints
        self.order = tuple(joint for joint in PARKING_JOINT_ORDER if joint in joints)
        if set(self.order) != set(joints):
            raise ValueError("unsupported startup joint selection")
        self.target = validate_action(dict(target), joints)
        self.timeout = timeout
        self.clock = clock or time.monotonic
        self.deadline = self.clock() + timeout
        self.overall_deadline = self.clock() + timeout * len(joints)
        self.index = 0
        self.goal = None
        self.final_generation = None
        self.phase = "syncing"

    @property
    def complete(self):
        return self.index == len(self.joints)

    @property
    def active_joint(self):
        return None if self.complete else self.order[self.index]

    def advance(self, feedback):
        if self.complete:
            return None
        if feedback.age > SYNC_FEEDBACK_STOP_S:
            raise feedback.error("during startup sync")
        if self.clock() >= min(self.deadline, self.overall_deadline):
            actual = (feedback["observation"] or {}).get(self.active_joint)
            raise RuntimeError(
                f"{joint_label(self.active_joint)} startup sync timed out: "
                f"follower={actual}, leader startup target={self.target[self.active_joint]:.2f}; inspect the arm"
            )
        if not feedback.fresh:
            self.phase = "waiting_feedback"
            return None if self.goal is None else dict(self.goal)
        self.phase = "syncing"
        measured = feedback["observation"]
        if self.goal is None:
            self.goal = dict(measured)  # First-frame alignment guard remains intact.
            return dict(self.goal)
        joint = self.active_joint
        if (
            self.final_generation is not None
            and feedback["generation"] > self.final_generation
            and abs(self.target[joint] - measured[joint]) <= SYNC_TOLERANCE
        ):
            self.goal[joint] = self.target[joint]
            self.index += 1
            self.final_generation = None
            self.deadline = self.clock() + self.timeout
            if self.complete:
                remaining = [
                    name for name in self.order
                    if abs(self.target[name] - measured[name]) > SYNC_TOLERANCE
                ]
                if not remaining:
                    self.phase = "complete"
                    return None
                self.index = self.order.index(remaining[0])
            joint = self.active_joint
        self.goal[joint] = max(
            measured[joint] - SYNC_STEP, min(measured[joint] + SYNC_STEP, self.target[joint]),
        )
        if self.goal[joint] == self.target[joint] and self.final_generation is None:
            self.final_generation = feedback["generation"]
        return dict(self.goal)


async def synchronize(socket, reader, feedback, target, seq, stop, on_status, timeout=SYNC_TIMEOUT_S):
    plan = StartupSync(target, feedback.joints, timeout)
    previous_phase = None
    last_report = 0
    gripper_only = feedback.joints == GRIPPER_JOINTS
    while not stop.is_set():
        if reader.done():
            reader.result()
        goal = plan.advance(feedback)
        phase = (plan.phase, plan.active_joint)
        if phase != previous_phase:
            if plan.complete:
                state = "J6 startup sync complete" if gripper_only else "startup sync complete"
            elif plan.phase == "waiting_feedback":
                state = "waiting for J6 feedback" if gripper_only else "waiting for follower feedback"
            else:
                state = f"synchronizing {plan.active_joint}"
            on_status({"type": "status", "state": state})
            previous_phase = phase
        if stop.is_set():
            return seq
        if plan.complete:
            return seq
        if goal is not None:
            seq += 1
            await socket.send(json.dumps(action_message(seq, goal, feedback.joints), separators=(",", ":")))
            on_status({"type": "sync_target", "target": goal, "seq": seq})
        if feedback.fresh and time.monotonic() - last_report >= 0.25:
            if gripper_only:
                on_status({
                    "type": "gripper_sample", "leader_percent": target["gripper.pos"],
                    "follower_percent": feedback["observation"]["gripper.pos"],
                })
            else:
                on_status({
                    "type": "teleop_sample", "target": dict(target),
                    "observation": dict(feedback["observation"]),
                })
            last_report = time.monotonic()
        try:
            await asyncio.wait_for(stop.wait(), timeout=SYNC_INTERVAL_S)
        except TimeoutError:
            pass
    return seq
