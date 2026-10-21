"""Small, versioned wire protocol shared by the leader and follower."""

import math
import time

VERSION = 1
JOINTS = (
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
)
GRIPPER_JOINTS = ("gripper.pos",)
MAX_AGE_S = 0.5
MAX_FUTURE_S = 0.25


class ProtocolError(ValueError):
    pass


def validate_action(action, joints=JOINTS):
    if not isinstance(action, dict) or set(action) != set(joints):
        if joints == GRIPPER_JOINTS:
            raise ProtocolError("gripper-only action must contain only gripper.pos")
        raise ProtocolError("action must contain exactly the six SO-ARM101 joints")
    cleaned = {}
    for name in joints:
        value = action[name]
        if isinstance(value, (bool, str, bytes)):
            raise ProtocolError(f"{name} must be a number")
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ProtocolError(f"{name} must be a number") from exc
        if not math.isfinite(value) or abs(value) > 360:
            raise ProtocolError(f"{name} is out of range")
        if name == "gripper.pos" and not 0 <= value <= 100:
            raise ProtocolError("gripper target must be between 0 and 100 percent")
        cleaned[name] = value
    return cleaned


def validate_command(message, last_seq, now=None, joints=JOINTS):
    if not isinstance(message, dict) or message.get("version") != VERSION or message.get("type") != "action":
        raise ProtocolError("unsupported command")
    seq = message.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq <= last_seq:
        raise ProtocolError("sequence must increase")
    sent_at = message.get("sent_at")
    if isinstance(sent_at, bool) or not isinstance(sent_at, (int, float)) or not math.isfinite(sent_at):
        raise ProtocolError("invalid timestamp")
    age = (time.time() if now is None else now) - sent_at
    if age > MAX_AGE_S or age < -MAX_FUTURE_S:
        raise ProtocolError("stale command or unsynchronized clocks")
    return seq, validate_action(message.get("action"), joints)


def action_message(seq, action, joints=JOINTS):
    return {
        "version": VERSION,
        "type": "action",
        "seq": seq,
        "sent_at": time.time(),
        "action": validate_action(action, joints),
    }
