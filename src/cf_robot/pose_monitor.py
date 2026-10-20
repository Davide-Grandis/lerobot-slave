"""Read-only LeRobot pose sampling for leader/follower mapping work."""

import time

from cf_robot.hardware import make_follower, make_leader, joint_positions
from cf_robot.protocol import JOINTS

POSE_MONITOR_INTERVAL_S = 0.1


def _sample(device, side):
    bus = device.bus
    if side == "leader":
        # This is the same normalized action value used by leader teleoperation.
        values = joint_positions(device.get_action())
    else:
        observed = bus.sync_read("Present_Position", num_retry=2)
        values = joint_positions(
            {f"{motor}.pos": value for motor, value in observed.items()}
        )

    raw = bus.sync_read("Present_Position", normalize=False, num_retry=2)
    return {
        "type": "pose_sample",
        "side": side,
        "values": values,
        "raw": {
            joint: int(raw[joint.removesuffix(".pos")])
            for joint in JOINTS
        },
        "sampled_at": time.time(),
    }


def stream_leader_pose(port, arm_id, stop, publish):
    """Sample the leader bus without calling connect/configure or writing torque."""
    leader = make_leader(port, arm_id)
    bus = leader.bus
    connected = False
    try:
        bus.connect()
        connected = True
        while not stop.is_set():
            started = time.monotonic()
            publish(_sample(leader, "leader"))
            remaining = POSE_MONITOR_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    finally:
        if connected:
            bus.disconnect(disable_torque=False)


def stream_follower_pose(port, arm_id, stop, publish):
    """Sample the follower bus read-only; do not configure motors or torque."""
    follower = make_follower(port, arm_id)
    bus = follower.bus
    connected = False
    try:
        bus.connect()
        connected = True
        while not stop.is_set():
            started = time.monotonic()
            publish(_sample(follower, "follower"))
            remaining = POSE_MONITOR_INTERVAL_S - (time.monotonic() - started)
            if remaining > 0 and stop.wait(remaining):
                break
    finally:
        if connected:
            bus.disconnect(disable_torque=False)
