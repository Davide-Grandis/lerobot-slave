"""LeRobot adapters; imports are deferred so protocol tests need no hardware stack."""

from cf_robot.angular_calibration import attach_angular_calibration
from cf_robot.protocol import JOINTS

CAMERA_WARMUP_S = 5


def make_leader(port, arm_id):
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

    leader = SO101Leader(SO101LeaderConfig(port=port, id=arm_id, use_degrees=True))
    return attach_angular_calibration(leader, "leader")


def make_follower(port, arm_id):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    follower = SO101Follower(SO101FollowerConfig(port=port, id=arm_id, use_degrees=True))
    return attach_angular_calibration(follower, "follower")


def make_camera(device, width, height, fps):
    from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

    return OpenCVCamera(
        OpenCVCameraConfig(
            index_or_path=device,
            width=width,
            height=height,
            fps=fps,
            warmup_s=CAMERA_WARMUP_S,
        )
    )


def joint_positions(observation):
    return {name: float(observation[name]) for name in JOINTS}
