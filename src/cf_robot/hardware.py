"""LeRobot adapters; imports are deferred so protocol tests need no hardware stack."""

from cf_robot.protocol import JOINTS


def make_leader(port, arm_id):
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

    return SO101Leader(SO101LeaderConfig(port=port, id=arm_id))


def make_follower(port, arm_id):
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    return SO101Follower(SO101FollowerConfig(port=port, id=arm_id))


def make_camera(device, width, height, fps):
    from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig

    return OpenCVCamera(
        OpenCVCameraConfig(index_or_path=device, width=width, height=height, fps=fps)
    )


def joint_positions(observation):
    return {name: float(observation[name]) for name in JOINTS}
