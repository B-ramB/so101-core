"""Create the lerobot follower robot (with optional wrist camera) and leader teleoperator from the arm config.

Both come as `RealLeader` / `RealFollower` (`so101.hardware.adapters`): joint values in model radians, the so101
standard.
"""

from collections.abc import Callable
from pathlib import Path

from so101.conventions import CAMERA_NAME, JointCalibration
from so101.hardware.adapters import RealFollower, RealLeader
from so101.paths import CALIBRATION_DIR


def make_leader(config: dict, calibration: Callable[[], JointCalibration]) -> RealLeader:
    from lerobot.teleoperators import make_teleoperator_from_config
    from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

    teleop = make_teleoperator_from_config(
        SOLeaderTeleopConfig(port=config["leader"]["port"], id=config["leader"]["id"], calibration_dir=CALIBRATION_DIR)
    )
    return RealLeader(teleop, calibration)


def make_follower(
    config: dict, use_camera: bool, calibration: Callable[[], JointCalibration]
) -> tuple[RealFollower, tuple[int, int] | None]:
    """The follower, and the wrist camera's (width, height) when the camera is in use."""
    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.robots import make_robot_from_config
    from lerobot.robots.so_follower import SOFollowerRobotConfig

    cameras = {}
    cam = config.get("camera")
    if use_camera and cam:
        if Path(str(cam["index_or_path"])).exists():
            cameras[CAMERA_NAME] = OpenCVCameraConfig(
                index_or_path=Path(cam["index_or_path"]), width=cam["width"], height=cam["height"], fps=cam["fps"]
            )
        else:
            print(f"Camera {cam['index_or_path']} not found, continuing without camera.")

    robot = make_robot_from_config(
        SOFollowerRobotConfig(
            port=config["follower"]["port"],
            id=config["follower"]["id"],
            calibration_dir=CALIBRATION_DIR,
            cameras=cameras,
        )
    )
    camera_cfg = cameras.get(CAMERA_NAME)
    camera_size = (camera_cfg.width, camera_cfg.height) if camera_cfg else None
    return RealFollower(robot, calibration), camera_size
