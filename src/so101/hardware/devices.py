"""Create the lerobot follower robot (with optional wrist camera) and leader teleoperator from the arm config."""

from pathlib import Path

from so101.conventions import CAMERA_NAME
from so101.paths import CALIBRATION_DIR


def make_devices(config: dict, use_camera: bool):
    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.robots import make_robot_from_config
    from lerobot.robots.so_follower import SOFollowerRobotConfig
    from lerobot.teleoperators import make_teleoperator_from_config
    from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

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
    teleop = make_teleoperator_from_config(
        SOLeaderTeleopConfig(port=config["leader"]["port"], id=config["leader"]["id"], calibration_dir=CALIBRATION_DIR)
    )
    return robot, teleop, cameras.get(CAMERA_NAME)
