"""Teleoperate the SO-101 follower with the leader arm, visualized in Rerun.

Uses the setup produced by `uv run setup/setup_arms.py` (setup/arm_config.json):
  - leader / follower ports and calibration files
  - the wrist camera

Rerun shows:
  - the follower and leader URDF models moving with the real arms (if urdf/ is present,
    fetch it with `setup/fetch_urdfs.sh`)
  - the wrist camera as a frustum attached to the follower gripper, with the live image projected in it
  - the joint positions (leader action vs. follower observation) as time series

How the models and the camera are drawn is configured in setup/visualization.json (created with
defaults on first run). The file is reloaded while running: edit, save, and Rerun updates.

Run with:
    uv run src/teleoperate.py            # default 30 fps
    uv run src/teleoperate.py --fps 60 --no-camera
"""

import argparse
import copy
import json
import math
import sys
import time
import warnings
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb

ROOT = Path(__file__).resolve().parent.parent
SETUP_DIR = ROOT / "setup"
CONFIG_PATH = SETUP_DIR / "arm_config.json"
VIS_CONFIG_PATH = SETUP_DIR / "visualization.json"
URDF_DIR = ROOT / "urdf"
FOLLOWER_URDF = URDF_DIR / "so101_new_calib.urdf"
LEADER_URDF = URDF_DIR / "so101_leader_new_calib.urdf"

CAMERA_NAME = "wrist"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

DEFAULT_VIS_CONFIG = {
    "_help": {
        "wrist_camera": (
            "Camera pose relative to the follower link `parent_link`. In gripper_link the gripper points "
            "along -Z. rpy_deg = mount rotation (roll, pitch, yaw about the link axes); the camera looks "
            "along its own +Z. roll_about_view_axis_deg rotates the camera (and image) around its viewing "
            "direction: use it to match the image's zero orientation to the wrist."
        ),
        "joints": (
            "Mapping from the real arm to the URDF model, per joint: model = sign * real + offset, "
            "where sign = -1 when invert is true. Units: degrees, gripper in percent (0-100). "
            "Use offset to match the model's zero orientation (e.g. wrist_roll) to the real arm."
        ),
        "leader_model_offset_m": "Where the leader model is drawn relative to the follower model.",
    },
    "wrist_camera": {
        "parent_link": "gripper_link",
        "xyz_m": [0.0, -0.04, -0.03],
        "rpy_deg": [180.0, 0.0, 0.0],
        "roll_about_view_axis_deg": 0.0,
        "fov_y_deg": 60.0,
        "image_plane_distance_m": 0.08,
    },
    "joints": {joint: {"offset": 0.0, "invert": False} for joint in JOINTS},
    "leader_model_offset_m": [0.0, 0.4, 0.0],
}


# --------------------------------------------------------------------------- #
# Config
# --------------------------------------------------------------------------- #
def load_config() -> dict:
    if not CONFIG_PATH.exists():
        sys.exit(f"{CONFIG_PATH.relative_to(ROOT)} not found. Run `uv run setup/setup_arms.py` first.")
    config = json.loads(CONFIG_PATH.read_text())

    calib_dir = ROOT / config["calibration_dir"]
    for role in ("leader", "follower"):
        calib = calib_dir / f"{config[role]['id']}.json"
        if not calib.exists():
            sys.exit(f"Calibration file {calib} is missing. Run `uv run setup/setup_arms.py`.")
        port = config[role]["port"]
        if not Path(port).exists():
            sys.exit(
                f"The {role} port {port} does not exist (arm unplugged or port changed). "
                "Run `uv run setup/setup_arms.py` to detect the ports again."
            )

    # Reuse the permission check from the setup script (sudo chmod 666 when needed).
    sys.path.insert(0, str(SETUP_DIR))
    from setup_arms import ensure_port_access

    for role in ("leader", "follower"):
        ensure_port_access(config[role]["port"])
    return config


def merge_defaults(defaults: dict, user: dict) -> dict:
    """Fill keys missing from the user config with defaults (recursively)."""
    merged = copy.deepcopy(defaults)
    for key, value in user.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_defaults(merged[key], value)
        else:
            merged[key] = value
    return merged


class VisualizationConfig:
    """setup/visualization.json, reloaded whenever the file changes."""

    def __init__(self, path: Path):
        self.path = path
        if not path.exists():
            path.write_text(json.dumps(DEFAULT_VIS_CONFIG, indent=4) + "\n")
            print(f"Created {path.relative_to(ROOT)} with default visualization settings.")
        self._mtime = 0.0
        self._last_check = 0.0
        self.data = copy.deepcopy(DEFAULT_VIS_CONFIG)
        self.reload_if_changed(force=True)

    def reload_if_changed(self, force: bool = False) -> bool:
        """Returns True when the config was (re)loaded. Checks the file at most twice per second."""
        now = time.monotonic()
        if not force and now - self._last_check < 0.5:
            return False
        self._last_check = now
        try:
            mtime = self.path.stat().st_mtime
        except OSError:
            return False
        if not force and mtime == self._mtime:
            return False
        self._mtime = mtime
        try:
            self.data = merge_defaults(DEFAULT_VIS_CONFIG, json.loads(self.path.read_text()))
        except json.JSONDecodeError as e:
            print(f"\n{self.path.name} is not valid JSON ({e}); keeping the previous settings.")
            return False
        if not force:
            print(f"\nReloaded {self.path.name}")
        return True

    @property
    def camera(self) -> dict:
        return self.data["wrist_camera"]

    def to_model_value(self, joint: str, value: float) -> float:
        cfg = self.data["joints"].get(joint, {})
        sign = -1.0 if cfg.get("invert", False) else 1.0
        return sign * value + float(cfg.get("offset", 0.0))


def make_devices(config: dict, use_camera: bool):
    from lerobot.cameras.opencv import OpenCVCameraConfig
    from lerobot.robots import make_robot_from_config
    from lerobot.robots.so_follower import SOFollowerRobotConfig
    from lerobot.teleoperators import make_teleoperator_from_config
    from lerobot.teleoperators.so_leader import SOLeaderTeleopConfig

    calib_dir = ROOT / config["calibration_dir"]
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
            calibration_dir=calib_dir,
            cameras=cameras,
        )
    )
    teleop = make_teleoperator_from_config(
        SOLeaderTeleopConfig(port=config["leader"]["port"], id=config["leader"]["id"], calibration_dir=calib_dir)
    )
    return robot, teleop, cameras.get(CAMERA_NAME)


# --------------------------------------------------------------------------- #
# Rerun
# --------------------------------------------------------------------------- #
def rpy_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_multiply(a: list[float], b: list[float]) -> list[float]:
    """Hamilton product a * b of xyzw quaternions (apply b first, then a)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


class ArmModel:
    """A URDF model in Rerun whose joints follow lerobot joint positions."""

    def __init__(self, name: str, urdf_path: Path, vis: VisualizationConfig):
        self.name = name
        self.vis = vis
        self.tree = rr.urdf.UrdfTree.from_file_path(
            urdf_path,
            entity_path_prefix=name,
            frame_prefix=f"{name}/",
            static_transform_entity_path=f"{name}/tf_static",
        )
        self.tree.log_urdf_to_recording()
        self.root_frame = f"{name}/{self.tree.root_link().name}"
        self.joints = {j.name: j for j in self.tree.joints() if j.joint_type in ("revolute", "continuous")}
        self.log_placement([0.0, 0.0, 0.0])

    def log_placement(self, offset: list[float]) -> None:
        """Attach the URDF root to the scene root so both arms share one 3D space."""
        rr.log(
            f"{self.name}/root",
            rr.Transform3D(translation=offset, parent_frame="tf#/", child_frame=self.root_frame),
            static=True,
        )

    def joint_value(self, joint: str, value: float) -> float:
        """lerobot position -> URDF joint value. Body joints are degrees, gripper is 0..100."""
        value = self.vis.to_model_value(joint, value)
        if joint == "gripper":
            urdf_joint = self.joints[joint]
            return urdf_joint.limit_lower + value / 100.0 * (urdf_joint.limit_upper - urdf_joint.limit_lower)
        return math.radians(value)

    def log_positions(self, positions: dict[str, float]) -> None:
        for joint, value in positions.items():
            if joint in self.joints:
                transform = self.joints[joint].compute_transform(self.joint_value(joint, value))
                rr.log(f"{self.name}/joint_transforms/{joint}", transform)


def log_wrist_camera(cam: dict, width: int, height: int) -> None:
    """Log the camera pose + pinhole on the follower. Static, so a re-log replaces the previous pose."""
    parent = f"follower/{cam['parent_link']}"
    frame = "follower/wrist_camera"
    mount = rpy_to_quat_xyzw(*(math.radians(a) for a in cam["rpy_deg"]))
    view_roll = rpy_to_quat_xyzw(0.0, 0.0, math.radians(cam["roll_about_view_axis_deg"]))  # about camera +Z
    focal = (height / 2) / math.tan(math.radians(cam["fov_y_deg"]) / 2)
    rr.log(
        "follower/wrist_camera",
        rr.Transform3D(
            translation=cam["xyz_m"],
            quaternion=quat_multiply(mount, view_roll),
            parent_frame=parent,
            child_frame=frame,
        ),
        rr.Pinhole(
            focal_length=focal,
            width=width,
            height=height,
            parent_frame=parent,
            child_frame=frame,
            image_plane_distance=cam["image_plane_distance_m"],
        ),
        static=True,
    )
    rr.log("follower/wrist_camera/image", rr.CoordinateFrame(frame), static=True)


class Visualizer:
    def __init__(self, vis: VisualizationConfig, show_urdf: bool, camera_size: tuple[int, int] | None):
        self.vis = vis
        self.camera_size = camera_size
        self.follower_model = self.leader_model = None
        rr.init("so101_teleoperation", spawn=True)

        if show_urdf:
            self.follower_model = ArmModel("follower", FOLLOWER_URDF, vis)
            if LEADER_URDF.exists():
                self.leader_model = ArmModel("leader", LEADER_URDF, vis)
        self.apply_config()

        views: list = []
        if show_urdf:
            views.append(rrb.Spatial3DView(origin="/", name="Arms"))
        side: list = []
        if camera_size:
            side.append(rrb.Spatial2DView(origin="/follower/wrist_camera/image", name="Wrist camera"))
        side.append(rrb.TimeSeriesView(origin="/joints", name="Joints (deg / gripper %)"))
        views.append(rrb.Vertical(*side))
        rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*views, column_shares=[2, 1][: len(views)])))

    def apply_config(self) -> None:
        """(Re)log everything that depends on setup/visualization.json."""
        if self.leader_model:
            self.leader_model.log_placement(self.vis.data["leader_model_offset_m"])
        if self.follower_model and self.camera_size:
            log_wrist_camera(self.vis.camera, *self.camera_size)

    def log_step(self, obs: dict, action: dict) -> None:
        if self.vis.reload_if_changed():
            self.apply_config()

        follower_pos = {k.removesuffix(".pos"): float(v) for k, v in obs.items() if k.endswith(".pos")}
        leader_pos = {k.removesuffix(".pos"): float(v) for k, v in action.items() if k.endswith(".pos")}

        for joint in JOINTS:
            if joint in follower_pos:
                rr.log(f"joints/{joint}/follower", rr.Scalars(follower_pos[joint]))
            if joint in leader_pos:
                rr.log(f"joints/{joint}/leader", rr.Scalars(leader_pos[joint]))

        if self.follower_model:
            self.follower_model.log_positions(follower_pos)
        if self.leader_model:
            self.leader_model.log_positions(leader_pos)

        if CAMERA_NAME in obs:
            rr.log("follower/wrist_camera/image", rr.Image(obs[CAMERA_NAME]).compress(jpeg_quality=80))


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fps", type=int, default=30, help="control loop frequency (default 30)")
    parser.add_argument("--no-camera", action="store_true", help="run without the wrist camera")
    parser.add_argument("--no-urdf", action="store_true", help="don't show the URDF models")
    args = parser.parse_args()

    config = load_config()
    vis = VisualizationConfig(VIS_CONFIG_PATH)

    show_urdf = not args.no_urdf and FOLLOWER_URDF.exists()
    if not args.no_urdf and not show_urdf:
        print(f"No URDF found at {FOLLOWER_URDF.relative_to(ROOT)} (run setup/fetch_urdfs.sh). Showing data only.")
    # Joint values outside the URDF limits are clamped; don't spam a warning every frame.
    warnings.filterwarnings("ignore", message=".*limit.*", category=UserWarning)

    robot, teleop, camera_cfg = make_devices(config, use_camera=not args.no_camera)
    camera_size = (camera_cfg.width, camera_cfg.height) if camera_cfg else None
    visualizer = Visualizer(vis, show_urdf, camera_size)

    print("Connecting to the arms...")
    robot.connect()
    teleop.connect()
    print(f"Teleoperating at {args.fps} fps. Press Ctrl+C to stop.")
    print(f"Edit {VIS_CONFIG_PATH.relative_to(ROOT)} while running to adjust the camera pose and joint offsets.")

    try:
        start = time.perf_counter()
        while True:
            loop_start = time.perf_counter()
            action = teleop.get_action()
            robot.send_action(action)
            obs = robot.get_observation()

            rr.set_time("time", duration=loop_start - start)
            visualizer.log_step(obs, action)

            dt = time.perf_counter() - loop_start
            time.sleep(max(1 / args.fps - dt, 0.0))
            print(f"\rLoop: {(time.perf_counter() - loop_start) * 1e3:5.1f} ms", end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
