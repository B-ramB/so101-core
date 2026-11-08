"""The SO-101 URDF models and wrist camera in Rerun."""

import math
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb

from so101.conventions import CAMERA_NAME, JOINTS, gripper_percent_to_joint
from so101.math3d import quat_multiply, rpy_to_quat_xyzw
from so101.paths import FOLLOWER_URDF, LEADER_URDF
from so101.viz.config import VisualizationConfig


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
            return gripper_percent_to_joint(value, urdf_joint.limit_lower, urdf_joint.limit_upper)
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
        """(Re)log everything that depends on so101_config/visualization.json."""
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
