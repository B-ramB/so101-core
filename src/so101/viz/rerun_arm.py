"""The SO-101 URDF models and wrist camera in Rerun."""

import math
import time
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb

from so101.conventions import CAMERA_NAME, GRIPPER, GRIPPER_LOWER, GRIPPER_UPPER, JOINTS, rescale_gripper
from so101.math3d import quat_multiply, rpy_to_quat_xyzw
from so101.paths import FOLLOWER_URDF, LEADER_URDF
from so101.viz.config import VisualizationConfig


class ArmModel:
    """A URDF model in Rerun whose joints follow joint positions in model radians."""

    def __init__(self, name: str, urdf_path: Path):
        self.name = name
        self.tree = rr.urdf.UrdfTree.from_file_path(
            urdf_path,
            entity_path_prefix=name,
            frame_prefix=f"{name}/",
            static_transform_entity_path=f"{name}/tf_static",
        )
        self.tree.log_urdf_to_recording()
        self.root_frame = f"{name}/{self.tree.root_link().name}"
        self.joints = {j.name: j for j in self.tree.joints() if j.joint_type in ("revolute", "continuous")}
        gripper = self.joints[GRIPPER]  # the leader's gripper range differs from the follower's
        self.gripper_limits = (gripper.limit_lower, gripper.limit_upper)
        self.log_placement([0.0, 0.0, 0.0])

    def log_placement(self, offset: list[float]) -> None:
        """Attach the URDF root to the scene root so both arms share one 3D space."""
        rr.log(
            f"{self.name}/root",
            rr.Transform3D(translation=offset, parent_frame="tf#/", child_frame=self.root_frame),
            static=True,
        )

    def log_positions(self, radians: dict[str, float]) -> None:
        """Pose the model from joint positions in model radians (the so101 standard, follower gripper range).

        The gripper is rescaled to this model's own gripper range, so the leader URDF (0 ... 0.785 rad) shows the
        same opening as the follower.
        """
        for joint, value in radians.items():
            if joint == GRIPPER:
                value = rescale_gripper(value, (GRIPPER_LOWER, GRIPPER_UPPER), self.gripper_limits)
            if joint in self.joints:
                rr.log(f"{self.name}/joint_transforms/{joint}", self.joints[joint].compute_transform(value))


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
        self._start: float | None = None
        rr.init("so101_teleoperation", spawn=True)

        if show_urdf:
            self.follower_model = ArmModel("follower", FOLLOWER_URDF)
            if LEADER_URDF.exists():
                self.leader_model = ArmModel("leader", LEADER_URDF)
        self.apply_config()

        views: list = []
        if show_urdf:
            views.append(rrb.Spatial3DView(origin="/", name="Arms"))
        side: list = []
        if camera_size:
            side.append(rrb.Spatial2DView(origin="/follower/wrist_camera/image", name="Wrist camera"))
        side.append(rrb.TimeSeriesView(origin="/joints", name="Joints (rad)"))
        views.append(rrb.Vertical(*side))
        rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(*views, column_shares=[2, 1][: len(views)])))

    def apply_config(self) -> None:
        """(Re)log everything that depends on so101_config/visualization.json."""
        if self.leader_model:
            self.leader_model.log_placement(self.vis.data["leader_model_offset_m"])
        if self.follower_model and self.camera_size:
            log_wrist_camera(self.vis.camera, *self.camera_size)

    def log_step(self, obs: dict, action: dict) -> None:
        """Log one teleop step: follower observation and leader action, joint values in model radians."""
        now = time.perf_counter()
        if self._start is None:
            self._start = now
        rr.set_time("time", duration=now - self._start)
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
