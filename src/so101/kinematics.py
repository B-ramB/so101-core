"""Forward kinematics of the SO-101 follower for the safety checks: plain Python + numpy, no mujoco.

`ArmPoints().positions(q)` returns, in the base frame (base_link at the origin, x forward, z up, metres), a few points
of the arm that can hit the table: the elbow and wrist joints, the gripper frame and the two jaw tips. `q` is in model
radians, keyed by joint name or `"<joint>.pos"`, so an observation or an action works directly.

Each point has a clearance `radius`: the elbow and wrist points sit on the joint axes, and the servo housings around
them stick out by about that much. `so101.safety.Workspace` keeps every point's sphere inside the workspace.

The joint origins are copied from the follower URDF `so101_new_calib.urdf` (the same file the MuJoCo model is generated
from), so the safety layer needs no downloaded file. The jaw tips were measured from its meshes (`gripper_link` and
`moving_jaw_so101_v1_link`). tests/test_sim_kinematics.py checks all of it against the MuJoCo model.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from so101.conventions import JOINTS

SUFFIX = ".pos"


def _origin(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    """A URDF <origin> as a 4x4 transform: translation xyz, rotation Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    transform = np.eye(4)
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = xyz
    return transform


# so101_new_calib.urdf: every joint is revolute about its own z axis. (joint, parent link, child link, origin)
_CHAIN = [
    ("shoulder_pan", "base_link", "shoulder_link", _origin((0.0388353, 0.0, 0.0624), (3.14159, 0.0, -3.14159))),
    ("shoulder_lift", "shoulder_link", "upper_arm_link",
     _origin((-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0))),
    ("elbow_flex", "upper_arm_link", "lower_arm_link", _origin((-0.11257, -0.028, 0.0), (0.0, 0.0, 1.5708))),
    ("wrist_flex", "lower_arm_link", "wrist_link", _origin((-0.1349, 0.0052, 0.0), (0.0, 0.0, -1.5708))),
    ("wrist_roll", "wrist_link", "gripper_link", _origin((0.0, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159))),
    ("gripper", "gripper_link", "moving_jaw_so101_v1_link",
     _origin((0.0202, 0.0188, -0.0234), (1.5708, -5.24284e-08, 0.0))),
]
# The fixed joint gripper_frame_joint: the URDF's gripper-tip frame on gripper_link.
_GRIPPER_FRAME = _origin((-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0))


@dataclass(frozen=True)
class ArmPoint:
    """A point on the arm: `offset` in the frame of `link`, with a clearance `radius` (metres)."""

    link: str
    offset: tuple[float, float, float]
    radius: float


# The points the workspace check uses. The housings around the elbow and wrist axes are servos (about 45 x 35 x 25 mm);
# 3 cm covers them. The jaw tips are the lowest points of the jaw meshes along the gripper.
POINTS = {
    "elbow": ArmPoint("lower_arm_link", (0.0, 0.0, 0.0), 0.03),
    "wrist": ArmPoint("wrist_link", (0.0, 0.0, 0.0), 0.03),
    "gripper_frame": ArmPoint("gripper_frame_link", (0.0, 0.0, 0.0), 0.0),
    "fixed_jaw_tip": ArmPoint("gripper_link", (-0.0089, -0.0002, -0.1044), 0.0),
    "moving_jaw_tip": ArmPoint("moving_jaw_so101_v1_link", (-0.0114, -0.0820, 0.0189), 0.0),
}


def _rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    rotation = np.eye(4)
    rotation[:2, :2] = [[c, -s], [s, c]]
    return rotation


def joint_value(q: Mapping[str, float], joint: str) -> float:
    """The position of `joint` in `q`, keyed `"<joint>.pos"` or by joint name. KeyError when neither is there."""
    key = f"{joint}{SUFFIX}"
    if key in q:
        return float(q[key])
    if joint in q:
        return float(q[joint])
    raise KeyError(f"No position for joint {joint!r}: expected key {key!r} or {joint!r}.")


class ArmPoints:
    """The base-frame positions of `POINTS` for given joint positions (model radians)."""

    points = POINTS

    def link_frames(self, q: Mapping[str, float]) -> dict[str, np.ndarray]:
        """4x4 base-frame transform of every link, for the joint positions `q` (every joint in `JOINTS`)."""
        frames = {"base_link": np.eye(4)}
        for joint, parent, child, origin in _CHAIN:
            frames[child] = frames[parent] @ origin @ _rot_z(joint_value(q, joint))
        frames["gripper_frame_link"] = frames["gripper_link"] @ _GRIPPER_FRAME
        return frames

    def positions(self, q: Mapping[str, float]) -> dict[str, np.ndarray]:
        """Base-frame position (x, y, z) of every point in `POINTS`, in metres."""
        frames = self.link_frames(q)
        return {
            name: frames[point.link][:3, :3] @ np.asarray(point.offset) + frames[point.link][:3, 3]
            for name, point in self.points.items()
        }


assert [joint for joint, *_ in _CHAIN] == JOINTS, "the chain must cover every joint, in order"
