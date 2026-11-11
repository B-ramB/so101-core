"""SO-101 conventions shared by the hardware, visualization and simulation code.

The standard unit for joint positions in so101 is **model radians**: the joint values of the follower URDF, which the
MuJoCo model uses as-is. lerobot reports and takes the body joints in degrees and the gripper in percent (0-100); those
units only exist at the edge with the real arm. `JointPositions` holds radians and converts to lerobot units on request.

The real arm maps to the model as `model = sign * real + offset` (in lerobot units, before the unit conversion), with
the per-joint offset / invert from visualization.json -> joints. See docs/units-and-joints.md.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass, field

LEADER_TYPE = "so101_leader"
FOLLOWER_TYPE = "so101_follower"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CAMERA_NAME = "wrist"
GRIPPER = "gripper"

# Follower gripper joint limits in radians, the same in so101_new_calib.urdf and the generated MJCF.
GRIPPER_LOWER = -0.174533
GRIPPER_UPPER = 1.74533


def real_to_model(value: float, offset: float = 0.0, invert: bool = False) -> float:
    """Real-arm joint value -> model value: model = sign * real + offset, where sign = -1 when inverted."""
    sign = -1.0 if invert else 1.0
    return sign * value + float(offset)


def model_to_real(value: float, offset: float = 0.0, invert: bool = False) -> float:
    """Model value -> real-arm joint value, the inverse of `real_to_model`: real = sign * (model - offset)."""
    sign = -1.0 if invert else 1.0
    return sign * (value - float(offset))


def gripper_percent_to_joint(percent: float, lower: float, upper: float) -> float:
    """Gripper opening in percent (0-100) -> URDF joint value between the joint's lower and upper limit."""
    return lower + percent / 100.0 * (upper - lower)


def joint_to_gripper_percent(value: float, lower: float, upper: float) -> float:
    """URDF gripper joint value -> opening in percent (0-100), the inverse of `gripper_percent_to_joint`."""
    return (value - lower) / (upper - lower) * 100.0


def rescale_gripper(value: float, from_limits: tuple[float, float], to_limits: tuple[float, float]) -> float:
    """A gripper joint value in one joint range -> the same opening (percent) in another range, e.g. follower -> leader."""
    return gripper_percent_to_joint(joint_to_gripper_percent(value, *from_limits), *to_limits)


@dataclass(frozen=True)
class JointCalibration:
    """How the real arm's lerobot units map to model radians.

    joints: per joint {"offset": float, "invert": bool}, the visualization.json -> joints block. The offset is in
        lerobot units (degrees, percent for the gripper). A joint missing here maps with offset 0 and no invert.
    gripper_limits: the gripper joint's (lower, upper) in radians; 0 % maps to lower, 100 % to upper.
    """

    joints: Mapping[str, Mapping] = field(default_factory=dict)
    gripper_limits: tuple[float, float] = (GRIPPER_LOWER, GRIPPER_UPPER)

    def _offset_invert(self, joint: str) -> tuple[float, bool]:
        cfg = self.joints.get(joint, {})
        return float(cfg.get("offset", 0.0)), bool(cfg.get("invert", False))

    def to_radians(self, joint: str, value: float) -> float:
        """lerobot value (degrees, gripper percent) -> model radians for one joint."""
        value = real_to_model(value, *self._offset_invert(joint))
        if joint == GRIPPER:
            return gripper_percent_to_joint(value, *self.gripper_limits)
        return math.radians(value)

    def to_degrees(self, joint: str, value: float) -> float:
        """Model radians -> lerobot value (degrees, gripper percent) for one joint."""
        if joint == GRIPPER:
            value = joint_to_gripper_percent(value, *self.gripper_limits)
        else:
            value = math.degrees(value)
        return model_to_real(value, *self._offset_invert(joint))


class JointPositions:
    """Joint positions in the so101 standard: model radians, keyed by joint name ("shoulder_pan", ...).

    Radians are what it stores. lerobot units come in only through `from_degrees` and go out only through
    `to_degrees`, both via the `JointCalibration` it holds.
    """

    def __init__(self, radians: Mapping[str, float], calibration: JointCalibration):
        self.radians = {joint: float(value) for joint, value in radians.items()}
        self.calibration = calibration

    @classmethod
    def from_degrees(cls, degrees: Mapping[str, float], calibration: JointCalibration) -> "JointPositions":
        """From lerobot units: body joints in degrees, gripper in percent (0-100)."""
        return cls({joint: calibration.to_radians(joint, value) for joint, value in degrees.items()}, calibration)

    def to_degrees(self) -> dict[str, float]:
        """In lerobot units: body joints in degrees, the gripper in percent (0-100)."""
        return {joint: self.calibration.to_degrees(joint, value) for joint, value in self.radians.items()}

    def __getitem__(self, joint: str) -> float:
        return self.radians[joint]

    def __contains__(self, joint: object) -> bool:
        return joint in self.radians

    def __repr__(self) -> str:
        return f"JointPositions({self.radians})"
