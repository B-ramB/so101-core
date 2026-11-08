"""SO-101 conventions shared by the hardware and visualization code.

lerobot reports the body joints in degrees and the gripper in percent (0-100). The URDF models use radians,
and the gripper joint has its own range. The real arm maps to the model as `model = sign * real + offset`.
"""

LEADER_TYPE = "so101_leader"
FOLLOWER_TYPE = "so101_follower"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CAMERA_NAME = "wrist"


def real_to_model(value: float, offset: float = 0.0, invert: bool = False) -> float:
    """Real-arm joint value -> model value: model = sign * real + offset, where sign = -1 when inverted."""
    sign = -1.0 if invert else 1.0
    return sign * value + float(offset)


def gripper_percent_to_joint(percent: float, lower: float, upper: float) -> float:
    """Gripper opening in percent (0-100) -> URDF joint value between the joint's lower and upper limit."""
    return lower + percent / 100.0 * (upper - lower)
