import json
from importlib.resources import files

from so101.conventions import JOINTS


def load_default_visualization() -> dict:
    return json.loads(files("so101").joinpath("defaults/visualization.json").read_text())


def test_visualization_defaults_is_valid_json():
    assert isinstance(load_default_visualization(), dict)


def test_visualization_defaults_cover_every_joint():
    joints = load_default_visualization()["joints"]
    assert set(joints) == set(JOINTS)
    for name in JOINTS:
        assert isinstance(joints[name]["offset"], (int, float))
        assert isinstance(joints[name]["invert"], bool)


# The follower URDF's joint ranges in radians (so101_new_calib.urdf, the same as the MuJoCo ctrlrange).
URDF_RANGES = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}


def load_default_safety() -> dict:
    return json.loads(files("so101").joinpath("defaults/safety.json").read_text())


def test_safety_defaults_cover_every_joint():
    safety = load_default_safety()
    assert set(safety["joint_limits"]) == set(JOINTS)
    assert set(safety["max_step"]) == set(JOINTS)


def test_safety_defaults_lie_inside_the_model_range():
    for joint, (lower, upper) in load_default_safety()["joint_limits"].items():
        urdf_lower, urdf_upper = URDF_RANGES[joint]
        assert urdf_lower < lower < upper < urdf_upper, joint


def test_safety_default_wrist_roll_is_well_under_a_full_turn():
    lower, upper = load_default_safety()["joint_limits"]["wrist_roll"]
    assert upper - lower < 2 * 3.14159 * 0.9
