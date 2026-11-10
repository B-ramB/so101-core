import json
import math
import random

import pytest

from so101.conventions import (
    GRIPPER_LOWER,
    GRIPPER_UPPER,
    JOINTS,
    JointCalibration,
    JointPositions,
    gripper_percent_to_joint,
    joint_to_gripper_percent,
    model_to_real,
    real_to_model,
    rescale_gripper,
)
from so101.paths import VIS_DEFAULTS_PATH


def test_real_to_model_identity():
    assert real_to_model(12.5) == 12.5


def test_real_to_model_offset():
    assert real_to_model(10.0, offset=-90.0) == -80.0


def test_real_to_model_invert():
    assert real_to_model(10.0, invert=True) == -10.0


def test_real_to_model_invert_then_offset():
    # model = sign * real + offset: the offset is applied after the sign flip.
    assert real_to_model(10.0, offset=5.0, invert=True) == -5.0


@pytest.mark.parametrize(
    ("percent", "expected"),
    [(0.0, -0.2), (50.0, 0.65), (100.0, 1.5)],
)
def test_gripper_percent_to_joint(percent, expected):
    assert gripper_percent_to_joint(percent, lower=-0.2, upper=1.5) == pytest.approx(expected)


# --- radians <-> lerobot units ---------------------------------------------------------------------------------------

MID_STANCE = {joint: 50.0 if joint == "gripper" else 0.0 for joint in JOINTS}
MIXED = JointCalibration(
    {
        "shoulder_pan": {"offset": 12.0, "invert": True},
        "elbow_flex": {"offset": -30.0, "invert": False},
        "wrist_roll": {"offset": -90.0, "invert": False},
        "gripper": {"offset": 100.0, "invert": True},
    }
)


def default_calibration() -> JointCalibration:
    return JointCalibration(json.loads(VIS_DEFAULTS_PATH.read_text())["joints"])


@pytest.mark.parametrize("offset", [0.0, -90.0, 17.5])
@pytest.mark.parametrize("invert", [False, True])
def test_model_to_real_inverts_real_to_model(offset, invert):
    for value in (-120.0, 0.0, 33.3):
        assert model_to_real(real_to_model(value, offset, invert), offset, invert) == pytest.approx(value)
        assert real_to_model(model_to_real(value, offset, invert), offset, invert) == pytest.approx(value)


@pytest.mark.parametrize(
    ("joint", "percent"),
    [(-0.2, 0.0), (0.65, 50.0), (1.5, 100.0)],
)
def test_joint_to_gripper_percent(joint, percent):
    assert joint_to_gripper_percent(joint, lower=-0.2, upper=1.5) == pytest.approx(percent)
    assert gripper_percent_to_joint(percent, lower=-0.2, upper=1.5) == pytest.approx(joint)


def test_round_trip_random_poses():
    rng = random.Random(0)
    for _ in range(200):
        degrees = {joint: rng.uniform(0, 100) if joint == "gripper" else rng.uniform(-180, 180) for joint in JOINTS}
        assert JointPositions.from_degrees(degrees, MIXED).to_degrees() == pytest.approx(degrees)

        radians = {joint: rng.uniform(-math.pi, math.pi) for joint in JOINTS}
        back = JointPositions.from_degrees(JointPositions(radians, MIXED).to_degrees(), MIXED)
        assert back.radians == pytest.approx(radians)


def test_mid_stance_in_radians_with_the_default_calibration():
    q = JointPositions.from_degrees(MID_STANCE, default_calibration())
    assert q.radians == pytest.approx(
        {
            "shoulder_pan": 0.0,
            "shoulder_lift": 0.0,
            "elbow_flex": 0.0,
            "wrist_flex": 0.0,
            "wrist_roll": -math.pi / 2,
            "gripper": (GRIPPER_LOWER + GRIPPER_UPPER) / 2,
        }
    )
    assert q.to_degrees() == pytest.approx(MID_STANCE)


def test_invert_flips_the_sign():
    cal = JointCalibration({"elbow_flex": {"offset": 0.0, "invert": True}})
    q = JointPositions.from_degrees({"elbow_flex": 30.0}, cal)
    assert q["elbow_flex"] == pytest.approx(-math.radians(30))
    assert q.to_degrees() == pytest.approx({"elbow_flex": 30.0})


def test_inverted_gripper_with_offset_100_swaps_open_and_closed():
    cal = JointCalibration({"gripper": {"offset": 100.0, "invert": True}})
    assert cal.to_radians("gripper", 0.0) == pytest.approx(GRIPPER_UPPER)
    assert cal.to_radians("gripper", 100.0) == pytest.approx(GRIPPER_LOWER)


def test_custom_gripper_limits():
    leader = JointCalibration(gripper_limits=(0.0, 0.785398163397))
    assert leader.to_radians("gripper", 50.0) == pytest.approx(0.3926990817)
    assert leader.to_degrees("gripper", 0.785398163397) == pytest.approx(100.0)


def test_joint_without_calibration_is_plain_degrees():
    q = JointPositions({"shoulder_lift": math.pi / 4}, JointCalibration())
    assert q.to_degrees() == pytest.approx({"shoulder_lift": 45.0})
    assert "shoulder_lift" in q and "gripper" not in q


@pytest.mark.parametrize("percent", [0.0, 50.0, 100.0])
def test_rescale_gripper_keeps_the_opening(percent):
    follower, leader = (GRIPPER_LOWER, GRIPPER_UPPER), (0.0, 0.785)
    value = gripper_percent_to_joint(percent, *follower)
    assert rescale_gripper(value, follower, leader) == pytest.approx(gripper_percent_to_joint(percent, *leader))
