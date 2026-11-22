"""so101.kinematics: the arm points from joint angles, in plain Python (tests/test_sim_kinematics.py checks MuJoCo)."""

import numpy as np
import pytest

from so101.conventions import JOINTS
from so101.kinematics import POINTS, ArmPoints

HOME = dict.fromkeys(JOINTS, 0.0)
HOME_TIP = (0.391, 0.000, 0.227)  # metres: the gripper frame with every joint at 0 rad


def test_home_gripper_frame():
    np.testing.assert_allclose(ArmPoints().positions(HOME)["gripper_frame"], HOME_TIP, rtol=0, atol=1e-3)


def test_every_point_is_returned():
    assert set(ArmPoints().positions(HOME)) == set(POINTS)


def test_shoulder_pan_rotates_about_the_base_z_axis():
    arm = ArmPoints()
    home = arm.positions(HOME)
    turned = arm.positions(HOME | {"shoulder_pan": np.pi / 2})
    for name in POINTS:
        # same height and distance from the base axis; the pan axis is 3.9 cm in front of the origin
        assert turned[name][2] == pytest.approx(home[name][2], abs=1e-4)
        axis = np.array([0.0388353, 0.0])
        assert np.linalg.norm(turned[name][:2] - axis) == pytest.approx(np.linalg.norm(home[name][:2] - axis), abs=1e-4)


def test_the_gripper_joint_moves_only_the_moving_jaw():
    arm = ArmPoints()
    closed, opened = arm.positions(HOME), arm.positions(HOME | {"gripper": 1.5})
    for name in POINTS:
        moved = not np.allclose(closed[name], opened[name])
        assert moved == (name == "moving_jaw_tip"), name


def test_observation_keys_work():
    arm = ArmPoints()
    q = dict(zip(JOINTS, np.linspace(-0.5, 0.5, len(JOINTS)), strict=True))
    keyed = {f"{joint}.pos": value for joint, value in q.items()}
    for name, position in arm.positions(q).items():
        np.testing.assert_array_equal(arm.positions(keyed)[name], position)


def test_a_missing_joint_raises():
    q = dict(HOME)
    del q["wrist_flex"]
    with pytest.raises(KeyError, match="wrist_flex"):
        ArmPoints().positions(q)
