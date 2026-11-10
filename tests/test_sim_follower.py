"""MujocoFollower: the sim follower with the real follower's interface, in model radians.

Needs the `sim` extra and the URDFs, like test_sim_model.py; the model is generated into a temp dir.
"""

import math

import numpy as np
import pytest

from so101.conventions import CAMERA_NAME, GRIPPER_LOWER, GRIPPER_UPPER, JOINTS, gripper_percent_to_joint
from so101.paths import CONFIG_DIR_NAME, find_project_root

mujoco = pytest.importorskip("mujoco")
from so101.sim.follower import MujocoFollower
from so101.sim.model import HOME_QPOS, ensure_model

pytestmark = pytest.mark.sim

URDF = find_project_root() / CONFIG_DIR_NAME / "urdf" / "so101_new_calib.urdf"
FPS = 30

# The calibration mid stance (lerobot: body 0°, gripper 50 %) in model radians, with the default wrist_roll offset.
MID_STANCE = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": -math.pi / 2,
    "gripper.pos": gripper_percent_to_joint(50.0, GRIPPER_LOWER, GRIPPER_UPPER),
}


@pytest.fixture(scope="module")
def model_dir(tmp_path_factory):
    if not URDF.exists():
        pytest.skip(f"no URDF at {URDF}; run `uv run so101-fetch-urdfs`")
    return ensure_model(URDF, tmp_path_factory.mktemp("mujoco"))


@pytest.fixture
def make_robot(model_dir):
    robots = []

    def make(**kwargs):
        robot = MujocoFollower(fps=FPS, urdf=URDF, dest=model_dir, **kwargs)
        robot.connect()
        robots.append(robot)
        return robot

    yield make
    for robot in robots:
        if robot.is_connected:
            robot.disconnect()


def hold(robot, action, seconds):
    for _ in range(round(seconds * FPS)):
        robot.send_action(action)


def test_keys_equal_the_real_follower(make_robot):
    real_motors = {f"{joint}.pos": float for joint in JOINTS}  # lerobot SOFollower._motors_ft
    robot = make_robot()
    assert robot.action_features == real_motors
    assert robot.observation_features == real_motors
    assert set(robot.get_observation()) == set(real_motors)
    assert MujocoFollower(camera=True).observation_features == {**real_motors, CAMERA_NAME: (480, 640, 3)}


def test_starts_at_home(make_robot):
    obs = make_robot().get_observation()
    for joint, value in HOME_QPOS.items():
        assert obs[f"{joint}.pos"] == pytest.approx(value, abs=1e-6)


def test_mid_stance_in_mid_stance_out(make_robot):
    robot = make_robot()
    hold(robot, MID_STANCE, 1.0)
    obs = robot.get_observation()
    for key, value in MID_STANCE.items():
        assert obs[key] == pytest.approx(value, abs=math.radians(1)), key


def test_wrist_flex_step_settles(make_robot):
    robot = make_robot()
    robot.send_action({"wrist_flex.pos": math.radians(20)})
    hold(robot, {}, 1.0 - 1 / FPS)
    assert robot.get_observation()["wrist_flex.pos"] == pytest.approx(math.radians(20), abs=math.radians(2))


def test_partial_action_changes_only_the_given_joints(make_robot):
    robot = make_robot()
    before = robot.data.ctrl.copy()
    sent = robot.send_action({"wrist_flex.pos": 0.3})
    assert sent == {"wrist_flex.pos": pytest.approx(0.3)}
    changed = np.flatnonzero(robot.data.ctrl != before)
    assert [robot.model.actuator(i).name for i in changed] == ["wrist_flex"]


def test_action_is_clipped_to_the_joint_range(make_robot):
    robot = make_robot()
    sent = robot.send_action({"gripper.pos": 10.0})
    assert sent["gripper.pos"] == pytest.approx(GRIPPER_UPPER, abs=1e-4)


def test_unknown_key_is_rejected(make_robot):
    with pytest.raises(ValueError, match="Unknown action key"):
        make_robot().send_action({"elbow.pos": 0.0})


def test_reset_returns_home(make_robot):
    robot = make_robot()
    hold(robot, {"shoulder_pan.pos": 0.5}, 0.5)
    robot.reset()
    assert robot.get_observation()["shoulder_pan.pos"] == pytest.approx(0.0, abs=1e-6)


def test_camera_image_has_the_configured_shape(make_robot):
    image = make_robot(camera=True, camera_size=(320, 240)).get_observation()[CAMERA_NAME]
    assert image.shape == (240, 320, 3)
    assert image.dtype == np.uint8
    assert image.max() > 0


def test_needs_connect(model_dir):
    robot = MujocoFollower(urdf=URDF, dest=model_dir)
    with pytest.raises(RuntimeError, match="not connected"):
        robot.get_observation()
    with robot:
        assert robot.is_connected
    assert not robot.is_connected
