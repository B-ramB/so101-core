"""The MuJoCo model generated from the follower URDF.

Needs the `sim` extra and the URDFs: the tests use the so101_config/urdf of this checkout (fetch it with
`uv run so101-fetch-urdfs`) and are skipped when it is missing. The model is generated into a temp dir.
"""

import numpy as np
import pytest

from so101.conventions import GRIPPER_LOWER, GRIPPER_UPPER, JOINTS
from so101.paths import CONFIG_DIR_NAME, find_project_root

mujoco = pytest.importorskip("mujoco")
from so101.sim.model import HOME_QPOS, ROBOT_XML, ensure_model, home, load_model

pytestmark = pytest.mark.sim

URDF = find_project_root() / CONFIG_DIR_NAME / "urdf" / "so101_new_calib.urdf"

# Joint targets for shoulder_pan .. wrist_roll that put the box of scene_box.xml between the open jaws, and that hold
# it 9 cm higher (found with a small IK on the jaw centre, gripper pointing down).
GRASP = [0.0, 0.19, 0.23, 1.15, 0.0]
LIFT = [0.0, 0.05, -0.26, 1.66, 0.0]
GRIPPER_OPEN, GRIPPER_CLOSED = 0.7, -0.17


@pytest.fixture(scope="module")
def model_dir(tmp_path_factory):
    if not URDF.exists():
        pytest.skip(f"no URDF at {URDF}; run `uv run so101-fetch-urdfs`")
    return ensure_model(URDF, tmp_path_factory.mktemp("mujoco"))


@pytest.fixture
def scene(model_dir):
    model = load_model(urdf=URDF, dest=model_dir)
    data = mujoco.MjData(model)
    home(model, data)
    return model, data


def test_robot_compiles(model_dir):
    model = load_model(ROBOT_XML, urdf=URDF, dest=model_dir)
    assert model.nu == len(JOINTS)


def test_actuator_names_equal_joints(scene):
    model, _ = scene
    assert [model.actuator(i).name for i in range(model.nu)] == JOINTS


def test_actuators_drive_their_joint_within_its_range(scene):
    model, _ = scene
    for name in JOINTS:
        actuator, joint = model.actuator(name), model.joint(name)
        assert model.actuator_trnid[actuator.id, 0] == joint.id
        np.testing.assert_allclose(model.actuator_ctrlrange[actuator.id], model.jnt_range[joint.id])


def test_gripper_range_matches_the_conventions(model_dir):
    model = load_model(ROBOT_XML, urdf=URDF, dest=model_dir)
    np.testing.assert_allclose(model.jnt_range[model.joint("gripper").id], [GRIPPER_LOWER, GRIPPER_UPPER])


def test_home_pose_has_no_self_collision(scene):
    model, data = scene
    box = model.body("box").id
    arm_contacts = [c for c in data.contact[: data.ncon] if box not in model.geom_bodyid[[c.geom1, c.geom2]]]
    assert arm_contacts == []


def test_1000_steps_from_home_are_stable(scene):
    model, data = scene
    for _ in range(1000):
        mujoco.mj_step(model, data)
    assert np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))
    for joint, value in HOME_QPOS.items():
        assert data.qpos[model.joint(joint).qposadr[0]] == pytest.approx(value, abs=0.02)


def test_regenerates_only_when_stale(model_dir):
    robot = model_dir / ROBOT_XML
    before = robot.stat().st_mtime_ns
    ensure_model(URDF, model_dir)
    assert robot.stat().st_mtime_ns == before
    ensure_model(URDF, model_dir, force=True)
    assert robot.stat().st_mtime_ns > before


def test_box_can_be_picked_up(scene):
    model, data = scene

    def move(arm, gripper, seconds):
        for name, value in zip(JOINTS, [*arm, gripper]):
            data.ctrl[model.actuator(name).id] = value
        for _ in range(round(seconds / model.opt.timestep)):
            mujoco.mj_step(model, data)

    start = data.body("box").xpos.copy()
    move(LIFT, GRIPPER_OPEN, 2)
    move(GRASP, GRIPPER_OPEN, 2)
    move(GRASP, GRIPPER_CLOSED, 1.5)
    assert data.qpos[model.joint("gripper").qposadr[0]] > 0, "the jaws closed completely: nothing between them"
    move(LIFT, GRIPPER_CLOSED, 2)
    assert data.body("box").xpos[2] > start[2] + 0.07
    move(LIFT, GRIPPER_OPEN, 2)
    assert data.body("box").xpos[2] == pytest.approx(start[2], abs=0.005)
