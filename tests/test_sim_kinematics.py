"""so101.kinematics against the MuJoCo model generated from the same URDF.

Needs the `sim` extra and the URDFs, like test_sim_model.py; the model is generated into a temp dir.
"""

import numpy as np
import pytest

from so101.conventions import JOINTS
from so101.kinematics import POINTS, ArmPoints
from so101.paths import CONFIG_DIR_NAME, find_project_root
from so101.safety import SafetyLimits

mujoco = pytest.importorskip("mujoco")
from so101.sim.model import ROBOT_XML, ensure_model, load_model

pytestmark = pytest.mark.sim

URDF = find_project_root() / CONFIG_DIR_NAME / "urdf" / "so101_new_calib.urdf"


@pytest.fixture(scope="module")
def sim(tmp_path_factory):
    if not URDF.exists():
        pytest.skip(f"no URDF at {URDF}; run `uv run so101-fetch-urdfs`")
    model = load_model(ROBOT_XML, URDF, ensure_model(URDF, tmp_path_factory.mktemp("mujoco")))
    return model, mujoco.MjData(model)


def random_poses(n, seed=0):
    rng = np.random.default_rng(seed)
    limits = SafetyLimits.defaults().joint_limits
    for _ in range(n):
        yield {joint: rng.uniform(*limits[joint]) for joint in JOINTS}


def set_pose(model, data, q):
    for joint, value in q.items():
        data.qpos[model.joint(joint).qposadr[0]] = value
    mujoco.mj_kinematics(model, data)


def test_link_frames_match_mujoco(sim):
    model, data = sim
    arm = ArmPoints()
    for q in random_poses(50):
        set_pose(model, data, q)
        for link, frame in arm.link_frames(q).items():
            body = data.body(link)
            np.testing.assert_allclose(frame[:3, 3], body.xpos, rtol=0, atol=1e-5, err_msg=link)
            np.testing.assert_allclose(frame[:3, :3], body.xmat.reshape(3, 3), rtol=0, atol=1e-4, err_msg=link)


def test_points_match_mujoco(sim):
    """Every point is its offset in its MuJoCo body, within 1 mm."""
    model, data = sim
    arm = ArmPoints()
    for q in random_poses(50, seed=1):
        set_pose(model, data, q)
        for name, position in arm.positions(q).items():
            point = POINTS[name]
            body = data.body(point.link)
            expected = body.xmat.reshape(3, 3) @ np.asarray(point.offset) + body.xpos
            np.testing.assert_allclose(position, expected, rtol=0, atol=1e-3, err_msg=name)


def test_the_jaw_tips_are_on_the_jaw_meshes(sim):
    """The jaw-tip offsets are vertices of the jaws' visual meshes (within 1 mm), the lowest along the gripper."""
    model, _ = sim
    for name in ("fixed_jaw_tip", "moving_jaw_tip"):
        point = POINTS[name]
        body = model.body(point.link).id
        nearest = np.inf
        for geom in np.flatnonzero((model.geom_bodyid == body) & (model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)):
            mesh = model.geom_dataid[geom]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, model.geom_quat[geom])
            vertices = model.mesh_vert[start : start + count] @ rotation.reshape(3, 3).T + model.geom_pos[geom]
            nearest = min(nearest, np.linalg.norm(vertices - point.offset, axis=1).min())
        assert nearest < 1e-3, name


def test_base_sits_just_below_z_0(sim):
    """The workspace takes the table as z = 0. The URDF puts the bottom of the base 2.4 mm below that, so z = 0 is
    2.4 mm above a real table: a little extra clearance, never less (docs/safety.md)."""
    model, data = sim
    set_pose(model, data, dict.fromkeys(JOINTS, 0.0))
    body = model.body("base_link").id
    lowest = np.inf
    for geom in np.flatnonzero((model.geom_bodyid == body) & (model.geom_type == mujoco.mjtGeom.mjGEOM_MESH)):
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = data.geom_xmat[geom].reshape(3, 3) @ model.mesh_vert[start : start + count].T
        lowest = min(lowest, (vertices.T + data.geom_xpos[geom])[:, 2].min())
    assert lowest == pytest.approx(-0.0024, abs=2e-4)
