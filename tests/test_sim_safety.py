"""SafeFollower around MujocoFollower: unsafe inputs never move the sim arm past the soft limits.

Needs the `sim` extra and the URDFs, like test_sim_follower.py; the model is generated into a temp dir.

The arm runs in `so101.xml` (no floor, no box): SafeFollower limits joint targets, not collisions, and in scene_box.xml
the box stops the arm long before the soft limits.
"""

import numpy as np
import pytest

from so101.conventions import JOINTS
from so101.paths import CONFIG_DIR_NAME, find_project_root
from so101.safety import SafeFollower, SafetyLimits, UnsafeActionError

pytest.importorskip("mujoco")
from so101.sim.follower import MujocoFollower
from so101.sim.model import ensure_model

pytestmark = pytest.mark.sim

URDF = find_project_root() / CONFIG_DIR_NAME / "urdf" / "so101_new_calib.urdf"
FPS = 30
LIMITS = SafetyLimits.defaults()
SCENE = "so101.xml"
# rad: the soft limits bound the targets; the servos overshoot a full max_step move by up to ~0.085 rad (measured
# over 5 seeds). 0.1 is the margin between the default soft limits and the model's joint range.
QPOS_TOLERANCE = 0.1


@pytest.fixture(scope="module")
def model_dir(tmp_path_factory):
    if not URDF.exists():
        pytest.skip(f"no URDF at {URDF}; run `uv run so101-fetch-urdfs`")
    return ensure_model(URDF, tmp_path_factory.mktemp("mujoco"))


@pytest.fixture
def robot(model_dir):
    with SafeFollower(MujocoFollower(fps=FPS, scene=SCENE, urdf=URDF, dest=model_dir), LIMITS) as robot:
        yield robot


def test_far_target_moves_gradually_and_stops_at_the_soft_limit(robot):
    upper = LIMITS.joint_limits["elbow_flex"][1]
    step = LIMITS.max_step["elbow_flex"]
    previous = robot.get_observation()["elbow_flex.pos"]
    for _ in range(2 * FPS):
        sent = robot.send_action({"elbow_flex.pos": 3.0})["elbow_flex.pos"]
        assert sent - previous <= step + 1e-9
        assert sent <= upper
        previous = sent
    assert sent == pytest.approx(upper)
    assert robot.last_clipped == {"elbow_flex.pos": (3.0, pytest.approx(upper))}
    sim = robot.robot
    assert sim.data.ctrl[sim.model.actuator("elbow_flex").id] == pytest.approx(upper)
    # The arm moved most of the way; self-collision (gripper against shoulder) stops it just short of the limit.
    assert 1.3 < robot.get_observation()["elbow_flex.pos"] <= upper + QPOS_TOLERANCE


def test_nan_raises_without_moving_anything(robot):
    sim = robot.robot
    ctrl, qpos = sim.data.ctrl.copy(), sim.data.qpos.copy()
    with pytest.raises(UnsafeActionError):
        robot.send_action({"shoulder_pan.pos": 0.5, "elbow_flex.pos": float("nan")})
    np.testing.assert_array_equal(sim.data.ctrl, ctrl)
    np.testing.assert_array_equal(sim.data.qpos, qpos)


def test_random_actions_stay_inside_the_soft_limits(robot):
    rng = np.random.default_rng(0)
    extremes = [-10.0, 10.0, -1e6, 1e6]
    for _ in range(500):
        joints = rng.choice(JOINTS, size=rng.integers(1, len(JOINTS) + 1), replace=False)
        action = {
            f"{joint}.pos": float(rng.choice(extremes) if rng.random() < 0.3 else rng.uniform(-4, 4))
            for joint in joints
        }
        sent = robot.send_action(action)
        for key, value in sent.items():
            lower, upper = LIMITS.joint_limits[key.removesuffix(".pos")]
            assert lower <= value <= upper, (key, value)
        obs = robot.get_observation()
        for joint in JOINTS:
            lower, upper = LIMITS.joint_limits[joint]
            assert lower - QPOS_TOLERANCE <= obs[f"{joint}.pos"] <= upper + QPOS_TOLERANCE, joint
