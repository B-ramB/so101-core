"""run_teleop_loop driving MujocoFollower with a fake leader (no hardware)."""

import math

import pytest

from so101.cli.teleop import run_teleop_loop
from so101.conventions import GRIPPER_LOWER, GRIPPER_UPPER, gripper_percent_to_joint
from so101.paths import CONFIG_DIR_NAME, find_project_root

pytest.importorskip("mujoco")
from so101.sim.follower import MujocoFollower
from so101.sim.model import ensure_model

pytestmark = pytest.mark.sim

URDF = find_project_root() / CONFIG_DIR_NAME / "urdf" / "so101_new_calib.urdf"
FPS = 30
MID_STANCE = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": -math.pi / 2,
    "gripper.pos": gripper_percent_to_joint(50.0, GRIPPER_LOWER, GRIPPER_UPPER),
}


class FakeLeader:
    """A leader held in the mid stance, already in radians (as RealLeader returns it)."""

    is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_action(self):
        return dict(MID_STANCE)


class LastStep:
    def log_step(self, obs, action):
        self.obs, self.action = obs, action


def test_leader_drives_the_sim(tmp_path):
    if not URDF.exists():
        pytest.skip(f"no URDF at {URDF}; run `uv run so101-fetch-urdfs`")
    robot = MujocoFollower(fps=FPS, urdf=URDF, dest=ensure_model(URDF, tmp_path))
    leader, last = FakeLeader(), LastStep()
    assert run_teleop_loop(robot, leader, fps=1000, visualizer=last, max_steps=FPS) == FPS  # 1 s of sim time
    for key, value in MID_STANCE.items():
        assert last.obs[key] == pytest.approx(value, abs=math.radians(1)), key
    assert not robot.is_connected and not leader.is_connected
