"""so101.hardware.rest_pose: recording the follower's rest pose with a fake real follower (no hardware)."""

import json

import pytest

from so101.conventions import JOINTS
from so101.hardware.rest_pose import record_rest_pose
from so101.safety import SafetyLimits

FOLDED = {"shoulder_pan": 0.05, "shoulder_lift": -1.745, "elbow_flex": 1.6, "wrist_flex": 0.95, "wrist_roll": -1.5}


class FakeRealFollower:
    """Records the order of calls; reports FOLDED (radians) once its torque is off and it was folded by hand."""

    def __init__(self):
        self.calls = []
        self.is_connected = False

    def connect(self):
        self.calls.append("connect")
        self.is_connected = True

    def disable_torque(self):
        self.calls.append("disable_torque")

    def get_observation(self):
        self.calls.append("get_observation")
        return {f"{joint}.pos": value for joint, value in (FOLDED | {"gripper": 0.1}).items()} | {"wrist": None}

    def disconnect(self):
        self.calls.append("disconnect")
        self.is_connected = False


def test_records_the_folded_pose_into_safety_json(tmp_path, capsys):
    path = tmp_path / "safety.json"
    follower, prompts = FakeRealFollower(), []

    def wait(prompt):
        prompts.append(prompt)
        follower.calls.append("wait")
        return ""

    stored = record_rest_pose(follower, path, wait=wait)

    # Torque goes off only after the user holds the arm; the pose is read after folding; it ends disconnected.
    assert follower.calls == ["connect", "wait", "disable_torque", "wait", "get_observation", "disconnect"]
    assert "Hold the follower" in prompts[0]
    limits = SafetyLimits.load(path)
    assert limits.rest_pose == stored
    assert set(stored) == set(JOINTS)
    # shoulder_lift and elbow_flex rest on their mechanical stops, beyond the soft limits: stored just inside them.
    assert stored["shoulder_lift"] == pytest.approx(limits.joint_limits["shoulder_lift"][0] + 0.02)
    assert stored["elbow_flex"] == pytest.approx(limits.joint_limits["elbow_flex"][1] - 0.02)
    assert stored["wrist_flex"] == 0.95
    assert "outside the soft limits" in capsys.readouterr().out
    assert json.loads(path.read_text())["rest_pose"] == stored


def test_disconnects_when_the_user_aborts(tmp_path):
    follower = FakeRealFollower()

    def abort(prompt):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        record_rest_pose(follower, tmp_path / "safety.json", wait=abort)
    assert follower.calls == ["connect", "disconnect"]
    assert not (tmp_path / "safety.json").exists()
