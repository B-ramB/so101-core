"""run_teleop_loop with a fake leader, follower and visualizer (no hardware, no Rerun)."""

import pytest

from so101.cli.teleop import run_teleop_loop
from so101.conventions import JOINTS
from so101.safety import SafeFollower, SafetyLimits, UnsafeActionError


class FakeDevice:
    def __init__(self):
        self.is_connected = False
        self.connects = self.disconnects = 0

    def connect(self):
        self.is_connected = True
        self.connects += 1

    def disconnect(self):
        self.is_connected = False
        self.disconnects += 1


class FakeLeader(FakeDevice):
    def __init__(self):
        super().__init__()
        self.count = 0

    def get_action(self):
        assert self.is_connected
        self.count += 1
        return {"wrist_flex.pos": 0.1 * self.count}


class FakeFollower(FakeDevice):
    def __init__(self, fail_after=None):
        super().__init__()
        self.sent = []
        self.fail_after = fail_after

    def send_action(self, action):
        assert self.is_connected
        if self.fail_after is not None and len(self.sent) == self.fail_after:
            raise RuntimeError("bus error")
        self.sent.append(action)
        return action

    def get_observation(self):
        return {"wrist_flex.pos": self.sent[-1]["wrist_flex.pos"], "step": len(self.sent)}


class FakeVisualizer:
    def __init__(self):
        self.steps = []

    def log_step(self, obs, action):
        self.steps.append((obs, action))


def test_forwards_each_action_and_logs_it():
    leader, follower, visualizer = FakeLeader(), FakeFollower(), FakeVisualizer()
    steps = run_teleop_loop(follower, leader, control_hz=1000, visualizer=visualizer, max_steps=5)
    assert steps == 5
    expected = [{"wrist_flex.pos": pytest.approx(0.1 * i)} for i in range(1, 6)]
    assert follower.sent == expected
    assert [action for _, action in visualizer.steps] == expected
    assert [obs["step"] for obs, _ in visualizer.steps] == [1, 2, 3, 4, 5]


def test_connects_and_disconnects_both():
    leader, follower = FakeLeader(), FakeFollower()
    run_teleop_loop(follower, leader, control_hz=1000, max_steps=2)
    assert (leader.connects, leader.disconnects) == (1, 1)
    assert (follower.connects, follower.disconnects) == (1, 1)


def test_disconnects_both_on_an_error():
    leader, follower = FakeLeader(), FakeFollower(fail_after=2)
    with pytest.raises(RuntimeError, match="bus error"):
        run_teleop_loop(follower, leader, control_hz=1000, max_steps=10)
    assert not leader.is_connected and not follower.is_connected


REST = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.5,
    "elbow_flex": 1.5,
    "wrist_flex": 0.9,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
LIMITS = SafetyLimits(
    joint_limits={joint: (-2.0, 2.0) for joint in JOINTS},
    max_step={joint: 0.17 for joint in JOINTS},
    rest_pose=REST,
)


class ArmFollower(FakeDevice):
    """A whole arm: every joint goes straight to its last target."""

    def __init__(self):
        super().__init__()
        self.position = {joint: 0.0 for joint in JOINTS}
        self.disconnected_at = None

    def send_action(self, action):
        assert self.is_connected
        self.position |= {key.removesuffix(".pos"): value for key, value in action.items()}
        return action

    def get_observation(self):
        assert self.is_connected
        return {f"{joint}.pos": value for joint, value in self.position.items()}

    def disconnect(self):
        self.disconnected_at = dict(self.position)
        super().disconnect()


class ScriptedLeader(FakeLeader):
    """Raises `error` on the action after `steps` good ones."""

    def __init__(self, steps, error):
        super().__init__()
        self.steps, self.error = steps, error

    def get_action(self):
        if self.count == self.steps:
            raise self.error
        return super().get_action()


@pytest.fixture
def safe_arm(monkeypatch):
    monkeypatch.setattr("so101.safety.REST_HZ", 1000.0)  # rest without waiting
    arm = ArmFollower()
    return arm, SafeFollower(arm, LIMITS)


def test_ctrl_c_moves_the_follower_to_rest_before_torque_off(safe_arm, capsys):
    arm, robot = safe_arm
    steps = run_teleop_loop(robot, ScriptedLeader(3, KeyboardInterrupt()), control_hz=1000)
    assert steps == 3
    assert arm.disconnected_at == pytest.approx(REST)
    assert not arm.is_connected
    assert "Moving the follower to its rest pose" in capsys.readouterr().out


def test_an_unsafe_action_moves_the_follower_to_rest_and_is_raised(safe_arm):
    arm, robot = safe_arm
    with pytest.raises(UnsafeActionError):
        run_teleop_loop(robot, ScriptedLeader(2, UnsafeActionError("nan")), control_hz=1000)
    assert arm.disconnected_at == pytest.approx(REST)
    assert not arm.is_connected


def test_a_failing_bus_still_disconnects(safe_arm):
    arm, robot = safe_arm

    def bus_error(action):
        raise ConnectionError("no status packet")

    arm.send_action = bus_error
    with pytest.raises(ConnectionError):
        run_teleop_loop(robot, FakeLeader(), control_hz=1000, max_steps=5)
    assert not arm.is_connected


def test_ctrl_c_during_the_rest_move_skips_it(safe_arm, monkeypatch, capsys):
    arm, robot = safe_arm

    def interrupt(**kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(robot, "rest", interrupt)
    run_teleop_loop(robot, FakeLeader(), control_hz=1000, max_steps=2)  # returns normally
    assert not arm.is_connected
    assert "Skipped the rest pose" in capsys.readouterr().out
