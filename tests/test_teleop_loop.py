"""run_teleop_loop with a fake leader, follower and visualizer (no hardware, no Rerun)."""

import pytest

from so101.cli.teleop import run_teleop_loop


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
    steps = run_teleop_loop(follower, leader, fps=1000, visualizer=visualizer, max_steps=5)
    assert steps == 5
    expected = [{"wrist_flex.pos": pytest.approx(0.1 * i)} for i in range(1, 6)]
    assert follower.sent == expected
    assert [action for _, action in visualizer.steps] == expected
    assert [obs["step"] for obs, _ in visualizer.steps] == [1, 2, 3, 4, 5]


def test_connects_and_disconnects_both():
    leader, follower = FakeLeader(), FakeFollower()
    run_teleop_loop(follower, leader, fps=1000, max_steps=2)
    assert (leader.connects, leader.disconnects) == (1, 1)
    assert (follower.connects, follower.disconnects) == (1, 1)


def test_disconnects_both_on_an_error():
    leader, follower = FakeLeader(), FakeFollower(fail_after=2)
    with pytest.raises(RuntimeError, match="bus error"):
        run_teleop_loop(follower, leader, fps=1000, max_steps=10)
    assert not leader.is_connected and not follower.is_connected
