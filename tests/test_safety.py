"""SafeFollower and SafetyLimits with a fake follower that records what it receives (no hardware, no mujoco)."""

import itertools
import json
import math

import pytest

from so101.conventions import JOINTS
from so101.robot import Follower
from so101.safety import SafeFollower, SafetyLimits, UnsafeActionError

STEP = 0.17
LIMIT = 2.0
LIMITS = SafetyLimits(
    joint_limits={joint: (-LIMIT, LIMIT) for joint in JOINTS},
    max_step={joint: STEP for joint in JOINTS},
)


class FakeFollower:
    """Records every action, returns it as sent, and reports the last goals as the observation."""

    def __init__(self, start=None):
        self.is_connected = False
        self.sent: list[dict] = []
        self.position = {joint: 0.0 for joint in JOINTS} | (start or {})
        self.observation_features = self.action_features = {f"{joint}.pos": float for joint in JOINTS}

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_observation(self):
        return {f"{joint}.pos": value for joint, value in self.position.items()}

    def send_action(self, action):
        self.sent.append(dict(action))
        for key, value in action.items():
            self.position[key.removesuffix(".pos")] = value
        return dict(action)


@pytest.fixture
def fake():
    return FakeFollower()


@pytest.fixture
def safe(fake):
    robot = SafeFollower(fake, LIMITS)
    robot.connect()
    return robot


@pytest.mark.parametrize(
    "action",
    [
        {"elbow_flex.pos": float("nan")},
        {"elbow_flex.pos": float("inf")},
        {"elbow_flex.pos": float("-inf")},
        {"elbow_flex.pos": "1.0"},
        {"elbow_flex.pos": True},
        {"elbow_flex.pos": None},
        {"elbow.pos": 0.1},
        {"elbow_flex": 0.1},
        {"wrist": 0.1},
        {"shoulder_pan.pos": 0.1, "elbow_flex.pos": float("nan")},  # a bad entry next to a good one
        {"shoulder_pan.pos": 0.1, "unknown.pos": 0.1},
    ],
)
def test_bad_action_raises_and_sends_nothing(safe, fake, action):
    with pytest.raises(UnsafeActionError):
        safe.send_action(action)
    assert fake.sent == []


def test_unsafe_action_error_is_a_value_error():
    assert issubclass(UnsafeActionError, ValueError)


def test_send_before_connect_raises(fake):
    with pytest.raises(RuntimeError, match="connect"):
        SafeFollower(fake, LIMITS).send_action({"elbow_flex.pos": 0.1})
    assert fake.sent == []


def test_value_outside_the_limits_is_clamped_and_reported(safe, fake):
    safe.send_action({"elbow_flex.pos": 3.0})
    assert safe.last_clipped == {"elbow_flex.pos": (3.0, pytest.approx(STEP))}
    for _ in range(20):
        sent = safe.send_action({"elbow_flex.pos": 3.0})
    assert sent == {"elbow_flex.pos": LIMIT}
    assert safe.last_clipped == {"elbow_flex.pos": (3.0, LIMIT)}
    assert max(action["elbow_flex.pos"] for action in fake.sent) == LIMIT


def test_a_far_target_is_reached_in_max_steps(safe, fake):
    target = math.radians(90)
    sent = safe.send_action({"wrist_flex.pos": target})
    assert sent["wrist_flex.pos"] == pytest.approx(STEP)
    expected = math.ceil(target / STEP)
    for _ in range(expected - 1):
        sent = safe.send_action({"wrist_flex.pos": target})
    assert sent["wrist_flex.pos"] == pytest.approx(target)
    assert safe.last_clipped == {}
    assert len(fake.sent) == expected
    values = [0.0] + [action["wrist_flex.pos"] for action in fake.sent]
    assert all(abs(b - a) <= STEP + 1e-12 for a, b in itertools.pairwise(values))


def test_step_limit_works_downwards(safe):
    sent = safe.send_action({"shoulder_lift.pos": -1.0})
    assert sent["shoulder_lift.pos"] == pytest.approx(-STEP)


def test_partial_action_changes_only_the_given_joints(safe, fake):
    safe.send_action({"wrist_flex.pos": 0.1})
    assert list(fake.sent[-1]) == ["wrist_flex.pos"]
    safe.send_action({"gripper.pos": 0.1})
    assert list(fake.sent[-1]) == ["gripper.pos"]
    # wrist_flex's goal was kept: the next step starts from 0.1, not from 0
    sent = safe.send_action({"wrist_flex.pos": 1.0})
    assert sent["wrist_flex.pos"] == pytest.approx(0.1 + STEP)


def test_first_goal_is_the_position_at_connect():
    fake = FakeFollower(start={"elbow_flex": 1.0})
    with SafeFollower(fake, LIMITS) as safe:
        sent = safe.send_action({"elbow_flex.pos": 0.0})
    assert sent["elbow_flex.pos"] == pytest.approx(1.0 - STEP)
    assert not fake.is_connected


def test_start_outside_the_limits_moves_back_gradually():
    fake = FakeFollower(start={"elbow_flex": 2.5})
    safe = SafeFollower(fake, LIMITS)
    safe.connect()
    sent = safe.send_action({"elbow_flex.pos": 2.5})
    assert sent["elbow_flex.pos"] == pytest.approx(2.5 - STEP)


def test_empty_action_is_forwarded(safe, fake):
    assert safe.send_action({}) == {}
    assert fake.sent == [{}]


def test_is_a_follower(fake):
    assert isinstance(SafeFollower(fake, LIMITS), Follower)


def test_delegates_features_and_observation(safe, fake):
    assert safe.action_features == fake.action_features
    assert safe.get_observation() == fake.get_observation()
    assert safe.is_connected


# --- SafetyLimits and the limits file ---


def test_defaults_are_valid():
    limits = SafetyLimits.defaults()
    assert set(limits.joint_limits) == set(JOINTS)


def test_load_creates_the_file_from_the_defaults(tmp_path):
    path = tmp_path / "safety.json"
    limits = SafetyLimits.load(path)
    assert path.exists()
    assert limits == SafetyLimits.defaults()


def test_load_fills_missing_keys_from_the_defaults(tmp_path):
    path = tmp_path / "safety.json"
    path.write_text(json.dumps({"joint_limits": {"elbow_flex": [-1.0, 1.0]}}))
    limits = SafetyLimits.load(path)
    defaults = SafetyLimits.defaults()
    assert limits.joint_limits["elbow_flex"] == (-1.0, 1.0)
    assert limits.joint_limits["wrist_flex"] == defaults.joint_limits["wrist_flex"]
    assert limits.max_step == defaults.max_step


@pytest.mark.parametrize(
    "user",
    [
        {"joint_limits": {"elbow_flex": [1.0, -1.0]}},  # min > max
        {"joint_limits": {"elbow_flex": [1.0, 1.0]}},  # empty range
        {"joint_limits": {"elbow_flex": [-1.0]}},
        {"joint_limits": {"elbow_flex": ["a", 1.0]}},
        {"joint_limits": {"elbow": [-1.0, 1.0]}},  # unknown joint
        {"max_step": {"gripper": 0}},
        {"max_step": {"gripper": -0.1}},
    ],
)
def test_load_rejects_invalid_limits(tmp_path, user):
    path = tmp_path / "safety.json"
    path.write_text(json.dumps(user))
    with pytest.raises(ValueError, match="safety.json"):
        SafetyLimits.load(path)


def test_limits_reject_a_missing_joint():
    with pytest.raises(ValueError, match="missing"):
        SafetyLimits({"elbow_flex": (-1.0, 1.0)}, {joint: STEP for joint in JOINTS})


def test_limits_reject_non_finite_values():
    with pytest.raises(ValueError, match="finite"):
        SafetyLimits(dict(LIMITS.joint_limits) | {"gripper": (0.0, math.inf)}, LIMITS.max_step)
