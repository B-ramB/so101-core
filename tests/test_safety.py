"""SafeFollower, SafetyLimits and Workspace with a fake follower that records what it receives (no hardware, no mujoco)."""

import itertools
import json
import math
import random
from dataclasses import replace

import pytest

from so101.conventions import JOINTS
from so101.kinematics import POINTS, ArmPoints
from so101.robot import Follower
from so101.safety import SafeFollower, SafetyLimits, UnsafeActionError, Workspace, leaves_workspace

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


# --- Workspace: the Cartesian limits (no mujoco: the FK is so101.kinematics) ---

TABLE = Workspace(z_min=0.01)
WITH_TABLE = replace(LIMITS, workspace=TABLE)
HOME = dict.fromkeys(JOINTS, 0.0)


def lowest_point(q) -> float:
    return min(p[2] - POINTS[name].radius for name, p in ArmPoints().positions(q).items())


def test_workspace_excess_and_contains():
    box = Workspace(x_min=0.0, x_max=1.0, z_min=0.1)
    assert box.contains((0.5, 5.0, 0.2))  # y is unbounded
    assert box.excess((0.5, 0.0, 0.05)) == pytest.approx(0.05)
    assert box.excess((-0.1, 0.0, 0.0)) == pytest.approx(0.1 + 0.1)  # summed over the bounds
    assert box.excess((0.5, 0.0, 0.12), radius=0.03) == pytest.approx(0.01)  # a sphere must fit
    ring = Workspace(r_min=0.1, r_max=0.4)
    assert ring.contains((0.0, -0.2, -5.0))
    assert ring.excess((0.03, 0.04, 0.0)) == pytest.approx(0.05)  # r = 0.05
    assert ring.excess((0.3, 0.4, 0.0)) == pytest.approx(0.1)  # r = 0.5


def test_an_empty_workspace_is_unbounded():
    assert not Workspace().bounded
    assert Workspace().excess((1e3, -1e3, -1e3)) == 0.0
    assert TABLE.bounded


@pytest.mark.parametrize(
    "bounds",
    [
        {"z_min": 0.2, "z_max": 0.1},
        {"x_min": 0.1, "x_max": 0.1},
        {"r_min": -0.1},
        {"z_min": math.nan},
        {"z_min": "0.1"},
        {"z_min": True},
    ],
)
def test_workspace_rejects_invalid_bounds(bounds):
    with pytest.raises(ValueError, match="workspace"):
        Workspace(**bounds)


def test_workspace_from_dict_rejects_unknown_keys():
    assert Workspace.from_dict({"z_min": 0.02, "x_max": None}) == Workspace(z_min=0.02)
    with pytest.raises(ValueError, match="unknown"):
        Workspace.from_dict({"z_minimum": 0.02})


def test_tightened_takes_the_stricter_bound_of_each():
    base = Workspace(z_min=0.01, x_max=0.5)
    task = Workspace(z_min=0.05, x_min=0.05, x_max=0.8, r_max=0.45)
    assert base.tightened(task) == Workspace(z_min=0.05, x_min=0.05, x_max=0.5, r_max=0.45)
    assert base.tightened(Workspace(z_min=-1.0)) == base  # a task cannot widen it
    assert base.tightened(None) == base


def test_tightened_is_the_intersection():
    rng = random.Random(0)

    def random_workspace():
        bounds = {}
        for axis in "xyz":
            lower, upper = sorted(rng.uniform(-1, 1) for _ in range(2))
            bounds[f"{axis}_min"] = lower if rng.random() < 0.6 else None
            bounds[f"{axis}_max"] = upper if rng.random() < 0.6 else None
        return Workspace(**bounds)

    for _ in range(200):
        a, b = random_workspace(), random_workspace()
        try:
            both = a.tightened(b)
        except ValueError:
            continue  # they do not overlap
        for _ in range(20):
            p = [rng.uniform(-1.2, 1.2) for _ in range(3)]
            assert both.contains(p) == (a.contains(p) and b.contains(p))


def test_tightened_rejects_disjoint_workspaces():
    with pytest.raises(ValueError, match="z_min"):
        Workspace(z_max=0.1).tightened(Workspace(z_min=0.2))


def test_limits_default_to_an_unbounded_workspace():
    assert LIMITS.workspace == Workspace()


def test_limits_with_workspace_only_narrows():
    limits = WITH_TABLE.with_workspace(Workspace(z_min=0.05))
    assert limits.workspace == Workspace(z_min=0.05)
    assert WITH_TABLE.with_workspace(Workspace(z_min=-0.5)).workspace == TABLE
    assert limits.joint_limits == WITH_TABLE.joint_limits


def test_default_workspace_keeps_the_arm_off_the_table():
    assert SafetyLimits.defaults().workspace.z_min == pytest.approx(0.01)


def test_load_reads_the_workspace(tmp_path):
    path = tmp_path / "safety.json"
    path.write_text(json.dumps({"workspace": {"z_min": 0.03, "r_max": 0.4}}))
    workspace = SafetyLimits.load(path).workspace
    assert workspace == Workspace(z_min=0.03, r_max=0.4)


def test_load_adds_the_workspace_to_an_older_file(tmp_path):
    path = tmp_path / "safety.json"
    path.write_text(json.dumps({"joint_limits": {"elbow_flex": [-1.0, 1.0]}}))  # written before the workspace existed
    assert SafetyLimits.load(path).workspace == SafetyLimits.defaults().workspace


@pytest.mark.parametrize("workspace", [{"z_min": 0.2, "z_max": 0.1}, {"zmin": 0.1}, {"z_min": "low"}])
def test_load_rejects_an_invalid_workspace(tmp_path, workspace):
    path = tmp_path / "safety.json"
    path.write_text(json.dumps({"workspace": workspace}))
    with pytest.raises(ValueError, match="safety.json"):
        SafetyLimits.load(path)


def test_leaves_workspace():
    bent = HOME | {"elbow_flex": 1.2}  # the gripper is under the table
    assert lowest_point(bent) < 0.0
    assert not leaves_workspace(HOME, HOME | {"elbow_flex": 0.17}, TABLE)
    assert leaves_workspace(HOME, bent, TABLE)
    assert not leaves_workspace(bent, HOME, TABLE)  # back in
    assert not leaves_workspace(bent, bent | {"shoulder_pan": 0.3}, TABLE)  # no worse
    assert leaves_workspace(bent, bent | {"elbow_flex": 1.3}, TABLE)  # worse
    assert not leaves_workspace(HOME, bent, Workspace())  # nothing to leave
    keyed = {f"{joint}.pos": value for joint, value in bent.items()}
    assert leaves_workspace(HOME, keyed, TABLE)  # observation / action keys work too


def test_pushing_the_gripper_through_the_table_is_held(fake):
    with SafeFollower(fake, WITH_TABLE) as safe:
        for _ in range(20):
            safe.send_action({"elbow_flex.pos": 3.0})
        assert safe.last_held
        assert "elbow_flex.pos" in safe.last_clipped
        held = fake.position["elbow_flex"]
    assert 0.3 < held < 1.0
    assert lowest_point(fake.position) >= TABLE.z_min
    # one more step would cross the table
    assert lowest_point(fake.position | {"elbow_flex": held + STEP}) < TABLE.z_min


def test_a_held_action_keeps_every_joint_in_it(fake):
    with SafeFollower(fake, WITH_TABLE) as safe:
        for _ in range(20):
            safe.send_action({"elbow_flex.pos": 3.0})
        before = dict(fake.position)
        sent = safe.send_action({"elbow_flex.pos": 3.0, "shoulder_pan.pos": 0.5})
    assert sent == {"elbow_flex.pos": before["elbow_flex"], "shoulder_pan.pos": before["shoulder_pan"]}


def test_moving_away_from_the_table_passes(fake):
    with SafeFollower(fake, WITH_TABLE) as safe:
        for _ in range(20):
            safe.send_action({"elbow_flex.pos": 3.0})
        held = fake.position["elbow_flex"]
        sent = safe.send_action({"elbow_flex.pos": 0.0})
        assert not safe.last_held
    assert sent["elbow_flex.pos"] == pytest.approx(held - STEP)


def test_moving_along_the_table_at_a_safe_height_works(fake):
    with SafeFollower(fake, WITH_TABLE) as safe:
        for _ in range(20):
            safe.send_action({"elbow_flex.pos": 3.0})  # gripper just above the table
        for _ in range(5):
            sent = safe.send_action({"shoulder_pan.pos": 1.0})  # panning keeps the height
            assert not safe.last_held
    assert sent["shoulder_pan.pos"] == pytest.approx(5 * STEP)


def test_starting_under_the_table_allows_moves_that_do_not_make_it_worse():
    fake = FakeFollower(start={"elbow_flex": 1.2})
    with SafeFollower(fake, WITH_TABLE) as safe:
        safe.send_action({"elbow_flex.pos": 2.0})  # deeper: held
        assert safe.last_held
        assert fake.position["elbow_flex"] == pytest.approx(1.2)
        safe.send_action({"shoulder_pan.pos": 0.2})  # sideways, no deeper: passes
        assert not safe.last_held
        safe.send_action({"elbow_flex.pos": 0.0})  # back up: passes
        assert not safe.last_held
    assert fake.position["elbow_flex"] == pytest.approx(1.2 - STEP)


def test_a_task_workspace_narrows_the_one_from_the_limits(fake):
    task = Workspace(z_min=0.1)
    with SafeFollower(fake, WITH_TABLE, workspace=task) as safe:
        assert safe.limits.workspace == task
        for _ in range(20):
            safe.send_action({"elbow_flex.pos": 3.0})
    assert lowest_point(fake.position) >= 0.1
    loose = SafeFollower(FakeFollower(), WITH_TABLE, workspace=Workspace(z_min=-1.0))
    assert loose.limits.workspace == TABLE
