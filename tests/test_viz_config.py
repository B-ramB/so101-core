"""so101.viz.config: merging the user file over the defaults, and reloading it while running."""

import json
import os

import pytest

from so101.viz import config as viz_config
from so101.viz.config import VisualizationConfig, merge_defaults


def test_merge_fills_missing_keys():
    assert merge_defaults({"a": 1, "b": 2}, {"a": 10}) == {"a": 10, "b": 2}


def test_merge_is_recursive_and_the_user_value_wins():
    defaults = {"camera": {"fov": 60, "pose": {"x": 0, "y": 0}}, "fps": 30}
    user = {"camera": {"pose": {"y": 5}}, "extra": True}
    assert merge_defaults(defaults, user) == {
        "camera": {"fov": 60, "pose": {"x": 0, "y": 5}},
        "fps": 30,
        "extra": True,
    }


def test_merge_user_value_replaces_a_dict_default():
    assert merge_defaults({"a": {"b": 1}}, {"a": None}) == {"a": None}


def test_merge_does_not_change_the_defaults():
    defaults = {"a": {"b": 1}}
    merge_defaults(defaults, {"a": {"b": 2}})
    assert defaults == {"a": {"b": 1}}


class Clock:
    """A stand-in for time.monotonic that only moves when told to."""

    def __init__(self):
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(viz_config.time, "monotonic", clock)
    return clock


@pytest.fixture
def vis(tmp_path, clock):
    """A VisualizationConfig on a fresh copy of the defaults in tmp_path."""
    return VisualizationConfig(tmp_path / "visualization.json")


def write(vis: VisualizationConfig, data: dict | str, mtime: float) -> None:
    """Write the file and give it a distinct mtime (a fast test could otherwise write twice within one tick)."""
    vis.path.write_text(data if isinstance(data, str) else json.dumps(data))
    os.utime(vis.path, (mtime, mtime))


def test_first_use_copies_the_defaults(vis):
    assert json.loads(vis.path.read_text()) == vis.defaults
    assert vis.data == vis.defaults


def test_reloads_when_the_file_changes(vis, clock):
    write(vis, {"joints": {"gripper": {"offset": 12.5}}}, mtime=1)
    clock.now += 0.6
    assert vis.reload_if_changed()
    assert vis.data["joints"]["gripper"]["offset"] == 12.5
    # Missing keys still come from the defaults.
    assert vis.data["joints"]["gripper"]["invert"] == vis.defaults["joints"]["gripper"]["invert"]
    assert vis.camera == vis.defaults["wrist_camera"]


def test_no_reload_when_the_file_is_unchanged(vis, clock):
    clock.now += 0.6
    assert not vis.reload_if_changed()


def test_checks_at_most_twice_per_second(vis, clock):
    write(vis, {"joints": {"gripper": {"offset": 1.0}}}, mtime=1)
    clock.now += 0.4
    assert not vis.reload_if_changed()
    clock.now += 0.2
    assert vis.reload_if_changed()


def test_throttle_counts_from_the_last_check(vis, clock):
    clock.now += 0.6
    assert not vis.reload_if_changed()  # checked: unchanged
    write(vis, {"joints": {"gripper": {"offset": 1.0}}}, mtime=1)
    clock.now += 0.3
    assert not vis.reload_if_changed()  # within 0.5 s of that check
    clock.now += 0.3
    assert vis.reload_if_changed()


def test_invalid_json_keeps_the_previous_settings(vis, clock, capsys):
    write(vis, {"joints": {"gripper": {"offset": 7.0}}}, mtime=1)
    clock.now += 0.6
    assert vis.reload_if_changed()
    before = vis.data

    write(vis, "{ not json", mtime=2)
    clock.now += 0.6
    assert not vis.reload_if_changed()
    assert vis.data == before
    assert "not valid JSON" in capsys.readouterr().out

    # Once fixed, the next change is picked up again.
    write(vis, {"joints": {"gripper": {"offset": 8.0}}}, mtime=3)
    clock.now += 0.6
    assert vis.reload_if_changed()
    assert vis.data["joints"]["gripper"]["offset"] == 8.0


def test_a_deleted_file_keeps_the_previous_settings(vis, clock):
    before = vis.data
    vis.path.unlink()
    clock.now += 0.6
    assert not vis.reload_if_changed()
    assert vis.data == before
