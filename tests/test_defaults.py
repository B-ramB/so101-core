import json
from importlib.resources import files

from so101.conventions import JOINTS


def load_default_visualization() -> dict:
    return json.loads(files("so101").joinpath("defaults/visualization.json").read_text())


def test_visualization_defaults_is_valid_json():
    assert isinstance(load_default_visualization(), dict)


def test_visualization_defaults_cover_every_joint():
    joints = load_default_visualization()["joints"]
    assert set(joints) == set(JOINTS)
    for name in JOINTS:
        assert isinstance(joints[name]["offset"], (int, float))
        assert isinstance(joints[name]["invert"], bool)
