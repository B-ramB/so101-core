"""load_config: which roles' calibration files and ports it checks."""

import json
import tempfile
from pathlib import Path

import pytest

from so101.config import load_config
from so101.paths import CALIBRATION_DIR, CONFIG_PATH


@pytest.fixture
def arm_config(tmp_path):
    """An arm_config.json whose leader port exists (a temp file) and whose follower port does not."""
    # conftest points SO101_HOME at a temp dir; never write over a real so101_config/.
    assert CONFIG_PATH.resolve().is_relative_to(Path(tempfile.gettempdir()).resolve()), CONFIG_PATH
    assert not CONFIG_PATH.exists()
    leader_port = tmp_path / "ttyLEADER"
    leader_port.touch()
    config = {
        "leader": {"port": str(leader_port), "id": "leader_test"},
        "follower": {"port": str(tmp_path / "ttyMISSING"), "id": "follower_test"},
    }
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    for role in ("leader", "follower"):
        (CALIBRATION_DIR / f"{config[role]['id']}.json").write_text("{}")
    CONFIG_PATH.write_text(json.dumps(config))
    yield config
    CONFIG_PATH.unlink()


def test_leader_only_ignores_the_follower(arm_config):
    assert load_config(roles=("leader",)) == arm_config


def test_default_checks_the_follower_port(arm_config):
    with pytest.raises(SystemExit, match="follower port"):
        load_config()
