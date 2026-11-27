"""save_config / load_config: the round trip, and the exit message for each missing piece."""

import json
import tempfile
from pathlib import Path

import pytest

from so101.config import load_config, save_config
from so101.paths import CALIBRATION_DIR, CONFIG_PATH


@pytest.fixture
def checked_ports(monkeypatch):
    """Replace ensure_port_access (which may run sudo chmod) and record the ports it is asked about."""
    import so101.hardware.ports

    checked = []
    monkeypatch.setattr(so101.hardware.ports, "ensure_port_access", checked.append)
    return checked


@pytest.fixture
def arm_config(tmp_path, checked_ports):
    """A config whose ports exist (temp files) and whose calibration files exist. Not saved yet."""
    # conftest points SO101_HOME at a temp dir; never write over a real so101_config/.
    assert CONFIG_PATH.resolve().is_relative_to(Path(tempfile.gettempdir()).resolve()), CONFIG_PATH
    assert not CONFIG_PATH.exists()
    config = {}
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    for role in ("leader", "follower"):
        port = tmp_path / f"tty_{role}"
        port.touch()
        config[role] = {"type": f"so101_{role}", "port": str(port), "id": f"{role}_test"}
        (CALIBRATION_DIR / f"{role}_test.json").write_text("{}")
    config["camera"] = None
    return config


def test_round_trip(arm_config, checked_ports):
    save_config(arm_config)
    assert load_config() == arm_config
    assert checked_ports == [arm_config["leader"]["port"], arm_config["follower"]["port"]]


def test_save_writes_indented_json_and_the_gitignore(arm_config):
    save_config(arm_config)
    assert json.loads(CONFIG_PATH.read_text()) == arm_config
    assert CONFIG_PATH.read_text().endswith("}\n")
    assert (CONFIG_PATH.parent / ".gitignore").exists()


def test_missing_config(checked_ports):
    with pytest.raises(SystemExit, match="arm_config.json not found. Run `uv run so101-setup` first."):
        load_config()
    assert checked_ports == []


def test_missing_calibration(arm_config, checked_ports):
    save_config(arm_config)
    (CALIBRATION_DIR / "follower_test.json").unlink()
    with pytest.raises(SystemExit, match=r"Calibration file .*follower_test\.json is missing"):
        load_config()
    assert checked_ports == []


def test_missing_port(arm_config, checked_ports):
    Path(arm_config["follower"]["port"]).unlink()
    save_config(arm_config)
    with pytest.raises(SystemExit, match=r"The follower port .*tty_follower does not exist"):
        load_config()
    assert checked_ports == []


def test_leader_only_ignores_the_follower(arm_config, checked_ports):
    Path(arm_config["follower"]["port"]).unlink()
    (CALIBRATION_DIR / "follower_test.json").unlink()
    save_config(arm_config)
    assert load_config(roles=("leader",)) == arm_config
    assert checked_ports == [arm_config["leader"]["port"]]

