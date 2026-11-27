"""so101.hardware.usb_config: reusing today's USB config, and detecting again when a device is gone."""

import json

import pytest

from so101.hardware import usb_config
from so101.hardware.usb_config import save_usb_config, setup_usb, usb_config_path

DAY = "2026-11-25"


@pytest.fixture
def devices(tmp_path, monkeypatch):
    """Two arm ports and a camera that exist (temp files); detection and permission fixes are recorded."""
    calls = {"detected": 0, "access": []}
    paths = {name: tmp_path / name for name in ("ttyLEADER", "ttyFOLLOWER", "video0")}
    for path in paths.values():
        path.touch()
    detected = {"leader_port": "/dev/ttyNEW0", "follower_port": "/dev/ttyNEW1", "camera": None}

    def detect_arm_ports():
        calls["detected"] += 1
        return detected["leader_port"], detected["follower_port"]

    monkeypatch.setattr(usb_config, "detect_arm_ports", detect_arm_ports)
    monkeypatch.setattr(usb_config, "detect_camera", lambda: detected["camera"])
    monkeypatch.setattr(usb_config, "ensure_port_access", calls["access"].append)
    calls["paths"] = paths
    calls["detected_usb"] = detected
    return calls


@pytest.fixture
def todays_config(devices):
    paths = devices["paths"]
    usb = {
        "leader_port": str(paths["ttyLEADER"]),
        "follower_port": str(paths["ttyFOLLOWER"]),
        "camera": {"index_or_path": str(paths["video0"]), "name": "Test camera"},
    }
    save_usb_config(DAY, usb)
    return usb


@pytest.fixture
def reuse(monkeypatch):
    """Answer the 'Use this USB config?' question."""

    def answer(yes: bool) -> None:
        monkeypatch.setattr(usb_config, "ask_yes_no", lambda question, default=None: yes)

    return answer


def test_save_writes_the_dated_file(todays_config):
    assert usb_config_path(DAY).name == f"usb_ports_{DAY}.json"
    assert json.loads(usb_config_path(DAY).read_text()) == todays_config


def test_todays_config_is_reused(todays_config, devices, reuse):
    reuse(True)
    assert setup_usb(DAY) == todays_config
    assert devices["detected"] == 0
    # Permissions are reset by a replug or reboot, so they are checked again.
    assert devices["access"] == [todays_config["leader_port"], todays_config["follower_port"]]


def test_no_config_for_today_runs_detection(devices, reuse):
    save_usb_config("2026-11-01", {"leader_port": "x", "follower_port": "y", "camera": None})
    assert setup_usb(DAY) == devices["detected_usb"]
    assert devices["detected"] == 1
    assert json.loads(usb_config_path(DAY).read_text()) == devices["detected_usb"]


def test_declining_todays_config_runs_detection(todays_config, devices, reuse):
    reuse(False)
    assert setup_usb(DAY) == devices["detected_usb"]
    assert devices["detected"] == 1


@pytest.mark.parametrize("gone", ["ttyLEADER", "ttyFOLLOWER", "video0"])
def test_a_device_that_disappeared_runs_detection(todays_config, devices, reuse, gone, capsys):
    devices["paths"][gone].unlink()
    reuse(True)
    assert setup_usb(DAY) == devices["detected_usb"]
    assert devices["detected"] == 1
    assert "not present anymore" in capsys.readouterr().out
    # The new detection result replaces today's file.
    assert json.loads(usb_config_path(DAY).read_text()) == devices["detected_usb"]


def test_a_config_without_camera_is_reused(devices, reuse):
    paths = devices["paths"]
    usb = {"leader_port": str(paths["ttyLEADER"]), "follower_port": str(paths["ttyFOLLOWER"]), "camera": None}
    save_usb_config(DAY, usb)
    reuse(True)
    assert setup_usb(DAY) == usb
    assert devices["detected"] == 0
