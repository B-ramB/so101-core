"""so101-setup: the cleanup of unused files, and the main flow with the hardware steps replaced."""

import json
from datetime import date

import pytest

from so101.cli import setup
from so101.config import load_config
from so101.paths import CALIBRATION_DIR, CONFIG_PATH, USB_CONFIG_DIR


@pytest.fixture
def replies(monkeypatch):
    """Answer ask_yes_no in so101.cli.setup from a queue. Returns the questions asked."""
    asked = []

    def script(*answers: bool) -> list[str]:
        queue = list(answers)

        def fake(question: str, default: bool | None = None) -> bool:
            asked.append(question)
            return queue.pop(0)

        monkeypatch.setattr(setup, "ask_yes_no", fake)
        return asked

    return script


@pytest.fixture
def files(tmp_path):
    paths = [tmp_path / name for name in ("a.json", "b.json", "c.json")]
    for path in paths:
        path.write_text("{}")
    return paths


def test_offer_delete_deletes_only_unused_files_after_yes(files, replies):
    asked = replies(True)
    setup.offer_delete("calibration files", files, in_use={files[1]})
    assert [p.exists() for p in files] == [False, True, False]
    assert asked == ["Delete these unused calibration files?"]


def test_offer_delete_keeps_everything_after_no(files, replies):
    replies(False)
    setup.offer_delete("calibration files", files, in_use={files[1]})
    assert all(p.exists() for p in files)


def test_offer_delete_does_not_ask_when_everything_is_in_use(files, replies, capsys):
    asked = replies()
    setup.offer_delete("USB configs", files, in_use=set(files))
    assert asked == []
    assert "No unused USB configs." in capsys.readouterr().out


def test_cleanup_keeps_the_calibration_and_usb_config_in_use(replies):
    CALIBRATION_DIR.mkdir(parents=True)
    USB_CONFIG_DIR.mkdir(parents=True)
    for name in ("leader_arm_new", "follower_arm_new", "leader_arm_old", "follower_arm_old"):
        (CALIBRATION_DIR / f"{name}.json").write_text("{}")
    for day in ("2026-11-01", "2026-11-25"):
        (USB_CONFIG_DIR / f"usb_ports_{day}.json").write_text("{}")

    replies(True, True)
    setup.cleanup_unused_files("leader_arm_new", "follower_arm_new", "2026-11-25")

    assert sorted(p.stem for p in CALIBRATION_DIR.iterdir()) == ["follower_arm_new", "leader_arm_new"]
    assert [p.name for p in USB_CONFIG_DIR.iterdir()] == ["usb_ports_2026-11-25.json"]


@pytest.fixture
def hardware(tmp_path, monkeypatch):
    """Replace every step of so101-setup that needs the arms, the camera or the network. Returns the calls made."""
    calls = {"calibrated": [], "fetched": 0, "rest_pose": []}
    ports = {}
    for role in ("leader", "follower"):
        ports[role] = tmp_path / f"tty_{role}"
        ports[role].touch()
    usb = {"leader_port": str(ports["leader"]), "follower_port": str(ports["follower"]), "camera": None}

    def run_calibration(role, port, device_id):
        calls["calibrated"].append((role, port, device_id))
        (CALIBRATION_DIR / f"{device_id}.json").write_text("{}")

    def fetch_urdfs():
        calls["fetched"] += 1

    monkeypatch.setattr(setup, "urdfs_present", lambda: True)
    monkeypatch.setattr(setup, "fetch_urdfs", fetch_urdfs)
    monkeypatch.setattr(setup, "setup_usb", lambda day: usb)
    monkeypatch.setattr(setup, "run_calibration", run_calibration)
    monkeypatch.setattr(setup, "setup_rest_pose", lambda config: calls["rest_pose"].append(config))
    monkeypatch.setattr("so101.hardware.ports.ensure_port_access", lambda port: None)
    calls["usb"] = usb
    return calls


def test_fresh_setup_calibrates_both_arms_and_saves_the_config(hardware, replies, capsys):
    asked = replies(False, True)  # no existing calibration files; record the rest pose; nothing to clean up afterwards
    setup.main()

    today = date.today().isoformat()
    leader_id, follower_id = f"leader_arm_{today}", f"follower_arm_{today}"
    usb = hardware["usb"]
    assert hardware["calibrated"] == [
        ("leader", usb["leader_port"], leader_id),
        ("follower", usb["follower_port"], follower_id),
    ]
    config = json.loads(CONFIG_PATH.read_text())
    assert config["leader"] == {"type": "so101_leader", "port": usb["leader_port"], "id": leader_id}
    assert config["follower"] == {"type": "so101_follower", "port": usb["follower_port"], "id": follower_id}
    assert config["camera"] is None
    assert load_config() == config  # what setup writes is what teleop accepts
    assert asked[1].startswith("\nRecord the follower's rest pose")
    assert hardware["rest_pose"] == [config]

    out = capsys.readouterr().out
    assert "    uv run so101-teleop\n" in out
    assert "    uv run --extra sim so101-teleop --sim\n" in out


def test_setup_with_existing_calibration_files_only_sets_up_usb(hardware, replies, monkeypatch):
    monkeypatch.setattr(setup, "select_existing_calibration", lambda role: f"{role}_arm_2026-11-08")
    for role in ("leader", "follower"):
        CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
        (CALIBRATION_DIR / f"{role}_arm_2026-11-08.json").write_text("{}")

    replies(True, False)  # the calibration files are already present; don't record a rest pose
    setup.main()

    assert hardware["calibrated"] == []
    assert hardware["rest_pose"] == []
    assert json.loads(CONFIG_PATH.read_text())["leader"]["id"] == "leader_arm_2026-11-08"


def test_setup_offers_to_download_missing_urdfs(hardware, replies, monkeypatch):
    monkeypatch.setattr(setup, "urdfs_present", lambda: False)
    asked = replies(True, False, False)  # download: yes; existing calibration files: no; rest pose: no
    setup.main()
    assert hardware["fetched"] == 1
    assert asked[0].startswith("No SO-101 URDFs")
