"""so101.hardware.calibration: listing calibration files, and the lerobot command run_calibration builds."""

import os
import subprocess
import sys

import pytest

from so101.hardware import calibration
from so101.hardware.calibration import list_calibration_files, run_calibration, select_existing_calibration
from so101.paths import CALIBRATION_DIR


def make_files(*names_and_mtimes: tuple[str, float]) -> None:
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    for name, mtime in names_and_mtimes:
        path = CALIBRATION_DIR / name
        path.write_text("{}")
        os.utime(path, (mtime, mtime))


def test_list_returns_newest_first():
    make_files(("leader_arm_a.json", 100), ("leader_arm_b.json", 300), ("leader_arm_c.json", 200))
    assert [p.name for p in list_calibration_files("leader_arm")] == [
        "leader_arm_b.json",
        "leader_arm_c.json",
        "leader_arm_a.json",
    ]


def test_list_only_returns_the_prefix():
    make_files(("leader_arm_a.json", 1), ("follower_arm_a.json", 2), ("leader_arm_a.txt", 3))
    assert [p.name for p in list_calibration_files("leader_arm")] == ["leader_arm_a.json"]


def test_list_without_folder_is_empty():
    assert list_calibration_files("leader_arm") == []


def test_select_returns_the_chosen_stem(monkeypatch):
    make_files(("follower_arm_old.json", 1), ("follower_arm_new.json", 2))
    monkeypatch.setattr(calibration, "ask_choice", lambda question, options: options.index("follower_arm_old.json"))
    assert select_existing_calibration("follower") == "follower_arm_old"


def test_select_without_files_returns_none():
    assert select_existing_calibration("leader") is None


@pytest.fixture
def lerobot(monkeypatch):
    """Replace subprocess.run. Each call pops a return code; on success the calibration file is created."""
    calls = []
    return_codes = []

    def fake_run(cmd, check=False):
        calls.append(cmd)
        code = return_codes.pop(0) if return_codes else 0
        if code == 0:
            device_id = next(arg for arg in cmd if ".id=" in arg).split("=", 1)[1]
            (CALIBRATION_DIR / f"{device_id}.json").write_text("{}")
        return subprocess.CompletedProcess(cmd, code)

    monkeypatch.setattr(calibration.subprocess, "run", fake_run)
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)
    return calls, return_codes


@pytest.fixture
def yes_no(monkeypatch):
    def answer(*replies: bool) -> list[str]:
        asked, queue = [], list(replies)

        def fake(question, default=None):
            asked.append(question)
            return queue.pop(0)

        monkeypatch.setattr(calibration, "ask_yes_no", fake)
        return asked

    return answer


@pytest.mark.parametrize(
    "role, kind, device_type",
    [("leader", "teleop", "so101_leader"), ("follower", "robot", "so101_follower")],
)
def test_run_calibration_builds_the_lerobot_command(lerobot, role, kind, device_type):
    calls, _ = lerobot
    run_calibration(role, "/dev/ttyACM0", f"{role}_arm_2026-11-25")
    assert calls == [
        [
            sys.executable, "-m", "lerobot.scripts.lerobot_calibrate",
            f"--{kind}.type={device_type}",
            f"--{kind}.port=/dev/ttyACM0",
            f"--{kind}.id={role}_arm_2026-11-25",
            f"--{kind}.calibration_dir={CALIBRATION_DIR}",
        ]
    ]
    assert (CALIBRATION_DIR / f"{role}_arm_2026-11-25.json").exists()


def test_existing_file_is_kept_unless_confirmed(lerobot, yes_no):
    calls, _ = lerobot
    make_files(("leader_arm_x.json", 1))
    yes_no(False)
    run_calibration("leader", "/dev/ttyACM0", "leader_arm_x")
    assert calls == []
    assert (CALIBRATION_DIR / "leader_arm_x.json").exists()


def test_existing_file_is_replaced_when_confirmed(lerobot, yes_no):
    calls, _ = lerobot
    make_files(("leader_arm_x.json", 1))
    yes_no(True)
    run_calibration("leader", "/dev/ttyACM0", "leader_arm_x")
    assert len(calls) == 1


def test_failure_retries_until_it_succeeds(lerobot, yes_no, capsys):
    calls, return_codes = lerobot
    return_codes.extend([1, 1, 0])
    asked = yes_no(True, True)
    run_calibration("follower", "/dev/ttyACM1", "follower_arm_x")
    assert len(calls) == 3
    assert asked == ["Fix the issue and retry?"] * 2
    assert "no status packet" in capsys.readouterr().out


def test_failure_without_retry_raises(lerobot, yes_no):
    _, return_codes = lerobot
    return_codes.append(1)
    yes_no(False)
    with pytest.raises(subprocess.CalledProcessError):
        run_calibration("follower", "/dev/ttyACM1", "follower_arm_x")


def test_success_without_a_file_raises(monkeypatch):
    monkeypatch.setattr(calibration.subprocess, "run", lambda cmd, check=False: subprocess.CompletedProcess(cmd, 0))
    with pytest.raises(RuntimeError, match="was not created"):
        run_calibration("leader", "/dev/ttyACM0", "leader_arm_x")
