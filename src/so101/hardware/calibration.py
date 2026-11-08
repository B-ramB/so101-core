"""Calibration files in so101_config/calibration: choosing existing ones and running lerobot's calibration."""

import subprocess
import sys
from pathlib import Path

from so101.conventions import FOLLOWER_TYPE, LEADER_TYPE
from so101.paths import CALIBRATION_DIR, pretty
from so101.prompts import ask_choice, ask_yes_no, header


def list_calibration_files(prefix: str) -> list[Path]:
    files = CALIBRATION_DIR.glob(f"{prefix}_*.json")
    return sorted(files, key=lambda p: p.stat().st_mtime, reverse=True)


def select_existing_calibration(role: str) -> str | None:
    """Return the id (filename stem) of an existing calibration file chosen by the user."""
    files = list_calibration_files(f"{role}_arm")
    if not files:
        print(f"No {role} calibration files found in {pretty(CALIBRATION_DIR)}.")
        return None
    print(f"\nExisting {role} calibration files (newest first):")
    index = ask_choice(f"Which {role} calibration should be used?", [f.name for f in files])
    return files[index].stem


def run_calibration(role: str, port: str, device_id: str) -> None:
    kind, device_type = ("teleop", LEADER_TYPE) if role == "leader" else ("robot", FOLLOWER_TYPE)
    target = CALIBRATION_DIR / f"{device_id}.json"

    if target.exists():
        if not ask_yes_no(f"{target.name} already exists. Recalibrate and overwrite it?", default=False):
            print(f"Keeping existing {target.name}.")
            return
        target.unlink()

    header(f"Calibrating {role} arm ({device_id} on {port})")
    cmd = [
        sys.executable, "-m", "lerobot.scripts.lerobot_calibrate",
        f"--{kind}.type={device_type}",
        f"--{kind}.port={port}",
        f"--{kind}.id={device_id}",
        f"--{kind}.calibration_dir={CALIBRATION_DIR}",
    ]
    while subprocess.run(cmd, check=False).returncode != 0:
        print(
            f"\nCalibration of the {role} arm failed. Common causes for 'There is no status packet':\n"
            "  - the arm's power supply is not connected / switched on\n"
            "  - a loose servo cable (the motor id in the error is the first one that did not answer;\n"
            "    check the cable between that motor and the previous one)\n"
            "  - a motor with a wrong id (fix with `lerobot-setup-motors`)"
        )
        if not ask_yes_no("Fix the issue and retry?", default=True):
            raise subprocess.CalledProcessError(1, cmd)

    if not target.exists():
        raise RuntimeError(f"Calibration finished but {pretty(target)} was not created.")
    print(f"Saved {pretty(target)}")
