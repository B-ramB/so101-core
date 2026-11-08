"""Interactive setup for the SO-101 leader and follower arms.

Steps:
  0. Offer to download the SO-101 URDFs into so101_config/urdf when they are missing.
  1. Ask whether calibration files already exist in so101_config/calibration.
  2. If so101_config/usb_configs/usb_ports_<today>.json exists, offer to reuse it and skip
     USB detection. Otherwise detect the USB port of each arm (unplug / replug, like
     `lerobot-find-port`) and the USB camera, and save them to that file.
     Port permissions are fixed (`sudo chmod 666 <port>`) when needed.
  3. If needed, calibrate both arms. Files are saved as
     so101_config/calibration/leader_arm_<date>.json and follower_arm_<date>.json.
  4. Save the result to so101_config/arm_config.json and print the teleoperate command.

so101_config/ is created at the root of the project you run this in (the nearest folder with a pyproject.toml).
The folder can be changed with the SO101_HOME environment variable.

Run with:
    uv run so101-setup
"""

import json
import subprocess
import sys
from datetime import date
from pathlib import Path

from so101.config import save_config
from so101.conventions import FOLLOWER_TYPE, LEADER_TYPE
from so101.hardware.calibration import run_calibration, select_existing_calibration
from so101.hardware.usb_config import setup_usb, usb_config_path
from so101.paths import (
    CALIBRATION_DIR,
    CONFIG_PATH,
    URDF_DIR,
    USB_CONFIG_DIR,
    VIS_CONFIG_PATH,
    ensure_home,
    pretty,
)
from so101.prompts import ask_yes_no, header
from so101.urdf import fetch_urdfs, urdfs_present
from so101.viz.config import ensure_user_config


def teleoperate_command(config: dict) -> str:
    lines = [
        "uv run lerobot-teleoperate",
        f"--robot.type={FOLLOWER_TYPE}",
        f"--robot.port={config['follower']['port']}",
        f"--robot.id={config['follower']['id']}",
        f"--robot.calibration_dir={CALIBRATION_DIR}",
    ]
    cam = config.get("camera")
    if cam:
        lines.append(
            f'--robot.cameras="{{ front: {{type: opencv, index_or_path: {cam["index_or_path"]}, '
            f'width: {cam["width"]}, height: {cam["height"]}, fps: {cam["fps"]}}}}}"'
        )
    lines += [
        f"--teleop.type={LEADER_TYPE}",
        f"--teleop.port={config['leader']['port']}",
        f"--teleop.id={config['leader']['id']}",
        f"--teleop.calibration_dir={CALIBRATION_DIR}",
    ]
    if cam:
        lines.append("--display_data=true")
    return " \\\n    ".join(lines)


def offer_delete(label: str, candidates: list[Path], in_use: set[Path]) -> None:
    """List the unused files among `candidates` and delete them after confirmation."""
    unused = sorted(p for p in candidates if p not in in_use)
    if not unused:
        print(f"No unused {label}.")
        return

    print(f"\nUnused {label}:")
    for path in unused:
        print(f"  {pretty(path)}")
    if not ask_yes_no(f"Delete these unused {label}?", default=False):
        return
    for path in unused:
        path.unlink()
    print(f"Deleted {len(unused)} file(s).")


def cleanup_unused_files(leader_id: str, follower_id: str, today: str) -> None:
    """Offer to delete calibration files and USB configs that are not in use anymore."""
    header("Cleanup")
    offer_delete(
        "calibration files",
        list(CALIBRATION_DIR.glob("*.json")),
        {CALIBRATION_DIR / f"{leader_id}.json", CALIBRATION_DIR / f"{follower_id}.json"},
    )
    offer_delete(
        "USB configs",
        list(USB_CONFIG_DIR.glob("usb_ports_*.json")),
        {usb_config_path(today)},
    )


def main() -> None:
    ensure_home()
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)

    header("SO-101 arm setup")
    print(f"Project config folder: {pretty(CALIBRATION_DIR.parent)}")
    if not urdfs_present():
        if ask_yes_no(f"No SO-101 URDFs in {pretty(URDF_DIR)}. Download them now?", default=True):
            fetch_urdfs()
        else:
            print("Skipping. Get them later with `uv run so101-fetch-urdfs`.")

    has_files = ask_yes_no(
        f"Are the calibration files already present in {pretty(CALIBRATION_DIR)}? "
        "(yes = only set up USB connections)",
        default=False,
    )

    leader_id = follower_id = None
    if has_files:
        leader_id = select_existing_calibration("leader")
        follower_id = select_existing_calibration("follower")
        missing = leader_id is None or follower_id is None
        if missing and not ask_yes_no("Missing calibration file(s). Calibrate the missing arm(s)?", default=True):
            sys.exit(1)

    today = date.today().isoformat()
    usb = setup_usb(today)
    leader_port, follower_port, camera = usb["leader_port"], usb["follower_port"], usb["camera"]

    if leader_id is None:
        leader_id = f"leader_arm_{today}"
        run_calibration("leader", leader_port, leader_id)
    if follower_id is None:
        follower_id = f"follower_arm_{today}"
        run_calibration("follower", follower_port, follower_id)

    config = {
        "leader": {"type": LEADER_TYPE, "port": leader_port, "id": leader_id},
        "follower": {"type": FOLLOWER_TYPE, "port": follower_port, "id": follower_id},
        "camera": camera,
    }
    save_config(config)

    header("Done")
    print(f"Configuration saved to {pretty(CONFIG_PATH)}:\n")
    print(json.dumps(config, indent=4))

    cleanup_unused_files(leader_id, follower_id, today)

    if ensure_user_config():
        print(f"\nCreated {pretty(VIS_CONFIG_PATH)} from the package defaults.")

    print("\nStart teleoperation with:\n")
    print(teleoperate_command(config))


def run() -> None:
    """Entry point of `so101-setup`."""
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"\nCommand failed (exit code {e.returncode}): {' '.join(map(str, e.cmd))}")
        sys.exit(1)


if __name__ == "__main__":
    run()
