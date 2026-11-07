"""Interactive setup for the SO-101 leader and follower arms.

Steps:
  1. Ask whether calibration files already exist in setup/calibration_files.
  2. If setup/usb_configs/usb_ports_<today>.json exists, offer to reuse it and skip
     USB detection. Otherwise detect the USB port of each arm (unplug / replug, like
     `lerobot-find-port`) and the USB camera, and save them to that file.
     Port permissions are fixed (`sudo chmod 666 <port>`) when needed.
  3. If needed, calibrate both arms. Files are saved as
     setup/calibration_files/leader_arm_<date>.json and follower_arm_<date>.json.
  4. Save the result to setup/arm_config.json and print the teleoperate command.

Run with:
    uv run setup/setup_arms.py
"""

import json
import os
import shutil
import subprocess
import sys
import time
from datetime import date
from pathlib import Path

from lerobot.scripts.lerobot_find_port import find_available_ports

SETUP_DIR = Path(__file__).resolve().parent
CALIBRATION_DIR = SETUP_DIR / "calibration_files"
CONFIG_PATH = SETUP_DIR / "arm_config.json"
USB_CONFIG_DIR = SETUP_DIR / "usb_configs"
VIS_EXAMPLE_PATH = SETUP_DIR / "visualization.example.json"
VIS_CONFIG_PATH = SETUP_DIR / "visualization.local.json"

LEADER_TYPE = "so101_leader"
FOLLOWER_TYPE = "so101_follower"


# --------------------------------------------------------------------------- #
# Small input helpers
# --------------------------------------------------------------------------- #
def ask_yes_no(question: str, default: bool | None = None) -> bool:
    suffix = {True: " [Y/n] ", False: " [y/N] ", None: " [y/n] "}[default]
    while True:
        answer = input(question + suffix).strip().lower()
        if not answer and default is not None:
            return default
        if answer in ("y", "yes"):
            return True
        if answer in ("n", "no"):
            return False
        print("Please answer 'y' or 'n'.")


def ask_choice(question: str, options: list[str], default: int = 0, allow_skip: bool = False) -> int | None:
    """Let the user pick one option by number. Returns the index, or None when skipped."""
    for i, option in enumerate(options):
        marker = " (default)" if i == default else ""
        print(f"  [{i}] {option}{marker}")
    hint = f"Choose 0-{len(options) - 1}" + (", 's' to skip" if allow_skip else "")
    while True:
        answer = input(f"{question} ({hint}): ").strip().lower()
        if not answer:
            return default
        if allow_skip and answer == "s":
            return None
        if answer.isdigit() and int(answer) < len(options):
            return int(answer)
        print("Invalid choice.")


def header(title: str) -> None:
    print("\n" + "=" * 60 + f"\n {title}\n" + "=" * 60)


# --------------------------------------------------------------------------- #
# USB port detection
# --------------------------------------------------------------------------- #
def serial_ports() -> set[str]:
    return set(find_available_ports())


def wait_for_port_change(baseline: set[str], expect: str, timeout: float = 10.0) -> list[str]:
    """Poll until a port disappears from / appears in `baseline` ('removed' or 'added')."""
    deadline = time.monotonic() + timeout
    while True:
        current = serial_ports()
        diff = baseline - current if expect == "removed" else current - baseline
        if diff or time.monotonic() > deadline:
            return sorted(diff)
        time.sleep(0.2)


def detect_port(role: str) -> str:
    """Detect the port of one arm by unplugging and replugging its USB cable."""
    while True:
        before = serial_ports()
        input(f"\nUnplug the USB cable of the {role.upper()} arm and press ENTER...")
        removed = wait_for_port_change(before, "removed", timeout=5.0)
        if len(removed) != 1:
            print(f"Could not detect the port (ports removed: {removed or 'none'}). Plug it back in and try again.")
            input("Press ENTER once the cable is plugged back in...")
            continue

        without = serial_ports()
        input(f"Detected {removed[0]}. Plug the {role.upper()} arm back in and press ENTER...")
        added = wait_for_port_change(without, "added", timeout=10.0)
        if len(added) != 1:
            print(f"Could not detect the port after replugging (ports added: {added or 'none'}). Try again.")
            continue

        port = added[0]
        print(f"The {role} arm is on {port}")
        return port


def ensure_port_access(port: str) -> None:
    """Run `sudo chmod 666 <port>` when the current user cannot read/write the port."""
    if os.access(port, os.R_OK | os.W_OK):
        print(f"Access to {port}: OK")
        return
    print(f"No read/write access to {port}, running: sudo chmod 666 {port}")
    subprocess.run(["sudo", "chmod", "666", port], check=True)
    if not os.access(port, os.R_OK | os.W_OK):
        raise PermissionError(f"Still no access to {port} after chmod.")


def detect_arm_ports() -> tuple[str, str]:
    header("Detecting USB ports")
    input("Make sure BOTH arms are powered and connected over USB, then press ENTER...")

    leader_port = detect_port("leader")
    follower_port = detect_port("follower")
    if leader_port == follower_port:
        raise RuntimeError(f"Leader and follower were detected on the same port ({leader_port}).")

    for port in (leader_port, follower_port):
        ensure_port_access(port)
    return leader_port, follower_port


# --------------------------------------------------------------------------- #
# Camera detection
# --------------------------------------------------------------------------- #
def video_device_name(path: str) -> str:
    name_file = Path("/sys/class/video4linux") / Path(path).name / "name"
    try:
        return name_file.read_text().strip()
    except OSError:
        return "unknown"


def detect_camera() -> dict | None:
    header("Detecting cameras")
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    cameras = OpenCVCamera.find_cameras()
    if not cameras:
        print("No camera found. Skipping camera setup.")
        return None

    options = []
    for cam in cameras:
        profile = cam.get("default_stream_profile", {})
        options.append(
            f"{cam['id']}  ({video_device_name(str(cam['id']))})  "
            f"{profile.get('width')}x{profile.get('height')} @ {profile.get('fps')} fps"
        )
    print("Found cameras (a built-in laptop webcam may also be listed):")
    index = ask_choice("Which camera should be used?", options, allow_skip=True)
    if index is None:
        return None

    cam = cameras[index]
    profile = cam.get("default_stream_profile", {})
    return {
        "type": "opencv",
        "index_or_path": str(cam["id"]),
        "name": video_device_name(str(cam["id"])),
        "width": int(profile.get("width") or 640),
        "height": int(profile.get("height") or 480),
        "fps": round(profile.get("fps") or 30),
    }


# --------------------------------------------------------------------------- #
# Dated USB configuration (ports + camera)
# --------------------------------------------------------------------------- #
def usb_config_path(day: str) -> Path:
    return USB_CONFIG_DIR / f"usb_ports_{day}.json"


def load_usb_config(day: str) -> dict | None:
    """Offer to reuse the USB config saved today. Returns it, or None to detect again."""
    path = usb_config_path(day)
    if not path.exists():
        return None

    usb = json.loads(path.read_text())
    camera = usb.get("camera")
    print(f"\nFound today's USB config ({path.relative_to(SETUP_DIR.parent)}):")
    print(f"  leader   : {usb['leader_port']}")
    print(f"  follower : {usb['follower_port']}")
    print(f"  camera   : {camera['index_or_path'] + ' (' + camera['name'] + ')' if camera else 'none'}")
    if not ask_yes_no("Use this USB config and skip USB detection?", default=True):
        return None

    missing = [p for p in (usb["leader_port"], usb["follower_port"]) if not Path(p).exists()]
    if camera and not Path(camera["index_or_path"]).exists():
        missing.append(camera["index_or_path"])
    if missing:
        print(f"These devices are not present anymore: {missing}. Running USB detection instead.")
        return None

    # Permissions are reset when a device is replugged or the PC reboots.
    for port in (usb["leader_port"], usb["follower_port"]):
        ensure_port_access(port)
    return usb


def save_usb_config(day: str, usb: dict) -> None:
    USB_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path = usb_config_path(day)
    path.write_text(json.dumps(usb, indent=4) + "\n")
    print(f"Saved USB config to {path.relative_to(SETUP_DIR.parent)}")


def setup_usb(day: str) -> dict:
    usb = load_usb_config(day)
    if usb is None:
        leader_port, follower_port = detect_arm_ports()
        usb = {"leader_port": leader_port, "follower_port": follower_port, "camera": detect_camera()}
        save_usb_config(day, usb)
    return usb


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #
def list_calibration_files(prefix: str) -> list[Path]:
    files = CALIBRATION_DIR.glob(f"{prefix}_*.json")
    return sorted(files, key=lambda p: p.stat().st_mtime, reverse=True)


def select_existing_calibration(role: str) -> str | None:
    """Return the id (filename stem) of an existing calibration file chosen by the user."""
    files = list_calibration_files(f"{role}_arm")
    if not files:
        print(f"No {role} calibration files found in {CALIBRATION_DIR}.")
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
        raise RuntimeError(f"Calibration finished but {target} was not created.")
    print(f"Saved {target}")


# --------------------------------------------------------------------------- #
# Output
# --------------------------------------------------------------------------- #
def teleoperate_command(config: dict) -> str:
    calib_dir = os.path.relpath(CALIBRATION_DIR, Path.cwd())
    lines = [
        "uv run lerobot-teleoperate",
        f"--robot.type={FOLLOWER_TYPE}",
        f"--robot.port={config['follower']['port']}",
        f"--robot.id={config['follower']['id']}",
        f"--robot.calibration_dir={calib_dir}",
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
        f"--teleop.calibration_dir={calib_dir}",
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
        print(f"  {path.relative_to(SETUP_DIR.parent)}")
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
    CALIBRATION_DIR.mkdir(parents=True, exist_ok=True)

    header("SO-101 arm setup")
    has_files = ask_yes_no(
        f"Are the calibration files already present in {CALIBRATION_DIR.relative_to(SETUP_DIR.parent)}? "
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
        "calibration_dir": str(CALIBRATION_DIR.relative_to(SETUP_DIR.parent)),
        "camera": camera,
    }
    CONFIG_PATH.write_text(json.dumps(config, indent=4) + "\n")

    header("Done")
    print(f"Configuration saved to {CONFIG_PATH.relative_to(SETUP_DIR.parent)}:\n")
    print(json.dumps(config, indent=4))

    cleanup_unused_files(leader_id, follower_id, today)

    if not VIS_CONFIG_PATH.exists() and VIS_EXAMPLE_PATH.exists():
        shutil.copyfile(VIS_EXAMPLE_PATH, VIS_CONFIG_PATH)
        print(f"\nCreated {VIS_CONFIG_PATH.relative_to(SETUP_DIR.parent)} from {VIS_EXAMPLE_PATH.name}.")

    print("\nStart teleoperation with:\n")
    print(teleoperate_command(config))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(1)
    except subprocess.CalledProcessError as e:
        print(f"\nCommand failed (exit code {e.returncode}): {' '.join(map(str, e.cmd))}")
        sys.exit(1)
