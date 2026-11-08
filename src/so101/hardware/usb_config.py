"""Dated USB configuration (arm ports + camera) in so101_config/usb_configs/usb_ports_<date>.json."""

import json
from pathlib import Path

from so101.hardware.cameras import detect_camera
from so101.hardware.ports import detect_arm_ports, ensure_port_access
from so101.paths import USB_CONFIG_DIR, ensure_home, pretty
from so101.prompts import ask_yes_no


def usb_config_path(day: str) -> Path:
    return USB_CONFIG_DIR / f"usb_ports_{day}.json"


def load_usb_config(day: str) -> dict | None:
    """Offer to reuse the USB config saved today. Returns it, or None to detect again."""
    path = usb_config_path(day)
    if not path.exists():
        return None

    usb = json.loads(path.read_text())
    camera = usb.get("camera")
    print(f"\nFound today's USB config ({pretty(path)}):")
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
    ensure_home()
    USB_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path = usb_config_path(day)
    path.write_text(json.dumps(usb, indent=4) + "\n")
    print(f"Saved USB config to {pretty(path)}")


def setup_usb(day: str) -> dict:
    usb = load_usb_config(day)
    if usb is None:
        leader_port, follower_port = detect_arm_ports()
        usb = {"leader_port": leader_port, "follower_port": follower_port, "camera": detect_camera()}
        save_usb_config(day, usb)
    return usb
