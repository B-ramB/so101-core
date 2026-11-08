"""The arm configuration written by `so101-setup` (so101_config/arm_config.json in your project)."""

import json
import sys
from pathlib import Path

from so101.paths import CALIBRATION_DIR, CONFIG_PATH, ensure_home, pretty


def load_config() -> dict:
    """Load the arm config and check that the calibration files and ports exist. Exits with a hint if not."""
    if not CONFIG_PATH.exists():
        sys.exit(f"{pretty(CONFIG_PATH)} not found. Run `uv run so101-setup` first.")
    config = json.loads(CONFIG_PATH.read_text())

    for role in ("leader", "follower"):
        calib = CALIBRATION_DIR / f"{config[role]['id']}.json"
        if not calib.exists():
            sys.exit(f"Calibration file {pretty(calib)} is missing. Run `uv run so101-setup`.")
        port = config[role]["port"]
        if not Path(port).exists():
            sys.exit(
                f"The {role} port {port} does not exist (arm unplugged or port changed). "
                "Run `uv run so101-setup` to detect the ports again."
            )

    # Imported here so the config can be read without loading lerobot.
    from so101.hardware.ports import ensure_port_access

    for role in ("leader", "follower"):
        ensure_port_access(config[role]["port"])
    return config


def save_config(config: dict) -> None:
    ensure_home()
    CONFIG_PATH.write_text(json.dumps(config, indent=4) + "\n")
