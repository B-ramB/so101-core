"""Where so101 keeps its files.

Everything robot-specific lives in `so101_config/` at the root of the project you run the command in: the nearest
folder, starting at the current directory, that contains a pyproject.toml (or .git). Each repository that installs
this package gets its own `so101_config/`, so it is clear which URDFs and calibration it uses:

    so101_config/
        urdf/                URDFs + meshes, fetched by `so101-fetch-urdfs`
        calibration/         lerobot calibration files
        visualization.json   camera pose and joint offsets
        arm_config.json      ports + which calibration is in use
        usb_configs/         dated port / camera detection results
        mujoco/              MuJoCo model generated from the follower URDF (`so101.sim.model`)

None of it is committed: the folder contains a .gitignore that ignores everything in it (itself included).

Override the folder with the SO101_HOME environment variable.
"""

import os
from pathlib import Path

CONFIG_DIR_NAME = "so101_config"


def find_project_root(start: Path | None = None) -> Path:
    """The nearest folder (start or a parent) with a pyproject.toml or .git. Falls back to `start`."""
    start = (start or Path.cwd()).resolve()
    for folder in (start, *start.parents):
        if (folder / "pyproject.toml").exists() or (folder / ".git").exists():
            return folder
    return start


def _env_dir(var: str, fallback: Path) -> Path:
    return Path(os.environ.get(var) or fallback).expanduser()


SO101_HOME = _env_dir("SO101_HOME", find_project_root() / CONFIG_DIR_NAME)

# Machine state, written by `so101-setup`
CONFIG_PATH = SO101_HOME / "arm_config.json"
CALIBRATION_DIR = SO101_HOME / "calibration"
USB_CONFIG_DIR = SO101_HOME / "usb_configs"
VIS_CONFIG_PATH = SO101_HOME / "visualization.json"

# Shipped with the package
PACKAGE_DIR = Path(__file__).resolve().parent
VIS_DEFAULTS_PATH = PACKAGE_DIR / "defaults" / "visualization.json"

# Downloaded by `so101-fetch-urdfs`
URDF_DIR = SO101_HOME / "urdf"
FOLLOWER_URDF = URDF_DIR / "so101_new_calib.urdf"
LEADER_URDF = URDF_DIR / "so101_leader_new_calib.urdf"

# Generated from the follower URDF by `so101.sim.model.ensure_model()`
MUJOCO_DIR = SO101_HOME / "mujoco"

GITIGNORE = """\
# Created by so101: local robot files (URDFs, calibration, ports, visualization) are not committed.
*
"""


def ensure_home() -> Path:
    """Create the config folder, with a .gitignore that keeps everything in it out of git."""
    SO101_HOME.mkdir(parents=True, exist_ok=True)
    gitignore = SO101_HOME / ".gitignore"
    if not gitignore.exists():
        gitignore.write_text(GITIGNORE)
    return SO101_HOME


def pretty(path: Path | str) -> str:
    """A path for messages: relative to the current directory when inside it, otherwise with ~ for home."""
    path = Path(path)
    for base, prefix in ((Path.cwd(), ""), (Path.home(), "~/")):
        if path.is_relative_to(base):
            return f"{prefix}{path.relative_to(base)}"
    return str(path)
