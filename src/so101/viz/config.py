"""Visualization settings: so101_config/visualization.json in your project, on top of the package defaults.

The defaults ship with the package (so101/defaults/visualization.json) and are the single source of default
values. Your own file is copied from them on first use, and keys missing from it are filled in from them.
"""

import copy
import json
import shutil
import time
from pathlib import Path

from so101.conventions import GRIPPER_LOWER, GRIPPER_UPPER, JointCalibration
from so101.paths import SO101_HOME, VIS_CONFIG_PATH, VIS_DEFAULTS_PATH, ensure_home, pretty


def merge_defaults(defaults: dict, user: dict) -> dict:
    """Fill keys missing from the user config with defaults (recursively)."""
    merged = copy.deepcopy(defaults)
    for key, value in user.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_defaults(merged[key], value)
        else:
            merged[key] = value
    return merged


def ensure_user_config(path: Path = VIS_CONFIG_PATH, defaults_path: Path = VIS_DEFAULTS_PATH) -> bool:
    """Copy the package defaults `defaults_path` to `path` when it does not exist yet. Returns True when created."""
    if path.exists():
        return False
    if path.parent == SO101_HOME:
        ensure_home()
    path.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(defaults_path, path)
    return True


class VisualizationConfig:
    """so101_config/visualization.json, reloaded whenever the file changes."""

    def __init__(self, path: Path = VIS_CONFIG_PATH):
        self.path = path
        if ensure_user_config(path):
            print(f"Created {pretty(path)} from the package defaults.")
        self.defaults = json.loads(VIS_DEFAULTS_PATH.read_text())
        self._mtime = 0.0
        self._last_check = 0.0
        self.data = copy.deepcopy(self.defaults)
        self.reload_if_changed(force=True)

    def reload_if_changed(self, force: bool = False) -> bool:
        """Returns True when the config was (re)loaded. Checks the file at most twice per second."""
        now = time.monotonic()
        if not force and now - self._last_check < 0.5:
            return False
        self._last_check = now
        try:
            mtime = self.path.stat().st_mtime
        except OSError:
            return False
        if not force and mtime == self._mtime:
            return False
        self._mtime = mtime
        try:
            self.data = merge_defaults(self.defaults, json.loads(self.path.read_text()))
        except json.JSONDecodeError as e:
            print(f"\n{self.path.name} is not valid JSON ({e}); keeping the previous settings.")
            return False
        if not force:
            print(f"\nReloaded {self.path.name}")
        return True

    @property
    def camera(self) -> dict:
        return self.data["wrist_camera"]

    def joint_calibration(
        self, gripper_limits: tuple[float, float] = (GRIPPER_LOWER, GRIPPER_UPPER)
    ) -> JointCalibration:
        """The real-arm -> model mapping from the `joints` block, as currently loaded (so it follows reloads)."""
        return JointCalibration(self.data["joints"], gripper_limits)
