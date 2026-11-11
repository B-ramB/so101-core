"""Reject or limit unsafe control inputs before they reach the arm, real or simulated.

`SafeFollower` wraps any `so101.robot.Follower` (`RealFollower`, `MujocoFollower`) and is a `Follower` itself. On each
`send_action` it validates the whole action, clamps every joint to its soft limits, limits the change per command, and
only then forwards it. Joint values are model radians, the so101 standard (see docs/units-and-joints.md).

    with SafeFollower(MujocoFollower(viewer=True)) as robot:
        robot.send_action({"elbow_flex.pos": 3.0})   # moves at most max_step, towards the soft limit
        robot.last_clipped                           # {"elbow_flex.pos": (3.0, 0.17)}

The limits come from so101_config/safety.json, created from the package defaults (so101/defaults/safety.json) on
first use. See docs/safety.md for what this does and does not protect against.

Plain Python: no lerobot or mujoco import, so all of it is unit-testable without hardware.
"""

import json
import logging
import math
import time
from dataclasses import dataclass
from numbers import Real
from pathlib import Path
from typing import Any, Self

from so101.conventions import JOINTS
from so101.paths import SAFETY_CONFIG_PATH, SAFETY_DEFAULTS_PATH, pretty
from so101.viz.config import ensure_user_config, merge_defaults

SUFFIX = ".pos"
WARN_INTERVAL_S = 1.0

log = logging.getLogger(__name__)


class UnsafeActionError(ValueError):
    """The action was rejected as a whole; nothing was sent to the robot."""


def _finite(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class SafetyLimits:
    """Soft joint limits and the largest change per command, both in radians, for every joint in `JOINTS`."""

    joint_limits: dict[str, tuple[float, float]]
    max_step: dict[str, float]

    def __post_init__(self):
        for name, block in (("joint_limits", self.joint_limits), ("max_step", self.max_step)):
            if set(block) != set(JOINTS):
                missing, unknown = set(JOINTS) - set(block), set(block) - set(JOINTS)
                raise ValueError(f"{name}: missing joints {sorted(missing)}, unknown joints {sorted(unknown)}")
        limits = {}
        for joint, pair in self.joint_limits.items():
            if not (isinstance(pair, (list, tuple)) and len(pair) == 2 and all(_finite(v) for v in pair)):
                raise ValueError(f"joint_limits.{joint}: expected [min, max] as two finite numbers, got {pair!r}")
            lower, upper = float(pair[0]), float(pair[1])
            if lower >= upper:
                raise ValueError(f"joint_limits.{joint}: min {lower} must be below max {upper}")
            limits[joint] = (lower, upper)
        steps = {}
        for joint, step in self.max_step.items():
            if not _finite(step) or step <= 0:
                raise ValueError(f"max_step.{joint}: expected a positive finite number, got {step!r}")
            steps[joint] = float(step)
        object.__setattr__(self, "joint_limits", limits)
        object.__setattr__(self, "max_step", steps)

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyLimits":
        return cls(dict(data["joint_limits"]), dict(data["max_step"]))

    @classmethod
    def defaults(cls) -> "SafetyLimits":
        """The package defaults (so101/defaults/safety.json). Writes nothing."""
        return cls.from_dict(json.loads(SAFETY_DEFAULTS_PATH.read_text()))

    @classmethod
    def load(cls, path: Path = SAFETY_CONFIG_PATH) -> "SafetyLimits":
        """Your limits file on top of the package defaults; the file is created from the defaults when missing.

        Keys missing from the file are filled from the defaults. Invalid limits raise a ValueError naming the file.
        """
        if ensure_user_config(path, SAFETY_DEFAULTS_PATH):
            print(f"Created {pretty(path)} from the package defaults.")
        defaults = json.loads(SAFETY_DEFAULTS_PATH.read_text())
        try:
            return cls.from_dict(merge_defaults(defaults, json.loads(path.read_text())))
        except (ValueError, TypeError) as e:
            raise ValueError(f"Invalid safety limits in {pretty(path)}: {e}") from e


class SafeFollower:
    """A `Follower` that validates, clamps and step-limits every action before forwarding it to `robot`.

    robot: any `so101.robot.Follower` (`RealFollower`, `MujocoFollower`), in model radians.
    limits: the soft limits; None loads so101_config/safety.json (created from the defaults when missing).

    `last_clipped` maps `"<joint>.pos"` -> (requested, sent) for the joints the last command changed.
    """

    def __init__(self, robot, limits: SafetyLimits | None = None):
        self.robot = robot
        self.limits = limits if limits is not None else SafetyLimits.load()
        self.last_clipped: dict[str, tuple[float, float]] = {}
        self._goal: dict[str, float] | None = None  # joint -> last target sent (radians)
        self._last_warning = -math.inf

    @property
    def observation_features(self) -> dict[str, Any]:
        return self.robot.observation_features

    @property
    def action_features(self) -> dict[str, Any]:
        return self.robot.action_features

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    def connect(self) -> None:
        """Connect the robot and take its current position as the first goal, so the first step starts from there."""
        self.robot.connect()
        obs = self.robot.get_observation()
        self._goal = {joint: float(obs[f"{joint}{SUFFIX}"]) for joint in JOINTS}
        self.last_clipped = {}

    def disconnect(self) -> None:
        self._goal = None
        self.robot.disconnect()

    def get_observation(self) -> dict[str, Any]:
        return self.robot.get_observation()

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        """Validate, clamp to the soft limits, limit the step, then forward. Returns what the robot actually sent.

        Raises UnsafeActionError (and sends nothing) for an unknown key or a value that is not a finite number.
        """
        if self._goal is None:
            raise RuntimeError("SafeFollower is not connected; call connect() first.")
        requested = self._validate(action)

        safe = {}
        for joint, value in requested.items():
            lower, upper = self.limits.joint_limits[joint]
            step = self.limits.max_step[joint]
            prev = self._goal[joint]
            clamped = min(max(value, lower), upper)
            safe[f"{joint}{SUFFIX}"] = prev + min(max(clamped - prev, -step), step)

        sent = self.robot.send_action(safe)

        self.last_clipped = {}
        for key, value in sent.items():
            joint = key.removesuffix(SUFFIX)
            if joint in self._goal:
                self._goal[joint] = float(value)
            if joint in requested and not math.isclose(value, requested[joint], abs_tol=1e-9):
                self.last_clipped[key] = (requested[joint], float(value))
        self._warn_if_clipped()
        return sent

    def _validate(self, action: dict) -> dict[str, float]:
        """The action as {joint: radians}, or UnsafeActionError for the first bad entry."""
        if not isinstance(action, dict):
            raise UnsafeActionError(f"Action must be a dict, got {type(action).__name__}.")
        requested = {}
        for key, value in action.items():
            joint = key.removesuffix(SUFFIX) if isinstance(key, str) else None
            if not isinstance(key, str) or not key.endswith(SUFFIX) or joint not in JOINTS:
                raise UnsafeActionError(f"Unknown action key {key!r}; expected one of {[j + SUFFIX for j in JOINTS]}.")
            if not _finite(value):
                raise UnsafeActionError(f"{key} must be a finite number, got {value!r}.")
            requested[joint] = float(value)
        return requested

    def _warn_if_clipped(self) -> None:
        if not self.last_clipped:
            return
        now = time.monotonic()
        if now - self._last_warning < WARN_INTERVAL_S:
            return
        self._last_warning = now
        details = ", ".join(f"{key} {req:.3f} -> {sent:.3f}" for key, (req, sent) in self.last_clipped.items())
        log.warning("SafeFollower limited the action (rad): %s", details)

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        if self.is_connected:
            self.disconnect()
