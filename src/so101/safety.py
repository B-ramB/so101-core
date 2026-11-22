"""Reject or limit unsafe control inputs before they reach the arm, real or simulated.

`SafeFollower` wraps any `so101.robot.Follower` (`RealFollower`, `MujocoFollower`) and is a `Follower` itself. On each
`send_action` it validates the whole action, clamps every joint to its soft limits, limits the change per command, holds
the arm when the move would take it out of the Cartesian workspace (e.g. into the table), and only then forwards it.
Joint values are model radians, the so101 standard (see docs/units-and-joints.md).

    with SafeFollower(MujocoFollower(viewer=True)) as robot:
        robot.send_action({"elbow_flex.pos": 3.0})   # moves at most max_step, towards the soft limit
        robot.last_clipped                           # {"elbow_flex.pos": (3.0, 0.17)}

A task can narrow the workspace for itself, never widen it: `SafeFollower(robot, workspace=Workspace(z_min=0.05))`.

The limits come from so101_config/safety.json, created from the package defaults (so101/defaults/safety.json) on
first use. See docs/safety.md for what this does and does not protect against.

Plain Python and numpy (the workspace check uses `so101.kinematics`): no lerobot or mujoco import, so all of it is
unit-testable without hardware.
"""

import json
import logging
import math
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, fields, replace
from numbers import Real
from pathlib import Path
from typing import Any, Self

from so101.conventions import JOINTS
from so101.kinematics import POINTS, ArmPoints
from so101.paths import SAFETY_CONFIG_PATH, SAFETY_DEFAULTS_PATH, pretty
from so101.viz.config import ensure_user_config, merge_defaults

SUFFIX = ".pos"
WARN_INTERVAL_S = 1.0
# m: how much "further outside the workspace" still counts as no worse. The URDF's rounded angles (3.14159 for pi) tilt
# the axes a little, so e.g. panning under the table changes the excess by ~0.2 µm.
WORKSPACE_TOLERANCE = 1e-6

log = logging.getLogger(__name__)


class UnsafeActionError(ValueError):
    """The action was rejected as a whole; nothing was sent to the robot."""


def _finite(value: Any) -> bool:
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class Workspace:
    """Where every checked point of the arm (`so101.kinematics.POINTS`) must stay, in metres in the base frame.

    The base frame has base_link at the origin, x forward, z up; the table is z = 0. `r` is the horizontal distance
    from the base's z axis, sqrt(x² + y²). Each bound is optional: None means no limit.
    """

    x_min: float | None = None
    x_max: float | None = None
    y_min: float | None = None
    y_max: float | None = None
    z_min: float | None = None
    z_max: float | None = None
    r_min: float | None = None
    r_max: float | None = None

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if value is None:
                continue
            if not _finite(value):
                raise ValueError(f"workspace.{f.name}: expected a finite number or null, got {value!r}")
            if f.name.startswith("r_") and value < 0:
                raise ValueError(f"workspace.{f.name}: a radius cannot be negative, got {value!r}")
            object.__setattr__(self, f.name, float(value))
        for axis in "xyzr":
            lower, upper = getattr(self, f"{axis}_min"), getattr(self, f"{axis}_max")
            if lower is not None and upper is not None and lower >= upper:
                raise ValueError(f"workspace: {axis}_min {lower} must be below {axis}_max {upper}")

    @classmethod
    def from_dict(cls, data: Mapping) -> "Workspace":
        names = {f.name for f in fields(cls)}
        unknown = sorted(set(data) - names)
        if unknown:
            raise ValueError(f"workspace: unknown keys {unknown}; expected some of {sorted(names)}")
        return cls(**data)

    @property
    def bounded(self) -> bool:
        """True when at least one bound is set."""
        return any(getattr(self, f.name) is not None for f in fields(self))

    def excess(self, point: Sequence[float], radius: float = 0.0) -> float:
        """How far a sphere of `radius` around `point` sticks out of the workspace, summed over the bounds (0 = inside)."""
        x, y, z = (float(v) for v in point)
        total = 0.0
        for value, lower, upper in (
            (x, self.x_min, self.x_max),
            (y, self.y_min, self.y_max),
            (z, self.z_min, self.z_max),
            (math.hypot(x, y), self.r_min, self.r_max),
        ):
            if lower is not None:
                total += max(0.0, lower - (value - radius))
            if upper is not None:
                total += max(0.0, value + radius - upper)
        return total

    def contains(self, point: Sequence[float], radius: float = 0.0) -> bool:
        return self.excess(point, radius) == 0.0

    def arm_excess(self, positions: Mapping[str, Sequence[float]]) -> float:
        """`excess` summed over the arm points from `ArmPoints.positions`, each with its clearance radius."""
        return sum(self.excess(position, POINTS[name].radius) for name, position in positions.items())

    def tightened(self, other: "Workspace | None") -> "Workspace":
        """The intersection of this workspace and `other`: every bound is the stricter of the two.

        A task uses this to narrow the workspace from safety.json for itself; it can never widen it. Raises a
        ValueError when the two do not overlap.
        """
        if other is None:
            return self
        values = {}
        for axis in "xyzr":
            lowers = [v for v in (getattr(self, f"{axis}_min"), getattr(other, f"{axis}_min")) if v is not None]
            uppers = [v for v in (getattr(self, f"{axis}_max"), getattr(other, f"{axis}_max")) if v is not None]
            values[f"{axis}_min"] = max(lowers) if lowers else None
            values[f"{axis}_max"] = min(uppers) if uppers else None
        return Workspace(**values)


def leaves_workspace(
    previous: Mapping[str, float], requested: Mapping[str, float], workspace: Workspace, arm: ArmPoints | None = None
) -> bool:
    """True when moving from the joint targets `previous` to `requested` must be held: the move takes an arm point out
    of `workspace`, or further out when the arm is already outside it (e.g. at connect). Moves back in pass.

    Both are complete joint positions in model radians, keyed by joint name or `"<joint>.pos"`. This is the single
    rule `SafeFollower` applies; a task that keeps its own targets (e.g. so101-rl's ReachIO) calls it too, so
    `SafeFollower` never has to hold that task's commands.
    """
    if not workspace.bounded:
        return False
    arm = arm if arm is not None else ArmPoints()
    after = workspace.arm_excess(arm.positions(requested))
    if after == 0.0:
        return False
    return after > workspace.arm_excess(arm.positions(previous)) + WORKSPACE_TOLERANCE


@dataclass(frozen=True)
class SafetyLimits:
    """Soft joint limits and the largest change per command, both in radians, for every joint in `JOINTS`, and the
    Cartesian `workspace` the arm must stay in (unbounded by default)."""

    joint_limits: dict[str, tuple[float, float]]
    max_step: dict[str, float]
    workspace: Workspace = Workspace()

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
        if not isinstance(self.workspace, Workspace):
            raise TypeError(f"workspace: expected a Workspace, got {type(self.workspace).__name__}")
        object.__setattr__(self, "joint_limits", limits)
        object.__setattr__(self, "max_step", steps)

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyLimits":
        workspace = Workspace.from_dict(data.get("workspace") or {})
        return cls(dict(data["joint_limits"]), dict(data["max_step"]), workspace)

    def with_workspace(self, workspace: Workspace | None) -> "SafetyLimits":
        """These limits with the workspace narrowed to `workspace` as well (`Workspace.tightened`); never widened."""
        return replace(self, workspace=self.workspace.tightened(workspace))

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
    workspace: a task's own workspace. It narrows `limits.workspace` (`Workspace.tightened`), never widens it;
        `self.limits.workspace` is the one in use.

    `last_clipped` maps `"<joint>.pos"` -> (requested, sent) for the joints the last command changed. `last_held` is
    True when the last command was held because it would have left the workspace.
    """

    def __init__(self, robot, limits: SafetyLimits | None = None, workspace: Workspace | None = None):
        self.robot = robot
        limits = limits if limits is not None else SafetyLimits.load()
        self.limits = limits.with_workspace(workspace)
        self.last_clipped: dict[str, tuple[float, float]] = {}
        self.last_held = False
        self._arm = ArmPoints()
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
        self.last_held = False

    def disconnect(self) -> None:
        self._goal = None
        self.robot.disconnect()

    def get_observation(self) -> dict[str, Any]:
        return self.robot.get_observation()

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        """Validate, clamp to the soft limits, limit the step, hold at the workspace, then forward.

        Returns what the robot actually sent. When the move would take the arm out of the workspace (`leaves_workspace`),
        the joints in this action keep their previous targets: steps are small, so that is a safe place to stop.
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

        proposed = self._goal | {key.removesuffix(SUFFIX): value for key, value in safe.items()}
        self.last_held = leaves_workspace(self._goal, proposed, self.limits.workspace, self._arm)
        if self.last_held:
            safe = {key: self._goal[key.removesuffix(SUFFIX)] for key in safe}

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
        reason = " (held at the workspace boundary)" if self.last_held else ""
        log.warning("SafeFollower limited the action%s (rad): %s", reason, details)

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        if self.is_connected:
            self.disconnect()
