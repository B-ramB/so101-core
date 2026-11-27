"""Reject or limit unsafe control inputs before they reach the arm, real or simulated.

`SafeFollower` wraps any `so101.robot.Follower` (`RealFollower`, `MujocoFollower`) and is a `Follower` itself. On each
`send_action` it validates the whole action, clamps every joint to its soft limits, limits the change per command, holds
the arm when the move would take it out of the Cartesian workspace (e.g. into the table), and only then forwards it.
Joint values are model radians, the so101 standard (see docs/units-and-joints.md).

    with SafeFollower(MujocoFollower(viewer=True)) as robot:
        robot.send_action({"elbow_flex.pos": 3.0})   # moves at most max_step, towards the soft limit
        robot.last_clipped                           # {"elbow_flex.pos": (3.0, 0.17)}

A task can narrow the workspace for itself, never widen it: `SafeFollower(robot, workspace=Workspace(z_min=0.05))`.

`disconnect()` first moves the arm to `rest_pose` (`rest()`, through the same clamp and step limit) and only then turns
the torque off, so the real arm doesn't drop.

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
# Commands per second while moving to the rest pose. With the default max_step of 0.17 rad that is at most ~1.7 rad/s
# (~100 deg/s); every joint gets there within the default 5 s timeout from anywhere inside the soft limits.
REST_HZ = 10.0
# rad: how far inside the soft limits `save_rest_pose` stores a measured joint that is outside them.
REST_MARGIN = 0.02

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
    """Soft joint limits and the largest change per command, both in radians, for every joint in `JOINTS`, the
    Cartesian `workspace` the arm must stay in (unbounded by default), and the `rest_pose` {joint: radians} that
    `SafeFollower.disconnect()` moves to before the torque goes off (None: disconnect without moving)."""

    joint_limits: dict[str, tuple[float, float]]
    max_step: dict[str, float]
    workspace: Workspace = Workspace()
    rest_pose: dict[str, float] | None = None

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
        if self.rest_pose is not None:
            object.__setattr__(self, "rest_pose", self._checked_rest_pose(self.rest_pose))

    def _checked_rest_pose(self, pose) -> dict[str, float]:
        if not isinstance(pose, Mapping):
            raise TypeError(f"rest_pose: expected {{joint: radians}}, got {pose!r}")
        if set(pose) != set(JOINTS):
            missing, unknown = set(JOINTS) - set(pose), set(pose) - set(JOINTS)
            raise ValueError(f"rest_pose: missing joints {sorted(missing)}, unknown joints {sorted(unknown)}")
        checked = {}
        for joint in JOINTS:
            value = pose[joint]
            if not _finite(value):
                raise ValueError(f"rest_pose.{joint}: expected a finite number, got {value!r}")
            lower, upper = self.joint_limits[joint]
            if not lower <= value <= upper:
                raise ValueError(f"rest_pose.{joint}: {value} is outside joint_limits.{joint} [{lower}, {upper}]")
            checked[joint] = float(value)
        return checked

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyLimits":
        workspace = Workspace.from_dict(data.get("workspace") or {})
        rest_pose = data.get("rest_pose")
        return cls(dict(data["joint_limits"]), dict(data["max_step"]), workspace, rest_pose)

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

        Keys missing from the file are filled from the defaults. `rest_pose` is the exception: it is one pose, so a
        `rest_pose` in your file is used as a whole and must name every joint. Invalid limits raise a ValueError naming
        the file.
        """
        if ensure_user_config(path, SAFETY_DEFAULTS_PATH):
            print(f"Created {pretty(path)} from the package defaults.")
        defaults = json.loads(SAFETY_DEFAULTS_PATH.read_text())
        try:
            user = json.loads(path.read_text())
            merged = merge_defaults(defaults, user)
            if "rest_pose" in user:
                merged["rest_pose"] = user["rest_pose"]
            return cls.from_dict(merged)
        except (ValueError, TypeError) as e:
            hint = ""
            if str(e).startswith("rest_pose"):
                hint = " Set a rest_pose inside your joint_limits there, or record one with `uv run so101-setup --rest-pose`."
            raise ValueError(f"Invalid safety limits in {pretty(path)}: {e}.{hint}") from e


def save_rest_pose(pose: Mapping[str, float], path: Path = SAFETY_CONFIG_PATH) -> tuple[dict[str, float], list[str]]:
    """Store `pose` ({joint: radians}, e.g. measured on the real arm) as `rest_pose` in the safety file `path`.

    A joint outside the file's `joint_limits` (a folded arm often rests on a mechanical stop, beyond the soft limits)
    is stored `REST_MARGIN` inside them. Returns the stored pose and the joints that had to be moved inwards. The file
    is created from the package defaults when missing; its other settings are kept.
    """
    limits = SafetyLimits.load(path)
    stored, moved = {}, []
    for joint in JOINTS:
        value = float(pose[joint])
        if not math.isfinite(value):
            raise ValueError(f"rest_pose.{joint}: expected a finite number, got {value!r}")
        value = round(value, 3)
        lower, upper = limits.joint_limits[joint]
        if not lower <= value <= upper:
            value = round(min(max(value, lower + REST_MARGIN), upper - REST_MARGIN), 3)
            moved.append(joint)
        stored[joint] = value
    data = json.loads(path.read_text())
    data["rest_pose"] = stored
    replace(limits, rest_pose=stored)  # validates before anything is written
    path.write_text(json.dumps(data, indent=4) + "\n")
    return stored, moved


class SafeFollower:
    """A `Follower` that validates, clamps and step-limits every action before forwarding it to `robot`.

    robot: any `so101.robot.Follower` (`RealFollower`, `MujocoFollower`), in model radians.
    limits: the soft limits; None loads so101_config/safety.json (created from the defaults when missing).
    workspace: a task's own workspace. It narrows `limits.workspace` (`Workspace.tightened`), never widens it;
        `self.limits.workspace` is the one in use.

    `last_clipped` maps `"<joint>.pos"` -> (requested, sent) for the joints the last command changed. `last_held` is
    True when the last command was held because it would have left the workspace.

    `disconnect()` moves the arm to `limits.rest_pose` first (`rest()`); `disconnect(rest=False)` skips that.
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

    def disconnect(self, rest: bool = True) -> None:
        """Move to the rest pose (`rest()`, unless `rest=False`), then disconnect the robot, which turns torque off.

        The robot is disconnected even when the move fails (e.g. the bus itself failed): the error is logged.
        """
        try:
            if rest and self._goal is not None:
                try:
                    self.rest()
                except Exception as e:  # noqa: BLE001 - disconnecting matters more than the reason
                    log.error("Could not move to the rest pose (%s: %s); disconnecting anyway.", type(e).__name__, e)
        finally:
            self._goal = None
            self.robot.disconnect()

    def rest(self, timeout_s: float = 5.0, tolerance: float = 0.05, hz: float | None = None) -> bool:
        """Move to `limits.rest_pose`, a pose where the arm can stay with torque off. Returns True when it got there.

        Sends the rest pose through the same clamp, step limit and workspace hold as `send_action`, `hz` times per
        second (default `REST_HZ`), until every observed joint is within `tolerance` (rad) of it. After `timeout_s` it
        stops and logs a warning. Without a rest pose (`limits.rest_pose` is None) it does nothing and returns False.
        """
        if self._goal is None:
            raise RuntimeError("SafeFollower is not connected; call connect() first.")
        pose = self.limits.rest_pose
        if pose is None:
            log.info("No rest_pose in the safety limits; not moving before torque off.")
            return False
        period = 1.0 / (hz or REST_HZ)
        action = {f"{joint}{SUFFIX}": value for joint, value in pose.items()}
        deadline = time.monotonic() + timeout_s
        while True:
            start = time.monotonic()
            obs = self.robot.get_observation()
            errors = {joint: abs(float(obs[f"{joint}{SUFFIX}"]) - value) for joint, value in pose.items()}
            off = {joint: error for joint, error in errors.items() if error > tolerance}
            if not off:
                return True
            if start >= deadline:
                details = ", ".join(f"{joint} {error:.3f}" for joint, error in off.items())
                reason = " (held at the workspace boundary)" if self.last_held else ""
                log.warning("Did not reach the rest pose within %.1f s%s; off by (rad): %s", timeout_s, reason, details)
                return False
            self._send(action)
            time.sleep(max(period - (time.monotonic() - start), 0.0))

    def get_observation(self) -> dict[str, Any]:
        return self.robot.get_observation()

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        """Validate, clamp to the soft limits, limit the step, hold at the workspace, then forward.

        Returns what the robot actually sent. When the move would take the arm out of the workspace (`leaves_workspace`),
        the joints in this action keep their previous targets: steps are small, so that is a safe place to stop.
        Raises UnsafeActionError (and sends nothing) for an unknown key or a value that is not a finite number.
        """
        sent = self._send(action)
        self._warn_if_clipped()
        return sent

    def _send(self, action: dict[str, float]) -> dict[str, float]:
        """`send_action` without the warning: `rest()` is limited on every step by design."""
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
