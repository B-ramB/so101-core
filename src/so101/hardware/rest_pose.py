"""Record the follower's rest pose: fold the real arm by hand, read its joints, store them in safety.json.

`SafeFollower.disconnect()` moves the arm to `rest_pose` before the torque goes off (see docs/safety.md). The
package default is a guess for a typical calibration; this measures it on your arm, in model radians through the same
joint mapping (visualization.json -> joints) that teleop uses, so the values match what `SafeFollower` sends.

`so101-setup` runs this after calibrating the follower; `uv run so101-setup --rest-pose` runs only this step.
"""

from collections.abc import Callable
from pathlib import Path

from so101.conventions import JOINTS
from so101.kinematics import ArmPoints
from so101.paths import SAFETY_CONFIG_PATH, VIS_CONFIG_PATH, pretty
from so101.prompts import header
from so101.safety import SafetyLimits, save_rest_pose

SUFFIX = ".pos"


def record_rest_pose(follower, path: Path = SAFETY_CONFIG_PATH, wait: Callable[[str], str] = input) -> dict[str, float]:
    """Connect `follower` (a `RealFollower`), turn its torque off, let the user fold it, store the pose in `path`.

    `wait(prompt)` blocks until the user is ready (`input` by default). Returns the stored pose in radians. The
    follower is disconnected at the end, with its torque off.
    """
    header("Rest pose of the follower")
    print(
        "When so101-teleop stops, the follower first moves to its rest pose, then its torque goes off.\n"
        "Choose a pose where the arm stays put without torque, folded onto its base."
    )
    follower.connect()
    try:
        wait("\nHold the follower arm: its torque turns off. Press ENTER when you hold it...")
        follower.disable_torque()
        wait("Fold the follower by hand into its rest pose, let go, and press ENTER...")
        obs = follower.get_observation()
    finally:
        follower.disconnect()
    measured = {joint: float(obs[f"{joint}{SUFFIX}"]) for joint in JOINTS}
    stored, moved = save_rest_pose(measured, path)

    print(f"\nSaved the rest pose to {pretty(path)} -> rest_pose (radians):")
    for joint in JOINTS:
        note = f"  (measured {measured[joint]:.3f}, outside the soft limits)" if joint in moved else ""
        print(f"  {joint:14s} {stored[joint]:7.3f}{note}")
    if moved:
        print(
            "Joints outside joint_limits were stored just inside them: the arm settles the last bit itself when its\n"
            "torque goes off."
        )
    limits = SafetyLimits.load(path)
    if limits.workspace.arm_excess(ArmPoints().positions(stored)) > 0:
        print(
            "Warning: this pose is outside the workspace in safety.json (e.g. below z_min). SafeFollower holds\n"
            "the arm at the boundary and won't reach it; record a pose higher above the table."
        )
    return stored


def setup_rest_pose(config: dict, path: Path = SAFETY_CONFIG_PATH) -> dict[str, float]:
    """`record_rest_pose` for the follower in the arm config (`so101.config.load_config()` / `save_config`)."""
    from so101.hardware.devices import make_follower
    from so101.viz.config import VisualizationConfig

    vis = VisualizationConfig(VIS_CONFIG_PATH)
    follower, _ = make_follower(config, use_camera=False, calibration=vis.joint_calibration)
    return record_rest_pose(follower, path)
