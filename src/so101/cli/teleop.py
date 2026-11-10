"""Teleoperate the SO-101 follower with the leader arm, visualized in Rerun.

Uses the setup produced by `uv run so101-setup` (so101_config/arm_config.json):
  - leader / follower ports and calibration files
  - the wrist camera

With --sim the follower is the MuJoCo simulation (`MujocoFollower`, needs the `sim` extra) instead of the real arm:
only the leader has to be connected. The MuJoCo viewer shows the sim arm.

Joint values pass through the loop in model radians, the so101 standard: the leader's degrees are converted with
the joint mapping in so101_config/visualization.json, and back to degrees for the real follower.

Rerun shows:
  - the follower and leader URDF models moving with the arms (the follower model shows the sim state with --sim; the
    URDFs must be in so101_config/urdf, fetch them with `uv run so101-fetch-urdfs`)
  - the wrist camera as a frustum attached to the follower gripper, with the live image projected in it
  - the joint positions in radians (leader action vs. follower observation) as time series

How the models and the camera are drawn is configured in so101_config/visualization.json (copied
from the package defaults on first run). The file is reloaded while running: edit, save, and Rerun updates.

Visualization uses Rerun (https://rerun.io): a viewer window is spawned automatically. Use --no-rerun to
teleoperate without it.

Run with:
    uv run so101-teleop            # default 30 fps
    uv run so101-teleop --fps 60 --no-camera
    uv run so101-teleop --no-rerun
    uv run --extra sim so101-teleop --sim
"""

import argparse
import sys
import time
import warnings

from so101.config import load_config
from so101.hardware.devices import make_follower, make_leader
from so101.paths import FOLLOWER_URDF, VIS_CONFIG_PATH, pretty
from so101.robot import Follower, Leader
from so101.viz.config import VisualizationConfig


def run_teleop_loop(
    robot: Follower, teleop: Leader, fps: int, visualizer=None, max_steps: int | None = None
) -> int:
    """Connect both, let the follower copy the leader at `fps` until Ctrl+C or `max_steps`, then disconnect both.

    `robot` is any `Follower` (`RealFollower` or `MujocoFollower`) and `teleop` any `Leader` (`RealLeader`); actions
    and observations are joint positions in model radians. `visualizer.log_step(obs, action)` is called after every step.
    Returns the number of steps run.
    """
    steps = 0
    try:
        robot.connect()
        teleop.connect()
        print(f"Teleoperating at {fps} fps. Press Ctrl+C to stop.")
        while max_steps is None or steps < max_steps:
            loop_start = time.perf_counter()
            action = teleop.get_action()
            robot.send_action(action)
            obs = robot.get_observation()
            if visualizer:
                visualizer.log_step(obs, action)
            steps += 1

            dt = time.perf_counter() - loop_start
            time.sleep(max(1 / fps - dt, 0.0))
            print(f"\rLoop: {(time.perf_counter() - loop_start) * 1e3:5.1f} ms", end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        for device in (teleop, robot):
            if device.is_connected:
                device.disconnect()
    return steps


def make_sim_follower(fps: int, use_camera: bool, scene: str) -> tuple[Follower, tuple[int, int] | None]:
    try:
        from so101.sim.follower import MujocoFollower
    except ImportError:
        sys.exit("MuJoCo is not installed. Run with the sim extra: `uv run --extra sim so101-teleop --sim`.")
    robot = MujocoFollower(fps=fps, camera=use_camera, viewer=True, scene=scene)
    return robot, robot.camera_size if use_camera else None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fps", type=int, default=30, help="control loop frequency (default 30)")
    parser.add_argument("--sim", action="store_true", help="drive the MuJoCo sim instead of the real follower")
    parser.add_argument("--scene", default="scene_box.xml", help="sim scene in so101_config/mujoco (with --sim)")
    parser.add_argument("--no-camera", action="store_true", help="run without the wrist camera")
    parser.add_argument("--no-urdf", action="store_true", help="don't show the URDF models in Rerun")
    parser.add_argument("--no-rerun", action="store_true", help="teleoperate without the Rerun visualization")
    args = parser.parse_args()

    config = load_config(roles=("leader",) if args.sim else ("leader", "follower"))
    vis = VisualizationConfig(VIS_CONFIG_PATH)  # the joint mapping between the leader's degrees and radians
    # The camera image is only used for the visualization.
    use_camera = not (args.no_camera or args.no_rerun)
    teleop = make_leader(config, vis.joint_calibration)
    if args.sim:
        robot, camera_size = make_sim_follower(args.fps, use_camera, args.scene)
    else:
        robot, camera_size = make_follower(config, use_camera, vis.joint_calibration)

    visualizer = None
    if not args.no_rerun:
        from so101.viz.rerun_arm import Visualizer

        show_urdf = not args.no_urdf and FOLLOWER_URDF.exists()
        if not args.no_urdf and not show_urdf:
            print(f"No URDF found at {pretty(FOLLOWER_URDF)} (run `uv run so101-fetch-urdfs`). Showing data only.")
        # Joint values outside the URDF limits are clamped; don't spam a warning every frame.
        warnings.filterwarnings("ignore", message=".*limit.*", category=UserWarning)
        visualizer = Visualizer(vis, show_urdf, camera_size)
        print(f"Edit {pretty(VIS_CONFIG_PATH)} while running to adjust the camera pose and joint offsets.")
    else:
        print("Rerun visualization is off (--no-rerun).")

    print("Connecting to the leader arm and the sim..." if args.sim else "Connecting to the arms...")
    run_teleop_loop(robot, teleop, args.fps, visualizer)


if __name__ == "__main__":
    main()
