"""Teleoperate the SO-101 follower with the leader arm, visualized in Rerun.

Uses the setup produced by `uv run so101-setup` (so101_config/arm_config.json):
  - leader / follower ports and calibration files
  - the wrist camera

Rerun shows:
  - the follower and leader URDF models moving with the real arms (if the URDFs are in
    so101_config/urdf, fetch them with `uv run so101-fetch-urdfs`)
  - the wrist camera as a frustum attached to the follower gripper, with the live image projected in it
  - the joint positions (leader action vs. follower observation) as time series

How the models and the camera are drawn is configured in so101_config/visualization.json (copied
from the package defaults on first run). The file is reloaded while running: edit, save, and Rerun updates.

Visualization uses Rerun (https://rerun.io): a viewer window is spawned automatically. Use --no-rerun to
teleoperate without it (no viewer, no visualization config needed).

Run with:
    uv run so101-teleop            # default 30 fps
    uv run so101-teleop --fps 60 --no-camera
    uv run so101-teleop --no-rerun
"""

import argparse
import time
import warnings

import rerun as rr

from so101.config import load_config
from so101.hardware.devices import make_devices
from so101.paths import FOLLOWER_URDF, VIS_CONFIG_PATH, pretty
from so101.viz.config import VisualizationConfig
from so101.viz.rerun_arm import Visualizer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fps", type=int, default=30, help="control loop frequency (default 30)")
    parser.add_argument("--no-camera", action="store_true", help="run without the wrist camera")
    parser.add_argument("--no-urdf", action="store_true", help="don't show the URDF models in Rerun")
    parser.add_argument("--no-rerun", action="store_true", help="teleoperate without the Rerun visualization")
    args = parser.parse_args()

    config = load_config()
    # The camera image is only used for the visualization.
    robot, teleop, camera_cfg = make_devices(config, use_camera=not (args.no_camera or args.no_rerun))

    visualizer = None
    if not args.no_rerun:
        vis = VisualizationConfig(VIS_CONFIG_PATH)
        show_urdf = not args.no_urdf and FOLLOWER_URDF.exists()
        if not args.no_urdf and not show_urdf:
            print(f"No URDF found at {pretty(FOLLOWER_URDF)} (run `uv run so101-fetch-urdfs`). Showing data only.")
        # Joint values outside the URDF limits are clamped; don't spam a warning every frame.
        warnings.filterwarnings("ignore", message=".*limit.*", category=UserWarning)
        camera_size = (camera_cfg.width, camera_cfg.height) if camera_cfg else None
        visualizer = Visualizer(vis, show_urdf, camera_size)

    print("Connecting to the arms...")
    robot.connect()
    teleop.connect()
    print(f"Teleoperating at {args.fps} fps. Press Ctrl+C to stop.")
    if visualizer:
        print(f"Edit {pretty(VIS_CONFIG_PATH)} while running to adjust the camera pose and joint offsets.")
    else:
        print("Rerun visualization is off (--no-rerun).")

    try:
        start = time.perf_counter()
        while True:
            loop_start = time.perf_counter()
            action = teleop.get_action()
            robot.send_action(action)
            obs = robot.get_observation()

            if visualizer:
                rr.set_time("time", duration=loop_start - start)
                visualizer.log_step(obs, action)

            dt = time.perf_counter() - loop_start
            time.sleep(max(1 / args.fps - dt, 0.0))
            print(f"\rLoop: {(time.perf_counter() - loop_start) * 1e3:5.1f} ms", end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
