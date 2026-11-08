"""Show the SO-101 follower and leader URDF models in Rerun at the calibration mid stance, without any arm connected.

lerobot calibrates every joint around the middle of its range of motion: in the mid stance every body joint reads
0 degrees and the gripper reads 50 %. This command draws both models in exactly that stance, passed through the joint
mapping in so101_config/visualization.json. Put the real arm in its mid stance (the pose you held during calibration)
and edit `joints.<joint>.offset` / `invert` until the model matches it. The file is reloaded while this runs.

Run with:
    uv run so101-view-urdf          # stop with Ctrl+C
"""

import argparse
import sys
import time
import warnings

import rerun as rr

from so101.conventions import JOINTS
from so101.paths import FOLLOWER_URDF, LEADER_URDF, VIS_CONFIG_PATH, pretty
from so101.viz.config import VisualizationConfig
from so101.viz.rerun_arm import ArmModel

# Real-arm readings in the calibration mid stance: body joints in degrees, gripper in percent.
MID_STANCE = {joint: 50.0 if joint == "gripper" else 0.0 for joint in JOINTS}


def main() -> None:
    argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter).parse_args()
    if not FOLLOWER_URDF.exists():
        sys.exit(f"No URDF found at {pretty(FOLLOWER_URDF)}. Fetch it with `uv run so101-fetch-urdfs`.")
    warnings.filterwarnings("ignore", message=".*limit.*", category=UserWarning)

    vis = VisualizationConfig(VIS_CONFIG_PATH)
    rr.init("so101_urdf_view", spawn=True)

    follower = ArmModel("follower", FOLLOWER_URDF, vis)
    leader = ArmModel("leader", LEADER_URDF, vis) if LEADER_URDF.exists() else None

    def draw() -> None:
        if leader:
            leader.log_placement(vis.data["leader_model_offset_m"])
        for model in (follower, leader):
            if model:
                model.log_positions(MID_STANCE)

    draw()
    print(f"Showing the mid stance (body joints 0 deg, gripper 50 %). Edit {pretty(VIS_CONFIG_PATH)} to align the")
    print("model with the real arm; changes show up within half a second. Press Ctrl+C to stop.")
    try:
        while True:
            if vis.reload_if_changed():
                draw()
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
