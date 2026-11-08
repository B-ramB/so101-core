"""Open the simulated SO-101 follower in the MuJoCo viewer.

The model is generated from the follower URDF in so101_config/urdf (fetch it with `uv run so101-fetch-urdfs`) and
regenerated whenever the URDF changes. The arm starts in the home pose (every joint at 0 rad, the calibration mid
stance). Move the joints with the sliders in the viewer's Control panel (right side): one per actuator, in radians.

Run with:
    uv run --extra sim so101-sim
    uv run --extra sim so101-sim --scene so101.xml     # the arm alone
    uv run --extra sim so101-sim --rebuild             # regenerate the model first
"""

import argparse
import sys

from so101.paths import pretty


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--scene", default="scene_box.xml", help="scene file in so101_config/mujoco (default: %(default)s)"
    )
    parser.add_argument("--rebuild", action="store_true", help="regenerate the model from the URDF first")
    args = parser.parse_args()

    try:
        import mujoco
        import mujoco.viewer
    except ImportError:
        sys.exit("MuJoCo is not installed. Run with the sim extra: `uv run --extra sim so101-sim`.")

    from so101.sim.model import ensure_model, home, load_model

    try:
        dest = ensure_model(force=args.rebuild)
        model = load_model(args.scene)
    except (FileNotFoundError, ValueError) as e:
        sys.exit(str(e))

    print(f"Loaded {pretty(dest / args.scene)}. Move the joints with the sliders in the Control panel.", flush=True)
    data = mujoco.MjData(model)
    home(model, data)
    mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
