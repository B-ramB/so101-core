# Use in your project

so101 is meant to be installed in other projects (RL, imitation learning, simulation), so they share the same setup,
units and safety layer.

## Install

From a local checkout, editable, so changes in so101-core are picked up immediately:

```bash
uv add --editable ../so101-core
```

Or from GitHub (pin a release with `--tag`, or a commit with `--rev`):

```bash
uv add "so101-core @ git+https://github.com/B-ramB/so101-core"
```

Add the `sim` extra for MuJoCo: `uv add "so101-core[sim] @ ..."`, or `--extra sim` on an editable install. The
commands (`so101-setup`, `so101-teleop`, …) are then available in your project with `uv run`.

## Project config folder

All robot files live in `so101_config/` at the root of the project you run the commands in (the nearest folder with
a `pyproject.toml`; change with the `SO101_HOME` environment variable). Every project gets its own folder:

```
so101_config/
├── urdf/               URDFs + meshes (so101-fetch-urdfs)
├── calibration/        leader / follower calibration files
├── visualization.json  camera pose and joint offsets for the Rerun models
├── safety.json         soft joint limits and max step for SafeFollower
├── arm_config.json     ports, which calibration is used, camera
├── usb_configs/        dated port / camera detection results
└── mujoco/             MuJoCo model generated from the follower URDF (so101-sim)
```

The folder is created with its own `.gitignore` (`*`), so none of these files are committed. To reuse a calibration in
another project, copy its `calibration/` folder over and answer "yes" when `so101-setup` asks whether calibration
files are present. Run [`so101-setup`](setup.md) once in the new project to write its `arm_config.json`.

## Writing control code

- Joint values are **model radians** everywhere: [units-and-joints.md](units-and-joints.md).
- Wrap the follower, real or sim, in `SafeFollower`: [safety.md](safety.md).
- `MujocoFollower` has the same interface as the real follower, so code can be tried in the sim first:
  [simulation.md](simulation.md).
