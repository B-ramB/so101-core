# SO-101 core

`so101` is an installable Python package to set up, calibrate, teleoperate, visualize and simulate the SO-101 leader
and follower arms, built on [lerobot](https://github.com/huggingface/lerobot), [Rerun](https://rerun.io) and
[MuJoCo](https://mujoco.org). Other projects (RL, imitation learning, simulation) install it to share the same setup,
units and safety layer.

| Command | What it does |
|---|---|
| `uv run so101-setup` | Detect ports and camera, calibrate the arms, write the config (offers to fetch the URDFs) |
| `uv run so101-fetch-urdfs` | Download or update the SO-101 URDFs and meshes |
| `uv run so101-view-urdf` | Show the URDF models in Rerun in the calibration mid stance, to align `visualization.json` (no arm needed) |
| `uv run so101-teleop` | Teleoperate the follower with the leader, visualized in Rerun (`--no-rerun` to turn it off, `--sim` for the MuJoCo sim) |
| `uv run --extra sim so101-sim` | Open the follower in the MuJoCo viewer with a box to pick up |

## Install

In this repository:

```bash
uv sync                 # add --extra sim for MuJoCo
```

In another project, from a local checkout (changes are picked up immediately) or from GitHub:

```bash
uv add --editable ../so101-core
uv add "so101-core @ git+https://github.com/B-ramB/so101-core"
```

See [docs/use-in-your-project.md](docs/use-in-your-project.md) for the config folder and where to start in your own code.

## Quick start

### Real arms

It is assumed that the robot is built and the motors are configured (`lerobot-setup-motors`).

```bash
uv sync
uv run so101-setup          # both arms powered and connected over USB
uv run so101-view-urdf      # optional: align the models with the real arm
uv run so101-teleop
```

### Simulation (no arms needed)

```bash
uv sync --extra sim
uv run so101-fetch-urdfs
uv run --extra sim so101-sim                    # MuJoCo viewer with sliders per joint
uv run --extra sim so101-teleop --sim           # drive the sim with the real leader arm
```

## Documentation

See [docs/index.md](docs/index.md) for all pages:

- [Setup](docs/setup.md): setup, calibration, URDFs, serial port access, teleoperation
- [Visualization](docs/visualization.md): aligning the models, `visualization.json`
- [Units and joints](docs/units-and-joints.md): model radians and the joint mapping
- [Simulation](docs/simulation.md): the MuJoCo model and `--sim`
- [Safety](docs/safety.md): `SafeFollower` and what it does not protect against
- [Use in your project](docs/use-in-your-project.md): installing so101 in another project
- [Development](docs/development.md): tests, lint, CI
