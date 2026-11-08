# Simulation

A MuJoCo model of the SO-101 follower, generated from the same URDF as the Rerun view, so you can look at the arm
and move it before writing any control code.

## Install and view

MuJoCo is an optional extra; the base install does not change.

```bash
uv sync --extra sim             # or prefix each command with `uv run --extra sim`
uv run so101-fetch-urdfs        # once: the follower URDF + meshes in so101_config/urdf/
uv run --extra sim so101-sim
```

The viewer opens with the arm in the home pose (every joint at 0 rad, the calibration mid stance) and a green 3 cm box
in front of it. Open the **Control** panel on the right: there is one slider per actuator (`shoulder_pan` …
`gripper`, in radians). Dragging a slider sets the target of that joint's position servo.

| Option | Effect |
|---|---|
| `--scene so101.xml` | the arm alone, without floor or box |
| `--rebuild` | regenerate the model from the URDF first |

Viewer tips: double-click a body to select it, Ctrl + right-drag to push it. In the **Rendering** panel, geom group 3
shows the collision shapes.

## The model

`so101.sim.model.ensure_model()` writes the model to `so101_config/mujoco/`:

```
so101_config/mujoco/
├── so101.xml       the arm (generated from so101_config/urdf/so101_new_calib.urdf)
└── scene_box.xml   floor, light and a box; includes so101.xml (copied from src/so101/sim/scenes/)
```

It regenerates them whenever the URDF, `model.py` or a shipped scene is newer, so the model always follows the current
URDF. `so101.xml` points at the URDF's meshes (`meshdir="../urdf"`); they are not copied. Don't edit the generated files:
change the code or add a scene to `src/so101/sim/scenes/`.

What comes from the URDF: links, joint names, axes and limits, masses and inertias, visual and collision meshes. The
joint zero is the "new_calib" zero, the same as the real arm after lerobot calibration.

What the generator adds:

- **Actuators**: one position actuator per joint, named after the joint (`so101.conventions.JOINTS`), with the
  control range equal to the joint range. Gains, force limit and joint damping / friction loss / armature are the
  STS3215 values from MuJoCo Menagerie's `robotstudio_so101` model (Apache-2.0).
- **Jaw collisions**: MuJoCo collides with the convex hull of a mesh. The hull of the part with the fixed jaw is a
  solid wedge that fills the space between the jaws, so nothing could be grasped. Both jaw meshes are therefore cut
  into 12 slabs along their length, and each slab gets its own hull.
- **Contact exclusions** between linked bodies, whose hulls overlap at the joints.
- **Solver options** for grasping: 2 ms timestep, implicitfast integrator, elliptic friction cones.
- Visual meshes in geom group 2, collision meshes in group 3 (hidden in the viewer by default).

From Python:

```python
import mujoco
from so101.sim.model import home, load_model

model = load_model()              # scene_box.xml; load_model("so101.xml") for the arm alone
data = mujoco.MjData(model)
home(model, data)                 # home pose, actuators holding it
data.ctrl[model.actuator("gripper").id] = 0.7   # open the gripper (rad)
mujoco.mj_step(model, data)
```

## Tests

`tests/test_sim_model.py` (marker `sim`) needs the `sim` extra and `so101_config/urdf/` in this checkout; it is
skipped otherwise. It checks that the model compiles, that the actuator names equal `JOINTS`, that 1000 steps from the
home pose stay finite and in place, and that the arm can pick up the box.

```bash
uv run --extra sim pytest -m sim
```
