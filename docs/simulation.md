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
`gripper`, in radians). Dragging a slider sets the target of that joint's position servo. Radians are the
standard unit in so101; see [units-and-joints.md](units-and-joints.md) for how they map to the real arm's degrees.

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
- **Wrist camera** `wrist_cam` on `gripper_link`, at the default `wrist_camera` pose of `visualization.json`.
  `MujocoFollower(camera=True)` moves it to the pose in your own `so101_config/visualization.json`.

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

## Control the sim from Python

`so101.sim.follower.MujocoFollower` has the interface of lerobot's SO-101 follower: `connect()`, `disconnect()`,
`is_connected`, `get_observation()`, `send_action()`, `observation_features` / `action_features`, plus `reset()`. A
control loop written against the follower runs against the sim. It also works as a context manager.

```python
import time
from so101.sim.follower import MujocoFollower

poses = [{"shoulder_lift.pos": -0.5, "elbow_flex.pos": 0.5}, {"wrist_flex.pos": 1.0},
         {"shoulder_pan.pos": 0.8, "gripper.pos": 1.2}, {"shoulder_pan.pos": 0.0, "wrist_flex.pos": 0.0}]
with MujocoFollower(viewer=True) as robot:
    for pose in poses:
        for _ in range(60):                       # 2 s at 30 fps
            robot.send_action(pose)
            time.sleep(1 / 30)
    print(robot.get_observation())
```

| | |
|---|---|
| Keys | the real follower's: `"shoulder_pan.pos"` … `"gripper.pos"`, and `"wrist"` for the camera image |
| Units | **model radians** for every joint, the gripper too (see [units-and-joints.md](units-and-joints.md)) |
| `send_action(action)` | sets the position targets, steps the physics for 1/fps s, returns the action sent |
| Partial actions | only the joints in the action change; the others keep their target |
| Out of range | targets are clipped to the joint range; the returned action shows the clipped value |
| `reset()` | the home pose (every joint 0 rad), and the box back in place |

Options: `MujocoFollower(fps=30, camera=False, viewer=False, scene="scene_box.xml", camera_size=(640, 480))`.

- `send_action` does not wait in real time; as with the real arm, your loop does the pacing. Without `sleep` the sim
  runs as fast as it can, which is what you want for tests or RL.
- `viewer=True` opens MuJoCo's passive viewer and updates it after every step.
- `camera=True` renders the wrist camera (`wrist_cam` in the model) into `obs["wrist"]`, an H×W×3 `uint8` image. Its
  pose is `wrist_camera` from `so101_config/visualization.json`, the same one the Rerun view draws, so tune it there.
  Headless (no display), set `MUJOCO_GL=egl`.

`MujocoFollower` and the real arm's `RealFollower` both implement `so101.robot.Follower`: the same methods, keys and
units. `RealFollower` wraps lerobot's follower and converts to and from degrees at the hardware edge.

## Teleoperate the sim with the leader arm

```bash
uv run --extra sim so101-teleop --sim
```

Only the leader arm has to be plugged in; `so101-teleop` reads its port and calibration from
`so101_config/arm_config.json` as usual and does not check the follower. The follower is a
`MujocoFollower(fps, viewer=True)`: the MuJoCo viewer opens with the sim arm, and moving the leader moves it.

Rerun works as with the real arm. The follower model there shows the sim state, the leader model the leader, and the
joint plots compare both in radians. The wrist camera image comes from the sim camera (`--no-camera` turns it off).
`--scene so101.xml` drives the arm without floor and box.

How the leader's degrees become sim radians: the leader reading goes through the same joint mapping as the real
follower (`joints` in `so101_config/visualization.json`, see [units-and-joints.md](units-and-joints.md)). So this is
the quickest check that the mapping is right: put the leader in the mid stance (the calibration pose) and the sim
should look like `uv run so101-view-urdf`. If a joint is off, fix its `offset` / `invert`; the file is reloaded while
Rerun runs. With the real follower the mapping cancels out (degrees → radians → the same degrees), so teleoperating
the real arm is not affected by it.

From Python, the loop is `so101.cli.teleop.run_teleop_loop(robot, teleop, fps, visualizer=None, max_steps=None)`.
It takes any follower and leader with the interfaces above, so the same code drives the sim or the real arm.

## Tests

`tests/test_sim_model.py`, `tests/test_sim_follower.py` and `tests/test_sim_teleop.py` (marker `sim`) need the `sim`
extra and `so101_config/urdf/` in this checkout; they are skipped otherwise. `test_sim_model.py` checks that the
model compiles, that the actuator names equal `JOINTS`, that 1000 steps from the home pose stay finite and in place,
and that the arm can pick up the box. `test_sim_follower.py` checks that the keys equal the real follower's, that the mid stance comes
back within 1° after 1 s, that a 20° step on `wrist_flex` settles within 2° in 1 s, partial actions, clipping, reset
and the camera image shape. `test_sim_teleop.py` runs the teleop loop with a fake leader held in the mid stance and
checks that the sim follows within 1°.

```bash
uv run --extra sim pytest -m sim
```
