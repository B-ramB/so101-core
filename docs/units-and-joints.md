# Units and joints

## The standard: model radians

so101 uses one unit for joint positions: **model radians**. These are the joint values of the follower URDF
(`so101_new_calib.urdf`). The MuJoCo model is generated from that URDF, so its `qpos` and `ctrl` are in this unit, and
the sim needs no conversion. Kinematics, RL environments and MuJoCo all work in radians too.

Joint names are `so101.conventions.JOINTS`: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`,
`gripper`. The follower's gripper ranges from `GRIPPER_LOWER = -0.174533` to `GRIPPER_UPPER = 1.74533` rad.

## lerobot units, at the real-arm edge

lerobot reads and writes the real arm in its own units:

| Joint | lerobot unit | Mid stance |
|---|---|---|
| body joints | degrees | 0° |
| gripper | percent, 0 = closed, 100 = open | 50 % |

lerobot calibration centres each joint in its range, so the pose you held during calibration (the **mid stance**)
reads 0° / 50 %. `uv run so101-view-urdf` draws the models in that stance.

## The conversion

The real arm and the model don't agree on every zero or direction. `so101_config/visualization.json` → `joints` stores,
per joint, the `offset` and `invert` that align them. Lerobot units are converted to model units first, then to radians:

```
model = sign · real + offset          sign = -1 if invert else 1    (degrees, or % for the gripper)
body:    radians = radians(model)
gripper: radians = lower + model / 100 · (upper − lower)
```

The inverse is `real = sign · (model − offset)`, after radians → degrees / percent.

Example: the default `wrist_roll` offset is −90°. Real wrist_roll 0° → model −90° → **−π/2 rad**. In the mid stance,
the model's joints are therefore:

| shoulder_pan | shoulder_lift | elbow_flex | wrist_flex | wrist_roll | gripper |
|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | −π/2 | 0.785 (50 %) |

The URDF zero (0 rad everywhere, the sim's home pose) is not the mid stance: wrist_roll there is real +90°, and the
gripper is about 9 % open.

The same file aligns the Rerun model with the real arm and converts real-arm readings to the standard. So an offset
you tune with `so101-view-urdf` also applies to every real ↔ sim comparison.

## From Python

`JointPositions` holds radians. Degrees only come in through `from_degrees` and go out through `to_degrees`, using a
`JointCalibration` (the offsets/inverts + gripper limits):

```python
from so101.conventions import JointPositions
from so101.viz.config import VisualizationConfig

cal = VisualizationConfig().joint_calibration()          # so101_config/visualization.json -> joints

q = JointPositions.from_degrees({"wrist_roll": 0.0, "gripper": 50.0}, cal)   # e.g. a real-arm reading
q.radians                    # {"wrist_roll": -1.5708, "gripper": 0.7854}
q["wrist_roll"]              # -1.5708

JointPositions({"wrist_flex": 0.5}, cal).to_degrees()    # {"wrist_flex": 28.65}
```

Keys are plain joint names. lerobot dicts use `"<joint>.pos"`, so strip the suffix first.
`to_degrees()` returns the gripper in percent, as lerobot does.

`JointCalibration(gripper_limits=...)` takes another gripper range. The Rerun view uses that to draw the leader, whose
URDF gripper range is 0 … 0.785 rad.

## Datasets

lerobot records datasets from the real arm in lerobot units. To compare or mix them with sim data, convert them with
`JointPositions.from_degrees`, using the calibration of the arm that recorded them.
