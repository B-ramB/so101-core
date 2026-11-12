# Safety: `SafeFollower`

`so101.safety.SafeFollower` wraps a follower, real or sim, and checks every command before it reaches the arm.
`so101-teleop` always uses it (real arm and `--sim`). Use it in your own control code too:

```python
from so101.safety import SafeFollower
from so101.sim.follower import MujocoFollower

with SafeFollower(MujocoFollower(viewer=True)) as robot:      # or SafeFollower(real_follower)
    robot.send_action({"elbow_flex.pos": 3.0})   # far beyond the limit: moves 0.17 rad towards it
    robot.last_clipped                           # {"elbow_flex.pos": (3.0, 0.17)}
```

`SafeFollower` is itself a `so101.robot.Follower`, so a controller works the same with or without it. Like everything
in so101, its values are **model radians** (see [units-and-joints.md](units-and-joints.md)).

## What already protects the arm, and what `SafeFollower` adds

This was checked in lerobot 0.6.1, the version this repo uses.

| Protection | Without SafeFollower | With SafeFollower |
|---|---|---|
| Calibration writes `Min/Max_Position_Limit` to each motor, so the firmware clamps goals to the calibrated range | ✅ But a `wrist_roll` calibrated over 0–4095 is a full turn, so in practice it has no limit | ✅ |
| Gripper torque limited to 50 % | ✅ | ✅ |
| Software clamp to the calibrated range | ❌ Not in degrees mode (lerobot's default `use_degrees=True`). `_unnormalize` clamps only the ±100 and 0–100 modes | ✅ clamp to the soft limits |
| Per-command step limit (`max_relative_target`) | ❌ Exists but off by default (`None`) | ✅ `max_step` |
| NaN, inf or a wrong joint name | ❌ Fails deep in the motor-bus code. The `finally: disconnect()` then disables torque and the arm drops | ✅ Rejected before anything is sent |
| Soft limits tighter than the calibration (table, cables, self-collision) | ❌ | ✅ `joint_limits` |
| The same protection in the sim | ❌ | ✅ |

On each `send_action`, in this order:

1. **Validate.** Every key must be `"<joint>.pos"` for a joint in `so101.conventions.JOINTS`, and every value must be a
   finite number. Otherwise it raises `UnsafeActionError` (a `ValueError`) and **sends nothing**, not even the valid
   joints of that action.
2. **Clamp** each joint to its soft limits (`joint_limits`).
3. **Limit the step:** the target changes by at most `max_step` compared with the last target sent. At `connect()`,
   that is the arm's current position. A far target is reached over several commands, not in one jump.
4. **Forward** the result. A partial action moves only the joints it names. `last_clipped` records
   `"<joint>.pos" -> (requested, sent)` for each joint the last command changed. A warning is logged at most once per
   second.

## The limits: `so101_config/safety.json`

The file is created from the package defaults (`src/so101/defaults/safety.json`) the first time a `SafeFollower` is made.
Keys missing from your file are filled in from the defaults. Invalid limits (min ≥ max, a missing or unknown joint,
`max_step` ≤ 0) are rejected at load with an error that names the file. The file is read at startup, so restart
`so101-teleop` after editing it.

Defaults, in radians (degrees in brackets):

| Joint | Model range | Soft limits | `max_step` |
|---|---|---|---|
| shoulder_pan | ±1.92 (±110°) | ±1.82 (±104°) | 0.17 (10°) |
| shoulder_lift | ±1.75 (±100°) | ±1.64 (±94°) | 0.17 |
| elbow_flex | ±1.69 (±97°) | ±1.59 (±91°) | 0.17 |
| wrist_flex | ±1.66 (±95°) | ±1.55 (±89°) | 0.17 |
| wrist_roll | −2.74 … 2.84 (−157° … 163°) | −2.6 … 2.7 (−149° … 155°) | 0.17 |
| gripper | −0.17 … 1.75 | −0.12 … 1.69 | 0.38 (≈ 20 % of the range) |

The soft limits are the model's joint range minus about 0.1 rad. `max_step` 0.17 rad per command is about 300°/s at
30 fps. It is a limit **per command**, so a faster loop moves faster.

### Choosing your own limits

1. Run `uv run so101-teleop` (or `--sim` first, to try it out).
2. Move the leader slowly to the edges of the workspace you consider safe: just above the table, before the cables
   pull, before the arm hits itself.
3. Read each follower joint's value (in radians) from the joint plots in Rerun at those edges.
4. Write them, with a small margin, into `so101_config/safety.json` → `joint_limits`, and restart.

The limits bound the **targets**. The servos overshoot a target a little: in the sim, up to about 0.085 rad (5°)
during a full-speed move into a limit. Keep a margin to anything you must not touch.

## What it does not protect against

- **Collisions** with the table, objects, people or the arm itself. Joint limits can't describe these: whether the
  gripper hits the table depends on all the joints together. Cartesian workspace limits and self-collision checks
  are not implemented yet.
- **Speed over time.** `max_step` is per command, not per second.
- **Torque off at the end.** `disconnect()` disables torque, so the real arm **drops**. Support it, or move it to a
  rest pose first. This also happens when `so101-teleop` stops because of an error, including an `UnsafeActionError`.
