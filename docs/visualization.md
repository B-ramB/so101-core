# Visualization

All visualization uses [Rerun](https://rerun.io) (the `rerun-sdk` dependency, installed with `uv sync`).
`so101-view-urdf` and `so101-teleop` spawn the Rerun viewer window automatically; no separate install or server is
needed. The viewer shows the 3D models (driven by the URDFs and `visualization.json`), the wrist camera and the joint
plots. `so101-teleop --no-rerun` runs the arms without any visualization.

## Align the models: `so101-view-urdf`

```bash
uv run so101-view-urdf
```

Opens Rerun with the follower and leader models in the **calibration mid stance**, without any arm connected.
lerobot calibrates every joint around the middle of its range, so in that stance every body joint reads 0° and the
gripper reads 50 %. The models show what these readings look like after the joint mapping in
`so101_config/visualization.json`.

To align the models with your arms:

1. Put the real arm in its mid stance (the pose you held during calibration).
2. Compare it with the model in Rerun, joint by joint.
3. Edit `joints.<joint>.offset` (degrees, or percent for the gripper) or `invert` in `visualization.json` until the
   model matches. The file is reloaded while the command runs; stop it with `Ctrl+C`.

The same mapping is used by `so101-teleop`, so the models then follow the real arms correctly. It also converts the
arm's readings to model radians, the unit used everywhere else; see [units-and-joints.md](units-and-joints.md).

## Visualization settings: `visualization.json`

`so101_config/visualization.json` controls how the models and the wrist camera are drawn. It is copied from the
package defaults (`src/so101/defaults/visualization.json`) the first time a command needs it; keys missing from your
file are filled in from the defaults. Edit your own file for your setup, and only change the package defaults when
they should change for everyone. To start over, delete your file and it is copied again.

**The file is reloaded while `so101-teleop` and `so101-view-urdf` run**: edit, save, and Rerun updates within half
a second.

| Key | Meaning |
|---|---|
| `joints.<joint>.offset` / `invert` | Maps the real arm to the model: `model = ±real + offset` (degrees, gripper in %). For example `joints.wrist_roll.offset` when the model's wrist zero differs from the real arm. |
| `wrist_camera.parent_link`, `xyz_m`, `rpy_deg` | Where the camera is mounted (default on `gripper_link`). |
| `wrist_camera.roll_about_view_axis_deg` | Rotates the camera (and its image) around its viewing direction, to match the camera's zero orientation to the wrist. |
| `wrist_camera.fov_y_deg` | Vertical field of view used for the projection. |
| `wrist_camera.image_plane_distance_m` | How far from the camera the image is drawn. |
| `leader_model_offset_m` | Where the leader model is drawn next to the follower. |
