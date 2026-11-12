# SO-101 core 

Installable Python package (`so101`, in `src/so101/`) to set up, calibrate, teleoperate and visualize the SO-101
leader and follower arms, built on [lerobot](https://github.com/huggingface/lerobot) and
[Rerun](https://rerun.io). Other projects (RL, imitation learning, simulation) install it to share the same setup.

| Command | What it does |
|---|---|
| `uv run so101-setup` | Detect ports and camera, calibrate the arms, write the config (offers to fetch the URDFs) |
| `uv run so101-fetch-urdfs` | Download or update the SO-101 URDFs and meshes |
| `uv run so101-view-urdf` | Show the URDF models in Rerun in the calibration mid stance, to align `visualization.json` (no arm needed) |
| `uv run so101-teleop` | Teleoperate the follower with the leader, visualized in Rerun (`--no-rerun` to turn it off, `--sim` for the MuJoCo sim) |
| `uv run --extra sim so101-sim` | Open the follower in the MuJoCo viewer with a box to pick up (see [docs/simulation.md](docs/simulation.md)) |

### Visualization with Rerun

All visualization uses [Rerun](https://rerun.io) (the `rerun-sdk` dependency, installed with `uv sync`).
`so101-view-urdf` and `so101-teleop` spawn the Rerun viewer window automatically; no separate install or server is
needed. The viewer shows the 3D models (driven by the URDFs and `visualization.json`), the wrist camera and the joint
plots. `so101-teleop --no-rerun` runs the arms without any visualization.

## Install

In this repository:

```bash
uv sync
```

In another project, install it from a local checkout (changes here are picked up immediately) or from GitHub:

```bash
uv add --editable ../psychic-meme
uv add "so101-core @ git+https://github.com/B-ramB/psychic-meme" --tag v0.1.0
```

## Project config folder

All robot files live in `so101_config/` at the root of the project you run the commands in (the nearest folder with
a `pyproject.toml`; change with the `SO101_HOME` environment variable). Every project gets its own folder:

```
so101_config/
├── urdf/               URDFs + meshes (so101-fetch-urdfs)
├── calibration/        leader / follower calibration files
├── visualization.json  camera pose and joint offsets for the Rerun models
├── arm_config.json     ports, which calibration is used, camera
├── usb_configs/        dated port / camera detection results
└── mujoco/             MuJoCo model generated from the follower URDF (so101-sim)
```

The folder is created with its own `.gitignore` (`*`), so none of these files are committed. To reuse a calibration in
another project, copy its `calibration/` folder over and answer "yes" when `so101-setup` asks whether calibration
files are present.

## Quick start

It is assumed that the robot is built and the motors are configured (`lerobot-setup-motors`).

```bash
uv sync
uv run so101-setup          # both arms powered and connected over USB
uv run so101-view-urdf      # optional: align the models with the real arm
uv run so101-teleop
```

### Optional: serial port access without sudo
<details>
The arms show up as serial ports (`/dev/ttyACM*`) that belong to the `dialout` group. If your user is not in that
group, `so101-setup` and `so101-teleop` run `sudo chmod 666 <port>` for you, which asks for your password and has
to be repeated after every replug or reboot. To avoid that, add yourself to the group once:

```bash
sudo usermod -aG dialout $USER
```

Then log out and back in (or reboot): group membership only applies to new sessions. `groups` should now list
`dialout`. Both ways work; the commands only fall back to `sudo chmod` when you have no access, and tell you when
you are in the group but still need to log in again.
</details>

## Set up the arms: `so101-setup`

```bash
uv run so101-setup
```

0. Offers to download the URDFs into `so101_config/urdf/` when they are missing.
1. Asks whether calibration files are already present in `so101_config/calibration/`.
   - **No**: both arms are calibrated after port detection.
   - **Yes**: you pick the existing leader and follower files and only the USB connections are set up.
     Use this after a reboot or after replugging the arms, when the ports may have changed.
2. Detects the port of each arm: you unplug and replug the leader, then the follower (as with `lerobot-find-port`).
   Without port access it runs `sudo chmod 666 <port>` (not needed in the `dialout` group, see above).
3. Detects the USB camera and lets you choose one (or skip). Ports and camera are saved to
   `so101_config/usb_configs/usb_ports_<YYYY-MM-DD>.json`. When you run the setup again on the same day, it offers
   to reuse that file and skips steps 2 and 3 (only the port access is checked again).
4. Calibrates the arms if needed, saved as `so101_config/calibration/leader_arm_<YYYY-MM-DD>.json` and
   `follower_arm_<YYYY-MM-DD>.json`. During calibration you hold each arm in the **middle of its range of motion**:
   this is the mid stance used by `so101-view-urdf`.
5. Writes the ports, calibration ids and camera to `so101_config/arm_config.json`, offers to delete unused
   calibration and USB files, and prints the equivalent `lerobot-teleoperate` command.

<details>
<summary>Manual steps (reference)</summary>

From the Hugging Face [SO-101 documentation](https://huggingface.co/docs/lerobot/main/en/so101?example=Linux&calibrate_leader=Command#calibrate):

```bash
sudo chmod 666 /dev/ttyACM0 /dev/ttyACM1
uv run lerobot-find-port
uv run lerobot-find-cameras opencv

uv run lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm_<date> \
    --teleop.calibration_dir=so101_config/calibration

uv run lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm_<date> \
    --robot.calibration_dir=so101_config/calibration
```
</details>

## URDFs: `so101-fetch-urdfs`

```bash
uv run so101-fetch-urdfs
```

Syncs the SO-101 URDFs and meshes from the lerobot Hugging Face bucket into `so101_config/urdf/` (only changed files
are downloaded). Use `--bucket hf://buckets/<namespace>/<bucket>/<prefix>` or the `URDF_BUCKET` environment variable
for another source.

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

The same mapping is used by `so101-teleop`, so the models then follow the real arms correctly.

## Teleoperate: `so101-teleop`

```bash
uv run so101-teleop
```

Reads `so101_config/arm_config.json`, lets the follower copy the leader, and opens the Rerun viewer with:

- the follower and leader models following the real arms (needs the URDFs, see `so101-fetch-urdfs`)
- the wrist camera as a frustum on the follower gripper, with the live image projected into the 3D view
- the leader and follower joint positions as time series, in radians (the so101 standard, see
  [docs/units-and-joints.md](docs/units-and-joints.md))

The follower is wrapped in `SafeFollower`: every target is clamped to the soft joint limits in
`so101_config/safety.json` and moves at most `max_step` per frame. See [docs/safety.md](docs/safety.md), also for what
it does not protect against.

| Option | Effect |
|---|---|
| `--fps 60` | Control loop frequency (default 30). |
| `--sim` | Drive the MuJoCo sim instead of the real follower; only the leader is needed. Needs the `sim` extra: `uv run --extra sim so101-teleop --sim`. See [docs/simulation.md](docs/simulation.md#teleoperate-the-sim-with-the-leader-arm). |
| `--no-camera` | Don't open the wrist camera. |
| `--no-urdf` | Don't draw the URDF models in Rerun (camera and joint plots only). |
| `--no-rerun` | Teleoperate without Rerun: no viewer window. The wrist camera is not opened either, since it is only used for the visualization. `visualization.json` is still read for the joint mapping. |

Stop with `Ctrl+C`. If an arm was replugged and its port changed, it tells you to run `so101-setup` again.

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
