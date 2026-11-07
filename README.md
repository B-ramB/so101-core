# psychic-meme

## Init workspace
To initialise the workspace, run the following command to install all dependencies in a uv enviroment.

```bash
uv sync
```

After this you can run any python code through ``` uv run [file_name]```
In the following commands uv is always added.

## URDFS
Get latest SO-101 URDFS  through:

```bash
setup/fetch_urdfs.sh
```

## Setup the SO-101 ARM

For this section it is assumed that you already have the robot built and the motors configured.

Run the interactive setup script with both arms powered and connected over USB:

```bash
uv run setup/setup_arms.py
```

The script:

1. Asks whether calibration files are already present in `setup/calibration_files`.
   - **No**: both arms are calibrated after port detection.
   - **Yes**: you pick the existing leader/follower files and only the USB connections are set up.
     Use this after a reboot or after replugging the arms, when the ports may have changed.
2. Detects the port of each arm. You unplug and replug the leader, then the follower, as with `lerobot-find-port`.
   If the port is not accessible, it runs `sudo chmod 666 <port>` (you may be asked for your password).
3. Detects the USB camera and lets you choose one (or skip).
   The ports and camera are saved to `setup/usb_configs/usb_ports_<YYYY-MM-DD>.json`. When you run the
   script again on the same day, it asks whether to reuse that file and then skips steps 2 and 3
   (only the port permissions are re-checked).
4. Calibrates the arms if needed. The files are saved as
   `setup/calibration_files/leader_arm_<YYYY-MM-DD>.json` and `follower_arm_<YYYY-MM-DD>.json`.
5. Writes the ports, calibration ids and camera to `setup/arm_config.json` and prints the
   `lerobot-teleoperate` command to use.

### Manual steps (reference)
<details>
These steps are originally from the huggingface [documentation](https://huggingface.co/docs/lerobot/main/en/so101?example=Linux&calibrate_leader=Command#calibrate)


```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
uv run lerobot-find-port
uv run lerobot-find-cameras opencv

uv run lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm_<date> \
    --teleop.calibration_dir=setup/calibration_files

uv run lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm_<date> \
    --robot.calibration_dir=setup/calibration_files
```
</details>

## Tele-operate

After running `setup/setup_arms.py`, start teleoperation with Rerun visualization using one command:

```bash
uv run src/teleoperate.py
```

It reads `setup/arm_config.json` (ports, calibration files, camera) and opens Rerun with:

- the follower and leader URDF models following the real arms (when `urdf/` is present; get it with `setup/fetch_urdfs.sh`)
- the wrist camera as a frustum on the follower gripper, with the live image projected into the 3D view
- the joint positions of leader and follower as time series

Options: `--fps 60`, `--no-camera`, `--no-urdf`. Stop with `Ctrl+C`.

If an arm was replugged and its port changed, the script tells you to run `setup/setup_arms.py` again.

### Aligning the camera and joints in Rerun

How the models and the wrist camera are drawn is set in `setup/visualization.local.json`. This file is git-ignored
and is copied from the tracked template `setup/visualization.example.json` on first setup (by `setup/setup_arms.py`,
or by `src/teleoperate.py` if it is still missing). Edit the local file for your own setup; only change the example
when the defaults for everyone should change. To start over, delete the local file and it is copied again.

**The file is reloaded while teleoperating**: edit, save, and Rerun updates within half a second.

- `wrist_camera.roll_about_view_axis_deg`: rotates the camera (and its image) around its viewing direction. Use this
  to match the camera's zero orientation to the wrist.
- `wrist_camera.xyz_m` / `rpy_deg` / `parent_link`: where the camera is mounted (default on `gripper_link`).
- `wrist_camera.fov_y_deg`: vertical field of view used for the projection.
- `joints.<joint>.offset` / `invert`: maps the real arm to the URDF model (`model = ±real + offset`, degrees, gripper
  in %). Use `joints.wrist_roll.offset` when the model's wrist zero differs from the real arm.
- `leader_model_offset_m`: where the leader model is drawn next to the follower.
