# Setup

It is assumed that the robot is built and the motors are configured (`lerobot-setup-motors`). All files below are
written to `so101_config/` in your project; see [use-in-your-project.md](use-in-your-project.md#project-config-folder).

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
   Without port access it runs `sudo chmod 666 <port>` (not needed in the `dialout` group, see
   [below](#serial-port-access-without-sudo)).
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

## Serial port access without sudo

The arms show up as serial ports (`/dev/ttyACM*`) that belong to the `dialout` group. If your user is not in that
group, `so101-setup` and `so101-teleop` run `sudo chmod 666 <port>` for you, which asks for your password and has
to be repeated after every replug or reboot. To avoid that, add yourself to the group once:

```bash
sudo usermod -aG dialout $USER
```

Then log out and back in (or reboot): group membership only applies to new sessions. `groups` should now list
`dialout`. Both ways work; the commands only fall back to `sudo chmod` when you have no access, and tell you when
you are in the group but still need to log in again.

## URDFs: `so101-fetch-urdfs`

```bash
uv run so101-fetch-urdfs
```

Syncs the SO-101 URDFs and meshes from the lerobot Hugging Face bucket into `so101_config/urdf/` (only changed files
are downloaded). Use `--bucket hf://buckets/<namespace>/<bucket>/<prefix>` or the `URDF_BUCKET` environment variable
for another source.

## Teleoperate: `so101-teleop`

```bash
uv run so101-teleop
```

Reads `so101_config/arm_config.json`, lets the follower copy the leader, and opens the Rerun viewer with:

- the follower and leader models following the real arms (needs the URDFs, see
  [`so101-fetch-urdfs`](#urdfs-so101-fetch-urdfs))
- the wrist camera as a frustum on the follower gripper, with the live image projected into the 3D view
- the leader and follower joint positions as time series, in radians (the so101 standard, see
  [units-and-joints.md](units-and-joints.md))

Align the models with your arms first; see [visualization.md](visualization.md).

The follower is wrapped in `SafeFollower`: every target is clamped to the soft joint limits in
`so101_config/safety.json` and moves at most `max_step` per frame. See [safety.md](safety.md), also for what it does
not protect against.

| Option | Effect |
|---|---|
| `--fps 60` | Control loop frequency (default 30). |
| `--sim` | Drive the MuJoCo sim instead of the real follower; only the leader is needed. Needs the `sim` extra: `uv run --extra sim so101-teleop --sim`. See [simulation.md](simulation.md#teleoperate-the-sim-with-the-leader-arm). |
| `--no-camera` | Don't open the wrist camera. |
| `--no-urdf` | Don't draw the URDF models in Rerun (camera and joint plots only). |
| `--no-rerun` | Teleoperate without Rerun: no viewer window. The wrist camera is not opened either, since it is only used for the visualization. `visualization.json` is still read for the joint mapping. |

Stop with `Ctrl+C`. If an arm was replugged and its port changed, it tells you to run `so101-setup` again.
