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

Use the command printed by `setup/setup_arms.py`. It looks like this:

```bash
uv run lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm_<date> \
    --robot.calibration_dir=setup/calibration_files \
    --robot.cameras="{ front: {type: opencv, index_or_path: /dev/video0, width: 640, height: 480, fps: 30}}" \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm_<date> \
    --teleop.calibration_dir=setup/calibration_files \
    --display_data=true
```


## Rerun

Visualize the urdf in rerun:

```bash
uv run src/urdf_visualizer.py
```
