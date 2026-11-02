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
/scripts/fetch_urdfs.sh
```

## Rerun

Visualize the urdf in rerun:

```bash
uv run src/urdf_visualizer.py
```


## Configuring the SO-101 ARM

For this section it is assumed that you already have the robot build and configured. Only the tuning is described here for this process.

On linux make sure you have access to the USB ports by running:

```bash
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

After run the below command to find each component. If you have a leader and follower arm, you run this twice to find each.

```bash
uv run lerobot-find-port
```



```bash

```

```bash
uv run lerobot-calibrate \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm_0
```

```bash
uv run lerobot-calibrate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm_0
```

## Tele-operate

```bash
uv run lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm_0 \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm_0
```