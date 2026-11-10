"""The simulated SO-101 follower, with the interface of lerobot's follower robot.

`MujocoFollower` has the same methods (`connect`, `get_observation`, `send_action`, ...) and the same keys
(`"shoulder_pan.pos"` ... `"gripper.pos"`, and `"wrist"` for the camera image) as lerobot's SO-101 follower, so a
control loop written against one runs against the other. The joint values are **model radians**, the so101 standard
(see docs/units-and-joints.md): the MuJoCo model is generated from the follower URDF, so no conversion is needed.

    with MujocoFollower(viewer=True) as robot:
        robot.send_action({"wrist_flex.pos": 0.5})   # only wrist_flex changes; steps the physics for 1/fps s
        robot.get_observation()                       # {"shoulder_pan.pos": 0.0, ..., "gripper.pos": 0.0}
"""

from pathlib import Path
from typing import Self

import mujoco
import numpy as np

from so101.conventions import CAMERA_NAME, JOINTS
from so101.paths import FOLLOWER_URDF, MUJOCO_DIR
from so101.sim.model import DEFAULT_SCENE, WRIST_CAMERA, home, load_model, wrist_camera_pose

SUFFIX = ".pos"


class MujocoFollower:
    """The SO-101 follower in MuJoCo. Joint positions in radians, keyed `"<joint>.pos"` as in lerobot.

    fps: control rate; each `send_action` advances the simulation by 1/fps s. It does not wait in real time: like
        with the real arm, the caller's loop does the pacing.
    camera: render the wrist camera into the observation as `"wrist"`, an HxWx3 uint8 image of `camera_size` (w, h).
        Its pose comes from so101_config/visualization.json -> wrist_camera, the same as in the Rerun view.
    viewer: open the passive MuJoCo viewer and sync it after every step.
    scene: scene file in `dest` (so101_config/mujoco by default).
    """

    name = "mujoco_so101_follower"

    def __init__(
        self,
        fps: int = 30,
        camera: bool = False,
        viewer: bool = False,
        scene: str = DEFAULT_SCENE,
        camera_size: tuple[int, int] = (640, 480),
        urdf: Path = FOLLOWER_URDF,
        dest: Path = MUJOCO_DIR,
    ):
        self.fps = fps
        self.camera = camera
        self.viewer = viewer
        self.scene = scene
        self.camera_size = camera_size
        self.urdf = urdf
        self.dest = dest
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self._renderer: mujoco.Renderer | None = None
        self._viewer = None
        self._actuators: dict[str, int] = {}  # joint -> actuator id
        self._qpos: dict[str, int] = {}  # joint -> qpos address

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        features: dict[str, type | tuple] = dict(self.action_features)
        if self.camera:
            width, height = self.camera_size
            features[CAMERA_NAME] = (height, width, 3)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}{SUFFIX}": float for joint in JOINTS}

    @property
    def is_connected(self) -> bool:
        return self.data is not None

    def connect(self) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self.name} is already connected.")
        model = load_model(self.scene, self.urdf, self.dest)
        self._apply_camera_config(model)
        self.model, self.data = model, mujoco.MjData(model)
        self._actuators = {joint: model.actuator(joint).id for joint in JOINTS}
        self._qpos = {joint: int(model.joint(joint).qposadr[0]) for joint in JOINTS}
        home(self.model, self.data)
        if self.camera:
            width, height = self.camera_size
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        if self.viewer:
            from mujoco import viewer

            self._viewer = viewer.launch_passive(self.model, self.data)

    def disconnect(self) -> None:
        self._check_connected()
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
        self.model = self.data = None

    def get_observation(self) -> dict:
        """Joint positions in radians (`"<joint>.pos"`), plus the `"wrist"` image when the camera is on."""
        self._check_connected()
        obs: dict = {f"{joint}{SUFFIX}": float(self.data.qpos[adr]) for joint, adr in self._qpos.items()}
        if self._renderer is not None:
            self._renderer.update_scene(self.data, camera=WRIST_CAMERA)
            obs[CAMERA_NAME] = self._renderer.render()
        return obs

    def send_action(self, action: dict) -> dict:
        """Set the position targets (radians) of the given joints, step the physics for 1/fps s.

        A partial action only changes the joints it names; the others keep their target. Targets outside a joint's
        range are clipped to it. Returns the action actually sent, after clipping.
        """
        self._check_connected()
        sent = {}
        for key, value in action.items():
            joint = key.removesuffix(SUFFIX)
            if not key.endswith(SUFFIX) or joint not in self._actuators:
                raise ValueError(f"Unknown action key {key!r}; expected one of {list(self.action_features)}.")
            act = self._actuators[joint]
            lower, upper = self.model.actuator_ctrlrange[act]
            self.data.ctrl[act] = sent[key] = float(np.clip(value, lower, upper))
        steps = max(1, round(1.0 / (self.fps * self.model.opt.timestep)))
        if self._viewer is not None:
            with self._viewer.lock():
                mujoco.mj_step(self.model, self.data, nstep=steps)
            self._viewer.sync()
        else:
            mujoco.mj_step(self.model, self.data, nstep=steps)
        return sent

    def reset(self) -> None:
        """Back to the home pose (every joint at 0 rad), with the actuators holding it. Resets the scene too."""
        self._check_connected()
        if self._viewer is not None:
            with self._viewer.lock():
                home(self.model, self.data)
            self._viewer.sync()
        else:
            home(self.model, self.data)

    def __enter__(self) -> Self:
        self.connect()
        return self

    def __exit__(self, *exc) -> None:
        if self.is_connected:
            self.disconnect()

    def _check_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError(f"{self.name} is not connected; call connect() first.")

    def _apply_camera_config(self, model: mujoco.MjModel) -> None:
        """Move the wrist camera to the pose in the user's visualization.json, if the camera is in use."""
        if not self.camera:
            return
        from so101.viz.config import VisualizationConfig

        cam = VisualizationConfig().camera
        camera = model.camera(WRIST_CAMERA)
        if model.body(camera.bodyid[0]).name != cam["parent_link"]:
            raise ValueError(
                f"visualization.json mounts the wrist camera on {cam['parent_link']!r}; the sim only supports "
                f"{model.body(camera.bodyid[0]).name!r}."
            )
        pos, quat, fovy = wrist_camera_pose(cam)
        model.cam_pos[camera.id] = pos
        model.cam_quat[camera.id] = quat
        model.cam_fovy[camera.id] = fovy
