"""The real SO-101 leader and follower: lerobot's devices in model radians, the edge where lerobot units exist.

lerobot reads and writes the arms in its own units (body joints in degrees, gripper in percent). so101 works in model
radians (see docs/units-and-joints.md). `RealLeader` and `RealFollower` wrap the lerobot devices, keep their
keys (`"<joint>.pos"`, camera names) and convert the joint values with a `JointCalibration`. They implement
`so101.robot.Leader` / `Follower`, like the sim's `MujocoFollower`.

`calibration` is a callable so the mapping can follow live edits of visualization.json, e.g.
`VisualizationConfig().joint_calibration`.
"""

from collections.abc import Callable

from so101.conventions import JointCalibration, JointPositions

SUFFIX = ".pos"


def _to_radians(values: dict, calibration: JointCalibration) -> dict:
    """lerobot dict -> the same dict with the `.pos` values in radians; other keys (camera images) pass through."""
    degrees = {key.removesuffix(SUFFIX): value for key, value in values.items() if key.endswith(SUFFIX)}
    radians = JointPositions.from_degrees(degrees, calibration).radians
    return {key: radians[key.removesuffix(SUFFIX)] if key.endswith(SUFFIX) else value for key, value in values.items()}


def _to_degrees(values: dict, calibration: JointCalibration) -> dict:
    """Radians dict (`"<joint>.pos"`) -> lerobot units."""
    radians = {key.removesuffix(SUFFIX): value for key, value in values.items() if key.endswith(SUFFIX)}
    degrees = JointPositions(radians, calibration).to_degrees()
    return {key: degrees[key.removesuffix(SUFFIX)] if key.endswith(SUFFIX) else value for key, value in values.items()}


class RealLeader:
    """The real leader arm (a lerobot teleoperator) as a `Leader`: `get_action()` returns model radians."""

    def __init__(self, teleop, calibration: Callable[[], JointCalibration]):
        self.teleop = teleop
        self.calibration = calibration

    @property
    def action_features(self) -> dict:
        return self.teleop.action_features

    @property
    def is_connected(self) -> bool:
        return self.teleop.is_connected

    def connect(self) -> None:
        self.teleop.connect()

    def disconnect(self) -> None:
        self.teleop.disconnect()

    def get_action(self) -> dict:
        return _to_radians(self.teleop.get_action(), self.calibration())


class RealFollower:
    """The real follower arm (a lerobot robot) as a `Follower`: takes and reports model radians, like `MujocoFollower`."""

    def __init__(self, robot, calibration: Callable[[], JointCalibration]):
        self.robot = robot
        self.calibration = calibration

    @property
    def observation_features(self) -> dict:
        return self.robot.observation_features

    @property
    def action_features(self) -> dict:
        return self.robot.action_features

    @property
    def is_connected(self) -> bool:
        return self.robot.is_connected

    def connect(self) -> None:
        self.robot.connect()

    def disconnect(self) -> None:
        self.robot.disconnect()

    def get_observation(self) -> dict:
        """Joint positions in radians (`"<joint>.pos"`), camera images as lerobot returns them."""
        return _to_radians(self.robot.get_observation(), self.calibration())

    def send_action(self, action: dict) -> dict:
        """Send joint targets in radians; returns the action lerobot actually sent, in radians."""
        calibration = self.calibration()
        sent = self.robot.send_action(_to_degrees(action, calibration))
        return _to_radians(sent, calibration)
