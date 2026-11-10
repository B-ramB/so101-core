"""RealLeader / RealFollower around lerobot's leader and follower, with fake lerobot devices (no hardware)."""

import math

import numpy as np
import pytest

from so101.conventions import GRIPPER_LOWER, GRIPPER_UPPER, JOINTS, JointCalibration, gripper_percent_to_joint
from so101.hardware.adapters import RealFollower, RealLeader
from so101.robot import Follower, Leader

CALIBRATION = JointCalibration({"wrist_roll": {"offset": -90.0}, "elbow_flex": {"invert": True}})
MID_STANCE_DEG = {f"{joint}.pos": 50.0 if joint == "gripper" else 0.0 for joint in JOINTS}


class FakeLeader:
    """lerobot leader: actions in degrees / gripper percent."""

    def __init__(self, action):
        self.action = action
        self.is_connected = False

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def get_action(self):
        return dict(self.action)


class FakeFollower:
    """lerobot follower: takes and reports degrees / gripper percent, and a camera image."""

    def __init__(self):
        self.sent = []
        self.positions = dict(MID_STANCE_DEG)
        self.is_connected = False
        self.image = np.zeros((4, 6, 3), dtype=np.uint8)

    def connect(self):
        self.is_connected = True

    def disconnect(self):
        self.is_connected = False

    def send_action(self, action):
        self.sent.append(action)
        self.positions.update(action)
        return dict(action)

    def get_observation(self):
        return {**self.positions, "wrist": self.image}


def test_leader_mid_stance_in_radians():
    leader = RealLeader(FakeLeader(MID_STANCE_DEG), lambda: CALIBRATION)
    action = leader.get_action()
    assert set(action) == set(MID_STANCE_DEG)
    assert action["wrist_roll.pos"] == pytest.approx(-math.pi / 2)
    assert action["gripper.pos"] == pytest.approx(gripper_percent_to_joint(50.0, GRIPPER_LOWER, GRIPPER_UPPER))
    assert action["shoulder_pan.pos"] == pytest.approx(0.0)


def test_leader_delegates_connection():
    leader = RealLeader(FakeLeader({}), lambda: CALIBRATION)
    leader.connect()
    assert leader.is_connected
    leader.disconnect()
    assert not leader.is_connected


def test_real_follower_gets_the_degrees_the_leader_read():
    """Leader deg -> rad -> follower deg uses the same mapping both ways, so the offsets and inverts cancel."""
    read = {"shoulder_pan.pos": 12.0, "elbow_flex.pos": -30.0, "wrist_roll.pos": 45.0, "gripper.pos": 20.0}
    leader = RealLeader(FakeLeader(read), lambda: CALIBRATION)
    real = FakeFollower()
    follower = RealFollower(real, lambda: CALIBRATION)
    sent = follower.send_action(leader.get_action())
    assert real.sent[0] == pytest.approx(read)
    assert sent == pytest.approx(leader.get_action())


def test_follower_partial_action():
    real = FakeFollower()
    RealFollower(real, lambda: CALIBRATION).send_action({"wrist_flex.pos": math.radians(20)})
    assert real.sent == [{"wrist_flex.pos": pytest.approx(20.0)}]


def test_follower_observation_in_radians_with_the_camera_image():
    real = FakeFollower()
    obs = RealFollower(real, lambda: CALIBRATION).get_observation()
    assert obs["wrist_roll.pos"] == pytest.approx(-math.pi / 2)
    assert obs["elbow_flex.pos"] == pytest.approx(0.0)
    assert obs["wrist"] is real.image


def test_calibration_is_read_on_every_call():
    calibrations = [JointCalibration(), CALIBRATION]
    leader = RealLeader(FakeLeader({"wrist_roll.pos": 0.0}), lambda: calibrations[-1])
    assert leader.get_action()["wrist_roll.pos"] == pytest.approx(-math.pi / 2)
    calibrations.pop()
    assert leader.get_action()["wrist_roll.pos"] == pytest.approx(0.0)


def test_implements_the_interfaces():
    assert isinstance(RealLeader(FakeLeader({}), JointCalibration), Leader)
    assert isinstance(RealFollower(FakeFollower(), JointCalibration), Follower)
