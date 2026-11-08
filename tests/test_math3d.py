import math

import pytest

from so101.math3d import quat_multiply, rpy_to_quat_xyzw

IDENTITY = [0.0, 0.0, 0.0, 1.0]
S = math.sqrt(0.5)


@pytest.mark.parametrize(
    ("rpy", "expected"),
    [
        ((0.0, 0.0, 0.0), IDENTITY),
        ((math.pi / 2, 0.0, 0.0), [S, 0.0, 0.0, S]),
        ((0.0, math.pi / 2, 0.0), [0.0, S, 0.0, S]),
        ((0.0, 0.0, math.pi / 2), [0.0, 0.0, S, S]),
        ((math.pi, 0.0, 0.0), [1.0, 0.0, 0.0, 0.0]),
    ],
)
def test_rpy_to_quat_known_angles(rpy, expected):
    assert rpy_to_quat_xyzw(*rpy) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("rpy", [(0.1, 0.2, 0.3), (-1.0, 0.5, 2.5), (math.pi, -math.pi / 3, 0.7)])
def test_rpy_to_quat_unit_norm(rpy):
    q = rpy_to_quat_xyzw(*rpy)
    assert math.fsum(c * c for c in q) == pytest.approx(1.0)


def test_rpy_matches_extrinsic_xyz_composition():
    # rpy = rotate about X (roll), then Y (pitch), then Z (yaw), all about fixed axes: q = qz * qy * qx.
    roll, pitch, yaw = 0.3, -0.4, 1.1
    qx = rpy_to_quat_xyzw(roll, 0.0, 0.0)
    qy = rpy_to_quat_xyzw(0.0, pitch, 0.0)
    qz = rpy_to_quat_xyzw(0.0, 0.0, yaw)
    assert quat_multiply(qz, quat_multiply(qy, qx)) == pytest.approx(rpy_to_quat_xyzw(roll, pitch, yaw))


@pytest.mark.parametrize("q", [[0.1, 0.2, 0.3, 0.927], rpy_to_quat_xyzw(0.4, -0.2, 1.3)])
def test_quat_multiply_identity(q):
    assert quat_multiply(IDENTITY, q) == pytest.approx(q)
    assert quat_multiply(q, IDENTITY) == pytest.approx(q)
