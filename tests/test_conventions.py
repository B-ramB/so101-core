import pytest

from so101.conventions import gripper_percent_to_joint, real_to_model


def test_real_to_model_identity():
    assert real_to_model(12.5) == 12.5


def test_real_to_model_offset():
    assert real_to_model(10.0, offset=-90.0) == -80.0


def test_real_to_model_invert():
    assert real_to_model(10.0, invert=True) == -10.0


def test_real_to_model_invert_then_offset():
    # model = sign * real + offset: the offset is applied after the sign flip.
    assert real_to_model(10.0, offset=5.0, invert=True) == -5.0


@pytest.mark.parametrize(
    ("percent", "expected"),
    [(0.0, -0.2), (50.0, 0.65), (100.0, 1.5)],
)
def test_gripper_percent_to_joint(percent, expected):
    assert gripper_percent_to_joint(percent, lower=-0.2, upper=1.5) == pytest.approx(expected)
