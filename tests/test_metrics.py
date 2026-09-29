import math
import pytest
from navigation.metrics import position_error, yaw_error, quaternion_to_yaw


@pytest.mark.parametrize('a,b,expected', [(0,0,0), (179,-179,2), (-179,179,2),
                                         (0,180,180), (720,0,0), (-10,350,0)])
def test_yaw_wrap(a, b, expected):
    assert yaw_error(a, b) == pytest.approx(expected)


def test_position():
    assert position_error(0, 0, 3, 4) == 5
    assert position_error(-2, 3, -2, 3) == 0


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_rejected(bad):
    with pytest.raises(ValueError):
        position_error(bad, 0, 0, 0)
    with pytest.raises(ValueError):
        yaw_error(0, bad)


def test_quaternion_normalization():
    assert quaternion_to_yaw(0, 0, 2, 2) == pytest.approx(90)
    with pytest.raises(ValueError):
        quaternion_to_yaw(0, 0, 0, 0)
