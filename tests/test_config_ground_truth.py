import subprocess
import pytest
from navigation.config import NavigationConfig, load_config
from navigation.ground_truth import parse_ground_truth, ground_truth


@pytest.mark.parametrize('value', [0, -1, float('inf'), float('nan'), True, '10'])
def test_timeout_validation(value):
    with pytest.raises(ValueError):
        NavigationConfig(navigation_timeout_sec=value)


def test_config_file():
    config = load_config()
    assert config.position_tolerance_m == 0.25
    assert config.yaw_tolerance_deg == 15.0


def test_ground_truth_model_and_omitted_zeros():
    raw = 'pose { name: "other" position { x: 99 } orientation { w: 1 } }'
    raw += 'pose { name: "burger" position { x: -1.2e-1 } orientation { z: 1 w: 0 } }'
    pose = parse_ground_truth(raw)
    assert pose['x'] == pytest.approx(-0.12)
    assert pose['y'] == 0
    assert abs(pose['yaw']) == pytest.approx(3.141592653589793)
    with pytest.raises(ValueError):
        parse_ground_truth(raw, 'missing')


def test_gz_timeout_propagates(monkeypatch):
    def timeout(command, **kwargs):
        assert kwargs['timeout'] == 0.5
        raise subprocess.TimeoutExpired(command, 0.5)
    monkeypatch.setattr('navigation.ground_truth.subprocess.check_output', timeout)
    with pytest.raises(subprocess.TimeoutExpired):
        ground_truth(timeout=0.5)


def test_ground_truth_omitted_quaternion_w():
    raw = 'pose { name: "burger" position { x: 1 } orientation { z: 1 } }'
    assert abs(parse_ground_truth(raw)['yaw']) == pytest.approx(3.141592653589793)
    with pytest.raises(ValueError):
        parse_ground_truth('pose { name: "burger" position {} orientation {} }')
