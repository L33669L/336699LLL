import json
import pytest
from navigation.result import NavigationResult, Pose2D


def test_three_metrics_and_json_roundtrip(tmp_path):
    result = NavigationResult(Pose2D(0, 0, 179))
    result.tf_pose, result.gazebo_pose = Pose2D(3, 4, -179), Pose2D(0, 4, 180)
    result.calculate_metrics()
    assert result.metrics['goal_vs_tf'] == {'position_error_m': 5, 'yaw_error_deg': 2}
    assert result.metrics['tf_vs_gazebo']['position_error_m'] == 3
    assert result.metrics['goal_vs_gazebo']['position_error_m'] == 4
    output = tmp_path / 'result.json'
    result.write_json(output)
    assert json.loads(output.read_text()) == result.to_dict()


def test_missing_pose_is_not_zero_error():
    result = NavigationResult(Pose2D(0, 0, 0), outcome='server_timeout')
    result.calculate_metrics()
    assert result.metrics == {}
    assert result.action_status is None
    assert result.tf_pose is None


def test_partial_measurement_and_recalculation():
    result = NavigationResult(Pose2D(0, 0, 0), tf_pose=Pose2D(1, 0, 0))
    result.calculate_metrics()
    assert set(result.metrics) == {'goal_vs_tf'}
    result.tf_pose = None
    result.calculate_metrics()
    assert result.metrics == {}


def test_nonfinite_pose_and_json(tmp_path):
    with pytest.raises(ValueError):
        Pose2D(float('nan'), 0, 0)
    result = NavigationResult(Pose2D(0, 0, 0), elapsed_sec=float('nan'))
    with pytest.raises(ValueError):
        result.write_json(tmp_path / 'bad.json')
