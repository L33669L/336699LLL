"""运行真实 ROS2：默认不执行，导航需额外显式指定目标。"""
from dataclasses import replace
import pytest

pytestmark = pytest.mark.integration


@pytest.fixture
def navigator():
    import rclpy
    from navigation.config import load_config
    from navigation.navigator import Navigator
    rclpy.init()
    node = None
    try:
        node = Navigator(load_config())
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_live_pose_and_sensors(navigator):
    from navigation.result import NavigationResult, Pose2D
    navigator._spin_for(2.0)
    report = NavigationResult(Pose2D(0, 0, 0))
    navigator.pose_reader.capture(report)
    assert not report.errors, report.errors
    assert report.tf_pose is not None and report.gazebo_pose is not None
    assert all(v['messages_during_navigation'] > 0 for v in navigator.sensors.snapshot().values())


def test_missing_action_server(navigator):
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from navigation.result import Pose2D
    navigator.action_client.destroy()
    navigator.action_client = ActionClient(navigator, NavigateToPose, '/p2_test_missing_action')
    navigator.config = replace(navigator.config, server_timeout_sec=0.2, warmup_sec=0.1)
    report = navigator.run(Pose2D(0, 0, 0))
    assert report.outcome == 'server_timeout'
    assert report.elapsed_sec < 3.0
    assert not report.cancel_requested


def test_navigation_result(navigator, request):
    from action_msgs.msg import GoalStatus
    from navigation.result import Pose2D
    from pathlib import Path
    goal = request.config.getoption('--navigation-goal')
    if goal is None:
        pytest.skip('导航会改变机器人位置；需 --navigation-goal x y yaw')
    report = navigator.run(Pose2D(*goal))
    report.write_json(Path(__file__).resolve().parents[1] / 'results'
                      / ('integration-' + report.goal_id + '.json'))
    assert report.outcome == 'succeeded', report.to_dict()
    assert report.action_status == GoalStatus.STATUS_SUCCEEDED
    assert not report.errors, report.errors
    # 仅验收 Goal vs TF；定位误差和真实到点误差记录，不擅自套用同一阈值。
    errors = report.metrics['goal_vs_tf']
    assert errors['position_error_m'] <= navigator.config.position_tolerance_m
    assert errors['yaw_error_deg'] <= navigator.config.yaw_tolerance_deg

