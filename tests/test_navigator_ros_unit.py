"""真实 ROS2 消息类型 + 模拟通信，不需要运行 Nav2/Gazebo。"""
from dataclasses import replace
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import time
import pytest
pytest.importorskip('rclpy', reason='ROS 消息模拟测试需 source env.sh；纯函数测试不需要')
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from navigation.config import NavigationConfig
from navigation.navigator import Navigator
from navigation.result import Pose2D
pytestmark = pytest.mark.ros_unit


class Future:

    def __init__(self, value=None, done=True):
        self.value = value
        self.ready = done

    def done(self):
        return self.ready

    def result(self):
        return self.value


def harness(status=GoalStatus.STATUS_SUCCEEDED, accepted=True, result_ready=True):
    rr = NS(status=status, result=NS(error_code=0, error_msg=''))
    result_future = Future(rr, result_ready)
    handle = NS(accepted=accepted, get_result_async=lambda: result_future)
    send = Future(handle)
    node = NS(
        config=NavigationConfig(),
        get_logger=lambda: Mock(),
        get_clock=lambda: NS(now=lambda: NS(to_msg=Time)),
        _spin_for=Mock(),
        sensors=NS(begin=Mock(), snapshot=lambda: {'scan': {}}),
        pose_reader=NS(capture=Mock()),
        action_client=NS(
            wait_for_server=Mock(return_value=True),
            send_goal_async=Mock(return_value=send),
        ),
        cancel_client=NS(wait_for_service=Mock(return_value=True)),
    )

    def ack():
        return Future(NS(goals_canceling=[NS(goal_id=node.goal_uuid)]))
    handle.cancel_goal_async = Mock(side_effect=ack)
    node.cancel_client.call_async = Mock(side_effect=lambda request: ack())
    node._wait = lambda future, seconds: future.done()
    for name in ('run', '_cancel', '_record_terminal'):
        setattr(node, name, MethodType(getattr(Navigator, name), node))
    return node, send, result_future, handle


@pytest.mark.parametrize('status, outcome', [
    (GoalStatus.STATUS_SUCCEEDED, 'succeeded'),
    (GoalStatus.STATUS_ABORTED, 'aborted'),
    (GoalStatus.STATUS_CANCELED, 'canceled'),
], ids=['导航成功', '导航中止', '导航取消'])
def test_terminal_results(status, outcome):
    node, _, _, handle = harness(status)
    report = node.run(Pose2D(0, 0, 0))
    assert report.outcome == outcome
    assert report.terminal_confirmed
    node.pose_reader.capture.assert_called_once()
    handle.cancel_goal_async.assert_not_called()


def test_rejected_goal_does_not_sample():
    node, _, _, _ = harness(accepted=False)
    report = node.run(Pose2D(0, 0, 0))
    assert report.outcome == 'rejected'
    assert not report.cancel_requested
    node.pose_reader.capture.assert_not_called()


def test_navigation_timeout_cancel_and_terminal_confirmation():
    node, _, result, handle = harness(GoalStatus.STATUS_CANCELED, result_ready=False)
    original = handle.cancel_goal_async.side_effect

    def cancel():
        result.ready = True
        return original()
    handle.cancel_goal_async.side_effect = cancel
    report = node.run(Pose2D(0, 0, 0))
    assert report.outcome == 'navigation_timeout'
    assert report.timeout_stage == 'navigation'
    assert report.cancel_acknowledged
    assert report.terminal_confirmed


def test_cancel_ack_is_not_terminal_confirmation():
    node, _, _, _ = harness(result_ready=False)
    report = node.run(Pose2D(0, 0, 0))
    assert report.cancel_acknowledged
    assert not report.terminal_confirmed
    assert report.errors
    node.pose_reader.capture.assert_not_called()


def test_late_goal_response_is_canceled():
    node, send, _, handle = harness(GoalStatus.STATUS_CANCELED)
    calls = 0

    def wait(future, seconds):
        nonlocal calls
        if future is send:
            calls += 1
            return calls > 1
        return future.done()
    node._wait = wait
    report = node.run(Pose2D(0, 0, 0))
    assert report.outcome == 'goal_response_timeout'
    assert report.cancel_acknowledged
    assert report.terminal_confirmed
    handle.cancel_goal_async.assert_called_once()


def test_missing_goal_response_cancels_only_own_uuid():
    node, send, _, _ = harness()
    send.ready = False
    report = node.run(Pose2D(0, 0, 0))
    request = node.cancel_client.call_async.call_args.args[0]
    assert list(request.goal_info.goal_id.uuid) == list(node.goal_uuid.uuid)
    assert any(request.goal_info.goal_id.uuid)
    assert report.cancel_acknowledged
    assert not report.terminal_confirmed


def test_cancel_rejected_is_recorded():
    node, _, _, handle = harness(result_ready=False)
    handle.cancel_goal_async.side_effect = lambda: Future(NS(goals_canceling=[]))
    report = node.run(Pose2D(0, 0, 0))
    assert not report.cancel_acknowledged
    assert not report.terminal_confirmed
    assert report.errors


def test_keyboard_interrupt_cancels():
    node, send, result, _ = harness(GoalStatus.STATUS_CANCELED)
    interrupted = False

    def wait(future, seconds):
        nonlocal interrupted
        if future is result and (not interrupted):
            interrupted = True
            raise KeyboardInterrupt
        return future.done()
    node._wait = wait
    report = node.run(Pose2D(0, 0, 0))
    assert report.outcome == 'interrupted'
    assert report.cancel_requested
    assert report.terminal_confirmed


def test_real_wait_has_wall_clock_deadline(monkeypatch):
    node = NS(config=replace(NavigationConfig(), spin_period_sec=0.005))
    monkeypatch.setattr('navigation.navigator.rclpy.ok', lambda: True)
    monkeypatch.setattr('navigation.navigator.rclpy.spin_once', lambda node, timeout_sec: time.sleep(timeout_sec))
    start = time.monotonic()
    assert not Navigator._wait(node, Future(done=False), 0.03)
    elapsed = time.monotonic() - start
    assert 0.02 <= elapsed < 0.5
