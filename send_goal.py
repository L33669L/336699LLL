"""兼容原来的 python3 send_goal.py x y yaw；这里只负责命令行与报告。"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import signal
import uuid

from navigation.config import DEFAULT_CONFIG, load_config
from navigation.result import Pose2D


def main(args=None):
    parser = argparse.ArgumentParser(description='发送 Nav2 目标并采集结构化结果，yaw 单位为度')
    parser.add_argument('x', type=float)
    parser.add_argument('y', type=float)
    parser.add_argument('yaw', type=float)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, help='指定 JSON 输出文件（存在则覆盖）')
    options = parser.parse_args(args)
    try:
        config = load_config(options.config)
        goal = Pose2D(options.x, options.y, options.yaw)
    except (ValueError, TypeError, OSError) as exc:
        parser.error(str(exc))

    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from navigation.navigator import Navigator
    # 保持 ROS context 有效，Ctrl+C / SIGTERM 时先有界取消，再释放资源。
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, interrupt)
    node = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Navigator(config)
        report = node.run(goal)
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        output = options.output or (Path(__file__).resolve().parent / 'results'
                                    / f'navigation-{stamp}-{uuid.uuid4().hex[:8]}.json')
        report.write_json(output)
        print(f'执行结果：{report.outcome}；Action：{report.action_status_name}')
        for name, value in report.metrics.items():
            print(f'{name}：位置误差 {value["position_error_m"]:.3f} m，'
                  f'朝向误差 {value["yaw_error_deg"]:.2f}°')
        print(f'传感器记录：{report.sensors}')
        print(f'结构化结果已保存：{output.resolve()}')
        # 退出码仅代表执行/采集完整性；精度验收交给 pytest。
        if report.outcome == 'interrupted':
            return 130
        return 0 if report.outcome == 'succeeded' and not report.errors else 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    raise SystemExit(main())
