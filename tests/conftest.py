from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def pytest_addoption(parser):
    parser.addoption('--run-integration', action='store_true', help='运行 ROS2 集成测试')
    parser.addoption('--navigation-goal', nargs=3, type=float, default=None,
                     help='显式启用真实仿真导航，输入 x y yaw_deg')


def pytest_collection_modifyitems(config, items):
    if not config.getoption('--run-integration'):
        for item in items:
            if 'integration' in item.keywords:
                item.add_marker(pytest.mark.skip(reason='需要 --run-integration 和运行中的 ROS2 环境'))
