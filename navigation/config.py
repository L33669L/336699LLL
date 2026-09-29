"""读取测试工具配置；不会修改 Nav2 baseline 参数。"""
from dataclasses import asdict, dataclass, fields
import math
from pathlib import Path
import yaml

DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'test_config.yaml'


@dataclass(frozen=True)
class NavigationConfig:
    action_name: str = 'navigate_to_pose'
    map_frame: str = 'map'
    base_frame: str = 'base_link'
    gazebo_topic: str = '/world/default/dynamic_pose/info'
    gazebo_model: str = 'burger'
    use_sim_time: bool = True
    server_timeout_sec: float = 10.0
    goal_response_timeout_sec: float = 10.0
    navigation_timeout_sec: float = 150.0
    cancel_timeout_sec: float = 5.0
    cancel_result_timeout_sec: float = 10.0
    gazebo_timeout_sec: float = 20.0
    tf_timeout_sec: float = 3.0
    warmup_sec: float = 1.0
    settle_sec: float = 0.5
    spin_period_sec: float = 0.05
    position_tolerance_m: float = 0.25
    yaw_tolerance_deg: float = 15.0

    def __post_init__(self):
        for f in fields(self):
            v = getattr(self, f.name)
            if f.type is float:
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
                    raise ValueError(f'{f.name} 必须是有限正数')
            elif f.type is str and (not isinstance(v, str) or not v.strip()):
                raise ValueError(f'{f.name} 必须是非空字符串')
        if not isinstance(self.use_sim_time, bool):
            raise ValueError('use_sim_time 必须是布尔值')

    def to_dict(self):
        return asdict(self)


def load_config(path=DEFAULT_CONFIG):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('配置必须是 YAML 对象')
    return NavigationConfig(**data)
