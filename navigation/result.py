"""可由 pytest、JSON 报告和日志复用的数据对象，不依赖 ROS2。"""
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import json
from pathlib import Path

from .metrics import finite, position_error, yaw_error


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw_deg: float

    def __post_init__(self):
        finite(self.x, self.y, self.yaw_deg)


@dataclass
class NavigationResult:
    goal: Pose2D
    schema_version: int = 1
    created_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    outcome: str = 'not_started'
    action_status: int | None = None
    action_status_name: str | None = None
    action_error_code: int | None = None
    action_error_message: str = ''
    elapsed_sec: float = 0.0
    timeout_stage: str | None = None
    cancel_requested: bool = False
    cancel_acknowledged: bool = False
    terminal_confirmed: bool = False
    goal_id: str | None = None
    tf_pose: Pose2D | None = None
    gazebo_pose: Pose2D | None = None
    metrics: dict = field(default_factory=dict)
    sensors: dict = field(default_factory=dict)
    sampling: dict = field(default_factory=dict)
    config: dict = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    def calculate_metrics(self):
        self.metrics.clear()
        for name, a, b in (
            ('goal_vs_tf', self.goal, self.tf_pose),
            ('tf_vs_gazebo', self.tf_pose, self.gazebo_pose),
            ('goal_vs_gazebo', self.goal, self.gazebo_pose),
        ):
            if a is not None and b is not None:
                self.metrics[name] = {
                    'position_error_m': position_error(a.x, a.y, b.x, b.y),
                    'yaw_error_deg': yaw_error(a.yaw_deg, b.yaw_deg),
                }

    def to_dict(self):
        return asdict(self)

    def write_json(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        # 禁止 NaN/Infinity 混入报告；每次 CLI 运行使用唯一文件名。
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2,
                                   allow_nan=False) + '\n', encoding='utf-8')
