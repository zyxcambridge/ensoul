"""
本体感知模块 —— 关节反馈检测

通过读取 G1 关节传感器数据，确认"我的手臂确实在动"。
ROS2 话题: /rt/joint_states (sensor_msgs/JointState)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


@dataclass
class BodyCheckResult:
    is_ok: bool
    active_joints: int
    total_joints: int
    score: float


def evaluate_body(positions: List[float], velocities: List[float], threshold: float = 0.8) -> BodyCheckResult:
    """
    判断关节是否有明显运动（速度绝对值 > 阈值的关节数占比）。
    """
    total = len(velocities)
    if total == 0:
        return BodyCheckResult(is_ok=False, active_joints=0, total_joints=0, score=0.0)

    active = sum(1 for v in velocities if abs(v) > 0.01)
    score = active / total
    return BodyCheckResult(
        is_ok=score >= threshold,
        active_joints=active,
        total_joints=total,
        score=score,
    )


if HAS_ROS2:

    class BodyCheckNode(Node):
        def __init__(self) -> None:
            super().__init__("body_check")
            self.latest_result: BodyCheckResult | None = None
            self.sub = self.create_subscription(
                JointState, "/rt/joint_states", self._on_joint, 10
            )

        def _on_joint(self, msg: JointState) -> None:
            self.latest_result = evaluate_body(
                list(msg.position), list(msg.velocity)
            )
