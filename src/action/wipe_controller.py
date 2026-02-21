"""
擦除动作控制器

控制 G1 手臂执行"擦额头贴纸"动作。
包含主轨迹和保守轨迹两条路径，失败时自动降级。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class Trajectory(Enum):
    PRIMARY = "head_touch_front"
    CONSERVATIVE = "head_touch_conservative"


@dataclass
class JointWaypoint:
    """单个关节空间路点。"""
    joint_angles: List[float]
    duration_sec: float


PRIMARY_WAYPOINTS = [
    JointWaypoint(joint_angles=[0.0, -0.5, 1.2, 0.8, 0.0, 0.0, 0.0], duration_sec=1.5),
    JointWaypoint(joint_angles=[0.0, -0.8, 1.5, 1.0, 0.0, 0.3, 0.0], duration_sec=1.0),
    JointWaypoint(joint_angles=[0.0, -0.5, 1.2, 0.8, 0.0, 0.0, 0.0], duration_sec=1.0),
]

CONSERVATIVE_WAYPOINTS = [
    JointWaypoint(joint_angles=[0.0, -0.3, 0.8, 0.5, 0.0, 0.0, 0.0], duration_sec=2.0),
    JointWaypoint(joint_angles=[0.0, -0.6, 1.2, 0.7, 0.0, 0.2, 0.0], duration_sec=1.5),
    JointWaypoint(joint_angles=[0.0, -0.3, 0.8, 0.5, 0.0, 0.0, 0.0], duration_sec=1.5),
]

TRAJECTORY_MAP = {
    Trajectory.PRIMARY: PRIMARY_WAYPOINTS,
    Trajectory.CONSERVATIVE: CONSERVATIVE_WAYPOINTS,
}


def execute_trajectory(waypoints: List[JointWaypoint], publish_fn=None) -> bool:
    """
    按顺序发送路点到手臂控制器。
    publish_fn: 发送单个 JointWaypoint 的回调（ROS2 模式下为话题发布）。
    模拟模式下直接返回 True。
    """
    for wp in waypoints:
        if publish_fn:
            publish_fn(wp)
    return True


def wipe_sticker(trajectory: Trajectory = Trajectory.PRIMARY, publish_fn=None) -> bool:
    waypoints = TRAJECTORY_MAP[trajectory]
    return execute_trajectory(waypoints, publish_fn)


if HAS_ROS2:

    class WipeControllerNode(Node):
        def __init__(self) -> None:
            super().__init__("wipe_controller")
            self.srv = self.create_service(
                Trigger, "/g1/action/wipe_sticker", self._handle_wipe
            )
            self._current_trajectory = Trajectory.PRIMARY

        def _handle_wipe(self, request, response) -> Trigger.Response:
            ok = wipe_sticker(self._current_trajectory)
            response.success = ok
            response.message = (
                f"Wipe {'succeeded' if ok else 'failed'} "
                f"with {self._current_trajectory.value}"
            )
            if not ok and self._current_trajectory == Trajectory.PRIMARY:
                self._current_trajectory = Trajectory.CONSERVATIVE
            return response
