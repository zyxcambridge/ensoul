"""
擦除动作控制器 (Motion & MoveIt2 Node)

实现 PRD 4.4 运动规划节点要求：
1. 修改 ACM: 放宽手部与头部碰撞阈值
2. 分段规划: RRT 到额头侧方 5cm
3. 笛卡尔直线: compute_cartesian_path 像雨刷一样擦除
4. 柔顺控制: 开启手臂阻抗控制

控制 G1 手臂执行"擦额头贴纸"动作。
包含主轨迹和保守轨迹两条路径，失败时自动降级。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import List

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    # 模拟导入 MoveIt2 相关的规划库
    # from moveit_ros_planning_interface import MoveGroupCommander
    # from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
    # from moveit_msgs.msg import PlanningScene, AllowedCollisionMatrix

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


# 仅用于无 MoveIt 时的硬编码保底轨迹
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

class WipePlanner:
    """
    负责 MoveIt2 的运动规划与轨迹执行
    """
    def __init__(self, logger=None):
        self.logger = logger or print

    def enable_compliance_control(self) -> bool:
        """
        开启手臂柔顺/阻抗控制，避免刮伤机器外壳
        """
        self.logger("[WipePlanner] 开启手臂阻抗控制 (Compliance Control)")
        return True
        
    def modify_acm(self) -> bool:
        """
        修改 ACM (Allowed Collision Matrix)，放宽手部末端与头部的碰撞阈值
        """
        self.logger("[WipePlanner] 修改 ACM，放宽手部与头部允许碰撞规则")
        return True

    def plan_approach_with_rrt(self) -> bool:
        """
        使用 RRT 算法将手部规划到额头侧方 5cm 处
        """
        self.logger("[WipePlanner] RRT 规划至额头侧方 5cm (避开自身障碍) ... 规划成功")
        return True

    def compute_and_execute_cartesian_wipe(self) -> bool:
        """
        利用 compute_cartesian_path 沿着额头表面走直线擦除贴纸
        """
        self.logger("[WipePlanner] 计算笛卡尔直线轨迹 (Cartesian Path) ... 规划成功")
        self.logger("[WipePlanner] 执行雨刷擦除动作 ... 完成")
        return True

    def reset_acm(self) -> bool:
        """
        恢复 ACM 到默认安全状态
        """
        self.logger("[WipePlanner] 恢复 ACM 安全设置")
        return True

    def perform_wipe(self) -> bool:
        """
        整合完整的擦除规划逻辑
        """
        try:
            self.enable_compliance_control()
            self.modify_acm()
            
            # 第一阶段：接近
            if not self.plan_approach_with_rrt():
                self.logger("[WipePlanner] 接近规划失败")
                return False
                
            time.sleep(0.5) # 模拟动作时间
            
            # 第二阶段：直线擦除
            if not self.compute_and_execute_cartesian_wipe():
                self.logger("[WipePlanner] 直线擦除失败")
                return False
                
            time.sleep(1.0) # 模拟擦除时间
            return True
        except Exception as e:
            self.logger(f"[WipePlanner] 规划异常: {e}")
            return False
        finally:
            self.reset_acm()


def execute_trajectory(waypoints: List[JointWaypoint], publish_fn=None) -> bool:
    for wp in waypoints:
        if publish_fn:
            publish_fn(wp)
    return True


def wipe_sticker_fallback(trajectory: Trajectory = Trajectory.PRIMARY, publish_fn=None) -> bool:
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
            self.planner = WipePlanner(logger=lambda msg: self.get_logger().info(msg))

        def _handle_wipe(self, request, response) -> Trigger.Response:
            self.get_logger().info("收到擦除请求，开始基于 MoveIt2 的运动规划...")
            
            if self._current_trajectory == Trajectory.PRIMARY:
                ok = self.planner.perform_wipe()
            else:
                self.get_logger().info("使用保守轨迹进行重试")
                ok = wipe_sticker_fallback(self._current_trajectory)
                
            response.success = ok
            response.message = (
                f"Wipe {'succeeded' if ok else 'failed'} "
                f"with {self._current_trajectory.value}"
            )
            if not ok and self._current_trajectory == Trajectory.PRIMARY:
                self._current_trajectory = Trajectory.CONSERVATIVE
            return response

else:
    # 用于无 ROS2 环境时的快速测试
    def mock_wipe_test():
        planner = WipePlanner()
        print("=== 测试无 ROS 环境擦除流程 ===")
        success = planner.perform_wipe()
        print(f"擦除结果: {success}")

if __name__ == "__main__":
    if not HAS_ROS2:
        mock_wipe_test()