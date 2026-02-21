"""
擦除动作控制器单元测试

覆盖：
- wipe_sticker_fallback: 主轨迹 / 保守轨迹成功
- execute_trajectory: 路点回调被正确调用 / 空路点
- WipePlanner: 完整擦除流程 / 各子步骤 / 异常降级
- 轨迹映射完整性
- JointWaypoint 数据结构
- 保守轨迹比主轨迹慢
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.action.wipe_controller import (
    Trajectory,
    JointWaypoint,
    PRIMARY_WAYPOINTS,
    CONSERVATIVE_WAYPOINTS,
    TRAJECTORY_MAP,
    execute_trajectory,
    wipe_sticker_fallback,
    WipePlanner,
)


# ---- wipe_sticker_fallback ----

def test_wipe_primary_success():
    assert wipe_sticker_fallback(Trajectory.PRIMARY) is True


def test_wipe_conservative_success():
    assert wipe_sticker_fallback(Trajectory.CONSERVATIVE) is True


# ---- 轨迹映射 ----

def test_trajectory_map_completeness():
    assert Trajectory.PRIMARY in TRAJECTORY_MAP
    assert Trajectory.CONSERVATIVE in TRAJECTORY_MAP


def test_primary_waypoints_not_empty():
    assert len(PRIMARY_WAYPOINTS) >= 2


def test_conservative_waypoints_not_empty():
    assert len(CONSERVATIVE_WAYPOINTS) >= 2


# ---- JointWaypoint 结构 ----

def test_waypoint_structure():
    for wp in PRIMARY_WAYPOINTS:
        assert isinstance(wp, JointWaypoint)
        assert isinstance(wp.joint_angles, list)
        assert len(wp.joint_angles) == 7
        assert isinstance(wp.duration_sec, float)
        assert wp.duration_sec > 0


def test_conservative_waypoint_structure():
    for wp in CONSERVATIVE_WAYPOINTS:
        assert isinstance(wp, JointWaypoint)
        assert len(wp.joint_angles) == 7
        assert wp.duration_sec > 0


# ---- execute_trajectory ----

def test_publish_fn_called():
    published = []
    wipe_sticker_fallback(Trajectory.PRIMARY, publish_fn=published.append)
    assert len(published) == len(PRIMARY_WAYPOINTS)
    for wp in published:
        assert isinstance(wp, JointWaypoint)


def test_execute_empty_waypoints():
    result = execute_trajectory([])
    assert result is True


def test_execute_with_callback():
    calls = []
    waypoints = [
        JointWaypoint(joint_angles=[0.0] * 7, duration_sec=1.0),
    ]
    result = execute_trajectory(waypoints, publish_fn=calls.append)
    assert result is True
    assert len(calls) == 1


# ---- 时间对比 ----

def test_conservative_slower_than_primary():
    primary_total = sum(wp.duration_sec for wp in PRIMARY_WAYPOINTS)
    conservative_total = sum(wp.duration_sec for wp in CONSERVATIVE_WAYPOINTS)
    assert conservative_total >= primary_total


# ---- 枚举值 ----

def test_trajectory_enum_values():
    assert Trajectory.PRIMARY.value == "head_touch_front"
    assert Trajectory.CONSERVATIVE.value == "head_touch_conservative"


# ---- WipePlanner ----

class TestWipePlanner:

    def test_perform_wipe_success(self):
        logs = []
        planner = WipePlanner(logger=logs.append)
        result = planner.perform_wipe()
        assert result is True
        assert len(logs) >= 4

    def test_enable_compliance(self):
        planner = WipePlanner(logger=lambda _: None)
        assert planner.enable_compliance_control() is True

    def test_modify_acm(self):
        planner = WipePlanner(logger=lambda _: None)
        assert planner.modify_acm() is True

    def test_plan_approach_rrt(self):
        planner = WipePlanner(logger=lambda _: None)
        assert planner.plan_approach_with_rrt() is True

    def test_cartesian_wipe(self):
        planner = WipePlanner(logger=lambda _: None)
        assert planner.compute_and_execute_cartesian_wipe() is True

    def test_reset_acm(self):
        planner = WipePlanner(logger=lambda _: None)
        assert planner.reset_acm() is True

    def test_perform_wipe_approach_fails(self):
        planner = WipePlanner(logger=lambda _: None)
        planner.plan_approach_with_rrt = lambda: False
        result = planner.perform_wipe()
        assert result is False

    def test_perform_wipe_cartesian_fails(self):
        planner = WipePlanner(logger=lambda _: None)
        planner.compute_and_execute_cartesian_wipe = lambda: False
        result = planner.perform_wipe()
        assert result is False

    def test_perform_wipe_exception_handled(self):
        planner = WipePlanner(logger=lambda _: None)
        planner.plan_approach_with_rrt = lambda: (_ for _ in ()).throw(
            RuntimeError("hardware fault")
        )
        result = planner.perform_wipe()
        assert result is False

    def test_reset_acm_called_even_on_failure(self):
        logs = []
        planner = WipePlanner(logger=logs.append)
        planner.plan_approach_with_rrt = lambda: False
        planner.perform_wipe()
        assert any("恢复 ACM" in l for l in logs)
