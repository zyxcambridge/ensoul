"""
擦除动作控制器单元测试

覆盖：
- 主轨迹擦除成功
- 保守轨迹擦除成功
- 路点回调被正确调用
- 轨迹映射完整性
- JointWaypoint 数据结构
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
    wipe_sticker,
)


def test_wipe_primary_success():
    assert wipe_sticker(Trajectory.PRIMARY) is True


def test_wipe_conservative_success():
    assert wipe_sticker(Trajectory.CONSERVATIVE) is True


def test_trajectory_map_completeness():
    assert Trajectory.PRIMARY in TRAJECTORY_MAP
    assert Trajectory.CONSERVATIVE in TRAJECTORY_MAP


def test_primary_waypoints_not_empty():
    assert len(PRIMARY_WAYPOINTS) >= 2


def test_conservative_waypoints_not_empty():
    assert len(CONSERVATIVE_WAYPOINTS) >= 2


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


def test_publish_fn_called():
    published = []
    wipe_sticker(Trajectory.PRIMARY, publish_fn=published.append)
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


def test_conservative_slower_than_primary():
    primary_total = sum(wp.duration_sec for wp in PRIMARY_WAYPOINTS)
    conservative_total = sum(wp.duration_sec for wp in CONSERVATIVE_WAYPOINTS)
    assert conservative_total >= primary_total


def test_trajectory_enum_values():
    assert Trajectory.PRIMARY.value == "head_touch_front"
    assert Trajectory.CONSERVATIVE.value == "head_touch_conservative"
