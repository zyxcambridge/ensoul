"""
本体感知模块单元测试

覆盖：
- 所有关节活跃 → is_ok=True
- 无关节数据 → is_ok=False
- 部分关节活跃低于阈值 → is_ok=False
- 自定义阈值
- score 计算正确性
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.perception.body_check import evaluate_body, BodyCheckResult


def test_all_joints_active():
    positions = [0.0] * 10
    velocities = [0.5] * 10
    result = evaluate_body(positions, velocities)
    assert result.is_ok is True
    assert result.active_joints == 10
    assert result.total_joints == 10
    assert abs(result.score - 1.0) < 1e-6


def test_no_joints():
    result = evaluate_body([], [])
    assert result.is_ok is False
    assert result.active_joints == 0
    assert result.total_joints == 0
    assert result.score == 0.0


def test_partial_below_threshold():
    positions = [0.0] * 10
    velocities = [0.5] * 5 + [0.001] * 5  # 50% active, below 80% threshold
    result = evaluate_body(positions, velocities)
    assert result.is_ok is False
    assert result.active_joints == 5
    assert abs(result.score - 0.5) < 1e-6


def test_partial_above_threshold():
    positions = [0.0] * 10
    velocities = [0.5] * 9 + [0.001] * 1  # 90% active
    result = evaluate_body(positions, velocities)
    assert result.is_ok is True
    assert result.active_joints == 9
    assert abs(result.score - 0.9) < 1e-6


def test_custom_threshold():
    positions = [0.0] * 10
    velocities = [0.5] * 3 + [0.001] * 7  # 30% active
    result = evaluate_body(positions, velocities, threshold=0.2)
    assert result.is_ok is True
    assert result.active_joints == 3


def test_exact_threshold_boundary():
    positions = [0.0] * 10
    velocities = [0.5] * 8 + [0.001] * 2  # exactly 0.8
    result = evaluate_body(positions, velocities, threshold=0.8)
    assert result.is_ok is True


def test_velocity_at_boundary():
    positions = [0.0] * 5
    velocities = [0.01, 0.011, -0.02, 0.005, 0.0]
    result = evaluate_body(positions, velocities)
    # 0.01 → not active (not > 0.01), 0.011 → active, -0.02 → active, 0.005 → not, 0.0 → not
    assert result.active_joints == 2
    assert abs(result.score - 0.4) < 1e-6


def test_result_dataclass_fields():
    result = evaluate_body([0.0], [1.0])
    assert isinstance(result, BodyCheckResult)
    assert hasattr(result, "is_ok")
    assert hasattr(result, "active_joints")
    assert hasattr(result, "total_joints")
    assert hasattr(result, "score")
