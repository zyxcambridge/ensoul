"""
镜像视觉匹配模块 —— 确认"镜中颜色与我头上一致"

通过 D435i 摄像头检测镜中贴纸的颜色，
与自身头顶（通过第二视角或推断）的颜色比对。
ROS2 话题: /camera/color/image_raw (sensor_msgs/Image)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import rclpy
    from rclpy.node import Node

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


COLOR_HSV_RANGES = {
    "red": ((0, 100, 100), (10, 255, 255)),
    "blue": ((100, 100, 100), (130, 255, 255)),
    "green": ((40, 100, 100), (80, 255, 255)),
}


@dataclass
class VisionCheckResult:
    detected_color: Optional[str]
    match: bool
    confidence: float


def detect_sticker_color(image_bgr: np.ndarray) -> tuple[Optional[str], float]:
    """
    在 BGR 图像中检测最大面积的彩色区域，返回颜色名和置信度。
    """
    if not HAS_CV2:
        return None, 0.0

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    best_color: Optional[str] = None
    best_area: float = 0.0
    total_pixels = hsv.shape[0] * hsv.shape[1]

    for color_name, (lo, hi) in COLOR_HSV_RANGES.items():
        mask = cv2.inRange(hsv, np.array(lo), np.array(hi))
        area = float(cv2.countNonZero(mask))
        if area > best_area:
            best_area = area
            best_color = color_name

    confidence = best_area / total_pixels if total_pixels > 0 else 0.0
    return best_color, confidence


def evaluate_vision(
    image_bgr: np.ndarray,
    expected_color: str,
    threshold: float = 0.85,
) -> VisionCheckResult:
    detected, confidence = detect_sticker_color(image_bgr)
    is_match = detected == expected_color and confidence >= threshold
    return VisionCheckResult(
        detected_color=detected,
        match=is_match,
        confidence=confidence,
    )
