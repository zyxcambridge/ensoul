"""
镜像视觉匹配模块 (Perception Node - Vision Check)

实现 PRD 4.2 视觉子模块要求：
调用 D435i 摄像头。
使用 OpenCV 识别镜中机器人的轮廓及贴纸颜色。
计算镜中影像的运动光流。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

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
    color_match: bool
    color_confidence: float
    optic_flow_magnitude: float


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


def compute_optic_flow(prev_gray: np.ndarray, curr_gray: np.ndarray) -> float:
    """
    计算镜中影像的运动光流 (Farneback)，返回光流整体强度。
    """
    if not HAS_CV2 or prev_gray is None or curr_gray is None:
        return 0.0
        
    flow = cv2.calcOpticalFlowFarneback(
        prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0
    )
    magnitude, _ = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    return float(np.mean(magnitude))


def evaluate_vision(
    image_bgr: np.ndarray,
    prev_image_gray: Optional[np.ndarray],
    expected_color: str,
    color_threshold: float = 0.85,
) -> VisionCheckResult:
    """
    识别贴纸颜色，并计算光流强度
    """
    detected, color_confidence = detect_sticker_color(image_bgr)
    is_match = detected == expected_color and color_confidence >= color_threshold
    
    optic_flow_mag = 0.0
    if HAS_CV2 and prev_image_gray is not None:
        curr_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        optic_flow_mag = compute_optic_flow(prev_image_gray, curr_gray)

    return VisionCheckResult(
        detected_color=detected,
        color_match=is_match,
        color_confidence=color_confidence,
        optic_flow_magnitude=optic_flow_mag
    )
