"""
镜像视觉匹配模块单元测试

覆盖：
- 纯红/蓝/绿图像正确识别
- 匹配时 match=True，不匹配时 match=False
- 黑色图像（无彩色区域）
- 自定义阈值
- 结果 dataclass 字段
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

import pytest

from src.perception.vision_check import (
    detect_sticker_color,
    evaluate_vision,
    VisionCheckResult,
    COLOR_HSV_RANGES,
)


def make_solid_bgr(b, g, r, size=100):
    img = np.zeros((size, size, 3), dtype=np.uint8)
    img[:, :] = (b, g, r)
    return img


@pytest.mark.skipif(not HAS_CV2, reason="OpenCV not installed")
class TestDetectStickerColor:

    def test_detect_red(self):
        img = make_solid_bgr(0, 0, 200)  # BGR pure red
        color, conf = detect_sticker_color(img)
        assert color == "red"
        assert conf > 0.5

    def test_detect_blue(self):
        img = make_solid_bgr(200, 0, 0)  # BGR pure blue
        color, conf = detect_sticker_color(img)
        assert color == "blue"
        assert conf > 0.5

    def test_detect_green(self):
        img = make_solid_bgr(0, 200, 0)  # BGR pure green
        color, conf = detect_sticker_color(img)
        assert color == "green"
        assert conf > 0.5

    def test_black_image(self):
        img = make_solid_bgr(0, 0, 0)
        color, conf = detect_sticker_color(img)
        assert conf < 0.01

    def test_white_image(self):
        img = make_solid_bgr(255, 255, 255)
        color, conf = detect_sticker_color(img)
        # white is not in red/blue/green HSV ranges (low saturation)
        assert conf < 0.1


@pytest.mark.skipif(not HAS_CV2, reason="OpenCV not installed")
class TestEvaluateVision:

    def test_match_red(self):
        img = make_solid_bgr(0, 0, 200)
        result = evaluate_vision(img, "red", threshold=0.3)
        assert result.match is True
        assert result.detected_color == "red"

    def test_no_match_wrong_expected(self):
        img = make_solid_bgr(0, 0, 200)  # red image
        result = evaluate_vision(img, "blue", threshold=0.3)
        assert result.match is False

    def test_low_confidence_below_threshold(self):
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[0:5, 0:5] = (0, 0, 200)  # tiny red patch
        result = evaluate_vision(img, "red", threshold=0.85)
        assert result.match is False

    def test_result_dataclass(self):
        img = make_solid_bgr(0, 200, 0)
        result = evaluate_vision(img, "green", threshold=0.3)
        assert isinstance(result, VisionCheckResult)
        assert hasattr(result, "detected_color")
        assert hasattr(result, "match")
        assert hasattr(result, "confidence")


class TestColorHSVRanges:

    def test_all_colors_defined(self):
        assert "red" in COLOR_HSV_RANGES
        assert "blue" in COLOR_HSV_RANGES
        assert "green" in COLOR_HSV_RANGES

    def test_range_format(self):
        for name, (lo, hi) in COLOR_HSV_RANGES.items():
            assert len(lo) == 3, f"{name} low range should have 3 values"
            assert len(hi) == 3, f"{name} high range should have 3 values"
            for i in range(3):
                assert lo[i] <= hi[i], f"{name} range[{i}]: low > high"
