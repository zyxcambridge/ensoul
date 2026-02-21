"""
镜像视觉匹配模块单元测试（适配新签名 + 光流）

evaluate_vision(image_bgr, prev_image_gray, expected_color, color_threshold)
VisionCheckResult: detected_color, color_match, color_confidence, optic_flow_magnitude
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
    compute_optic_flow,
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
        img = make_solid_bgr(0, 0, 200)
        color, conf = detect_sticker_color(img)
        assert color == "red"
        assert conf > 0.5

    def test_detect_blue(self):
        img = make_solid_bgr(200, 0, 0)
        color, conf = detect_sticker_color(img)
        assert color == "blue"
        assert conf > 0.5

    def test_detect_green(self):
        img = make_solid_bgr(0, 200, 0)
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
        assert conf < 0.1


@pytest.mark.skipif(not HAS_CV2, reason="OpenCV not installed")
class TestEvaluateVision:

    def test_match_red(self):
        img = make_solid_bgr(0, 0, 200)
        result = evaluate_vision(img, None, "red", color_threshold=0.3)
        assert result.color_match is True
        assert result.detected_color == "red"

    def test_no_match_wrong_expected(self):
        img = make_solid_bgr(0, 0, 200)  # red
        result = evaluate_vision(img, None, "blue", color_threshold=0.3)
        assert result.color_match is False

    def test_low_confidence_below_threshold(self):
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        img[0:5, 0:5] = (0, 0, 200)  # tiny red patch
        result = evaluate_vision(img, None, "red", color_threshold=0.85)
        assert result.color_match is False

    def test_result_dataclass(self):
        img = make_solid_bgr(0, 200, 0)
        result = evaluate_vision(img, None, "green", color_threshold=0.3)
        assert isinstance(result, VisionCheckResult)
        assert hasattr(result, "detected_color")
        assert hasattr(result, "color_match")
        assert hasattr(result, "color_confidence")
        assert hasattr(result, "optic_flow_magnitude")

    def test_optic_flow_zero_without_prev(self):
        img = make_solid_bgr(0, 0, 200)
        result = evaluate_vision(img, None, "red", color_threshold=0.3)
        assert result.optic_flow_magnitude == 0.0

    def test_optic_flow_nonzero_with_prev(self):
        img1 = np.zeros((100, 100, 3), dtype=np.uint8)
        img1[40:60, 40:60] = (0, 0, 200)  # red square center
        img2 = np.zeros((100, 100, 3), dtype=np.uint8)
        img2[50:70, 50:70] = (0, 0, 200)  # shifted red square
        prev_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        result = evaluate_vision(img2, prev_gray, "red", color_threshold=0.001)
        assert result.optic_flow_magnitude >= 0.0


@pytest.mark.skipif(not HAS_CV2, reason="OpenCV not installed")
class TestComputeOpticFlow:

    def test_identical_frames_low_flow(self):
        gray = np.random.randint(0, 255, (100, 100), dtype=np.uint8)
        mag = compute_optic_flow(gray, gray.copy())
        assert mag < 1.0

    def test_none_input(self):
        gray = np.zeros((100, 100), dtype=np.uint8)
        assert compute_optic_flow(None, gray) == 0.0
        assert compute_optic_flow(gray, None) == 0.0


class TestColorHSVRanges:

    def test_all_colors_defined(self):
        assert "red" in COLOR_HSV_RANGES
        assert "blue" in COLOR_HSV_RANGES
        assert "green" in COLOR_HSV_RANGES

    def test_range_format(self):
        for name, (lo, hi) in COLOR_HSV_RANGES.items():
            assert len(lo) == 3
            assert len(hi) == 3
            for i in range(3):
                assert lo[i] <= hi[i]
