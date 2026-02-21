"""
语音回听模块单元测试（适配新签名：tts_is_playing 参数）

evaluate_audio(transcript, tts_is_playing, expected_keywords, threshold)
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.perception.audio_check import evaluate_audio, AudioCheckResult, SELF_KEYWORDS


# ---- tts_is_playing = True，关键词全命中 ----

def test_all_keywords_found():
    transcript = "我不知那是什么"
    result = evaluate_audio(transcript, True, SELF_KEYWORDS)
    assert result.heard_self is True
    assert result.confidence == 1.0


def test_no_keywords_found():
    transcript = "今天天气很好，适合出门散步"
    result = evaluate_audio(transcript, True, SELF_KEYWORDS)
    assert result.heard_self is False
    assert result.confidence == 0.0


# ---- tts_is_playing = False 时必然返回 False ----

def test_not_playing_returns_false():
    transcript = "我不知那是什么"
    result = evaluate_audio(transcript, False, SELF_KEYWORDS)
    assert result.heard_self is False
    assert result.confidence == 0.0


def test_not_playing_with_keywords():
    result = evaluate_audio("不知", False, ["不知"])
    assert result.heard_self is False


# ---- 阈值边界 ----

def test_partial_above_threshold():
    transcript = "我不知道那是什么"
    result = evaluate_audio(transcript, True, ["不知", "镜子"], threshold=0.4)
    # "不知" ✓, "镜子" ✗ → 0.5 >= 0.4
    assert result.heard_self is True
    assert abs(result.confidence - 0.5) < 1e-6


def test_partial_below_threshold():
    transcript = "我看到了"
    result = evaluate_audio(transcript, True, SELF_KEYWORDS, threshold=0.5)
    assert result.heard_self is False


# ---- 空输入 ----

def test_empty_transcript():
    result = evaluate_audio("", True, SELF_KEYWORDS)
    assert result.heard_self is False
    assert result.confidence == 0.0
    assert result.transcript == ""


def test_empty_keywords():
    result = evaluate_audio("任何文本", True, [])
    assert result.heard_self is False
    assert result.confidence == 0.0


# ---- 自定义关键词 ----

def test_custom_threshold_low():
    result = evaluate_audio("不知", True, SELF_KEYWORDS, threshold=0.5)
    assert result.heard_self is True


def test_custom_keywords():
    custom_kw = ["你好", "世界"]
    transcript = "你好世界"
    result = evaluate_audio(transcript, True, custom_kw)
    assert result.heard_self is True
    assert result.confidence == 1.0


# ---- transcript 保留 ----

def test_result_preserves_transcript():
    text = "原始输入文本"
    result = evaluate_audio(text, True, ["不存在"])
    assert result.transcript == text


# ---- dataclass 结构 ----

def test_result_dataclass_fields():
    result = evaluate_audio("test", True, ["test"])
    assert isinstance(result, AudioCheckResult)
    assert hasattr(result, "heard_self")
    assert hasattr(result, "transcript")
    assert hasattr(result, "confidence")
