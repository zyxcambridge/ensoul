"""
语音回听模块单元测试

覆盖：
- 全部关键词命中 → heard_self=True
- 无关键词 → heard_self=False
- 部分命中高于/低于阈值
- 空 transcript
- 空 expected_keywords 列表
- 自定义阈值
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.perception.audio_check import evaluate_audio, AudioCheckResult, SELF_KEYWORDS


def test_all_keywords_found():
    transcript = "我不确定那是不是我，不知道镜子里的标记是谁的"
    result = evaluate_audio(transcript, SELF_KEYWORDS)
    assert result.heard_self is True
    assert result.confidence == 1.0


def test_no_keywords_found():
    transcript = "今天天气很好，适合出门散步"
    result = evaluate_audio(transcript, SELF_KEYWORDS)
    assert result.heard_self is False
    assert result.confidence == 0.0


def test_partial_above_threshold():
    transcript = "我不确定，也不知道那是什么，但我看到了什么东西"
    # "不确定" ✓, "不知道" ✓, "镜子" ✗, "标记" ✗ → 2/4 = 0.5
    result = evaluate_audio(transcript, SELF_KEYWORDS, threshold=0.5)
    assert result.heard_self is True
    assert abs(result.confidence - 0.5) < 1e-6


def test_partial_below_threshold():
    transcript = "我不确定那是什么"
    # "不确定" ✓, rest ✗ → 1/4 = 0.25
    result = evaluate_audio(transcript, SELF_KEYWORDS)
    assert result.heard_self is False
    assert abs(result.confidence - 0.25) < 1e-6


def test_empty_transcript():
    result = evaluate_audio("", SELF_KEYWORDS)
    assert result.heard_self is False
    assert result.confidence == 0.0
    assert result.transcript == ""


def test_empty_keywords():
    result = evaluate_audio("任何文本", [])
    assert result.heard_self is False
    assert result.confidence == 0.0


def test_custom_threshold_low():
    transcript = "我看到了镜子"
    # "不确定" ✗, "不知道" ✗, "镜子" ✓, "标记" ✗ → 1/4 = 0.25
    result = evaluate_audio(transcript, SELF_KEYWORDS, threshold=0.2)
    assert result.heard_self is True


def test_custom_keywords():
    custom_kw = ["你好", "世界"]
    transcript = "你好世界，这是测试"
    result = evaluate_audio(transcript, custom_kw)
    assert result.heard_self is True
    assert result.confidence == 1.0


def test_result_preserves_transcript():
    text = "原始输入文本"
    result = evaluate_audio(text, ["不存在的"])
    assert result.transcript == text


def test_result_dataclass_fields():
    result = evaluate_audio("test", ["test"])
    assert isinstance(result, AudioCheckResult)
    assert hasattr(result, "heard_self")
    assert hasattr(result, "transcript")
    assert hasattr(result, "confidence")
