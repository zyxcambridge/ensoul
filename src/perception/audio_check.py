"""
语音回听模块 (Perception Node - Audio Check)

实现 PRD 4.2 听觉子模块要求：
订阅 /g1/mic_asr（语音识别）和内部 TTS 状态。
执行比对逻辑（发声状态 == True 且 监听到自身文本）。
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


@dataclass
class AudioCheckResult:
    heard_self: bool
    transcript: str
    confidence: float


def evaluate_audio(transcript: str, tts_is_playing: bool, expected_keywords: list[str], threshold: float = 0.5) -> AudioCheckResult:
    """
    PRD逻辑：发声状态 == True 且 监听到自身文本。
    如果 transcript 中包含 expected_keywords 的比例超过阈值，并且 TTS 正在发声，
    则认为机器人"听到了自己"。
    """
    if not tts_is_playing:
        return AudioCheckResult(heard_self=False, transcript=transcript, confidence=0.0)

    if not expected_keywords:
        return AudioCheckResult(heard_self=False, transcript=transcript, confidence=0.0)

    hits = sum(1 for kw in expected_keywords if kw in transcript)
    confidence = hits / len(expected_keywords)
    return AudioCheckResult(
        heard_self=confidence >= threshold,
        transcript=transcript,
        confidence=confidence,
    )


# 预期自己刚说过的关键词
SELF_KEYWORDS = ["不知"]


if HAS_ROS2:

    class AudioCheckNode(Node):
        def __init__(self) -> None:
            super().__init__("audio_check")
            self.latest_result: AudioCheckResult | None = None
            self.tts_is_playing: bool = False
            
            # 订阅麦克风 ASR 结果
            self.sub_asr = self.create_subscription(
                String, "/g1/mic_asr", self._on_transcript, 10
            )
            # 订阅 TTS 状态
            self.sub_tts = self.create_subscription(
                Bool, "/g1/tts_status", self._on_tts_status, 10
            )

        def _on_tts_status(self, msg: Bool) -> None:
            self.tts_is_playing = msg.data

        def _on_transcript(self, msg: String) -> None:
            self.latest_result = evaluate_audio(
                msg.data, 
                self.tts_is_playing, 
                SELF_KEYWORDS
            )
