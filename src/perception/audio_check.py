"""
语音回听模块 —— 确认"我听到了自己的声音"

利用 G1 的 4 麦克风阵列，检测最近 N 秒内
是否捕获到与机器人自身 TTS 输出匹配的音频。
ROS2 话题: /rt/audio_msg
"""

from __future__ import annotations

from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


@dataclass
class AudioCheckResult:
    heard_self: bool
    transcript: str
    confidence: float


def evaluate_audio(transcript: str, expected_keywords: list[str], threshold: float = 0.7) -> AudioCheckResult:
    """
    简单关键词匹配：如果 transcript 中包含 expected_keywords 的比例超过阈值，
    则认为机器人"听到了自己"。
    """
    if not expected_keywords:
        return AudioCheckResult(heard_self=False, transcript=transcript, confidence=0.0)

    hits = sum(1 for kw in expected_keywords if kw in transcript)
    confidence = hits / len(expected_keywords)
    return AudioCheckResult(
        heard_self=confidence >= threshold,
        transcript=transcript,
        confidence=confidence,
    )


SELF_KEYWORDS = ["不确定", "不知道", "镜子", "标记"]


if HAS_ROS2:

    class AudioCheckNode(Node):
        def __init__(self) -> None:
            super().__init__("audio_check")
            self.latest_result: AudioCheckResult | None = None
            self.sub = self.create_subscription(
                String, "/rt/audio_transcript", self._on_transcript, 10
            )

        def _on_transcript(self, msg: String) -> None:
            self.latest_result = evaluate_audio(msg.data, SELF_KEYWORDS)
