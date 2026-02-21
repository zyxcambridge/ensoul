"""
语音播报服务 (TTS)

调用 edge-tts 或系统 TTS 引擎播报文本。
ROS2 模式下暴露 /g1/speech/say 服务。
"""

from __future__ import annotations

import subprocess
import sys

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


def speak_text(text: str, lang: str = "zh-CN") -> bool:
    """
    使用系统命令播报文本。
    macOS 用 say，Linux 尝试 espeak-ng，
    若有 edge-tts 则优先使用。
    """
    try:
        if sys.platform == "darwin":
            subprocess.run(["say", "-v", "Ting-Ting", text], check=True, timeout=30)
        else:
            subprocess.run(
                ["espeak-ng", "-v", lang, text], check=True, timeout=30
            )
        return True
    except (FileNotFoundError, subprocess.SubprocessError):
        print(f"[TTS-FALLBACK] {text}")
        return False


if HAS_ROS2:

    class TTSNode(Node):
        def __init__(self) -> None:
            super().__init__("tts_service")
            self.sub = self.create_subscription(
                String, "/g1/speech/say", self._on_say, 10
            )

        def _on_say(self, msg: String) -> None:
            self.get_logger().info(f"Speaking: {msg.data}")
            speak_text(msg.data)
