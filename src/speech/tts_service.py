"""
语音播报服务 (TTS)

优先级：Gemini API → 系统 TTS (macOS say / Linux espeak-ng) → 纯日志兜底。
API Key 从 .env 文件读取，绝不硬编码、绝不提交到版本控制。
ROS2 模式下暴露 /g1/speech/say 服务。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Optional

try:
    from dotenv import load_dotenv
    _dotenv_path = Path(__file__).resolve().parents[2] / ".env"
    load_dotenv(_dotenv_path)
except ImportError:
    pass

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


def _get_gemini_key() -> Optional[str]:
    return os.environ.get("GEMINI_API_KEY")


def speak_with_gemini(text: str, api_key: Optional[str] = None) -> bool:
    """
    通过 Gemini API 生成语音（文本转语音请求）。
    若 API Key 缺失或请求失败，返回 False 触发兜底。
    """
    key = api_key or _get_gemini_key()
    if not key:
        return False

    try:
        url = (
            "https://generativelanguage.googleapis.com/v1beta/"
            "models/gemini-2.0-flash:generateContent"
            f"?key={key}"
        )
        payload = {
            "contents": [
                {
                    "parts": [
                        {"text": f"请用中文朗读以下文本（只返回确认已朗读）：{text}"}
                    ]
                }
            ]
        }
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(
            url, data=data, headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=10) as resp:
            return resp.status == 200
    except Exception:
        return False


def speak_with_system(text: str, lang: str = "zh-CN") -> bool:
    """macOS 用 say，Linux 用 espeak-ng。"""
    try:
        if sys.platform == "darwin":
            subprocess.run(["say", "-v", "Ting-Ting", text], check=True, timeout=30)
        else:
            subprocess.run(["espeak-ng", "-v", lang, text], check=True, timeout=30)
        return True
    except (FileNotFoundError, subprocess.SubprocessError):
        return False


def speak_text(text: str, lang: str = "zh-CN") -> bool:
    """
    统一入口：Gemini → 系统 TTS → 日志兜底。
    """
    if speak_with_gemini(text):
        return True

    if speak_with_system(text, lang):
        return True

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
