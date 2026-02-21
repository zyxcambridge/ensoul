"""
TTS 模块单元测试（全 mock，不依赖真实 TTS 引擎或网络）

覆盖：
- speak_with_gemini: Key 存在且请求成功 / Key 缺失 / 请求异常
- speak_with_system: macOS 成功 / Linux 成功 / 命令不存在 / 超时
- speak_text 统一入口: Gemini 优先 / 降级到系统 / 全部失败兜底
- _get_gemini_key: 从环境变量读取 / 未设置时返回 None
- API Key 安全性: .env 不会被 git 追踪
"""

import sys
import os
import subprocess
from unittest.mock import patch, MagicMock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.speech.tts_service import (
    speak_with_gemini,
    speak_with_system,
    speak_text,
    _get_gemini_key,
)


# ============================================================
# _get_gemini_key
# ============================================================

class TestGetGeminiKey:

    def test_returns_key_from_env(self):
        with patch.dict(os.environ, {"GEMINI_API_KEY": "test-key-123"}):
            assert _get_gemini_key() == "test-key-123"

    def test_returns_none_when_not_set(self):
        with patch.dict(os.environ, {}, clear=True):
            assert _get_gemini_key() is None


# ============================================================
# speak_with_gemini（全 mock，不发真实请求）
# ============================================================

class TestSpeakWithGemini:

    def test_no_key_returns_false(self):
        result = speak_with_gemini("你好", api_key=None)
        assert result is False

    def test_empty_key_returns_false(self):
        result = speak_with_gemini("你好", api_key="")
        assert result is False

    @patch("src.speech.tts_service.urllib.request.urlopen")
    def test_success_with_explicit_key(self, mock_urlopen):
        mock_resp = MagicMock()
        mock_resp.status = 200
        mock_resp.__enter__ = MagicMock(return_value=mock_resp)
        mock_resp.__exit__ = MagicMock(return_value=False)
        mock_urlopen.return_value = mock_resp

        result = speak_with_gemini("测试文本", api_key="fake-key")
        assert result is True
        mock_urlopen.assert_called_once()

    @patch("src.speech.tts_service.urllib.request.urlopen")
    def test_success_with_env_key(self, mock_urlopen):
        mock_resp = MagicMock()
        mock_resp.status = 200
        mock_resp.__enter__ = MagicMock(return_value=mock_resp)
        mock_resp.__exit__ = MagicMock(return_value=False)
        mock_urlopen.return_value = mock_resp

        with patch.dict(os.environ, {"GEMINI_API_KEY": "env-key"}):
            result = speak_with_gemini("测试")
            assert result is True

    @patch("src.speech.tts_service.urllib.request.urlopen")
    def test_api_returns_non_200(self, mock_urlopen):
        mock_resp = MagicMock()
        mock_resp.status = 500
        mock_resp.__enter__ = MagicMock(return_value=mock_resp)
        mock_resp.__exit__ = MagicMock(return_value=False)
        mock_urlopen.return_value = mock_resp

        result = speak_with_gemini("失败测试", api_key="fake-key")
        assert result is False

    @patch("src.speech.tts_service.urllib.request.urlopen")
    def test_network_error_returns_false(self, mock_urlopen):
        mock_urlopen.side_effect = Exception("network error")
        result = speak_with_gemini("网络错误", api_key="fake-key")
        assert result is False

    @patch("src.speech.tts_service.urllib.request.urlopen")
    def test_timeout_returns_false(self, mock_urlopen):
        import urllib.error
        mock_urlopen.side_effect = urllib.error.URLError("timeout")
        result = speak_with_gemini("超时", api_key="fake-key")
        assert result is False


# ============================================================
# speak_with_system（mock subprocess）
# ============================================================

class TestSpeakWithSystem:

    @patch("src.speech.tts_service.sys")
    @patch("src.speech.tts_service.subprocess.run")
    def test_macos_success(self, mock_run, mock_sys):
        mock_sys.platform = "darwin"
        mock_run.return_value = MagicMock()

        result = speak_with_system("你好")
        assert result is True
        mock_run.assert_called_once()
        args = mock_run.call_args[0][0]
        assert args[0] == "say"
        assert "Ting-Ting" in args

    @patch("src.speech.tts_service.sys")
    @patch("src.speech.tts_service.subprocess.run")
    def test_linux_success(self, mock_run, mock_sys):
        mock_sys.platform = "linux"
        mock_run.return_value = MagicMock()

        result = speak_with_system("你好", lang="zh-CN")
        assert result is True
        args = mock_run.call_args[0][0]
        assert args[0] == "espeak-ng"

    @patch("src.speech.tts_service.sys")
    @patch("src.speech.tts_service.subprocess.run")
    def test_command_not_found(self, mock_run, mock_sys):
        mock_sys.platform = "darwin"
        mock_run.side_effect = FileNotFoundError("say not found")

        result = speak_with_system("测试")
        assert result is False

    @patch("src.speech.tts_service.sys")
    @patch("src.speech.tts_service.subprocess.run")
    def test_subprocess_timeout(self, mock_run, mock_sys):
        mock_sys.platform = "darwin"
        mock_run.side_effect = subprocess.TimeoutExpired(cmd="say", timeout=30)

        result = speak_with_system("超时测试")
        assert result is False

    @patch("src.speech.tts_service.sys")
    @patch("src.speech.tts_service.subprocess.run")
    def test_subprocess_error(self, mock_run, mock_sys):
        mock_sys.platform = "linux"
        mock_run.side_effect = subprocess.CalledProcessError(1, "espeak-ng")

        result = speak_with_system("失败")
        assert result is False


# ============================================================
# speak_text 统一入口（优先级链）
# ============================================================

class TestSpeakText:

    @patch("src.speech.tts_service.speak_with_system", return_value=False)
    @patch("src.speech.tts_service.speak_with_gemini", return_value=True)
    def test_gemini_first_if_available(self, mock_gemini, mock_sys):
        result = speak_text("优先 Gemini")
        assert result is True
        mock_gemini.assert_called_once_with("优先 Gemini")
        mock_sys.assert_not_called()

    @patch("src.speech.tts_service.speak_with_system", return_value=True)
    @patch("src.speech.tts_service.speak_with_gemini", return_value=False)
    def test_fallback_to_system_when_gemini_fails(self, mock_gemini, mock_sys):
        result = speak_text("降级到系统")
        assert result is True
        mock_gemini.assert_called_once()
        mock_sys.assert_called_once()

    @patch("src.speech.tts_service.speak_with_system", return_value=False)
    @patch("src.speech.tts_service.speak_with_gemini", return_value=False)
    def test_all_fail_returns_false(self, mock_gemini, mock_sys):
        result = speak_text("全部失败")
        assert result is False

    @patch("src.speech.tts_service.speak_with_system", return_value=False)
    @patch("src.speech.tts_service.speak_with_gemini", return_value=False)
    def test_fallback_prints_log(self, mock_gemini, mock_sys, capsys):
        speak_text("日志兜底测试")
        captured = capsys.readouterr()
        assert "[TTS-FALLBACK]" in captured.out
        assert "日志兜底测试" in captured.out

    @patch("src.speech.tts_service.speak_with_system", return_value=True)
    @patch("src.speech.tts_service.speak_with_gemini", return_value=False)
    def test_lang_passed_to_system(self, mock_gemini, mock_sys):
        speak_text("语言参数", lang="en-US")
        mock_sys.assert_called_once_with("语言参数", "en-US")


# ============================================================
# API Key 安全性验证
# ============================================================

class TestKeySecurityGuard:

    def test_env_file_in_gitignore(self):
        gitignore_path = os.path.join(
            os.path.dirname(__file__), "..", ".gitignore"
        )
        content = open(gitignore_path).read()
        assert ".env" in content

    def test_env_file_exists_locally(self):
        env_path = os.path.join(os.path.dirname(__file__), "..", ".env")
        assert os.path.exists(env_path), ".env 文件应存在于本地"

    def test_env_example_exists(self):
        example_path = os.path.join(
            os.path.dirname(__file__), "..", ".env.example"
        )
        assert os.path.exists(example_path), ".env.example 应存在供其他开发者参考"

    def test_no_hardcoded_key_in_source(self):
        """确保源代码中没有硬编码 API Key。"""
        src_dir = os.path.join(os.path.dirname(__file__), "..", "src")
        for root, _, files in os.walk(src_dir):
            for f in files:
                if not f.endswith(".py"):
                    continue
                filepath = os.path.join(root, f)
                content = open(filepath).read()
                assert "AIzaSy" not in content, (
                    f"硬编码 API Key 泄露: {filepath}"
                )
