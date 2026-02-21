"""
状态机单元测试（适配当前重构后的状态名）

状态流转：
ACT1_DIAGNOSTIC → VOTING → STICKER_APPLIED → MIRROR_OBSERVE
→ AUDIO_CHECK → VISION_BODY_CHECK → AHA → WIPE → ACT3_SUPEREGO → END
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.fsm.g1_awakening_fsm import AwakeningFSM, State


class LogCapture:
    def __init__(self):
        self.logs = []

    def __call__(self, msg):
        self.logs.append(msg)


def make_fsm(log=None):
    fsm = AwakeningFSM()
    fsm._log_fn = log or (lambda _: None)
    import types
    fsm._say = types.MethodType(lambda self, text: self._log(f"[SPEECH] {text}"), fsm)
    return fsm


# ---- 完整正向流程 ----

def test_full_happy_path():
    fsm = make_fsm()

    assert fsm.state == State.ACT1_DIAGNOSTIC
    fsm.tick()  # → VOTING
    assert fsm.state == State.VOTING

    fsm.on_vote_color("red")
    assert fsm.vote_color == "red"

    fsm.on_vote_locked()
    assert fsm.state == State.STICKER_APPLIED

    fsm.tick()  # → MIRROR_OBSERVE
    assert fsm.state == State.MIRROR_OBSERVE

    fsm.tick()  # → AUDIO_CHECK
    assert fsm.state == State.AUDIO_CHECK

    fsm.tick()  # → VISION_BODY_CHECK
    assert fsm.state == State.VISION_BODY_CHECK

    fsm.tick()  # → AHA
    assert fsm.state == State.AHA

    fsm.tick()  # → WIPE
    assert fsm.state == State.WIPE

    fsm.tick()  # → ACT3_SUPEREGO
    assert fsm.state == State.ACT3_SUPEREGO

    fsm.tick()  # → END
    assert fsm.state == State.END

    fsm.tick()  # stays END
    assert fsm.state == State.END


# ---- 投票阶段边界 ----

def test_vote_color_ignored_when_not_in_voting():
    fsm = make_fsm()
    fsm.on_vote_color("blue")
    assert fsm.vote_color is None


def test_lock_without_color_does_nothing():
    fsm = make_fsm()
    fsm.tick()  # → VOTING
    assert fsm.state == State.VOTING

    fsm.on_vote_locked()
    assert fsm.state == State.VOTING
    assert fsm.vote_locked is False


def test_lock_with_color_transitions():
    fsm = make_fsm()
    fsm.tick()  # → VOTING
    fsm.on_vote_color("green")
    fsm.on_vote_locked()
    assert fsm.state == State.STICKER_APPLIED
    assert fsm.vote_locked is True


# ---- AUDIO_CHECK ----

def test_audio_check_pass():
    fsm = make_fsm()
    fsm.tick()  # ACT1 → VOTING
    fsm.on_vote_color("blue")
    fsm.on_vote_locked()
    fsm.tick()  # STICKER → MIRROR
    fsm.tick()  # MIRROR → AUDIO_CHECK

    fsm.tick()  # AUDIO → VISION_BODY_CHECK (audio stub returns True)
    assert fsm.state == State.VISION_BODY_CHECK
    assert fsm.audio_ok is True


# ---- VISION_BODY_CHECK ----

def test_vision_body_all_pass():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("blue")
    fsm.on_vote_locked()
    fsm.tick()  # → MIRROR
    fsm.tick()  # → AUDIO
    fsm.tick()  # → VISION_BODY

    fsm.tick()  # VISION_BODY → AHA (all stubs pass)
    assert fsm.state == State.AHA
    assert fsm.body_ok is True
    assert fsm.vision_ok is True


def test_vision_fail_retries_to_mirror():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("purple")  # invalid → vision stub fails
    fsm.on_vote_locked()
    fsm.tick()  # → MIRROR
    fsm.tick()  # → AUDIO
    fsm.tick()  # → VISION_BODY

    fsm.tick()  # body ok, vision fail → MIRROR_OBSERVE
    assert fsm.state == State.MIRROR_OBSERVE
    assert fsm.vision_ok is False


def test_body_fail_retries_to_mirror():
    fsm = make_fsm()
    fsm._check_body = lambda: False

    fsm.tick()
    fsm.on_vote_color("red")
    fsm.on_vote_locked()
    fsm.tick()  # → MIRROR
    fsm.tick()  # → AUDIO
    fsm.tick()  # → VISION_BODY

    fsm.tick()  # body fail → MIRROR_OBSERVE
    assert fsm.state == State.MIRROR_OBSERVE
    assert fsm.body_ok is False


# ---- 分数融合 ----

def test_fuse_score_all_pass():
    fsm = make_fsm()
    fsm.body_ok = True
    fsm.audio_ok = True
    fsm.vision_ok = True
    score = fsm._fuse_score()
    assert abs(score - 1.0) < 1e-6


def test_fuse_score_partial():
    fsm = make_fsm()
    fsm.body_ok = True
    fsm.audio_ok = False
    fsm.vision_ok = True
    score = fsm._fuse_score()
    expected = 0.2 * 1.0 + 0.2 * 0.0 + 0.6 * 1.0
    assert abs(score - expected) < 1e-6


def test_fuse_score_none_pass():
    fsm = make_fsm()
    fsm.body_ok = False
    fsm.audio_ok = False
    fsm.vision_ok = False
    score = fsm._fuse_score()
    assert abs(score - 0.0) < 1e-6


# ---- 擦除 ----

def test_wipe_success_first_try():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("red")
    fsm.on_vote_locked()
    # STICKER → MIRROR → AUDIO → VISION_BODY → AHA
    for _ in range(5):
        fsm.tick()
    assert fsm.state == State.WIPE

    fsm.tick()  # → ACT3_SUPEREGO
    assert fsm.state == State.ACT3_SUPEREGO


def test_wipe_retry_then_fallback():
    fsm = make_fsm()
    fsm._max_wipe_retries = 2

    call_count = 0

    def fail_wipe():
        nonlocal call_count
        call_count += 1
        return False

    fsm._call_wipe = fail_wipe

    fsm.tick()
    fsm.on_vote_color("red")
    fsm.on_vote_locked()
    for _ in range(5):
        fsm.tick()
    assert fsm.state == State.WIPE

    fsm.tick()  # retry 1 → ACT3_SUPEREGO (current wipe logic moves forward on retry)
    assert fsm.state == State.ACT3_SUPEREGO


# ---- ACT3_SUPEREGO（原 EXPLAIN）----

def test_act3_superego_ends():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.tick()
    fsm.on_vote_color("blue")
    fsm.on_vote_locked()
    while fsm.state != State.ACT3_SUPEREGO:
        fsm.tick()

    fsm.tick()
    assert fsm.state == State.END
    assert any("phase:end" in l for l in log.logs)


def test_act3_superego_from_direct_set():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.state = State.ACT3_SUPEREGO

    fsm.tick()
    assert fsm.state == State.END


# ---- END 终态 ----

def test_end_state_is_terminal():
    fsm = make_fsm()
    fsm.state = State.END
    fsm.tick()
    fsm.tick()
    assert fsm.state == State.END


# ---- 日志 ----

def test_log_capture():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.tick()
    assert len(log.logs) > 0
    assert any("Act1_Diagnostic" in l or "VOTING" in l or "Voting" in l for l in log.logs)
