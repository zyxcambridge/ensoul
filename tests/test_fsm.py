"""
状态机单元测试

覆盖：
- 完整正向流程 (Idle → End)
- 投票颜色未设定时不能锁票
- 非 VOTING 状态下投票被忽略
- 三重检查失败时回退到 MirrorObserve
- 擦除失败重试 + 超过最大次数兜底
- 分数融合权重计算正确性
- 颜色中文映射
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
    return fsm


# ---- 完整正向流程 ----

def test_full_happy_path():
    fsm = make_fsm()

    assert fsm.state == State.IDLE
    fsm.tick()
    assert fsm.state == State.VOTING

    fsm.on_vote_color("red")
    assert fsm.vote_color == "red"

    fsm.on_vote_locked()
    assert fsm.state == State.STICKER_APPLIED

    fsm.tick()
    assert fsm.state == State.MIRROR_OBSERVE

    fsm.tick()
    assert fsm.state == State.TRIPLE_CHECK

    fsm.tick()
    assert fsm.state == State.AHA

    fsm.tick()
    assert fsm.state == State.WIPE

    fsm.tick()
    assert fsm.state == State.EXPLAIN

    fsm.tick()
    assert fsm.state == State.END

    fsm.tick()
    assert fsm.state == State.END


# ---- 投票阶段边界 ----

def test_vote_color_ignored_when_not_voting():
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


# ---- 三重检查 ----

def test_triple_check_all_pass():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("blue")
    fsm.on_vote_locked()
    fsm.tick()  # STICKER → MIRROR
    fsm.tick()  # MIRROR → TRIPLE

    fsm.tick()  # TRIPLE → AHA (all pass by default stubs)
    assert fsm.state == State.AHA
    assert fsm.body_ok is True
    assert fsm.audio_ok is True
    assert fsm.vision_ok is True


def test_triple_check_vision_fail_retries():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("purple")  # 不在合法颜色中，视觉桩会返回 False
    fsm.on_vote_locked()
    fsm.tick()  # → MIRROR
    fsm.tick()  # → TRIPLE

    fsm.tick()  # TRIPLE check → vision fails → MIRROR_OBSERVE
    assert fsm.state == State.MIRROR_OBSERVE
    assert fsm.vision_ok is False


def test_triple_check_body_fail():
    fsm = make_fsm()
    fsm._check_body = lambda: False

    fsm.tick()
    fsm.on_vote_color("red")
    fsm.on_vote_locked()
    fsm.tick()  # → MIRROR
    fsm.tick()  # → TRIPLE

    fsm.tick()
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
    expected = 0.2 * 1.0 + 0.2 * 0.0 + 0.6 * 1.0  # 0.8
    assert abs(score - expected) < 1e-6


def test_fuse_score_none_pass():
    fsm = make_fsm()
    fsm.body_ok = False
    fsm.audio_ok = False
    fsm.vision_ok = False
    score = fsm._fuse_score()
    assert abs(score - 0.0) < 1e-6


# ---- 擦除重试 ----

def test_wipe_success_first_try():
    fsm = make_fsm()
    fsm.tick()
    fsm.on_vote_color("red")
    fsm.on_vote_locked()
    for _ in range(4):  # STICKER → MIRROR → TRIPLE → AHA
        fsm.tick()
    assert fsm.state == State.WIPE

    fsm.tick()
    assert fsm.state == State.EXPLAIN


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
    for _ in range(4):
        fsm.tick()
    assert fsm.state == State.WIPE

    fsm.tick()  # retry 1
    assert fsm.state == State.WIPE
    assert fsm._wipe_retries == 1

    fsm.tick()  # retry 2
    assert fsm.state == State.WIPE
    assert fsm._wipe_retries == 2

    fsm.tick()  # max retries exceeded → EXPLAIN (manual fallback)
    assert fsm.state == State.EXPLAIN


# ---- Explain 阶段颜色中文 ----

def test_explain_uses_chinese_color_name():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.tick()
    fsm.on_vote_color("blue")
    fsm.on_vote_locked()
    while fsm.state != State.EXPLAIN:
        fsm.tick()

    fsm.tick()
    assert fsm.state == State.END
    assert any("蓝色" in l for l in log.logs)


def test_explain_unknown_color_fallback():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.state = State.EXPLAIN
    fsm.vote_color = "yellow"

    fsm.tick()
    assert fsm.state == State.END
    assert any("yellow" in l for l in log.logs)


# ---- END 状态不再变化 ----

def test_end_state_is_terminal():
    fsm = make_fsm()
    fsm.state = State.END
    fsm.tick()
    fsm.tick()
    assert fsm.state == State.END


# ---- 日志输出 ----

def test_log_capture():
    log = LogCapture()
    fsm = make_fsm(log=log)
    fsm.tick()
    assert len(log.logs) > 0
    assert any("Idle" in l for l in log.logs)
