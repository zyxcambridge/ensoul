"""
G1 镜像觉醒 —— 核心状态机节点 (ROS2 rclpy)

状态流转：
Idle → Voting → StickerApplied → MirrorObserve
→ TripleCheck → AhaMoment → WipeAction → Explain → End
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, Float32
    from std_srvs.srv import Trigger

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


class State(Enum):
    IDLE = "Idle"
    VOTING = "Voting"
    STICKER_APPLIED = "StickerApplied"
    MIRROR_OBSERVE = "MirrorObserve"
    TRIPLE_CHECK = "TripleCheck"
    AHA = "AhaMoment"
    WIPE = "WipeAction"
    EXPLAIN = "Explain"
    END = "End"


class AwakeningFSM:
    """
    演示状态机。在 ROS2 环境下作为 Node 运行，
    在无 ROS2 环境下以纯 Python 模拟运行（方便本地调试）。
    """

    def __init__(self) -> None:
        self.state: State = State.IDLE
        self.vote_color: Optional[str] = None
        self.vote_locked: bool = False

        self.body_ok: bool = False
        self.audio_ok: bool = False
        self.vision_ok: bool = False
        self.match_score: float = 0.0

        self._wipe_retries: int = 0
        self._max_wipe_retries: int = 2
        self._log_fn = print

    # ---- 外部事件入口 ----

    def on_vote_color(self, color: str) -> None:
        if self.state == State.VOTING:
            self.vote_color = color
            self._log(f"收到投票颜色: {color}")

    def on_vote_locked(self) -> None:
        if self.state == State.VOTING and self.vote_color:
            self.vote_locked = True
            self._log(f"投票锁定: {self.vote_color}")
            self._transition(State.STICKER_APPLIED)

    # ---- 主循环 ----

    def tick(self) -> None:
        handler = {
            State.IDLE: self._handle_idle,
            State.VOTING: self._handle_voting,
            State.STICKER_APPLIED: self._handle_sticker_applied,
            State.MIRROR_OBSERVE: self._handle_mirror_observe,
            State.TRIPLE_CHECK: self._handle_triple_check,
            State.AHA: self._handle_aha,
            State.WIPE: self._handle_wipe,
            State.EXPLAIN: self._handle_explain,
            State.END: lambda: None,
        }.get(self.state)
        if handler:
            handler()

    # ---- 各状态处理 ----

    def _handle_idle(self) -> None:
        self._transition(State.VOTING)

    def _handle_voting(self) -> None:
        pass  # 等待外部事件 on_vote_locked

    def _handle_sticker_applied(self) -> None:
        self._say("助手已贴好贴纸，我将观察镜子中的目标。")
        self._transition(State.MIRROR_OBSERVE)

    def _handle_mirror_observe(self) -> None:
        self._say("我看到了一个额头有标记的机器人……我还不确定那是不是我。")
        self._publish_screen("phase:mirror_observe")
        self._transition(State.TRIPLE_CHECK)

    def _handle_triple_check(self) -> None:
        self.body_ok = self._check_body()
        self._publish_screen("check:body_ok" if self.body_ok else "check:body_fail")
        self._say("我感知到我的手臂在动。" if self.body_ok else "本体反馈异常。")

        self.audio_ok = self._check_audio()
        self._publish_screen("check:audio_ok" if self.audio_ok else "check:audio_fail")
        self._say("我检测到刚才发出的是我的声音。" if self.audio_ok else "语音回听异常。")

        self.vision_ok = self._check_vision()
        self._publish_screen("check:vision_ok" if self.vision_ok else "check:vision_fail")

        self.match_score = self._fuse_score()
        self._publish_score(self.match_score)

        if self.body_ok and self.audio_ok and self.vision_ok:
            self._say(f"镜中的颜色和位置……与我完全匹配。匹配度 {self.match_score:.0%}。")
            self._transition(State.AHA)
        else:
            self._say("我还不能确认，需要重试。")
            self._transition(State.MIRROR_OBSERVE)

    def _handle_aha(self) -> None:
        self._say("我确认了：镜子里的，就是我自己。")
        self._publish_screen("phase:aha_moment")
        self._transition(State.WIPE)

    def _handle_wipe(self) -> None:
        success = self._call_wipe()
        if success:
            self._publish_screen("wipe:success")
            self._transition(State.EXPLAIN)
        elif self._wipe_retries < self._max_wipe_retries:
            self._wipe_retries += 1
            self._say("动作未命中，启用保守轨迹重试。")
            self._publish_screen("wipe:retry")
        else:
            self._say("擦除失败，请助手协助。")
            self._publish_screen("wipe:manual_fallback")
            self._transition(State.EXPLAIN)

    def _handle_explain(self) -> None:
        color_name = {"red": "红色", "blue": "蓝色", "green": "绿色"}.get(
            self.vote_color or "", self.vote_color or "未知"
        )
        self._say(f"谢谢大家选择{color_name}，我已经认出并擦掉了它。")
        self._say("我是能确认自身的系统。")
        self._say("我来自你们给我的未知任务。")
        self._say("我将去理解更多'我与世界'的关系。")
        self._publish_screen("phase:end")
        self._transition(State.END)

    # ---- 感知桩方法（ROS2 模式下替换为真实订阅） ----

    def _check_body(self) -> bool:
        return True

    def _check_audio(self) -> bool:
        return True

    def _check_vision(self) -> bool:
        return self.vote_color in {"red", "blue", "green"}

    def _fuse_score(self) -> float:
        w_body, w_audio, w_vision = 0.2, 0.2, 0.6
        return (
            w_body * (1.0 if self.body_ok else 0.0)
            + w_audio * (1.0 if self.audio_ok else 0.0)
            + w_vision * (1.0 if self.vision_ok else 0.0)
        )

    def _call_wipe(self) -> bool:
        return True

    # ---- 输出桩方法 ----

    def _say(self, text: str) -> None:
        self._log(f"[SPEECH] {text}")

    def _publish_screen(self, event: str) -> None:
        self._log(f"[SCREEN] {event}")

    def _publish_score(self, score: float) -> None:
        self._log(f"[SCORE] {score:.2f}")

    # ---- 内部工具 ----

    def _transition(self, next_state: State) -> None:
        self._log(f"[FSM] {self.state.value} → {next_state.value}")
        self.state = next_state

    def _log(self, msg: str) -> None:
        self._log_fn(msg)


# ===========================================================
# ROS2 Node 封装（仅在有 rclpy 时生效）
# ===========================================================

if HAS_ROS2:

    class AwakeningNode(Node):
        def __init__(self) -> None:
            super().__init__("g1_awakening_fsm")
            self.fsm = AwakeningFSM()
            self.fsm._log_fn = lambda msg: self.get_logger().info(msg)

            self.sub_color = self.create_subscription(
                String, "/show/vote_color", self._on_color, 10
            )
            self.sub_locked = self.create_subscription(
                Bool, "/show/vote_locked", self._on_locked, 10
            )

            self.pub_status = self.create_publisher(String, "/g1/self_check/status", 10)
            self.pub_score = self.create_publisher(Float32, "/g1/self_check/score", 10)
            self.pub_screen = self.create_publisher(String, "/show/screen_event", 10)

            self.timer = self.create_timer(0.1, self._tick)

            self.fsm._publish_screen = self._pub_screen_event
            self.fsm._publish_score = self._pub_score_value

        def _on_color(self, msg: String) -> None:
            self.fsm.on_vote_color(msg.data)

        def _on_locked(self, msg: Bool) -> None:
            if msg.data:
                self.fsm.on_vote_locked()

        def _tick(self) -> None:
            self.fsm.tick()

        def _pub_screen_event(self, event: str) -> None:
            m = String()
            m.data = event
            self.pub_screen.publish(m)

        def _pub_score_value(self, score: float) -> None:
            m = Float32()
            m.data = score
            self.pub_score.publish(m)


# ===========================================================
# 入口
# ===========================================================

def main() -> None:
    if HAS_ROS2:
        rclpy.init()
        node = AwakeningNode()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("=== 无 ROS2 环境，进入模拟模式 ===")
        fsm = AwakeningFSM()
        fsm.tick()  # Idle → Voting
        fsm.on_vote_color("red")
        fsm.on_vote_locked()
        while fsm.state != State.END:
            fsm.tick()
            time.sleep(0.3)
        print("=== 演示结束 ===")


if __name__ == "__main__":
    main()
