"""
投票桥接服务 —— FastAPI → ROS2

手机端扫码投票，通过 HTTP 接口接收颜色选择，
再桥接到 ROS2 话题 /show/vote_color 和 /show/vote_locked。
"""

from __future__ import annotations

import json
import asyncio
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool

    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False


ALLOWED_COLORS = {"red", "blue", "green"}

vote_state = {
    "red": 0,
    "blue": 0,
    "green": 0,
    "locked": False,
    "winner": None,
}

screen_clients: list[WebSocket] = []

ros2_node: Optional[object] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros2_node
    if HAS_ROS2:
        rclpy.init()
        ros2_node = rclpy.create_node("vote_bridge")
    yield
    if HAS_ROS2 and ros2_node:
        ros2_node.destroy_node()
        rclpy.shutdown()


app = FastAPI(title="ensoul vote bridge", lifespan=lifespan)


TEMPLATES_DIR = Path(__file__).parent / "templates"
SCREEN_DIR = Path(__file__).parent / "screen"


class VoteRequest(BaseModel):
    color: str


@app.get("/", response_class=HTMLResponse)
async def vote_page():
    html_path = TEMPLATES_DIR / "vote.html"
    return HTMLResponse(html_path.read_text(encoding="utf-8"))


@app.get("/screen", response_class=HTMLResponse)
async def screen_page():
    html_path = SCREEN_DIR / "screen_ui.html"
    return HTMLResponse(html_path.read_text(encoding="utf-8"))


@app.post("/api/vote")
async def cast_vote(req: VoteRequest):
    if vote_state["locked"]:
        return {"ok": False, "error": "投票已结束"}
    color = req.color.lower()
    if color not in ALLOWED_COLORS:
        return {"ok": False, "error": f"颜色必须为 {ALLOWED_COLORS}"}

    vote_state[color] += 1

    if HAS_ROS2 and ros2_node:
        pub = ros2_node.create_publisher(String, "/show/vote_color", 10)
        msg = String()
        msg.data = color
        pub.publish(msg)

    await _broadcast({"type": "vote_update", "state": _safe_state()})
    return {"ok": True, "state": _safe_state()}


@app.post("/api/lock")
async def lock_vote():
    if vote_state["locked"]:
        return {"ok": False, "error": "已锁定"}

    vote_state["locked"] = True
    winner = max(ALLOWED_COLORS, key=lambda c: vote_state[c])
    vote_state["winner"] = winner

    if HAS_ROS2 and ros2_node:
        pub_color = ros2_node.create_publisher(String, "/show/vote_color", 10)
        pub_lock = ros2_node.create_publisher(Bool, "/show/vote_locked", 10)
        m1 = String()
        m1.data = winner
        pub_color.publish(m1)
        m2 = Bool()
        m2.data = True
        pub_lock.publish(m2)

    await _broadcast({"type": "vote_locked", "winner": winner, "state": _safe_state()})
    return {"ok": True, "winner": winner, "state": _safe_state()}


@app.post("/api/reset")
async def reset_vote():
    for c in ALLOWED_COLORS:
        vote_state[c] = 0
    vote_state["locked"] = False
    vote_state["winner"] = None
    await _broadcast({"type": "vote_reset", "state": _safe_state()})
    return {"ok": True}


@app.websocket("/ws/screen")
async def ws_screen(ws: WebSocket):
    await ws.accept()
    screen_clients.append(ws)
    try:
        await ws.send_json({"type": "init", "state": _safe_state()})
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        screen_clients.remove(ws)


def _safe_state() -> dict:
    return {k: v for k, v in vote_state.items()}


async def _broadcast(payload: dict) -> None:
    data = json.dumps(payload, ensure_ascii=False)
    for ws in list(screen_clients):
        try:
            await ws.send_text(data)
        except Exception:
            screen_clients.remove(ws)
