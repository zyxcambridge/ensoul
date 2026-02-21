"""
投票 API 单元测试 (FastAPI TestClient)

覆盖：
- 投票成功
- 投票后锁定
- 锁定后再投票被拒绝
- 非法颜色被拒绝
- 重置后可重新投票
- 投票页 / 大屏页 HTML 返回
- 投票计数正确性
- 多色投票胜者正确
"""

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from fastapi.testclient import TestClient

from src.web import vote_bridge


@pytest.fixture(autouse=True)
def reset_state():
    """每个测试前重置投票状态。"""
    for c in vote_bridge.ALLOWED_COLORS:
        vote_bridge.vote_state[c] = 0
    vote_bridge.vote_state["locked"] = False
    vote_bridge.vote_state["winner"] = None
    vote_bridge.screen_clients.clear()
    yield


@pytest.fixture
def client():
    return TestClient(vote_bridge.app)


class TestCastVote:

    def test_vote_red(self, client):
        resp = client.post("/api/vote", json={"color": "red"})
        data = resp.json()
        assert data["ok"] is True
        assert data["state"]["red"] == 1

    def test_vote_blue(self, client):
        resp = client.post("/api/vote", json={"color": "blue"})
        assert resp.json()["ok"] is True
        assert resp.json()["state"]["blue"] == 1

    def test_vote_green(self, client):
        resp = client.post("/api/vote", json={"color": "green"})
        assert resp.json()["ok"] is True
        assert resp.json()["state"]["green"] == 1

    def test_vote_invalid_color(self, client):
        resp = client.post("/api/vote", json={"color": "purple"})
        data = resp.json()
        assert data["ok"] is False
        assert "error" in data

    def test_vote_case_insensitive(self, client):
        resp = client.post("/api/vote", json={"color": "RED"})
        assert resp.json()["ok"] is True
        assert resp.json()["state"]["red"] == 1

    def test_multiple_votes_accumulate(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "blue"})
        resp = client.post("/api/vote", json={"color": "red"})
        state = resp.json()["state"]
        assert state["red"] == 3
        assert state["blue"] == 1
        assert state["green"] == 0


class TestLockVote:

    def test_lock_determines_winner(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "blue"})
        resp = client.post("/api/lock")
        data = resp.json()
        assert data["ok"] is True
        assert data["winner"] == "red"

    def test_vote_after_lock_rejected(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/lock")
        resp = client.post("/api/vote", json={"color": "blue"})
        assert resp.json()["ok"] is False

    def test_double_lock_rejected(self, client):
        client.post("/api/vote", json={"color": "green"})
        client.post("/api/lock")
        resp = client.post("/api/lock")
        assert resp.json()["ok"] is False

    def test_lock_with_tie_picks_one(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "blue"})
        resp = client.post("/api/lock")
        data = resp.json()
        assert data["ok"] is True
        assert data["winner"] in {"red", "blue", "green"}


class TestReset:

    def test_reset_clears_state(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/lock")
        resp = client.post("/api/reset")
        assert resp.json()["ok"] is True

        resp2 = client.post("/api/vote", json={"color": "blue"})
        assert resp2.json()["ok"] is True
        assert resp2.json()["state"]["red"] == 0
        assert resp2.json()["state"]["blue"] == 1

    def test_reset_unlocks(self, client):
        client.post("/api/vote", json={"color": "red"})
        client.post("/api/lock")
        client.post("/api/reset")

        resp = client.post("/api/vote", json={"color": "green"})
        assert resp.json()["ok"] is True


class TestPages:

    def test_vote_page_returns_html(self, client):
        resp = client.get("/")
        assert resp.status_code == 200
        assert "text/html" in resp.headers["content-type"]

    def test_screen_page_returns_html(self, client):
        resp = client.get("/screen")
        assert resp.status_code == 200
        assert "text/html" in resp.headers["content-type"]
