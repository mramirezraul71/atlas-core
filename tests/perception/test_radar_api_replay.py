from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_scanner.api.radar as radar_api
from atlas_scanner.store import JsonlRadarSnapshotStore


def test_radar_api_recent_and_replay_endpoints(tmp_path) -> None:
    radar_api._SNAPSHOT_PERSISTENCE = JsonlRadarSnapshotStore(tmp_path / "radar_api.jsonl")
    radar_api.build_realtime_snapshot(symbol="SPY", timeframes=("1m", "5m"), runtime_mode="paper")

    app = FastAPI()
    app.include_router(radar_api.router)
    client = TestClient(app)

    recent = client.get("/api/radar/recent", params={"limit": 10})
    assert recent.status_code == 200
    body_recent = recent.json()
    assert body_recent["ok"] is True
    assert body_recent["count"] >= 1

    replay = client.get("/api/radar/replay/SPY", params={"limit": 10})
    assert replay.status_code == 200
    body_replay = replay.json()
    assert body_replay["ok"] is True
    assert body_replay["symbol"] == "SPY"
    assert body_replay["count"] >= 1

    stats = client.get("/api/radar/store/stats")
    assert stats.status_code == 200
    body_stats = stats.json()
    assert body_stats["ok"] is True
    assert body_stats["store"]["total_records"] >= 1

    freshness = client.get("/api/radar/diagnostics/freshness/SPY", params={"timeframes": ["1m", "5m"]})
    assert freshness.status_code == 200
    body_freshness = freshness.json()
    assert body_freshness["ok"] is True
    assert len(body_freshness["items"]) >= 1

    calendar = client.get("/api/radar/macro/calendar/SPY")
    assert calendar.status_code == 200
    body_calendar = calendar.json()
    assert body_calendar["ok"] is True
    assert "calendar_risk_score" in body_calendar
