from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_scanner.api.radar as radar_api
from atlas_scanner.decision.store import JsonlDecisionGateStore


def test_decision_endpoints_expose_gate_evaluations(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "radar_gate.jsonl"))
    monkeypatch.setenv("ATLAS_DECISION_GATE_ENABLED", "true")
    monkeypatch.setenv("ATLAS_DECISION_GATE_MODE", "paper_supervised")
    radar_api._DECISION_STORE = JsonlDecisionGateStore(tmp_path / "decision_gate.jsonl")  # noqa: SLF001
    radar_api.build_realtime_snapshot(symbol="SPY", timeframes=("1m", "5m"), runtime_mode="paper")
    app = FastAPI()
    app.include_router(radar_api.router)
    client = TestClient(app)

    recent = client.get("/api/radar/decisions/recent", params={"limit": 20})
    assert recent.status_code == 200
    body_recent = recent.json()
    assert body_recent["ok"] is True
    assert body_recent["count"] >= 1
    assert "decision" in body_recent["items"][0]

    by_symbol = client.get("/api/radar/decisions/SPY", params={"limit": 20})
    assert by_symbol.status_code == 200
    body_symbol = by_symbol.json()
    assert body_symbol["ok"] is True
    assert body_symbol["symbol"] == "SPY"
    assert body_symbol["count"] >= 1

    replay = client.get("/api/radar/decisions/replay/SPY", params={"limit": 20})
    assert replay.status_code == 200
    assert replay.json()["ok"] is True
    assert replay.json()["count"] >= 1

    stats = client.get("/api/radar/decisions/stats")
    assert stats.status_code == 200
    assert stats.json()["ok"] is True
    assert stats.json()["stats"]["total_decisions"] >= 1

    summary = client.get("/api/radar/dashboard/summary", params={"symbol": "SPY", "limit": 10})
    assert summary.status_code == 200
    body_summary = summary.json()
    assert body_summary["ok"] is True
    assert "decision_gate_stats" in body_summary
    assert "provider_health_summary" in body_summary
