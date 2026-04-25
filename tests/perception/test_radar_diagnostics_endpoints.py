from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_scanner.api.radar as radar_api


def test_radar_diagnostics_endpoints_operational(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "radar_diag.jsonl"))
    radar_api.build_realtime_snapshot(symbol="SPY", timeframes=("1m", "5m", "1h"), runtime_mode="paper")
    app = FastAPI()
    app.include_router(radar_api.router)
    client = TestClient(app)

    providers = client.get("/api/radar/diagnostics/providers")
    assert providers.status_code == 200
    assert providers.json()["ok"] is True
    assert "summary" in providers.json()

    structural = client.get("/api/radar/diagnostics/structural/SPY")
    assert structural.status_code == 200
    assert structural.json()["ok"] is True
    assert "structural_confidence_score" in structural.json()

    fast = client.get("/api/radar/diagnostics/fast/SPY")
    assert fast.status_code == 200
    assert fast.json()["ok"] is True
    assert "fast_pressure_score" in fast.json()

    dealer = client.get("/api/radar/dealer/SPY")
    assert dealer.status_code == 200
    assert dealer.json()["ok"] is True
    assert "dealer" in dealer.json()

    political = client.get("/api/radar/political/SPY")
    assert political.status_code == 200
    assert political.json()["ok"] is True
    assert "political_context" in political.json()
