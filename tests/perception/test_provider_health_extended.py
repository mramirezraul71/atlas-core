from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_scanner.api.radar as radar_api


def test_provider_health_exposes_extended_fields(tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "health_ext.jsonl"))
    radar_api.build_realtime_snapshot(symbol="SPY", timeframes=("1m",), runtime_mode="paper")
    app = FastAPI()
    app.include_router(radar_api.router)
    client = TestClient(app)
    response = client.get("/api/radar/health/providers")
    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    providers = body["providers"]
    assert isinstance(providers, dict)
    first = next(iter(providers.values()))
    assert "latency_ms" in first
    assert "error_rate" in first
    assert "circuit_state" in first
    assert "fallback_active" in first
    assert "provider_name" in first
    assert "latency_ms" in first
    assert "p95_latency_ms" in first
    assert "burst_error_indicator" in first
    assert "circuit_open_indicator" in first
    assert any(key.startswith("ownership:") for key in providers.keys())
    assert any(key.startswith("insider:") for key in providers.keys())
    assert any(key.startswith("political:") for key in providers.keys())
    assert any(key.startswith("regulatory:") for key in providers.keys())
    assert any(key.startswith("options_chain:") for key in providers.keys())
