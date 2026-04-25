from __future__ import annotations

import sys

from fastapi.testclient import TestClient


def _reload_http_api(monkeypatch, tmp_path, *, v4_enabled: bool = True) -> TestClient:
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_ROUTER", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_DASHBOARD", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_STREAM", "true")
    monkeypatch.setenv("ATLAS_RADAR_STREAM_MODE", "sse")
    monkeypatch.setenv("ATLAS_RADAR_STREAM_HEARTBEAT_SEC", "1")
    monkeypatch.setenv("ATLAS_RADAR_V4_INTEGRATION_MODE", "true" if v4_enabled else "false")
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "v4_snapshots.jsonl"))
    monkeypatch.setenv("ATLAS_DECISION_GATE_STORE_PATH", str(tmp_path / "v4_decisions.jsonl"))
    for mod_name in (
        "atlas_scanner.api.radar",
        "atlas_scanner.ui.dashboard",
        "atlas_scanner.ui",
        "atlas_adapter.atlas_http_api",
    ):
        sys.modules.pop(mod_name, None)
    import atlas_adapter.atlas_http_api as http_api  # noqa: WPS433

    return TestClient(http_api.app)


def test_v4_summary_contract(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, v4_enabled=True)
    warm = client.get("/api/radar/snapshot/SPY", params={"timeframes": ["1m", "5m"], "refresh": True})
    assert warm.status_code == 200

    response = client.get("/api/radar/v4/summary", params={"symbol": "SPY", "top_n": 5, "decisions_limit": 20})
    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["integration_mode"] == "v4"
    assert "radar_status" in body
    assert "top_signals" in body
    assert "gate_recent_decisions" in body
    assert "provider_health_summary" in body
    assert "degradations_active" in body
    assert "structural_context_summary" in body
    assert "fast_context_summary" in body
    assert "freshness" in body
    assert "last_update" in body
    assert "stream_available" in body


def test_v4_config_capabilities(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, v4_enabled=True)
    response = client.get("/api/radar/v4/config")
    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert "domains_active" in body
    assert "providers_available" in body
    assert "modes_supported" in body
    assert "timeframes_supported" in body
    assert "streaming" in body
    assert "decision_gate_thresholds" in body


def test_v4_endpoints_disabled_by_flag(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, v4_enabled=False)
    summary = client.get("/api/radar/v4/summary")
    config = client.get("/api/radar/v4/config")
    assert summary.status_code == 404
    assert config.status_code == 404


def test_existing_radar_endpoints_unchanged(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, v4_enabled=True)
    snapshot = client.get("/api/radar/snapshot/SPY", params={"timeframes": ["1m", "5m"], "refresh": True})
    decisions = client.get("/api/radar/decisions/recent", params={"limit": 10})
    dashboard = client.get("/api/radar/dashboard/summary", params={"symbol": "SPY"})
    assert snapshot.status_code == 200
    assert decisions.status_code == 200
    assert dashboard.status_code == 200
