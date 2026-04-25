from __future__ import annotations

import sys
from fastapi.testclient import TestClient


def _reload_http_api(monkeypatch, tmp_path, *, stream_enabled: bool = True, heartbeat_sec: str = "1") -> TestClient:
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_ROUTER", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_DASHBOARD", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_STREAM", "true" if stream_enabled else "false")
    monkeypatch.setenv("ATLAS_RADAR_STREAM_MODE", "sse")
    monkeypatch.setenv("ATLAS_RADAR_STREAM_HEARTBEAT_SEC", heartbeat_sec)
    monkeypatch.setenv("ATLAS_RADAR_BACKPLANE_TYPE", "memory")
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "radar_stream_snapshot.jsonl"))
    monkeypatch.setenv("ATLAS_DECISION_GATE_STORE_PATH", str(tmp_path / "radar_stream_decisions.jsonl"))
    for mod_name in (
        "atlas_scanner.api.radar",
        "atlas_scanner.ui.events",
        "atlas_scanner.ui.backplane.redis",
        "atlas_scanner.ui.dashboard",
        "atlas_scanner.ui",
        "atlas_adapter.atlas_http_api",
    ):
        sys.modules.pop(mod_name, None)
    import atlas_adapter.atlas_http_api as http_api  # noqa: WPS433

    return TestClient(http_api.app)
def test_sse_endpoint_responds(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, stream_enabled=True, heartbeat_sec="1")
    response = client.get("/api/radar/stream?once=true")
    assert response.status_code == 200
    assert "text/event-stream" in response.headers.get("content-type", "")


def test_sse_emits_heartbeat(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, stream_enabled=True, heartbeat_sec="1")
    response = client.get("/api/radar/stream?once=true")
    assert "event: heartbeat" in response.text


def test_sse_emits_snapshot_and_decision(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, stream_enabled=True, heartbeat_sec="1")
    snapshot = client.get("/api/radar/snapshot/SPY", params={"timeframes": ["1m", "5m"], "refresh": True})
    assert snapshot.status_code == 200

    response = client.get("/api/radar/stream?once=true", headers={"last-event-id": "0"})
    assert "event: snapshot" in response.text
    assert "event: decision" in response.text


def test_dashboard_js_has_stream_fallback_hooks() -> None:
    with open("atlas_scanner/ui/assets/dashboard.js", "r", encoding="utf-8") as handle:
        source = handle.read()
    assert "setupStreamingWithFallback" in source
    assert "typeof EventSource === \"undefined\"" in source
    assert "Polling activo" in source


def test_stream_disabled_keeps_safe_response(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, stream_enabled=False, heartbeat_sec="1")
    response = client.get("/api/radar/stream?once=true")
    assert response.status_code == 200
    assert "stream_disabled" in response.text


def test_stream_metrics_endpoint(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path, stream_enabled=True, heartbeat_sec="1")
    response = client.get("/api/radar/stream/metrics")
    assert response.status_code == 200
    body = response.json()
    assert body["ok"] is True
    assert body["configured_backplane"] == "memory"
    assert "metrics" in body
