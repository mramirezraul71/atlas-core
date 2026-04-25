from __future__ import annotations

import sys

from fastapi.testclient import TestClient


def _reload_http_api(monkeypatch, tmp_path) -> TestClient:
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_ROUTER", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_DASHBOARD", "true")
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "camera_snapshots.jsonl"))
    monkeypatch.setenv("ATLAS_DECISION_GATE_STORE_PATH", str(tmp_path / "camera_decisions.jsonl"))
    monkeypatch.setenv("ATLAS_CAMERA_PROVIDER", "stub")
    monkeypatch.setenv("ATLAS_CAMERA_CAPTURE_ENABLED", "false")
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


def test_camera_health_endpoint_and_snapshot_meta(monkeypatch, tmp_path) -> None:
    client = _reload_http_api(monkeypatch, tmp_path)
    health = client.get("/api/radar/sensors/camera/health")
    assert health.status_code == 200
    health_body = health.json()
    assert health_body["ok"] is True
    assert "camera" in health_body
    assert health_body["camera"]["status"] in {"disabled", "degraded", "ready"}

    capture = client.get("/api/radar/sensors/camera/capture")
    assert capture.status_code == 200
    capture_body = capture.json()
    assert capture_body["ok"] is True
    assert "camera" in capture_body
