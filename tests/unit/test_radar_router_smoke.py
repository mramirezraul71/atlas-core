from __future__ import annotations

import sys

from fastapi.testclient import TestClient


def test_radar_router_mount_and_smoke(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_ROUTER", "true")
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "radar_smoke.jsonl"))
    monkeypatch.setenv("ATLAS_FLOW_PROVIDER", "synthetic")

    for mod_name in (
        "atlas_scanner.api.radar",
        "atlas_scanner.api",
        "atlas_adapter.atlas_http_api",
    ):
        sys.modules.pop(mod_name, None)

    import atlas_adapter.atlas_http_api as http_api  # noqa: WPS433

    client = TestClient(http_api.app)
    health = client.get("/api/radar/health/providers")
    assert health.status_code == 200
    snapshot = client.get("/api/radar/snapshot/SPY", params={"timeframes": ["1m", "5m"], "refresh": True})
    assert snapshot.status_code == 200
    replay = client.get("/api/radar/recent", params={"limit": 5})
    assert replay.status_code == 200
    assert replay.json()["ok"] is True
