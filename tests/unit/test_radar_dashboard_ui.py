from __future__ import annotations

import sys

from fastapi.testclient import TestClient


def _reload_http_api(monkeypatch) -> TestClient:
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_ROUTER", "true")
    monkeypatch.setenv("ATLAS_ENABLE_RADAR_DASHBOARD", "true")
    for mod_name in (
        "atlas_scanner.api.radar",
        "atlas_scanner.ui.dashboard",
        "atlas_scanner.ui",
        "atlas_adapter.atlas_http_api",
    ):
        sys.modules.pop(mod_name, None)
    import atlas_adapter.atlas_http_api as http_api  # noqa: WPS433

    return TestClient(http_api.app)


def test_radar_dashboard_route_serves_html(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "dashboard_radar.jsonl"))
    client = _reload_http_api(monkeypatch)
    response = client.get("/radar/dashboard")
    assert response.status_code == 200
    assert "text/html" in response.headers.get("content-type", "")
    assert "Institutional Radar" in response.text


def test_radar_dashboard_assets_served(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "dashboard_assets.jsonl"))
    client = _reload_http_api(monkeypatch)

    css = client.get("/radar/dashboard/assets/dashboard.css")
    assert css.status_code == 200
    assert "text/css" in css.headers.get("content-type", "")
    assert "--bg-primary" in css.text

    js = client.get("/radar/dashboard/assets/dashboard.js")
    assert js.status_code == 200
    assert "application/javascript" in js.headers.get("content-type", "")
    assert "refreshAll" in js.text


def test_radar_dashboard_invalid_asset_path(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ATLAS_SCANNER_RADAR_SNAPSHOT_PATH", str(tmp_path / "dashboard_invalid.jsonl"))
    client = _reload_http_api(monkeypatch)
    response = client.get("/radar/dashboard/assets/..%5Csecrets.txt")
    assert response.status_code == 400
