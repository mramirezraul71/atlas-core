from __future__ import annotations

from fastapi.testclient import TestClient

from atlas_adapter import atlas_http_api
from atlas_adapter.services.update_center import UpdateCenterService


def _temp_update_center(monkeypatch, tmp_path):
    svc = UpdateCenterService(tmp_path)
    monkeypatch.setattr(atlas_http_api, "_UPDATE_CENTER", svc)
    monkeypatch.setattr(atlas_http_api, "_ensure_update_center_auto_scheduler", lambda: None)
    return svc


def test_update_center_config_endpoints(monkeypatch, tmp_path):
    _temp_update_center(monkeypatch, tmp_path)
    client = TestClient(atlas_http_api.app)

    res_get = client.get("/api/update-center/config")
    assert res_get.status_code == 200
    assert res_get.json()["ok"] is True

    res_post = client.post("/api/update-center/config", json={"enabled": False, "dry_run": True, "scan_interval_sec": 900})
    assert res_post.status_code == 200
    payload = res_post.json()["data"]
    assert payload["enabled"] is False
    assert payload["dry_run"] is True
    assert payload["scan_interval_sec"] == 900


def test_software_apply_one_blocked_by_policy(monkeypatch, tmp_path):
    _temp_update_center(monkeypatch, tmp_path)
    client = TestClient(atlas_http_api.app)

    fake_registry = {
        "ok": True,
        "software": [
            {
                "id": "trading-runtime-core",
                "name": "Trading Runtime Core",
                "category": "runtime",
                "install_method": "pip",
                "install_target": "trading-runtime-core",
                "update_ready": True,
            }
        ],
        "network_candidates": [],
    }

    monkeypatch.setattr(atlas_http_api, "_read_json_dict", lambda path: fake_registry)

    res = client.post("/api/software/apply-one", json={"item_id": "trading-runtime-core", "background": True})
    assert res.status_code == 200
    body = res.json()
    assert body["ok"] is False
    assert body["error"] == "blocked_by_policy"
    assert body["data"]["status"] == "blocked"
