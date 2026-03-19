from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient

import atlas_adapter.routes.trading_quant as trading_quant
from atlas_adapter.routes.trading_quant import build_router


def test_trading_quant_router_exposes_expected_paths():
    router = build_router()
    paths = {route.path for route in router.routes}

    assert "/api/trading/quant/vision-bridge/status" in paths
    assert "/api/trading/quant/vision-cycle" in paths


def test_trading_quant_router_delegates_to_bridge(monkeypatch):
    monkeypatch.setattr(
        trading_quant,
        "get_quant_bridge_status",
        lambda: {"ok": True, "quant_api_base": "http://127.0.0.1:8792"},
    )
    monkeypatch.setattr(
        trading_quant,
        "run_quant_vision_cycle",
        lambda **kwargs: {
            "ok": True,
            "generated_at": "2026-03-19T00:00:00Z",
            "vision_snapshot": {"capture_ok": True},
            "quant": {"ok": True, "payload": {"ok": True}},
            "kwargs": kwargs,
        },
    )

    app = FastAPI()
    app.include_router(build_router())
    client = TestClient(app)

    status = client.get("/api/trading/quant/vision-bridge/status").json()
    cycle = client.post(
        "/api/trading/quant/vision-cycle",
        json={
            "order": {"symbol": "SPY"},
            "action": "preview",
            "include_vision": True,
            "capture_target": "nexus:camera",
            "screen_target": {"x": 2000, "y": 900},
        },
    ).json()

    assert status["quant_api_base"] == "http://127.0.0.1:8792"
    assert cycle["ok"] is True
    assert cycle["kwargs"]["order"] == {"symbol": "SPY"}
    assert cycle["kwargs"]["action"] == "preview"
    assert cycle["kwargs"]["capture_target"] == "nexus:camera"
