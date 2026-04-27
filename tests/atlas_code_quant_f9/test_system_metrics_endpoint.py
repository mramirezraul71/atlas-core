"""Tests F9.4 — /api/system/metrics + /api/paper/{open,close,monitor}."""
from __future__ import annotations

import os

import pytest
from fastapi.testclient import TestClient

from atlas_code_quant.execution.runtime import reset_runtime


@pytest.fixture()
def client(monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.setenv("ATLAS_LIVE_TRADING_ENABLED", "false")
    monkeypatch.setenv("ATLAS_TRADIER_DRY_RUN", "true")
    monkeypatch.setenv("ATLAS_SKIP_LIVE_SERVICE_TESTS", "1")
    reset_runtime()
    # Importar después de setenv para que startup respete paper-first
    from atlas_adapter.atlas_http_api import app  # noqa: WPS433
    return TestClient(app)


def test_metrics_endpoint_is_paper_first(client: TestClient) -> None:
    r = client.get("/api/system/metrics")
    assert r.status_code == 200
    data = r.json()
    for key in ("radar", "strategy", "execution", "risk", "journal"):
        assert key in data
    assert data["execution"]["live_trading_enabled"] is False
    assert data["execution"]["tradier_dry_run"] is True
    assert data["execution"]["positions_open"] == 0
    assert data["execution"]["positions_closed"] == 0
    assert data["journal"]["entries_total"] == 0


def _open_payload(trace_id: str) -> dict:
    return {
        "strategy": "vertical_spread",
        "symbol": "SPY",
        "direction": "long",
        "legs": [
            {"side": "buy", "right": "call", "strike_offset": 0.0,
             "qty": 1, "expiry_dte": 14},
            {"side": "sell", "right": "call", "strike_offset": 5.0,
             "qty": 1, "expiry_dte": 14},
        ],
        "notional_estimate_usd": 500.0,
        "max_loss_estimate_usd": 200.0,
        "horizon_min": 30,
        "rationale": "test",
        "trace_id": trace_id,
        "entry_price": 2.0,
    }


def test_paper_open_close_flow_updates_metrics(client: TestClient) -> None:
    # 1. Abrir posición
    r = client.post("/api/paper/open", json=_open_payload("e2e-fix-1"))
    assert r.status_code == 200, r.text
    pos = r.json()["position"]
    pos_id = pos["position_id"]
    assert pos["status"] == "open"
    assert pos["take_profit_price"] == 3.0  # 2.0 * 1.5

    # 2. Métricas reflejan 1 abierta
    m = client.get("/api/system/metrics").json()
    assert m["execution"]["positions_open"] == 1
    assert m["execution"]["positions_closed"] == 0
    assert m["journal"]["trade_opens"] == 1

    # 3. Monitor sugiere TP
    mon = client.post(
        "/api/paper/monitor",
        json={"position_id": pos_id, "last_price": 3.10},
    ).json()
    assert mon["action"] == "close"
    assert mon["reason"] == "take_profit"

    # 4. Cerrar al precio sugerido
    r2 = client.post(
        "/api/paper/close",
        json={
            "position_id": pos_id,
            "exit_price": mon["suggested_exit_price"],
            "reason": "take_profit",
        },
    )
    assert r2.status_code == 200, r2.text
    closed = r2.json()["position"]
    assert closed["status"] == "closed"
    assert closed["realized_pnl_usd"] == 200.0  # (3-2)*2*100

    # 5. Métricas finales
    m2 = client.get("/api/system/metrics").json()
    assert m2["execution"]["positions_open"] == 0
    assert m2["execution"]["positions_closed"] == 1
    assert m2["execution"]["realized_pnl_usd_total"] == 200.0
    assert m2["journal"]["trade_opens"] == 1
    assert m2["journal"]["trade_closes"] == 1
    assert m2["journal"]["complete_cycles"] == 1


def test_paper_open_blocked_when_live_enabled(
    client: TestClient, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("ATLAS_LIVE_TRADING_ENABLED", "true")
    r = client.post("/api/paper/open", json=_open_payload("e2e-fix-block"))
    assert r.status_code == 403
    assert "live_trading_enabled" in r.json()["detail"]
