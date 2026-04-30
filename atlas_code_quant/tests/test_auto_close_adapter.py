"""Tests para options.auto_close_adapter y endpoint close-candidates."""
from __future__ import annotations

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine
from atlas_code_quant.options.auto_close_adapter import AutoCloseReportBuilder

pytest.importorskip("fastapi")


class _FakePortfolio:
    def __init__(self, entries):
        self._entries = list(entries)

    def get_open_entries(self):
        return list(self._entries)


def _sample_entries():
    return [
        {
            "position_id": "tp",
            "strategy_type": "bull_put_credit_spread",
            "entry_credit": 2.0,
            "current_value": 1.0,
            "remaining_dte": 35,
            "is_credit": True,
            "underlying": "SPY",
            "opened_at": "2026-04-01T12:00:00",
            "tags": ["income"],
        },
        {
            "position_id": "sl",
            "strategy_type": "bear_call_credit_spread",
            "entry_credit": 2.0,
            "current_value": 7.0,
            "remaining_dte": 30,
            "is_credit": True,
            "underlying": "SPY",
        },
        {
            "position_id": "dte",
            "strategy_type": "iron_condor",
            "entry_credit": 2.0,
            "current_value": 2.1,
            "remaining_dte": 10,
            "is_credit": True,
            "underlying": "QQQ",
        },
        {
            "position_id": "hold",
            "strategy_type": "iron_condor",
            "entry_credit": 2.0,
            "current_value": 1.8,
            "remaining_dte": 40,
            "is_credit": True,
            "underlying": "IWM",
        },
    ]


def test_build_report_groups_candidates_and_holds() -> None:
    portfolio = _FakePortfolio(_sample_entries())
    builder = AutoCloseReportBuilder(portfolio, AutoCloseEngine())

    report = builder.build_report(include_hold=True)

    assert report["total_positions"] == 4
    assert len(report["close_candidates"]) == 3
    assert len(report["held_positions"]) == 1
    assert report["held_positions"][0]["position_id"] == "hold"

    ids = [row["position_id"] for row in report["close_candidates"]]
    # stop_loss tiene prioridad high, debe ir primero.
    assert ids[0] == "sl"
    reason_map = {row["position_id"]: set(row["reasons"]) for row in report["close_candidates"]}
    assert "take_profit" in reason_map["tp"]
    assert "stop_loss" in reason_map["sl"]
    assert "dte_gate" in reason_map["dte"]


def test_build_report_omits_hold_list_by_default() -> None:
    portfolio = _FakePortfolio(_sample_entries())
    builder = AutoCloseReportBuilder(portfolio, AutoCloseEngine())
    report = builder.build_report()
    assert report["held_positions"] == []


def test_close_candidates_endpoint_returns_payload(monkeypatch) -> None:
    from atlas_code_quant.api.routes import options as options_routes

    fake_portfolio = _FakePortfolio(_sample_entries())
    monkeypatch.setattr(options_routes, "_portfolio", fake_portfolio)

    app = FastAPI()
    app.include_router(options_routes.router)
    with TestClient(app) as client:
        resp = client.get("/options/close-candidates?include_hold=true")

    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    data = body["data"]
    assert data["total_positions"] == 4
    assert "close_candidates" in data
    assert "held_positions" in data
