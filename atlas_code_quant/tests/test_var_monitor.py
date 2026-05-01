from __future__ import annotations

import json
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

from atlas_code_quant.data.var_monitor import VarMonitor
from atlas_code_quant.journal import service as journal_service


def _state_file(tmp_path: Path, *, enabled: bool = True) -> Path:
    path = tmp_path / "operation_center_state.json"
    path.write_text(json.dumps({"var_monitor_enabled": enabled}), encoding="utf-8")
    return path


def _position(risk_budget: float) -> dict[str, float]:
    return {"risk_budget": risk_budget}


def _entry(
    *,
    symbol: str,
    strategy_type: str,
    hours_ago: int,
    unrealized_pnl: float,
    risk_at_entry: float,
    entry_notional: float,
    account_type: str = "paper",
) -> SimpleNamespace:
    now = datetime(2026, 3, 28, 16, 0, 0)
    return SimpleNamespace(
        status="open",
        symbol=symbol,
        strategy_type=strategy_type,
        account_type=account_type,
        entry_time=now - timedelta(hours=hours_ago),
        updated_at=now,
        journal_key=f"{symbol}-{strategy_type}",
        strategy_id=f"{strategy_type}:{symbol}",
        unrealized_pnl=unrealized_pnl,
        risk_at_entry=risk_at_entry,
        entry_notional=entry_notional,
        win_rate_at_entry=60.0,
        current_win_rate_pct=50.0,
    )


def test_var_monitor_disabled_returns_zero_snapshot(tmp_path: Path) -> None:
    monitor = VarMonitor(state_path=_state_file(tmp_path, enabled=False))

    snapshot = monitor.compute(
        [_position(300.0)],
        total_risk_budget=300.0,
        total_entry_notional=3000.0,
        max_symbol_heat_pct=50.0,
        adverse_positions_count=1,
        total_unrealized_pnl=-100.0,
    )

    assert not snapshot.enabled
    assert snapshot.var_95_usd == 0.0
    assert snapshot.status == "disabled"
    assert snapshot.method == "disabled"


def test_var_monitor_escalates_status_for_concentrated_losing_book(tmp_path: Path) -> None:
    monitor = VarMonitor(state_path=_state_file(tmp_path, enabled=True))

    snapshot = monitor.compute(
        [_position(350.0), _position(250.0), _position(150.0)],
        total_risk_budget=750.0,
        total_entry_notional=4200.0,
        max_symbol_heat_pct=78.0,
        adverse_positions_count=2,
        total_unrealized_pnl=-260.0,
    )

    assert snapshot.enabled
    assert snapshot.var_95_usd > 0.0
    assert snapshot.cvar_95_usd > snapshot.var_95_usd
    assert snapshot.monte_carlo_var_usd > 0.0
    assert snapshot.monte_carlo_cvar_usd >= snapshot.monte_carlo_var_usd
    assert snapshot.method == "monte_carlo_portfolio"
    assert snapshot.status in {"warning", "critical"}
    assert "symbol_concentration" in snapshot.drivers


def test_var_monitor_monte_carlo_penalizes_concentration_more_than_diversified_book(tmp_path: Path) -> None:
    monitor = VarMonitor(state_path=_state_file(tmp_path, enabled=True))

    concentrated = monitor.compute(
        [
            {"symbol": "AAPL", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -50.0},
            {"symbol": "AAPL", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -35.0},
            {"symbol": "AAPL", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -25.0},
        ],
        total_risk_budget=750.0,
        total_entry_notional=7500.0,
        max_symbol_heat_pct=100.0,
        adverse_positions_count=3,
        total_unrealized_pnl=-110.0,
    )
    diversified = monitor.compute(
        [
            {"symbol": "AAPL", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -50.0},
            {"symbol": "MSFT", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -35.0},
            {"symbol": "NVDA", "strategy_type": "equity_long", "risk_budget": 250.0, "entry_notional": 2500.0, "unrealized_pnl": -25.0},
        ],
        total_risk_budget=750.0,
        total_entry_notional=7500.0,
        max_symbol_heat_pct=33.34,
        adverse_positions_count=3,
        total_unrealized_pnl=-110.0,
    )

    assert concentrated.monte_carlo_var_usd > diversified.monte_carlo_var_usd
    assert concentrated.var_95_usd > diversified.var_95_usd


def test_position_management_snapshot_exposes_var_monitor_payload(tmp_path: Path, monkeypatch) -> None:
    monitor = VarMonitor(state_path=_state_file(tmp_path, enabled=True))
    monkeypatch.setattr(journal_service, "VarMonitor", lambda: monitor)
    now = datetime(2026, 3, 28, 16, 0, 0)
    entries = [
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=40,
            unrealized_pnl=-150.0,
            risk_at_entry=300.0,
            entry_notional=3000.0,
        ),
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=8,
            unrealized_pnl=-35.0,
            risk_at_entry=240.0,
            entry_notional=2500.0,
        ),
        _entry(
            symbol="MSFT",
            strategy_type="equity_short",
            hours_ago=6,
            unrealized_pnl=20.0,
            risk_at_entry=120.0,
            entry_notional=1600.0,
        ),
    ]

    snapshot = journal_service.build_position_management_snapshot(entries, account_type="paper", limit=6, now=now)

    assert snapshot["var_monitor"]["enabled"] is True
    assert snapshot["var_monitor"]["method"] == "monte_carlo_portfolio"
    assert snapshot["summary"]["var_method"] == "monte_carlo_portfolio"
    assert snapshot["summary"]["var_95_usd"] == snapshot["var_monitor"]["var_95_usd"]
    assert snapshot["summary"]["cvar_95_usd"] == snapshot["var_monitor"]["cvar_95_usd"]
    assert snapshot["summary"]["var_status"] == snapshot["var_monitor"]["status"]
    assert snapshot["var_monitor"]["simulation_count"] > 0
    assert snapshot["var_monitor"]["monte_carlo_var_usd"] > 0.0
    assert snapshot["var_monitor"]["var_95_usd"] > 0.0
