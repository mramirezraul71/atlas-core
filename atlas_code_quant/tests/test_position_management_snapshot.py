from __future__ import annotations

from datetime import datetime, timedelta
from types import SimpleNamespace

from atlas_code_quant.data.var_monitor import VarSnapshot
from atlas_code_quant.journal import service as journal_service
from atlas_code_quant.journal.service import (
    build_exit_governance_snapshot,
    build_position_management_snapshot,
    build_post_trade_learning_snapshot,
)


def _entry(
    *,
    symbol: str,
    strategy_type: str,
    hours_ago: int,
    unrealized_pnl: float,
    risk_at_entry: float,
    entry_notional: float,
    win_rate_at_entry: float | None,
    current_win_rate_pct: float | None,
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
        win_rate_at_entry=win_rate_at_entry,
        current_win_rate_pct=current_win_rate_pct,
    )


def test_build_position_management_snapshot_flags_heat_staleness_and_thesis_drift() -> None:
    now = datetime(2026, 3, 28, 16, 0, 0)
    entries = [
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=80,
            unrealized_pnl=-120.0,
            risk_at_entry=300.0,
            entry_notional=3000.0,
            win_rate_at_entry=68.0,
            current_win_rate_pct=45.0,
        ),
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=10,
            unrealized_pnl=-30.0,
            risk_at_entry=200.0,
            entry_notional=2200.0,
            win_rate_at_entry=62.0,
            current_win_rate_pct=50.0,
        ),
        _entry(
            symbol="MSFT",
            strategy_type="equity_short",
            hours_ago=8,
            unrealized_pnl=40.0,
            risk_at_entry=100.0,
            entry_notional=1500.0,
            win_rate_at_entry=55.0,
            current_win_rate_pct=58.0,
        ),
    ]

    snapshot = build_position_management_snapshot(entries, account_type="paper", limit=6, now=now)

    assert snapshot["summary"]["open_positions"] == 3
    assert snapshot["summary"]["adverse_positions_count"] == 1
    assert snapshot["summary"]["stale_positions_count"] == 1
    assert snapshot["summary"]["thesis_deteriorated_count"] == 1
    assert snapshot["summary"]["max_symbol_heat_pct"] > 60.0
    assert snapshot["alerts"]
    assert any(alert["code"] == "symbol_heat_exceeded" for alert in snapshot["alerts"])
    assert snapshot["watchlist"]
    assert snapshot["watchlist"][0]["symbol"] == "AAPL"
    assert "stale_loser" in snapshot["watchlist"][0]["alert_reasons"]
    assert "thesis_drift" in snapshot["watchlist"][0]["alert_reasons"]


def test_build_exit_governance_snapshot_classifies_recommendations() -> None:
    now = datetime(2026, 3, 28, 16, 0, 0)
    entries = [
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=20,
            unrealized_pnl=-180.0,
            risk_at_entry=300.0,
            entry_notional=3000.0,
            win_rate_at_entry=68.0,
            current_win_rate_pct=44.0,
        ),
        _entry(
            symbol="NVDA",
            strategy_type="equity_long",
            hours_ago=18,
            unrealized_pnl=210.0,
            risk_at_entry=200.0,
            entry_notional=2500.0,
            win_rate_at_entry=63.0,
            current_win_rate_pct=61.0,
        ),
        _entry(
            symbol="MSFT",
            strategy_type="equity_long",
            hours_ago=100,
            unrealized_pnl=-5.0,
            risk_at_entry=120.0,
            entry_notional=1800.0,
            win_rate_at_entry=57.0,
            current_win_rate_pct=57.0,
        ),
    ]

    snapshot = build_exit_governance_snapshot(entries, account_type="paper", limit=6, now=now)

    assert snapshot["summary"]["exit_now_count"] >= 1
    assert snapshot["summary"]["take_profit_count"] >= 1
    assert snapshot["alerts"]
    assert snapshot["recommendations"][0]["recommendation"] == "exit_now"
    assert snapshot["recommendations"][0]["exit_reason"] in {"hard_stop_loss_r", "thesis_invalidated"}


def test_build_exit_governance_snapshot_surfaces_portfolio_var_pressure(monkeypatch) -> None:
    class _CriticalVarMonitor:
        def is_enabled(self) -> bool:
            return True

        def compute(self, *_: object, **__: object) -> VarSnapshot:
            return VarSnapshot(
                enabled=True,
                method="monte_carlo_portfolio",
                simulation_count=750,
                confidence_level_pct=95.0,
                horizon_days=1,
                var_95_usd=620.0,
                cvar_95_usd=760.0,
                var_95_pct_of_book=8.6,
                threshold_usd=400.0,
                status="critical",
                drivers=["symbol_concentration"],
                diversified_risk_usd=320.0,
                gross_risk_usd=540.0,
                concentration_multiplier=1.4,
                loss_multiplier=1.2,
                monte_carlo_var_usd=700.0,
                monte_carlo_cvar_usd=820.0,
                expected_loss_usd=210.0,
                worst_case_loss_usd=980.0,
                net_directional_exposure_pct=68.0,
            )

    monkeypatch.setattr(journal_service, "VarMonitor", lambda: _CriticalVarMonitor())
    now = datetime(2026, 3, 28, 16, 0, 0)
    entries = [
        _entry(
            symbol="AAPL",
            strategy_type="equity_long",
            hours_ago=80,
            unrealized_pnl=-5.0,
            risk_at_entry=200.0,
            entry_notional=2500.0,
            win_rate_at_entry=60.0,
            current_win_rate_pct=60.0,
        ),
    ]
    entries.extend(
        [
            _entry(
                symbol=symbol,
                strategy_type="equity_long",
                hours_ago=12,
                unrealized_pnl=10.0,
                risk_at_entry=200.0,
                entry_notional=2200.0,
                win_rate_at_entry=59.0,
                current_win_rate_pct=57.0,
            )
            for symbol in ("MSFT", "NVDA", "AMZN", "META", "TSLA", "AMD", "GOOGL", "QQQ")
        ]
    )

    snapshot = build_exit_governance_snapshot(entries, account_type="paper", limit=6, now=now)

    assert snapshot["summary"]["var_status"] == "critical"
    assert snapshot["summary"]["var_method"] == "monte_carlo_portfolio"
    assert snapshot["summary"]["var_95_usd"] == 620.0
    assert any(alert["code"] == "portfolio_var_exit_pressure" for alert in snapshot["alerts"])
    assert any(item["exit_reason"] == "portfolio_var_limit" for item in snapshot["recommendations"])


def test_build_post_trade_learning_snapshot_surfaces_policy_candidates() -> None:
    now = datetime(2026, 3, 28, 16, 0, 0)
    closed_entries = [
        SimpleNamespace(
            status="closed",
            account_type="paper",
            strategy_type="equity_long",
            symbol="AAPL",
            realized_pnl=-120.0,
            updated_at=now,
            post_mortem_json='{"root_cause":"direccion","driver":"delta"}',
        ),
        SimpleNamespace(
            status="closed",
            account_type="paper",
            strategy_type="equity_long",
            symbol="MSFT",
            realized_pnl=-40.0,
            updated_at=now - timedelta(hours=1),
            post_mortem_json='{"root_cause":"direccion","driver":"delta"}',
        ),
        SimpleNamespace(
            status="closed",
            account_type="paper",
            strategy_type="equity_short",
            symbol="NVDA",
            realized_pnl=80.0,
            updated_at=now - timedelta(hours=2),
            post_mortem_json='{"root_cause":"timing","driver":"theta"}',
        ),
    ]

    snapshot = build_post_trade_learning_snapshot(closed_entries, account_type="paper", limit=5)

    assert snapshot["summary"]["closed_trades"] == 3
    assert snapshot["summary"]["post_mortem_coverage_pct"] == 100.0
    assert snapshot["root_cause_breakdown"][0]["root_cause"] == "direccion"
    assert snapshot["policy_candidates"]
    assert snapshot["policy_candidates"][0]["strategy_type"] == "equity_long"
