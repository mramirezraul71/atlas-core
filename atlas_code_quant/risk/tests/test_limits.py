"""Tests F6 — Risk Limits."""
from __future__ import annotations

from atlas_code_quant.risk.limits import (
    RiskLimits,
    RiskState,
    TradeIntent,
    check,
)


def _limits() -> RiskLimits:
    return RiskLimits(
        max_position_notional_usd=1_000.0,
        max_daily_loss_usd=500.0,
        max_open_positions=3,
        per_symbol_max_notional_usd=2_000.0,
    )


def test_allowed_when_all_ok() -> None:
    state = RiskState(realized_pnl_today_usd=-100.0, open_positions=1, notional_per_symbol_usd={})
    intent = TradeIntent(symbol="AAPL", notional_usd=500.0, side="open")
    d = check(intent, state, _limits())
    assert d.allowed is True


def test_blocked_by_daily_loss() -> None:
    state = RiskState(realized_pnl_today_usd=-501.0, open_positions=1)
    intent = TradeIntent(symbol="AAPL", notional_usd=100.0)
    d = check(intent, state, _limits())
    assert d.allowed is False
    assert d.reason == "max_daily_loss_breached"


def test_blocked_by_max_open_positions() -> None:
    state = RiskState(realized_pnl_today_usd=0.0, open_positions=3)
    intent = TradeIntent(symbol="AAPL", notional_usd=100.0)
    d = check(intent, state, _limits())
    assert d.allowed is False
    assert d.reason == "max_open_positions_reached"


def test_blocked_by_position_notional() -> None:
    state = RiskState(realized_pnl_today_usd=0.0, open_positions=0)
    intent = TradeIntent(symbol="AAPL", notional_usd=1_500.0)
    d = check(intent, state, _limits())
    assert d.allowed is False
    assert d.reason == "position_notional_exceeds_limit"


def test_blocked_by_per_symbol_cap() -> None:
    state = RiskState(
        realized_pnl_today_usd=0.0,
        open_positions=1,
        notional_per_symbol_usd={"AAPL": 1_800.0},
    )
    intent = TradeIntent(symbol="AAPL", notional_usd=500.0)
    d = check(intent, state, _limits())
    assert d.allowed is False
    assert d.reason == "per_symbol_notional_exceeds_limit"


def test_close_intent_always_allowed() -> None:
    state = RiskState(realized_pnl_today_usd=-10_000.0, open_positions=999)
    intent = TradeIntent(symbol="X", notional_usd=99_999.0, side="close")
    d = check(intent, state, _limits())
    assert d.allowed is True
    assert d.reason == "closing"
