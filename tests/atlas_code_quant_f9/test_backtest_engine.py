"""Tests F9.1 — Backtest engine determinista."""
from __future__ import annotations

from atlas_code_quant.backtest import BacktestEngine, BacktestResult
from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyPlan,
)


def _make_plan(strategy: str = "vertical_spread", symbol: str = "SPY") -> StrategyPlan:
    return StrategyPlan(
        strategy=strategy,
        symbol=symbol,
        direction="long",
        legs=[
            OptionLeg(side="buy", right="call", strike_offset=0.0, qty=1, expiry_dte=14),
            OptionLeg(side="sell", right="call", strike_offset=5.0, qty=1, expiry_dte=14),
        ],
        notional_estimate_usd=500.0,
        max_loss_estimate_usd=250.0,
        horizon_min=60,
        rationale="test",
        status="planned",
        trace_id="t-1",
    )


def test_engine_produces_full_metrics() -> None:
    plan = _make_plan()
    engine = BacktestEngine(n_trades=50)
    res = engine.evaluate(plan, opportunity_score=75.0, trace_id="trace-1")

    assert isinstance(res, BacktestResult)
    assert res.trades_count == 50
    assert res.symbol == "SPY"
    assert res.strategy == "vertical_spread"
    assert res.trace_id == "trace-1"
    assert res.rejected is False
    # Métricas estándar presentes y con tipos numéricos
    for field_name in (
        "sharpe", "profit_factor", "win_rate", "expectancy",
        "max_drawdown", "total_pnl", "avg_win", "avg_loss",
    ):
        assert isinstance(getattr(res, field_name), float)
    assert 0.0 <= res.win_rate <= 1.0
    assert res.profit_factor >= 0.0


def test_engine_is_deterministic() -> None:
    plan = _make_plan()
    engine = BacktestEngine(n_trades=30)
    a = engine.evaluate(plan, opportunity_score=70.0)
    b = engine.evaluate(plan, opportunity_score=70.0)
    assert a.sharpe == b.sharpe
    assert a.profit_factor == b.profit_factor
    assert a.win_rate == b.win_rate
    assert a.total_pnl == b.total_pnl


def test_engine_rejects_non_actionable_plan() -> None:
    rejected = StrategyPlan(
        strategy="iron_condor",
        symbol="QQQ",
        direction="long",
        legs=[],
        status="rejected",
        rationale="needs neutral",
        trace_id="t-2",
    )
    engine = BacktestEngine()
    res = engine.evaluate(rejected, opportunity_score=70.0)
    assert res.rejected is True
    assert res.trades_count == 0
    assert res.profit_factor == 0.0
    assert res.rejection_reasons


def test_score_modifier_changes_win_rate() -> None:
    plan = _make_plan(strategy="iron_condor", symbol="QQQ")
    plan.direction = "neutral"  # condor exige neutral, pero engine no le exige eso
    engine = BacktestEngine(n_trades=120)
    high = engine.evaluate(plan, opportunity_score=85.0)
    low = engine.evaluate(plan, opportunity_score=55.0)
    # Score alto debe producir un WR estrictamente mayor que score bajo
    assert high.win_rate > low.win_rate


def test_strategy_profiles_differentiate() -> None:
    engine = BacktestEngine(n_trades=80)
    base = _make_plan("vertical_spread", "AAPL")
    cond = _make_plan("iron_condor", "AAPL")
    strad = _make_plan("straddle_strangle", "AAPL")
    rs = {p.strategy: engine.evaluate(p, opportunity_score=70.0) for p in (base, cond, strad)}
    # No deben colapsar al mismo número
    pfs = {k: v.profit_factor for k, v in rs.items()}
    assert len(set(pfs.values())) >= 2
