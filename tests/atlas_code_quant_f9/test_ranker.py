"""Tests F9.2 — Strategy ranker + factory.build_candidates."""
from __future__ import annotations

from atlas_code_quant.backtest.engine import BacktestEngine
from atlas_code_quant.strategies.contracts import (
    OptionLeg,
    StrategyOpportunityRef,
    StrategyPlan,
)
from atlas_code_quant.strategies.factory import StrategyFactory
from atlas_code_quant.strategies.ranker import (
    RankerPolicy,
    compute_fitness,
    rank_strategies,
)


def _bt_result(**kw):
    from atlas_code_quant.backtest.engine import BacktestResult
    base = dict(
        strategy="vertical_spread",
        symbol="SPY",
        trades_count=60,
        sharpe=0.6,
        profit_factor=1.50,
        win_rate=0.62,
        expectancy=20.0,
        max_drawdown=0.04,
        total_pnl=1200.0,
        avg_win=110.0,
        avg_loss=-150.0,
        trace_id="t",
        source="stub",
        rejected=False,
        rejection_reasons=[],
    )
    base.update(kw)
    return BacktestResult(**base)


def test_compute_fitness_in_unit_range() -> None:
    f = compute_fitness(_bt_result())
    assert 0.0 <= f <= 1.0


def test_rank_picks_highest_fitness() -> None:
    a = _bt_result(strategy="iron_condor",     profit_factor=1.20, win_rate=0.55, max_drawdown=0.06)
    b = _bt_result(strategy="vertical_spread", profit_factor=1.80, win_rate=0.65, max_drawdown=0.03)
    c = _bt_result(strategy="iron_butterfly",  profit_factor=1.10, win_rate=0.54, max_drawdown=0.07)
    out = rank_strategies([a, b, c])
    assert out.winner is not None
    assert out.winner.strategy == "vertical_spread"
    # Justificación menciona ganadora y al menos una otra
    assert "vertical_spread" in out.justification


def test_rank_rejects_below_thresholds() -> None:
    bad_pf = _bt_result(strategy="iron_condor", profit_factor=1.05)
    bad_wr = _bt_result(strategy="iron_butterfly", win_rate=0.40)
    bad_dd = _bt_result(strategy="straddle_strangle", max_drawdown=0.20)
    bad_n = _bt_result(strategy="vertical_spread", trades_count=5)
    out = rank_strategies([bad_pf, bad_wr, bad_dd, bad_n])
    assert out.winner is None
    assert all(not r.accepted for r in out.ranked)
    assert "No candidate met acceptance policy" in out.justification


def test_rank_handles_mixed_pool() -> None:
    good = _bt_result(strategy="vertical_spread", profit_factor=2.0, win_rate=0.66, max_drawdown=0.03)
    bad = _bt_result(strategy="iron_condor", profit_factor=0.9)
    out = rank_strategies([good, bad])
    assert out.winner is not None
    assert out.winner.strategy == "vertical_spread"
    rejected = [r for r in out.ranked if not r.accepted]
    assert any(r.strategy == "iron_condor" for r in rejected)


def test_factory_build_candidates_neutral_yields_three() -> None:
    opp = StrategyOpportunityRef(
        symbol="SPY", score=72.0, direction="neutral",
        horizon_min=60, trace_id="trace-1",
    )
    plans = StrategyFactory.build_candidates(opp)
    # Neutral debe producir condor + butterfly + straddle/strangle (3 plans) y descartar vertical
    names = {p.strategy for p in plans}
    # straddle_strangle puede emitir 'straddle_strangle.straddle' o equivalente
    assert "iron_condor" in names
    assert "iron_butterfly" in names
    assert any(n.startswith("straddle_strangle") for n in names)
    assert "vertical_spread" not in names
    assert all(p.status == "planned" for p in plans)
    assert len(plans) >= 3


def test_factory_build_candidates_long_yields_vertical_only() -> None:
    opp = StrategyOpportunityRef(
        symbol="QQQ", score=80.0, direction="long",
        horizon_min=30, trace_id="trace-2",
    )
    plans = StrategyFactory.build_candidates(opp)
    names = {p.strategy for p in plans}
    # Long bias solo califica vertical_spread
    assert "vertical_spread" in names
    assert all(p.status == "planned" for p in plans)


def test_factory_then_ranker_end_to_end() -> None:
    opp = StrategyOpportunityRef(
        symbol="NVDA", score=78.0, direction="neutral",
        horizon_min=60, trace_id="trace-3",
    )
    plans = StrategyFactory.build_candidates(opp)
    assert len(plans) >= 2
    engine = BacktestEngine(n_trades=50)
    results = [engine.evaluate(p, opportunity_score=opp.score, trace_id=opp.trace_id) for p in plans]
    out = rank_strategies(results)
    # Debe haber un ganador o justificación explícita
    if out.winner is not None:
        assert out.winner.fitness > 0.0
        assert out.winner.strategy in {p.strategy for p in plans}
    else:
        assert "No candidate" in out.justification
