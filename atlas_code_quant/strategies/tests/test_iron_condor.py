"""Tests F5 — IronCondorStrategy.build_plan."""
from __future__ import annotations

from atlas_code_quant.strategies.contracts import (
    StrategyConfig,
    StrategyOpportunityRef,
)
from atlas_code_quant.strategies.options.iron_condor import IronCondorStrategy


def _opp(direction: str, trace_id: str = "t-ic") -> StrategyOpportunityRef:
    return StrategyOpportunityRef(
        symbol="SPY",
        score=58.0,
        direction=direction,  # type: ignore[arg-type]
        horizon_min=45,
        trace_id=trace_id,
    )


def test_build_plan_neutral_four_legs() -> None:
    strat = IronCondorStrategy()
    plan = strat.build_plan(_opp("neutral"))
    assert plan.status == "planned"
    assert plan.direction == "neutral"
    assert plan.n_legs() == 4
    # 2 puts (1 short, 1 long deep OTM) + 2 calls (1 short, 1 long deep OTM)
    rights = [leg.right for leg in plan.legs]
    sides = [leg.side for leg in plan.legs]
    assert rights.count("put") == 2
    assert rights.count("call") == 2
    assert sides.count("buy") == 2
    assert sides.count("sell") == 2
    assert plan.is_actionable() is True


def test_build_plan_long_rejected() -> None:
    strat = IronCondorStrategy()
    plan = strat.build_plan(_opp("long"))
    assert plan.status == "rejected"
    assert plan.is_actionable() is False
    assert "neutral" in plan.rationale.lower()
    assert plan.legs == []


def test_build_plan_short_rejected() -> None:
    strat = IronCondorStrategy()
    plan = strat.build_plan(_opp("short"))
    assert plan.status == "rejected"
    assert plan.legs == []


def test_build_plan_max_loss_bounded_and_wings_consistent() -> None:
    strat = IronCondorStrategy()
    cfg = StrategyConfig(max_loss_usd=999_999.0, width_usd=4.0, qty=1)
    plan = strat.build_plan(_opp("neutral"), cfg)
    assert plan.status == "planned"
    # Si max_loss_usd es muy grande, debe limitarse por wing*100*qty
    assert plan.max_loss_estimate_usd <= 4.0 * 100 * 1
    # Long puts strike más bajo que short put; long call strike más alto que short call
    put_short = next(l for l in plan.legs if l.right == "put" and l.side == "sell")
    put_long = next(l for l in plan.legs if l.right == "put" and l.side == "buy")
    call_short = next(l for l in plan.legs if l.right == "call" and l.side == "sell")
    call_long = next(l for l in plan.legs if l.right == "call" and l.side == "buy")
    assert put_long.strike_offset < put_short.strike_offset
    assert call_long.strike_offset > call_short.strike_offset


def test_build_plan_preserves_trace_id_and_horizon() -> None:
    strat = IronCondorStrategy()
    plan = strat.build_plan(_opp("neutral", trace_id="abc-123"))
    assert plan.trace_id == "abc-123"
    assert plan.horizon_min == 45
