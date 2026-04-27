"""Tests F5 — VerticalSpreadStrategy.build_plan."""
from __future__ import annotations

from atlas_code_quant.strategies.contracts import (
    StrategyConfig,
    StrategyOpportunityRef,
)
from atlas_code_quant.strategies.options.vertical_spread import (
    VerticalSpreadStrategy,
)


def _opp(direction: str = "long", trace_id: str = "t-vs") -> StrategyOpportunityRef:
    return StrategyOpportunityRef(
        symbol="AAPL",
        score=72.5,
        direction=direction,  # type: ignore[arg-type]
        horizon_min=30,
        trace_id=trace_id,
    )


def test_build_plan_long_call_debit_spread() -> None:
    strat = VerticalSpreadStrategy()
    plan = strat.build_plan(_opp("long"))
    assert plan.status == "planned"
    assert plan.symbol == "AAPL"
    assert plan.direction == "long"
    assert plan.n_legs() == 2
    # buy call (long) + sell call (short)
    assert plan.legs[0].side == "buy"
    assert plan.legs[0].right == "call"
    assert plan.legs[1].side == "sell"
    assert plan.legs[1].right == "call"
    # short strike OTM (positivo para call)
    assert plan.legs[1].strike_offset > plan.legs[0].strike_offset
    assert plan.is_actionable() is True
    assert plan.trace_id == "t-vs"


def test_build_plan_short_put_debit_spread() -> None:
    strat = VerticalSpreadStrategy()
    plan = strat.build_plan(_opp("short"))
    assert plan.status == "planned"
    assert plan.direction == "short"
    assert plan.n_legs() == 2
    assert plan.legs[0].right == "put"
    assert plan.legs[1].right == "put"
    # short strike OTM hacia abajo (negativo para put)
    assert plan.legs[1].strike_offset < plan.legs[0].strike_offset


def test_build_plan_neutral_rejected() -> None:
    strat = VerticalSpreadStrategy()
    plan = strat.build_plan(_opp("neutral"))
    assert plan.status == "rejected"
    assert plan.is_actionable() is False
    assert "directional" in plan.rationale.lower()
    assert plan.legs == []


def test_build_plan_max_loss_bounded_by_config() -> None:
    strat = VerticalSpreadStrategy()
    cfg = StrategyConfig(max_loss_usd=80.0, cash_alloc_usd=2_000.0, width_usd=5.0, qty=2)
    plan = strat.build_plan(_opp("long"), cfg)
    assert plan.status == "planned"
    # max_loss <= max_loss_usd configurado
    assert plan.max_loss_estimate_usd <= 80.0
    # notional ~ width * 100 * qty
    assert plan.notional_estimate_usd == 5.0 * 100 * 2


def test_build_plan_accepts_dict_input() -> None:
    strat = VerticalSpreadStrategy()
    plan = strat.build_plan(
        {"symbol": "msft", "score": 65.0, "direction": "long", "trace_id": "x"}
    )
    assert plan.symbol == "MSFT"
    assert plan.status == "planned"
    assert plan.trace_id == "x"
