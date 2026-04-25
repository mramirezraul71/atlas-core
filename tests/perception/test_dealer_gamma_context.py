from __future__ import annotations

from datetime import datetime, timezone

from atlas_scanner.contracts import FlowPerceptionSnapshot
from atlas_scanner.perception.market.gamma_context import build_gamma_context, to_dealer_positioning_snapshot


def test_gamma_context_with_real_chain_builds_flip_walls_and_composite() -> None:
    now = datetime.now(timezone.utc)
    flow = FlowPerceptionSnapshot(symbol="SPY", timeframe="5m", as_of=now, net_bias=0.1, net_gamma=0.2)
    context = build_gamma_context(
        symbol="SPY",
        as_of=now,
        spot_price=500.0,
        options_chain={
            "gamma_by_strike": {495.0: -120.0, 500.0: -20.0, 505.0: 80.0, 510.0: 110.0},
            "oi_by_strike": {495.0: 35000.0, 500.0: 85000.0, 505.0: 90000.0, 510.0: 25000.0},
            "call_oi_by_strike": {500.0: 40000.0, 505.0: 70000.0},
            "put_oi_by_strike": {495.0: 30000.0, 500.0: 45000.0},
            "dte_weights": {"0-7": 1.3, "8-30": 1.0, "31+": 0.7},
        },
        flow_snapshot=flow,
    )
    dealer = to_dealer_positioning_snapshot(context)
    assert context.degraded is False
    assert context.gamma_flip_level is not None
    assert context.call_wall is not None
    assert context.put_wall is not None
    assert isinstance(context.meta.get("dealer_pressure_score"), float)
    assert dealer.confidence >= 0.5
    assert dealer.meta.get("acceleration_direction") in {"bullish", "bearish", "neutral"}


def test_gamma_context_proxy_path_marks_degradation() -> None:
    now = datetime.now(timezone.utc)
    flow = FlowPerceptionSnapshot(symbol="QQQ", timeframe="1m", as_of=now, net_bias=-0.3, net_gamma=None)
    context = build_gamma_context(
        symbol="QQQ",
        as_of=now,
        spot_price=420.0,
        options_chain={},
        flow_snapshot=flow,
    )
    dealer = to_dealer_positioning_snapshot(context)
    assert context.degraded is True
    assert "missing_full_options_chain" in context.degradation_reasons
    assert dealer.quality_flags.get("degraded") is True
    assert dealer.freshness_sec == 180
