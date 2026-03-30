from __future__ import annotations

from atlas_code_quant.context.market_context_engine import MarketContextEngine


def test_market_context_engine_blocks_ambiguous_event_driven_setup() -> None:
    engine = MarketContextEngine()

    context = engine.build(
        candidate={
            "symbol": "NVDA",
            "timeframe": "15m",
            "direction": "alcista",
            "strategy_key": "breakout_event",
            "selection_score": 59.0,
            "local_win_rate_pct": 51.0,
            "predicted_move_pct": 3.4,
            "relative_strength_pct": -1.1,
            "iv_rank": 84.0,
            "iv_hv_ratio": 1.45,
            "liquidity_score": 0.42,
            "event_near": True,
            "confirmation": {"higher_timeframe": "1h"},
        },
        tracker_summary={"totals": {"positions": 24}},
    )

    assert context["decision_gate"]["blocked"] is True
    assert context["context_report"]["permission"] == "block"
    assert context["source_trace"]["taxonomy"] == "macro->global->asset->session->history->gate"


def test_market_context_engine_allows_clean_trend_setup() -> None:
    engine = MarketContextEngine()

    context = engine.build(
        candidate={
            "symbol": "AAPL",
            "timeframe": "4h",
            "direction": "alcista",
            "strategy_key": "trend_ema_stack",
            "selection_score": 84.0,
            "local_win_rate_pct": 63.0,
            "predicted_move_pct": 1.7,
            "relative_strength_pct": 3.8,
            "iv_rank": 34.0,
            "iv_hv_ratio": 0.98,
            "liquidity_score": 0.93,
            "confirmation": {"higher_timeframe": "1d"},
            "regime": "BULL",
        },
        tracker_summary={"totals": {"positions": 6}},
    )

    assert context["decision_gate"]["blocked"] is False
    assert context["confidence_pct"] >= 60.0
    assert context["asset_state"]["profile"]["profile_key"] == "trend_follow_profile"
