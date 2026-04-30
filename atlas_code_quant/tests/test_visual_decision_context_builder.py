from __future__ import annotations

from atlas_code_quant.operations.visual_decision_context_builder import (
    build_visual_decision_context,
)


def test_visual_decision_context_capture_ok_high_bucket() -> None:
    out = build_visual_decision_context(
        symbol="SPY",
        timeframe="5m",
        visual_evidence={
            "chart_bias": "bullish",
            "moving_average_state": "rising",
            "volume_confirmation": "strong",
            "pattern_signals": ["breakout"],
            "visual_confidence": 0.9,
        },
        operational_status={
            "navigation_ok": True,
            "capture_ok": True,
            "symbol_verified": True,
            "timeframe_verified": True,
        },
    )
    assert out["symbol"] == "SPY"
    assert out["timeframe"] == "5m"
    assert out["operational_context"]["capture_ok"] is True
    assert out["confluence_score"] is not None
    assert out["confluence_bucket"] in {"high", "medium"}
    assert out["recommended_action"] in {
        "execute",
        "reduce_aggressiveness",
        "postpone",
        "discard",
        "manual_review",
    }
    assert isinstance(out["pattern_signals"], list)
    assert out["pattern_signals"][0]["type"] == "breakout"
    assert isinstance(out["seasonality_context"], dict)
    assert isinstance(out["multi_timeframe_view"], dict)
    assert isinstance(out["fusion_components"], dict)


def test_visual_decision_context_capture_failed_unknown_bucket() -> None:
    out = build_visual_decision_context(
        symbol="QQQ",
        timeframe="1h",
        visual_evidence={"chart_bias": "neutral", "pattern_signals": ["no_clear_pattern"]},
        operational_status={
            "navigation_ok": False,
            "capture_ok": False,
            "symbol_verified": False,
            "timeframe_verified": False,
        },
    )
    assert out["operational_context"]["capture_ok"] is False
    assert out["confluence_score"] is None
    assert out["confluence_bucket"] == "unknown"
    assert out["recommended_action"] == "manual_review"
    assert "capture" in out["fusion_decision_reason"].lower()


def test_visual_decision_context_symbol_not_verified_degrades_reason() -> None:
    out = build_visual_decision_context(
        symbol="IWM",
        timeframe="15m",
        visual_evidence={
            "chart_bias": "bearish",
            "moving_average_state": "falling",
            "volume_confirmation": "moderate",
            "pattern_signals": [{"type": "reversal", "strength": "high"}],
        },
        operational_status={
            "navigation_ok": False,
            "capture_ok": True,
            "symbol_verified": False,
            "timeframe_verified": True,
        },
    )
    assert out["operational_context"]["symbol_verified"] is False
    assert out["confluence_bucket"] in {"low", "medium"}
    assert out["recommended_action"] in {"reduce_aggressiveness", "postpone", "manual_review", "discard"}
    assert "verification" in out["fusion_decision_reason"].lower()
