from __future__ import annotations

from atlas_code_quant.vision.multi_timeframe_analyzer import analyze_multi_timeframe_view


def test_multi_timeframe_alignment_bullish() -> None:
    out = analyze_multi_timeframe_view(
        primary_timeframe="5m",
        primary_visual_evidence={"chart_bias": "bullish", "support_levels": [0.2, 0.3]},
        requested_timeframes=["5m", "15m", "1h"],
        timeframe_evidence={
            "15m": {"chart_bias": "bullish", "support_levels": [0.3]},
            "1h": {"chart_bias": "bullish", "support_levels": [0.5]},
        },
    )
    assert out["available"] is True
    assert out["alignment"] == "aligned_bullish"
    assert out["alignment_score"] is not None
    assert out["mtf_reason"] == "mtf_aligned_bullish"


def test_multi_timeframe_alignment_mixed() -> None:
    out = analyze_multi_timeframe_view(
        primary_timeframe="15m",
        primary_visual_evidence={"chart_bias": "bearish"},
        requested_timeframes=["15m", "1h"],
        timeframe_evidence={"1h": {"chart_bias": "bullish"}},
    )
    assert out["alignment"] == "mixed"
    assert out["available"] is True
