from __future__ import annotations

from atlas_code_quant.operations.live_transition_realism import estimate_live_transition_realism


def test_live_transition_realism_penalty_shape() -> None:
    out = estimate_live_transition_realism(
        baseline_score=88.0,
        paper_realism={
            "estimated_slippage_bps": 22.0,
            "partial_fill_risk": 0.35,
            "fill_quality_assumption": "degraded",
            "fee_model": "paper_options_flat",
        },
        operational_context={
            "navigation_ok": False,
            "capture_ok": True,
            "symbol_verified": True,
            "timeframe_verified": False,
        },
    )
    assert out["paper_live_drift_bucket"] in {"low", "medium", "high"}
    assert 0.0 <= out["paper_live_drift_score"] <= 1.0
    assert out["latency_penalty"] >= 0.0
    assert out["stale_chart_risk"] >= 0.0
