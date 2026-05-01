from __future__ import annotations

import pytest

from atlas_code_quant.options import options_engine_metrics as oem


def test_record_paper_aggressive_decision_v2_dimensions() -> None:
    pytest.importorskip("prometheus_client")
    oem.record_paper_aggressive_decision(
        decision_path="final",
        mode="paper_aggressive",
        pre_trade_score=82.0,
        executed=True,
        session="intraday",
        regime="trend",
        strategy="iron_condor",
        model_path="llama3.1:8b",
        policy_variant="high_conviction_v1",
        cohort_id="cohort_test",
        error_taxonomy_v2={"timing_error": 1, "validation_error": 2},
        realism={
            "estimated_slippage_bps": 12.5,
            "fee_model": "paper_options_flat",
            "fill_quality_assumption": "good",
            "partial_fill_risk": 0.22,
        },
        visual_evidence={
            "provider_used": "desktop_capture",
            "chart_bias": "bullish",
            "visual_confidence": 0.77,
            "confluence_score": 0.73,
            "confluence_bucket": "high",
            "visual_decision_context": {
                "confluence_score": 0.73,
                "confluence_bucket": "high",
                "recommended_action": "execute",
                "seasonality_context": {"seasonal_bias": "favorable"},
                "multi_timeframe_view": {"alignment": "aligned_bullish"},
                "fusion_components": {"operational_reliability_bucket": "high"},
            },
        },
    )
    snap = oem.get_ui_snapshot()
    assert snap.get("ok") is True
