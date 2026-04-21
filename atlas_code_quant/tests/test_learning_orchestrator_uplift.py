from __future__ import annotations

from atlas_code_quant.learning import learning_orchestrator as lo


def test_compute_advisory_uplift_from_events(monkeypatch) -> None:
    class _FakeStore:
        def query(self, **kwargs):
            return [
                {"data": {"baseline_decision": "accept", "advisory_decision": "accept", "executed_in_paper": True, "requested_model": "llama3.1:8b", "strategy_hint": "iron_condor", "policy_variant": "aggressive_v1", "cohort_id": "c1", "regime": "trend", "opportunity_id": "opp_a", "trace_id": "tr_a", "decision_id": "d_a", "reasons": [], "confluence_bucket": "high", "recommended_action": "execute", "seasonality_context": {"seasonal_bias": "favorable"}, "multi_timeframe_view": {"alignment": "aligned_bullish"}}},
                {"data": {"baseline_decision": "accept", "advisory_decision": "discard", "executed_in_paper": False, "requested_model": "llama3.1:8b", "strategy_hint": "iron_condor", "policy_variant": "aggressive_local_models_v1", "cohort_id": "c2", "regime": "range", "opportunity_id": "opp_b", "trace_id": "tr_b", "decision_id": "d_b", "error_taxonomy_v2": {"timing_error": 1}, "recommended_action": "postpone", "visual_context": {"confluence_bucket": "low", "seasonality_context": {"seasonal_bias": "hostile"}, "multi_timeframe_view": {"alignment": "mixed"}}}},
                {"data": {"baseline_decision": "discard", "advisory_decision": "accept", "executed_in_paper": True, "requested_model": "qwen", "strategy_hint": "debit_spread", "policy_variant": "high_conviction_v1", "cohort_id": "c3", "regime": "trend", "opportunity_id": "opp_c", "trace_id": "tr_c", "decision_id": "d_c", "reasons": [], "recommended_action": "reduce_aggressiveness", "visual_evidence": {"confluence_bucket": "medium"}, "seasonality_context": {"seasonal_bias": "neutral"}, "multi_timeframe_view": {"alignment": "aligned_bearish"}}},
            ]

    monkeypatch.setattr("atlas_code_quant.learning.event_store.get_event_store", lambda: _FakeStore())
    out = lo.compute_advisory_uplift(window_days=5)
    assert out["total_decisions"] == 3
    assert "advisory_uplift_ratio" in out
    assert "strong_setups" in out
    assert "cohort_metrics" in out
    assert out["traceability_coverage_ratio"] == 1.0
    assert "timing_error" in out["error_buckets"]
    assert out["confluence_bucket_counts"]["high"] == 1
    assert out["confluence_bucket_counts"]["medium"] == 1
    assert out["confluence_bucket_counts"]["low"] == 1
    assert out["seasonal_bias_counts"]["favorable"] == 1
    assert out["seasonal_bias_counts"]["hostile"] == 1
    assert out["mtf_alignment_counts"]["mixed"] == 1
    assert out["recommended_action_counts"]["postpone"] == 1
    readiness = lo.build_transition_readiness_insights(out)
    assert "runtime_stability_score" in readiness
    assert "realism_penalty_score" in readiness
    assert "transition_ready_variants" in readiness
