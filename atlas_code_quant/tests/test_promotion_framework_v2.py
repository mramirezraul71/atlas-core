from __future__ import annotations

from atlas_code_quant.operations.promotion_framework import evaluate_promotion_stage


def test_promotion_framework_v2_contract_and_recommendation() -> None:
    out = evaluate_promotion_stage(
        stage="paper_aggressive",
        metrics={
            "n_trades_evaluated": 210,
            "profit_factor": 1.52,
            "win_rate_pct": 56.0,
            "fallback_rate": 0.10,
            "stability_score": 76.0,
            "runtime_stability_score": 0.92,
            "visual_reliability_ratio": 0.81,
            "temporal_consistency_score": 0.83,
            "error_taxonomy_rate": 0.08,
            "realism_penalty_score": 0.18,
            "seasonal_hostile_execution_rate": 0.18,
            "mtf_mixed_execution_rate": 0.20,
            "low_confluence_execution_rate": 0.14,
            "cohort_metrics": [
                {"regime": "trend", "delta_vs_baseline": 0.06, "samples": 50, "policy_variant": "aggressive_v1"},
                {"regime": "range", "delta_vs_baseline": 0.01, "samples": 40, "policy_variant": "aggressive_v1"},
            ],
        },
    )
    assert out["current_stage"] == "paper_aggressive"
    assert out["target_stage"] == "supervised_live_candidate"
    assert out["recommendation"] in {"supervised_live_candidate", "guarded_live_candidate"}
    assert isinstance(out["passed_checks"], list)
    assert isinstance(out["failed_checks"], list)
    assert isinstance(out["warnings"], list)
    assert isinstance(out["scorecard"], dict)
