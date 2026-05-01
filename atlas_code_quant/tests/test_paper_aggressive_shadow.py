from __future__ import annotations

from atlas_code_quant.learning.event_store import EventStore
from atlas_code_quant.operations.decision_shadow import (
    build_advisory_shadow,
    build_baseline_decision,
    record_shadow_triplet,
    resolve_final_paper_decision,
)
from atlas_code_quant.operations.promotion_framework import evaluate_promotion_stage


def test_shadow_triplet_records_events(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr(
        "atlas_code_quant.operations.decision_shadow.classify_journal_event",
        lambda payload: {
            "label": "trading_event",
            "confidence": 0.8,
            "requested_model": "llama3.1:8b",
            "provider_role": "classifier",
            "used_fallback": False,
            "fallback_reason": "",
            "summary": "ok",
            "tags": ["shadow"],
        },
    )
    baseline = build_baseline_decision(
        candidate={"symbol": "SPY", "selection_score": 78.0, "strategy_type": "iron_condor"},
        order_seed={"symbol": "SPY", "strategy_type": "iron_condor"},
        action="submit",
        auton_mode="paper_aggressive",
    )
    advisory = build_advisory_shadow(baseline=baseline, candidate={"symbol": "SPY", "selection_score": 78.0})
    final = resolve_final_paper_decision(
        baseline=baseline,
        advisory=advisory,
        evaluate_result={"blocked": False, "reasons": []},
    )
    store = EventStore(path=tmp_path / "events.jsonl")
    record_shadow_triplet(event_store=store, baseline=baseline, advisory=advisory, final=final)
    topics = [e["topic"] for e in store.query(limit=10)]
    assert "decision.baseline" in topics
    assert "decision.advisory.local_models" in topics
    assert "decision.final" in topics
    assert baseline.get("opportunity_id")
    assert baseline.get("trace_id")
    assert baseline.get("cohort_id")
    assert final.get("paper_realism", {}).get("estimated_slippage_bps") is not None
    assert "error_taxonomy_v2" in final
    assert baseline.get("visual_context") is None
    assert final.get("fusion_score") is None
    assert final.get("confluence_bucket") == "unknown"
    assert final.get("recommended_action") == "manual_review"
    assert isinstance(final.get("seasonality_context"), dict)
    assert isinstance(final.get("multi_timeframe_view"), dict)
    assert isinstance(final.get("operational_context"), dict)
    assert isinstance(final.get("decision_without_visual"), dict)
    assert isinstance(final.get("decision_with_visual"), dict)
    assert isinstance(final.get("live_transition_realism"), dict)
    assert isinstance(final.get("live_guardrails"), dict)
    assert isinstance(final.get("authority_transition_assessment"), dict)
    assert isinstance(final.get("live_readiness_candidate"), bool)


def test_promotion_framework_stage_recommendation() -> None:
    payload = evaluate_promotion_stage(
        metrics={
            "n_trades_evaluated": 120,
            "profit_factor": 1.35,
            "win_rate_pct": 53.0,
            "fallback_rate": 0.2,
            "stability_score": 61.0,
            "cohort_metrics": [
                {"policy_variant": "aggressive_v1", "cohort_id": "c1", "regime": "trend", "samples": 30, "delta_vs_baseline": 0.04},
                {"policy_variant": "aggressive_v1", "cohort_id": "c2", "regime": "chop", "samples": 20, "delta_vs_baseline": -0.01},
            ],
        },
        stage="paper_aggressive",
    )
    assert payload["stage"] == "paper_aggressive"
    assert payload["recommendation"] in {
        "stay_paper",
        "expand_paper",
        "supervised_live_candidate",
        "guarded_live_candidate",
        "reject_variant",
    }
    assert "regime_uplift" in payload
    assert "failed_checks" in payload
    assert payload["current_stage"] == "paper_aggressive"
    assert payload["target_stage"] in {"supervised_live_candidate", "guarded_live_candidate"}
    assert isinstance(payload.get("scorecard"), dict)


def test_shadow_decision_includes_fusion_fields_when_visual_context_present(monkeypatch) -> None:
    monkeypatch.setattr(
        "atlas_code_quant.operations.decision_shadow.classify_journal_event",
        lambda payload: {
            "label": "trading_event",
            "confidence": 0.9,
            "requested_model": "llama3.1:8b",
            "provider_role": "classifier",
            "used_fallback": False,
            "fallback_reason": "",
            "summary": "ok",
            "tags": ["shadow"],
        },
    )
    visual = {
        "visual_decision_context": {
            "confluence_score": 0.81,
            "confluence_bucket": "high",
            "fusion_decision_reason": "High confluence.",
            "recommended_action": "execute",
            "seasonality_context": {"seasonal_bias": "favorable"},
            "multi_timeframe_view": {"alignment": "aligned_bullish"},
            "fusion_components": {"operational_reliability_bucket": "high"},
            "operational_context": {
                "navigation_ok": True,
                "capture_ok": True,
                "symbol_verified": True,
                "timeframe_verified": True,
            },
        },
        "confluence_score": 0.81,
        "confluence_bucket": "high",
        "fusion_decision_reason": "High confluence.",
    }
    baseline = build_baseline_decision(
        candidate={"symbol": "QQQ", "selection_score": 88.0, "strategy_type": "debit_spread"},
        order_seed={"symbol": "QQQ", "strategy_type": "debit_spread"},
        action="submit",
        auton_mode="paper_aggressive",
        visual_context=visual,
    )
    advisory = build_advisory_shadow(baseline=baseline, candidate={"symbol": "QQQ", "selection_score": 88.0})
    final = resolve_final_paper_decision(
        baseline=baseline,
        advisory=advisory,
        evaluate_result={"blocked": False, "reasons": []},
    )
    assert baseline.get("fusion_score") == 0.81
    assert baseline.get("confluence_bucket") == "high"
    assert isinstance(final.get("visual_context"), dict)
    assert final.get("fusion_score") == 0.81
    assert final.get("confluence_bucket") == "high"
    assert final.get("recommended_action") == "execute"
    assert final.get("seasonality_context", {}).get("seasonal_bias") == "favorable"
    assert final.get("multi_timeframe_view", {}).get("alignment") == "aligned_bullish"
    assert final.get("decision_with_visual", {}).get("confluence_bucket") == "high"
    assert "live_readiness_candidate" in final
    assert isinstance(final.get("live_risk_flags"), list)
