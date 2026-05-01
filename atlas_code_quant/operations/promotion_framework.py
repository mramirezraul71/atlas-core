"""Promotion framework for staged paper->live maturity recommendations.

Phase C: recommendation-only framework. No automatic live execution changes.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class PromotionStageThresholds:
    min_trades: int
    min_profit_factor: float
    min_win_rate_pct: float
    max_fallback_rate: float
    min_stability_score: float
    max_error_rate: float
    max_realism_penalty: float
    min_visual_reliability: float
    min_temporal_consistency: float


_THRESHOLDS: dict[str, PromotionStageThresholds] = {
    "paper_baseline": PromotionStageThresholds(
        min_trades=30,
        min_profit_factor=1.1,
        min_win_rate_pct=45.0,
        max_fallback_rate=0.60,
        min_stability_score=40.0,
        max_error_rate=0.40,
        max_realism_penalty=0.55,
        min_visual_reliability=0.40,
        min_temporal_consistency=0.45,
    ),
    "paper_aggressive": PromotionStageThresholds(
        min_trades=80,
        min_profit_factor=1.25,
        min_win_rate_pct=50.0,
        max_fallback_rate=0.40,
        min_stability_score=55.0,
        max_error_rate=0.28,
        max_realism_penalty=0.45,
        min_visual_reliability=0.55,
        min_temporal_consistency=0.60,
    ),
    "supervised_live_candidate": PromotionStageThresholds(
        min_trades=150,
        min_profit_factor=1.40,
        min_win_rate_pct=52.0,
        max_fallback_rate=0.25,
        min_stability_score=65.0,
        max_error_rate=0.20,
        max_realism_penalty=0.35,
        min_visual_reliability=0.65,
        min_temporal_consistency=0.70,
    ),
    "guarded_live_candidate": PromotionStageThresholds(
        min_trades=250,
        min_profit_factor=1.55,
        min_win_rate_pct=54.0,
        max_fallback_rate=0.15,
        min_stability_score=72.0,
        max_error_rate=0.14,
        max_realism_penalty=0.25,
        min_visual_reliability=0.72,
        min_temporal_consistency=0.78,
    ),
}


_STAGE_ORDER = [
    "paper_baseline",
    "paper_aggressive",
    "supervised_live_candidate",
    "guarded_live_candidate",
]


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _target_stage(current_stage: str) -> str:
    if current_stage not in _STAGE_ORDER:
        return "paper_aggressive"
    idx = _STAGE_ORDER.index(current_stage)
    if idx >= len(_STAGE_ORDER) - 1:
        return current_stage
    return _STAGE_ORDER[idx + 1]


def evaluate_promotion_stage(*, metrics: dict[str, Any], stage: str = "paper_aggressive") -> dict[str, Any]:
    current_stage = str(stage or "paper_aggressive").strip().lower()
    if current_stage == "supervised_live":
        current_stage = "supervised_live_candidate"
    if current_stage == "guarded_live":
        current_stage = "guarded_live_candidate"
    t = _THRESHOLDS.get(current_stage, _THRESHOLDS["paper_aggressive"])
    target_stage = _target_stage(current_stage)

    n_trades = int(metrics.get("n_trades_evaluated") or metrics.get("n_trades_analyzed") or 0)
    profit_factor = _safe_float(metrics.get("profit_factor"), 0.0)
    win_rate_pct = _safe_float(metrics.get("win_rate_pct"), 0.0)
    fallback_rate = _safe_float(metrics.get("fallback_rate"), 0.0)
    stability_score = _safe_float(metrics.get("stability_score"), 0.0)
    runtime_stability = _safe_float(metrics.get("runtime_stability_score"), stability_score / 100.0)
    visual_reliability = _safe_float(metrics.get("visual_reliability_ratio"), 0.0)
    temporal_consistency = _safe_float(metrics.get("temporal_consistency_score"), 0.0)
    realism_penalty = _safe_float(metrics.get("realism_penalty_score"), 0.0)
    error_rate = _safe_float(metrics.get("error_taxonomy_rate"), 0.0)

    seasonal_outcome_rate = _safe_float(metrics.get("seasonal_hostile_execution_rate"), 0.0)
    mtf_mixed_rate = _safe_float(metrics.get("mtf_mixed_execution_rate"), 0.0)
    confluence_low_rate = _safe_float(metrics.get("low_confluence_execution_rate"), 0.0)

    checks = {
        "min_trades": n_trades >= t.min_trades,
        "min_profit_factor": profit_factor >= t.min_profit_factor,
        "min_win_rate_pct": win_rate_pct >= t.min_win_rate_pct,
        "max_fallback_rate": fallback_rate <= t.max_fallback_rate,
        "min_stability_score": stability_score >= t.min_stability_score,
        "runtime_stability_ok": runtime_stability >= 0.70,
        "visual_reliability_ok": visual_reliability >= t.min_visual_reliability,
        "temporal_consistency_ok": temporal_consistency >= t.min_temporal_consistency,
        "max_error_taxonomy_rate": error_rate <= t.max_error_rate,
        "max_realism_penalty": realism_penalty <= t.max_realism_penalty,
        "seasonality_hostile_controlled": seasonal_outcome_rate <= 0.45,
        "mtf_conflict_controlled": mtf_mixed_rate <= 0.50,
        "low_confluence_controlled": confluence_low_rate <= 0.40,
    }
    passed = [k for k, ok in checks.items() if ok]
    failed = [k for k, ok in checks.items() if not ok]
    cohort_metrics = metrics.get("cohort_metrics")
    cohort_rows = cohort_metrics if isinstance(cohort_metrics, list) else []
    by_regime: dict[str, list[float]] = {}
    for row in cohort_rows:
        if not isinstance(row, dict):
            continue
        regime = str(row.get("regime") or "unknown")
        delta = _safe_float(row.get("delta_vs_baseline"), 0.0)
        by_regime.setdefault(regime, []).append(delta)
    regime_effect = {
        regime: (sum(deltas) / len(deltas) if deltas else 0.0)
        for regime, deltas in by_regime.items()
    }
    improves_regimes = [k for k, v in regime_effect.items() if v > 0.02]
    degrades_regimes = [k for k, v in regime_effect.items() if v < -0.02]
    has_negative_core = bool({"min_profit_factor", "min_win_rate_pct"} & set(failed))
    has_data = n_trades >= max(20, int(t.min_trades * 0.5))
    warnings: list[str] = []
    if n_trades < t.min_trades:
        warnings.append("insufficient_sample_size")
    if fallback_rate > t.max_fallback_rate:
        warnings.append("fallback_rate_above_threshold")
    if degrades_regimes:
        warnings.append("regime_specific_degradation_detected")
    if realism_penalty > t.max_realism_penalty:
        warnings.append("paper_live_realism_drift_high")

    if has_negative_core:
        recommendation = "reject_variant"
    elif failed and has_data:
        recommendation = "stay_paper"
    elif failed:
        recommendation = "expand_paper"
    elif current_stage == "paper_aggressive" and improves_regimes and not degrades_regimes and has_data:
        recommendation = "supervised_live_candidate"
    elif current_stage == "supervised_live_candidate" and not degrades_regimes and has_data:
        recommendation = "guarded_live_candidate"
    elif improves_regimes and degrades_regimes:
        recommendation = "expand_paper"
    else:
        recommendation = "stay_paper"

    readiness_score = 0.0
    if checks:
        readiness_score = round(sum(1.0 for ok in checks.values() if ok) / float(len(checks)), 4)

    return {
        "stage": current_stage,  # backward compatibility
        "current_stage": current_stage,
        "target_stage": target_stage,
        "recommendation": recommendation,
        "passed_checks": passed,
        "failed_checks": failed,
        "warnings": warnings,
        "regime_uplift": regime_effect,
        "regimes_improving": improves_regimes,
        "regimes_degrading": degrades_regimes,
        "thresholds": {
            "min_trades": t.min_trades,
            "min_profit_factor": t.min_profit_factor,
            "min_win_rate_pct": t.min_win_rate_pct,
            "max_fallback_rate": t.max_fallback_rate,
            "min_stability_score": t.min_stability_score,
            "max_error_rate": t.max_error_rate,
            "max_realism_penalty": t.max_realism_penalty,
            "min_visual_reliability": t.min_visual_reliability,
            "min_temporal_consistency": t.min_temporal_consistency,
        },
        "metrics": {
            "n_trades": n_trades,
            "profit_factor": profit_factor,
            "win_rate_pct": win_rate_pct,
            "fallback_rate": fallback_rate,
            "stability_score": stability_score,
            "runtime_stability_score": runtime_stability,
            "visual_reliability_ratio": visual_reliability,
            "temporal_consistency_score": temporal_consistency,
            "error_taxonomy_rate": error_rate,
            "realism_penalty_score": realism_penalty,
            "seasonal_hostile_execution_rate": seasonal_outcome_rate,
            "mtf_mixed_execution_rate": mtf_mixed_rate,
            "low_confluence_execution_rate": confluence_low_rate,
        },
        "ready_for_supervised_live": (
            readiness_score >= 0.82
            and current_stage in {"paper_aggressive", "supervised_live_candidate", "guarded_live_candidate"}
            and "min_profit_factor" in passed
            and "min_win_rate_pct" in passed
            and "max_realism_penalty" in passed
        ),
        "ready_for_guarded_live": (
            readiness_score >= 0.90
            and current_stage in {"supervised_live_candidate", "guarded_live_candidate"}
            and "runtime_stability_ok" in passed
            and "visual_reliability_ok" in passed
            and "temporal_consistency_ok" in passed
        ),
        "ready_for_full_live": False,
        "blocking_reasons": [
            "full_live_globally_locked_until_human_unlock_and_formal_readiness"
        ],
        "scorecard": {
            "readiness_score": readiness_score,
            "checks_total": len(checks),
            "checks_passed": len(passed),
            "checks_failed": len(failed),
            "critical_failures": [
                check
                for check in failed
                if check in {"min_profit_factor", "min_win_rate_pct", "max_realism_penalty"}
            ],
        },
    }
