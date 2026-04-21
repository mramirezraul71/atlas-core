"""Paper-vs-live realism estimators for transition readiness.

No live execution is performed here. This module only computes penalties/risk.
"""
from __future__ import annotations

from typing import Any


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _bounded(value: float, *, min_v: float = 0.0, max_v: float = 1.0) -> float:
    return max(min_v, min(max_v, value))


def estimate_live_transition_realism(
    *,
    baseline_score: float,
    paper_realism: dict[str, Any] | None,
    operational_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    realism = dict(paper_realism or {})
    operational = dict(operational_context or {})

    slippage_bps = _safe_float(realism.get("estimated_slippage_bps"), 10.0)
    partial_fill_risk = _bounded(_safe_float(realism.get("partial_fill_risk"), 0.2))
    fill_quality = str(realism.get("fill_quality_assumption") or "normal").lower()
    fee_model = str(realism.get("fee_model") or "paper_flat")

    latency_penalty = _bounded(slippage_bps / 120.0)
    rejected_order_risk = _bounded(0.15 + (0.20 if fill_quality == "degraded" else 0.05))
    duplicate_signal_risk = _bounded(0.08 + (0.10 if baseline_score > 85 else 0.03))
    stale_chart_risk = _bounded(
        0.35
        if not bool(operational.get("timeframe_verified", False))
        else 0.20
        if not bool(operational.get("navigation_ok", False))
        else 0.08
    )
    stale_decision_risk = _bounded(
        0.30
        if not bool(operational.get("capture_ok", False))
        else 0.14
    )

    paper_live_drift_score = _bounded(
        (latency_penalty * 0.24)
        + (partial_fill_risk * 0.24)
        + (rejected_order_risk * 0.18)
        + (duplicate_signal_risk * 0.14)
        + (stale_chart_risk * 0.12)
        + (stale_decision_risk * 0.08)
    )

    return {
        "slippage_assumption_bps": slippage_bps,
        "fee_assumption": fee_model,
        "fill_quality_assumption": fill_quality,
        "partial_fill_risk": partial_fill_risk,
        "latency_penalty": latency_penalty,
        "rejected_order_risk": rejected_order_risk,
        "duplicate_signal_risk": duplicate_signal_risk,
        "stale_chart_risk": stale_chart_risk,
        "stale_decision_risk": stale_decision_risk,
        "paper_live_drift_score": paper_live_drift_score,
        "paper_live_drift_bucket": (
            "low"
            if paper_live_drift_score < 0.25
            else "medium"
            if paper_live_drift_score < 0.5
            else "high"
        ),
    }
