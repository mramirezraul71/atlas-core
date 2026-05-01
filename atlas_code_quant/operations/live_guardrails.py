"""Guardrails for future live transition (simulation-only)."""
from __future__ import annotations

from typing import Any


DEFAULT_LIVE_GUARDRAILS: dict[str, Any] = {
    "emergency_stop_ready": True,
    "rollback_ready": True,
    "max_fallback_rate": 0.30,
    "max_visual_reliability_drop": 0.35,
    "max_seasonality_conflict_rate": 0.45,
    "max_mtf_conflict_rate": 0.50,
    "max_realism_penalty": 0.45,
    "max_error_taxonomy_rate": 0.22,
    "pause_conditions": [
        "fallback_rate_spike",
        "visual_reliability_degraded",
        "seasonality_conflict_spike",
        "mtf_conflict_spike",
        "realism_penalty_exceeded",
        "error_taxonomy_rate_exceeded",
    ],
}


def evaluate_live_guardrails(metrics: dict[str, Any] | None) -> dict[str, Any]:
    payload = dict(metrics or {})
    fallback_rate = float(payload.get("fallback_rate") or 0.0)
    visual_reliability = float(payload.get("visual_reliability_ratio") or 0.0)
    seasonal_conflict_rate = float(payload.get("seasonality_conflict_rate") or 0.0)
    mtf_conflict_rate = float(payload.get("mtf_conflict_rate") or 0.0)
    realism_penalty = float(payload.get("realism_penalty_score") or 0.0)
    error_rate = float(payload.get("error_taxonomy_rate") or 0.0)

    exceeded: list[str] = []
    if fallback_rate > float(DEFAULT_LIVE_GUARDRAILS["max_fallback_rate"]):
        exceeded.append("fallback_rate_spike")
    if visual_reliability < (1.0 - float(DEFAULT_LIVE_GUARDRAILS["max_visual_reliability_drop"])):
        exceeded.append("visual_reliability_degraded")
    if seasonal_conflict_rate > float(DEFAULT_LIVE_GUARDRAILS["max_seasonality_conflict_rate"]):
        exceeded.append("seasonality_conflict_spike")
    if mtf_conflict_rate > float(DEFAULT_LIVE_GUARDRAILS["max_mtf_conflict_rate"]):
        exceeded.append("mtf_conflict_spike")
    if realism_penalty > float(DEFAULT_LIVE_GUARDRAILS["max_realism_penalty"]):
        exceeded.append("realism_penalty_exceeded")
    if error_rate > float(DEFAULT_LIVE_GUARDRAILS["max_error_taxonomy_rate"]):
        exceeded.append("error_taxonomy_rate_exceeded")

    return {
        "emergency_stop_ready": bool(DEFAULT_LIVE_GUARDRAILS["emergency_stop_ready"]),
        "rollback_ready": bool(DEFAULT_LIVE_GUARDRAILS["rollback_ready"]),
        "pause_conditions_triggered": exceeded,
        "pause_recommended": bool(exceeded),
        "guardrails": dict(DEFAULT_LIVE_GUARDRAILS),
    }
