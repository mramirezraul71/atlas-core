from __future__ import annotations

from typing import Any


def apply_grok_review_policy(opportunity: dict[str, Any], review: dict[str, Any]) -> dict[str, Any]:
    updated = dict(opportunity)
    verdict = str(review.get("verdict") or "neutral").strip().lower()
    score_adjustment = _safe_float(review.get("score_adjustment"))
    contracts_multiplier = _safe_float(review.get("contracts_multiplier"))
    prefer_strategy = review.get("prefer_strategy")

    if score_adjustment is not None:
        updated["score"] = round(float(updated.get("score") or 0.0) + score_adjustment, 6)
    if contracts_multiplier is not None:
        updated["grok_contracts_multiplier"] = max(0.0, min(1.0, contracts_multiplier))
    if prefer_strategy:
        updated["grok_prefer_strategy"] = str(prefer_strategy).strip().upper()
        updated["grok_prefer_strategy_matches"] = updated["grok_prefer_strategy"] == str(updated.get("strategy") or "").upper()
    updated["grok_review"] = dict(review)

    blocked = verdict == "reject"
    return {
        "blocked": blocked,
        "block_reason": "grok_reject" if blocked else None,
        "opportunity": updated,
    }


def _safe_float(value: Any) -> float | None:
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None
