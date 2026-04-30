"""Multi-timeframe synthesis for paper_aggressive visual fusion."""
from __future__ import annotations

from typing import Any


_DEFAULT_TIMEFRAMES = ["5m", "15m", "1h", "1d"]


def _bias_to_direction(chart_bias: str) -> int:
    token = str(chart_bias or "").strip().lower()
    if token == "bullish":
        return 1
    if token == "bearish":
        return -1
    return 0


def _normalize_levels(value: Any) -> list[float]:
    out: list[float] = []
    if not isinstance(value, list):
        return out
    for item in value:
        try:
            out.append(round(float(item), 4))
        except (TypeError, ValueError):
            continue
    return out


def _default_payload() -> dict[str, Any]:
    return {
        "chart_bias": "unknown",
        "support_levels": [],
        "resistance_levels": [],
        "moving_average_state": "unknown",
        "volume_confirmation": "unknown",
        "confluence_score": None,
    }


def analyze_multi_timeframe_view(
    *,
    primary_timeframe: str,
    primary_visual_evidence: dict[str, Any] | None,
    requested_timeframes: list[str] | None = None,
    timeframe_evidence: dict[str, dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Create a first-pass MTF view from existing visual payloads."""
    req = [str(tf).lower() for tf in (requested_timeframes or _DEFAULT_TIMEFRAMES)]
    if str(primary_timeframe or "").lower() not in req:
        req.insert(0, str(primary_timeframe or "5m").lower())

    per_timeframe: dict[str, dict[str, Any]] = {}
    source = timeframe_evidence if isinstance(timeframe_evidence, dict) else {}

    for tf in req:
        raw = source.get(tf)
        if not isinstance(raw, dict) and tf == str(primary_timeframe or "").lower():
            raw = dict(primary_visual_evidence or {})
        if not isinstance(raw, dict):
            raw = {}
        payload = {
            "chart_bias": str(raw.get("chart_bias") or "unknown"),
            "support_levels": _normalize_levels(raw.get("support_levels")),
            "resistance_levels": _normalize_levels(raw.get("resistance_levels")),
            "moving_average_state": str(raw.get("moving_average_state") or "unknown"),
            "volume_confirmation": str(raw.get("volume_confirmation") or "unknown"),
            "confluence_score": raw.get("confluence_score"),
        }
        if not payload["support_levels"] and not payload["resistance_levels"] and not raw:
            payload = _default_payload()
        per_timeframe[tf] = payload

    directions = [_bias_to_direction(v.get("chart_bias", "unknown")) for v in per_timeframe.values()]
    non_zero = [d for d in directions if d != 0]
    available = any(v.get("chart_bias") not in {"unknown", "neutral", ""} for v in per_timeframe.values())
    if not non_zero:
        alignment = "unknown"
        alignment_score = None
    elif all(d > 0 for d in non_zero):
        alignment = "aligned_bullish"
        alignment_score = round(len(non_zero) / max(1, len(req)), 4)
    elif all(d < 0 for d in non_zero):
        alignment = "aligned_bearish"
        alignment_score = round(len(non_zero) / max(1, len(req)), 4)
    else:
        alignment = "mixed"
        aligned = max(sum(1 for d in non_zero if d > 0), sum(1 for d in non_zero if d < 0))
        alignment_score = round(aligned / max(1, len(req)), 4)

    primary_levels = set(_normalize_levels((primary_visual_evidence or {}).get("support_levels")))
    overlap = None
    if primary_levels:
        overlap = False
        for tf, info in per_timeframe.items():
            if tf == str(primary_timeframe or "").lower():
                continue
            if primary_levels.intersection(set(info.get("support_levels") or [])):
                overlap = True
                break

    if alignment in {"aligned_bullish", "aligned_bearish"}:
        reason = f"mtf_{alignment}"
    elif alignment == "mixed":
        reason = "mtf_conflict_across_timeframes"
    else:
        reason = "mtf_insufficient_signal"

    return {
        "available": bool(available),
        "requested_timeframes": req,
        "per_timeframe": per_timeframe,
        "alignment": alignment,
        "alignment_score": alignment_score,
        "key_level_overlap": overlap,
        "mtf_reason": reason,
    }
