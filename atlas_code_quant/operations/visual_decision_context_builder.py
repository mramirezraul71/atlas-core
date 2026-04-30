"""Build a stable visual decision context for paper_aggressive."""
from __future__ import annotations

from typing import Any

from atlas_code_quant.learning.seasonality_context import build_seasonality_context
from atlas_code_quant.vision.multi_timeframe_analyzer import analyze_multi_timeframe_view

_ALLOWED_BUCKETS = {"high", "medium", "low", "unknown"}
_ALLOWED_RECOMMENDED_ACTIONS = {
    "execute",
    "reduce_aggressiveness",
    "postpone",
    "discard",
    "manual_review",
}


def _pattern_type(label: str) -> str:
    token = str(label or "").strip().lower().replace(" ", "_")
    if not token:
        return "unknown"
    return token


def _pattern_strength(pattern_type: str) -> str:
    if pattern_type in {"breakout", "reversal", "expansion"}:
        return "high"
    if pattern_type in {"compression"}:
        return "medium"
    if pattern_type in {"no_clear_pattern", "unknown"}:
        return "low"
    return "medium"


def _normalize_pattern_signals(value: Any) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    if not isinstance(value, list):
        return out
    for item in value:
        if isinstance(item, dict):
            p_type = _pattern_type(item.get("type") or item.get("pattern") or item.get("label"))
            p_strength = str(item.get("strength") or _pattern_strength(p_type)).lower()
        else:
            p_type = _pattern_type(item)
            p_strength = _pattern_strength(p_type)
        out.append({"type": p_type, "strength": p_strength})
    return out


def _bucket_from_score(score: float | None, *, capture_ok: bool) -> str:
    if not capture_ok or score is None:
        return "unknown"
    if score >= 0.75:
        return "high"
    if score >= 0.48:
        return "medium"
    if score > 0.0:
        return "low"
    return "unknown"


def _build_reason(
    *,
    bucket: str,
    capture_ok: bool,
    symbol_verified: bool,
    timeframe_verified: bool,
    chart_bias: str,
    volume_confirmation: str,
) -> str:
    if not capture_ok:
        return "Visual capture failed or empty frame."
    if not symbol_verified or not timeframe_verified:
        return "Visual capture is present but symbol/timeframe verification is incomplete."
    if bucket == "high":
        return f"High confluence: {chart_bias} bias with {volume_confirmation} volume confirmation and verified capture context."
    if bucket == "medium":
        return "Medium confluence: visual signals are partially aligned with verified context."
    if bucket == "low":
        return "Low confluence: visual context is verified but lacks strong pattern/volume alignment."
    return "Confluence unknown due to insufficient visual evidence."


def _safe_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _seasonality_weight(seasonality_context: dict[str, Any]) -> tuple[float, str]:
    bias = str(seasonality_context.get("seasonal_bias") or "unknown").lower()
    if bias == "favorable":
        return (0.1, "seasonality_favorable")
    if bias == "neutral":
        return (0.0, "seasonality_neutral")
    if bias == "hostile":
        return (-0.12, "seasonality_hostile")
    return (-0.04, "seasonality_unknown")


def _operational_reliability(op: dict[str, bool]) -> tuple[float, str]:
    if op.get("capture_ok") and op.get("symbol_verified") and op.get("timeframe_verified"):
        return (1.0, "high")
    if op.get("capture_ok") and (op.get("symbol_verified") or op.get("timeframe_verified")):
        return (0.65, "medium")
    if op.get("capture_ok"):
        return (0.45, "low")
    return (0.25, "critical")


def _recommended_action(
    *,
    score: float | None,
    bucket: str,
    operational_reliability: float,
    mtf_alignment: str,
    seasonal_bias: str,
) -> str:
    if operational_reliability < 0.35:
        return "manual_review"
    if score is None or bucket == "unknown":
        return "manual_review"
    if mtf_alignment == "mixed" and seasonal_bias == "hostile":
        return "postpone"
    if bucket == "high" and mtf_alignment in {"aligned_bullish", "aligned_bearish"}:
        return "execute"
    if bucket in {"medium", "high"} and seasonal_bias in {"neutral", "favorable"}:
        return "reduce_aggressiveness"
    if bucket == "low":
        return "postpone"
    return "discard"


def build_visual_decision_context(
    *,
    symbol: str,
    timeframe: str,
    visual_evidence: dict[str, Any] | None,
    seasonality_snapshot: dict[str, Any] | None = None,
    operational_status: dict[str, Any] | None = None,
    chart_plan: dict[str, Any] | None = None,
    camera_plan: dict[str, Any] | None = None,
    quant_score: float | None = None,
) -> dict[str, Any]:
    raw = dict(visual_evidence or {})
    op = dict(operational_status or {})

    capture_ok = bool(op.get("capture_ok", bool(raw.get("capture_ok"))))
    symbol_verified = bool(op.get("symbol_verified", bool(raw.get("symbol_verified"))))
    timeframe_verified = bool(op.get("timeframe_verified", bool(raw.get("timeframe_verified"))))
    navigation_ok = bool(
        op.get(
            "navigation_ok",
            capture_ok and symbol_verified and timeframe_verified,
        )
    )

    chart_bias = str(raw.get("chart_bias") or "neutral")
    volume_confirmation = str(raw.get("volume_confirmation") or "unknown")
    moving_average_state = str(raw.get("moving_average_state") or "unknown")
    pattern_signals = _normalize_pattern_signals(raw.get("pattern_signals") or [])
    quant_score_value = _safe_float(
        quant_score if quant_score is not None else raw.get("selection_score")
    )

    requested_timeframes = raw.get("requested_timeframes")
    if not isinstance(requested_timeframes, list):
        requested_timeframes = list((chart_plan or {}).get("requested_timeframes") or [])
    timeframe_evidence = raw.get("multi_timeframe_snapshots")
    if not isinstance(timeframe_evidence, dict):
        timeframe_evidence = dict((chart_plan or {}).get("timeframe_snapshots") or {})

    multi_timeframe_view = analyze_multi_timeframe_view(
        primary_timeframe=str(timeframe or "").lower(),
        primary_visual_evidence=raw,
        requested_timeframes=requested_timeframes,
        timeframe_evidence=timeframe_evidence,
    )
    mtf_alignment = str(multi_timeframe_view.get("alignment") or "unknown")
    mtf_alignment_score = _safe_float(multi_timeframe_view.get("alignment_score"))
    mtf_component = (mtf_alignment_score or 0.0) * 0.20

    seasonality_context = dict(seasonality_snapshot or build_seasonality_context())
    seasonality_component, seasonality_reason = _seasonality_weight(seasonality_context)

    score = None
    if capture_ok:
        score_val = 0.0
        score_val += 0.26 if navigation_ok else 0.08
        score_val += 0.16 if chart_bias in {"bullish", "bearish"} else 0.05
        score_val += (
            0.14
            if volume_confirmation == "strong"
            else 0.08
            if volume_confirmation == "moderate"
            else 0.02
        )
        score_val += (
            0.12
            if moving_average_state in {"rising", "falling", "stacked_bullish", "stacked_bearish"}
            else 0.04
        )
        if any(p["type"] in {"breakout", "reversal", "expansion"} for p in pattern_signals):
            score_val += 0.11
        else:
            score_val += 0.04
        score_val += mtf_component
        score_val += seasonality_component
        if quant_score_value is not None:
            score_val += max(0.0, min(1.0, quant_score_value / 100.0)) * 0.12
        visual_conf = raw.get("visual_confidence")
        if visual_conf is not None:
            try:
                score_val += min(0.05, max(0.0, float(visual_conf)) * 0.05)
            except Exception:
                pass
        score = round(min(1.0, max(0.0, score_val)), 4)

    bucket = str(raw.get("confluence_bucket") or _bucket_from_score(score, capture_ok=capture_ok)).lower()
    if bucket not in _ALLOWED_BUCKETS:
        bucket = "unknown"

    reason = str(raw.get("fusion_decision_reason") or "").strip()
    if not reason:
        base_reason = _build_reason(
            bucket=bucket,
            capture_ok=capture_ok,
            symbol_verified=symbol_verified,
            timeframe_verified=timeframe_verified,
            chart_bias=chart_bias,
            volume_confirmation=volume_confirmation,
        )
        reason = f"{base_reason} MTF={mtf_alignment}; seasonality={seasonality_reason}."

    operational_context = {
        "navigation_ok": navigation_ok,
        "capture_ok": capture_ok,
        "symbol_verified": symbol_verified,
        "timeframe_verified": timeframe_verified,
    }
    op_reliability_score, op_reliability_bucket = _operational_reliability(operational_context)
    recommended_action = _recommended_action(
        score=score,
        bucket=bucket,
        operational_reliability=op_reliability_score,
        mtf_alignment=mtf_alignment,
        seasonal_bias=str(seasonality_context.get("seasonal_bias") or "unknown"),
    )
    if recommended_action not in _ALLOWED_RECOMMENDED_ACTIONS:
        recommended_action = "manual_review"

    fusion_components = {
        "quant_score": quant_score_value,
        "visual_confluence_component": score,
        "mtf_alignment": mtf_alignment,
        "mtf_alignment_score": mtf_alignment_score,
        "seasonal_bias": str(seasonality_context.get("seasonal_bias") or "unknown"),
        "seasonality_component": seasonality_component,
        "operational_reliability_score": op_reliability_score,
        "operational_reliability_bucket": op_reliability_bucket,
    }

    return {
        "symbol": str(symbol or "").upper(),
        "timeframe": str(timeframe or "").lower(),
        "chart_bias": chart_bias,
        "support_levels": list(raw.get("support_levels") or []),
        "resistance_levels": list(raw.get("resistance_levels") or []),
        "moving_average_state": moving_average_state,
        "volume_confirmation": volume_confirmation,
        "pattern_signals": pattern_signals,
        "multi_timeframe_view": multi_timeframe_view,
        "seasonality_context": seasonality_context,
        "operational_context": operational_context,
        "confluence_score": score,
        "confluence_bucket": bucket,
        "fusion_decision_reason": reason,
        "recommended_action": recommended_action,
        "fusion_components": fusion_components,
        "chart_plan": dict(chart_plan or {}),
        "camera_plan": dict(camera_plan or {}),
    }
