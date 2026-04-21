"""Paper-only visual evidence pipeline for paper_aggressive decisions."""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore[assignment]

from atlas_code_quant.local_models.integrations import analyze_dashboard_screenshot
from atlas_code_quant.operations.visual_decision_context_builder import (
    build_visual_decision_context,
)
from atlas_code_quant.operations.visual_input_provider import VisualInputProvider, VisualFrame


@dataclass
class VisualReviewResult:
    symbol: str
    timeframe: str
    chart_bias: str
    support_levels: list[float]
    resistance_levels: list[float]
    moving_average_state: str
    volume_confirmation: str
    pattern_signals: list[str]
    confluence_score: float
    visual_confidence: float
    screenshot_path: str
    provider_used: str
    used_fallback: bool
    fallback_reason: str
    opportunity_id: str
    trace_id: str
    timestamp_utc: str
    estimated_slippage_bps: float
    fee_model: str
    fill_quality_assumption: str
    partial_fill_risk: float


def _utc_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        x = float(v)
        if math.isnan(x) or math.isinf(x):
            return default
        return x
    except Exception:
        return default


def _supports_resistances_from_frame(frame: np.ndarray) -> tuple[list[float], list[float]]:
    if cv2 is None or frame is None:
        return ([], [])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, _ = gray.shape[:2]
    # Vertical profile as lightweight proxy for frequently visited price zones.
    prof = gray.mean(axis=1)
    idx = np.argsort(prof)
    supports = sorted({round((h - int(i)) / max(h, 1), 3) for i in idx[:3]})
    resistances = sorted({round((h - int(i)) / max(h, 1), 3) for i in idx[-3:]})
    return (supports, resistances)


def _moving_average_state(frame: np.ndarray, *, hint: str = "") -> str:
    if frame is None:
        return "unknown"
    if "bull" in hint:
        return "stacked_bullish"
    if "bear" in hint:
        return "stacked_bearish"
    h, w = frame.shape[:2]
    if cv2 is None or h < 4 or w < 4:
        return "mixed"
    # Approximate slope by comparing average brightness left vs right center-band.
    band = frame[h // 3 : (2 * h) // 3, :, :]
    left = float(np.mean(band[:, : w // 2]))
    right = float(np.mean(band[:, w // 2 :]))
    if right > left + 2.0:
        return "rising"
    if left > right + 2.0:
        return "falling"
    return "flat"


def _volume_confirmation(frame: np.ndarray) -> str:
    if frame is None:
        return "unknown"
    h, _, _ = frame.shape
    lower = frame[int(h * 0.78) :, :, :]
    v = float(np.std(lower))
    if v >= 35.0:
        return "strong"
    if v >= 20.0:
        return "moderate"
    return "weak"


def _pattern_signals(provider_payload: dict[str, Any], *, direction_hint: str = "") -> list[str]:
    signals: list[str] = []
    summary = str(provider_payload.get("summary") or "").lower()
    tags = [str(t).lower() for t in (provider_payload.get("tags") or [])]
    blob = " ".join([summary, *tags, direction_hint.lower()])
    if any(k in blob for k in ("breakout", "ruptura")):
        signals.append("breakout")
    if any(k in blob for k in ("reversal", "revers", "giro")):
        signals.append("reversal")
    if any(k in blob for k in ("compression", "compres")):
        signals.append("compression")
    if any(k in blob for k in ("expansion", "expan")):
        signals.append("expansion")
    if not signals:
        signals.append("no_clear_pattern")
    return signals


def _chart_bias(direction_hint: str, ma_state: str, volume_conf: str) -> str:
    d = str(direction_hint or "").lower()
    if "bear" in d or d in {"short", "down"}:
        return "bearish" if ma_state in {"falling", "stacked_bearish"} else "mixed"
    if "bull" in d or d in {"long", "up"}:
        return "bullish" if ma_state in {"rising", "stacked_bullish"} else "mixed"
    if ma_state in {"rising", "stacked_bullish"} and volume_conf != "weak":
        return "bullish"
    if ma_state in {"falling", "stacked_bearish"} and volume_conf != "weak":
        return "bearish"
    return "neutral"


def _realism_from_candidate(candidate: dict[str, Any], *, confidence: float) -> dict[str, float | str]:
    spread_hint = _safe_float(candidate.get("spread_pct"), 0.12)
    slippage = max(2.0, min(45.0, spread_hint * 100.0))
    partial = max(0.02, min(0.8, spread_hint * (1.25 if confidence < 0.6 else 0.9)))
    strategy = str(candidate.get("strategy_type") or "").lower()
    fee_model = "paper_options_flat" if any(k in strategy for k in ("condor", "spread", "option", "call", "put")) else "paper_equity_flat"
    fill_q = "good" if confidence >= 0.75 else "normal" if confidence >= 0.55 else "degraded"
    return {
        "estimated_slippage_bps": round(slippage, 3),
        "fee_model": fee_model,
        "fill_quality_assumption": fill_q,
        "partial_fill_risk": round(partial, 4),
    }


def build_visual_review_contract(
    *,
    symbol: str,
    timeframe: str,
    opportunity_id: str,
    trace_id: str,
    frame: VisualFrame,
    provider_payload: dict[str, Any],
    candidate: dict[str, Any],
) -> VisualReviewResult:
    supports, resistances = _supports_resistances_from_frame(frame.frame) if frame.frame is not None else ([], [])
    direction_hint = str(candidate.get("direction") or "")
    ma_state = _moving_average_state(frame.frame, hint=direction_hint)
    vol = _volume_confirmation(frame.frame) if frame.frame is not None else "unknown"
    patterns = _pattern_signals(provider_payload, direction_hint=direction_hint)
    confidence_model = _safe_float(provider_payload.get("confidence"), 0.0)
    confidence = max(0.1, min(1.0, confidence_model if confidence_model > 0 else (0.55 if frame.ok else 0.2)))
    bias = _chart_bias(direction_hint, ma_state, vol)
    confluence = round(min(1.0, max(0.0, confidence * (1.0 if vol != "weak" else 0.8))), 4)
    realism = _realism_from_candidate(candidate, confidence=confidence)
    return VisualReviewResult(
        symbol=symbol,
        timeframe=timeframe,
        chart_bias=bias,
        support_levels=supports,
        resistance_levels=resistances,
        moving_average_state=ma_state,
        volume_confirmation=vol,
        pattern_signals=patterns,
        confluence_score=confluence,
        visual_confidence=round(confidence, 4),
        screenshot_path=frame.screenshot_path,
        provider_used=frame.provider_used,
        used_fallback=bool(frame.used_fallback),
        fallback_reason=str(frame.fallback_reason or ""),
        opportunity_id=opportunity_id,
        trace_id=trace_id,
        timestamp_utc=_utc_iso(),
        estimated_slippage_bps=float(realism["estimated_slippage_bps"]),
        fee_model=str(realism["fee_model"]),
        fill_quality_assumption=str(realism["fill_quality_assumption"]),
        partial_fill_risk=float(realism["partial_fill_risk"]),
    )


def collect_visual_evidence(
    *,
    symbol: str,
    timeframe: str,
    opportunity_id: str,
    trace_id: str,
    chart_source: str,
    candidate: dict[str, Any],
    chart_plan: dict[str, Any] | None = None,
    camera_plan: dict[str, Any] | None = None,
) -> dict[str, Any]:
    provider = VisualInputProvider()
    chart_frame = provider.get_chart_frame(source=chart_source, symbol=symbol, timeframe=timeframe)
    if not chart_frame.ok:
        # Final fallback: desktop frame to always capture evidence.
        screen = provider.get_screen_frame()
        if screen.ok:
            screen.used_fallback = True
            screen.fallback_reason = chart_frame.fallback_reason or "chart_frame_unavailable"
            chart_frame = screen

    vision_payload: dict[str, Any] = {}
    if chart_frame.screenshot_path:
        try:
            vision_payload = analyze_dashboard_screenshot(chart_frame.screenshot_path)
        except Exception as exc:
            vision_payload = {"ok": False, "error": str(exc), "confidence": 0.0}

    contract = build_visual_review_contract(
        symbol=symbol,
        timeframe=timeframe,
        opportunity_id=opportunity_id,
        trace_id=trace_id,
        frame=chart_frame,
        provider_payload=vision_payload if isinstance(vision_payload, dict) else {},
        candidate=candidate,
    )
    payload = asdict(contract)
    frame_meta = chart_frame.metadata if isinstance(chart_frame.metadata, dict) else {}
    capture_ok = bool(chart_frame.ok and chart_frame.frame is not None)
    observed_symbol = str(frame_meta.get("symbol") or "").strip().upper()
    observed_timeframe = str(frame_meta.get("timeframe") or "").strip().lower()
    symbol_verified = bool(observed_symbol) and observed_symbol == str(symbol).strip().upper()
    timeframe_verified = bool(observed_timeframe) and observed_timeframe == str(timeframe).strip().lower()
    navigation_ok = capture_ok and symbol_verified and timeframe_verified

    chart_plan_payload = (
        dict(chart_plan)
        if isinstance(chart_plan, dict)
        else dict(candidate.get("chart_plan") or {})
    )
    camera_plan_payload = (
        dict(camera_plan)
        if isinstance(camera_plan, dict)
        else dict(candidate.get("camera_plan") or {})
    )
    visual_context = build_visual_decision_context(
        symbol=symbol,
        timeframe=timeframe,
        visual_evidence=payload,
        seasonality_snapshot=None,
        operational_status={
            "navigation_ok": navigation_ok,
            "capture_ok": capture_ok,
            "symbol_verified": symbol_verified,
            "timeframe_verified": timeframe_verified,
        },
        chart_plan=chart_plan_payload,
        camera_plan=camera_plan_payload,
        quant_score=_safe_float(candidate.get("selection_score"), 0.0),
    )
    payload["capture_ok"] = capture_ok
    payload["symbol_verified"] = symbol_verified
    payload["timeframe_verified"] = timeframe_verified
    payload["navigation_ok"] = navigation_ok
    payload["visual_decision_context"] = visual_context
    payload["confluence_score"] = visual_context.get("confluence_score")
    payload["confluence_bucket"] = visual_context.get("confluence_bucket", "unknown")
    payload["fusion_decision_reason"] = visual_context.get("fusion_decision_reason")
    payload["fusion_score"] = visual_context.get("confluence_score")
    payload["recommended_action"] = visual_context.get("recommended_action", "manual_review")
    payload["seasonality_context"] = visual_context.get("seasonality_context")
    payload["multi_timeframe_view"] = visual_context.get("multi_timeframe_view")
    payload["operational_context"] = visual_context.get("operational_context")
    payload["fusion_components"] = visual_context.get("fusion_components")
    evidence_path = Path(payload["screenshot_path"]).parent if payload.get("screenshot_path") else Path(
        Path(__file__).resolve().parents[1] / "runtime_artifacts" / "visual_evidence"
    )
    evidence_path.mkdir(parents=True, exist_ok=True)
    meta = evidence_path / f"visual_{symbol}_{timeframe}_{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}.json"
    try:
        meta.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        payload["meta_path"] = str(meta)
    except Exception:
        payload["meta_path"] = ""
    return payload
