"""Transforma el reporte del OpportunityScanner (Quant) al contrato JSON del radar PUSH."""
from __future__ import annotations

import logging
from datetime import datetime, timezone
from typing import Any

logger = logging.getLogger("atlas.radar.mapper")


def _utc_from_report(report: dict[str, Any]) -> str:
    return str(report.get("generated_at") or "")


def pick_candidate(symbol: str, report: dict[str, Any]) -> dict[str, Any] | None:
    sym = (symbol or "").strip().upper()
    if not sym:
        return None
    for item in report.get("candidates") or []:
        if not isinstance(item, dict):
            continue
        if str(item.get("symbol") or "").strip().upper() == sym:
            return item
    return None


def _norm_0_1_from_pct(value: Any, *, scale: float = 100.0) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not v:
        return 0.0
    return max(0.0, min(1.0, v / scale))


def _bias_from_candidate(cand: dict[str, Any] | None) -> str:
    if not cand:
        return "neutral"
    direction = str(cand.get("direction") or "").strip().lower()
    if direction in ("long", "largo", "alcista", "buy"):
        return "long"
    if direction in ("short", "corto", "bajista", "sell"):
        return "short"
    return "neutral"


def compute_degradations_active(
    transport: dict[str, Any],
    provider_health_summary: dict[str, Any],
    camera_context: dict[str, Any],
) -> list[dict[str, str]]:
    """Entradas explicativas de degradación para el dashboard (M2).

    Códigos: STUB_MODE, QUANT_UNREACHABLE, PROVIDERS_DEGRADED, CAMERA_UNAVAILABLE.
    """
    out: list[dict[str, str]] = []
    if transport.get("stub") is True:
        out.append(
            {
                "code": "STUB_MODE",
                "label": "Modo demostración: no hay reporte en vivo del escáner en este ciclo (stub).",
                "severity": "warning",
                "source": "transport",
            }
        )
    if transport.get("quant") is False:
        out.append(
            {
                "code": "QUANT_UNREACHABLE",
                "label": "Puente PUSH→Quant no conectado o clave de API no configurada en el servidor.",
                "severity": "critical",
                "source": "quant",
            }
        )
    try:
        dc = int(provider_health_summary.get("degraded_count") or 0)
    except (TypeError, ValueError):
        dc = 0
    if dc > 0:
        checked = provider_health_summary.get("providers_checked")
        out.append(
            {
                "code": "PROVIDERS_DEGRADED",
                "label": (
                    f"Proveedores en estado degradado: {dc}"
                    + (f" (revisados: {checked})" if checked is not None else "")
                ),
                "severity": "warning",
                "source": "provider",
            }
        )
    pr = camera_context.get("provider_ready")
    st = str(camera_context.get("state") or "").strip().lower()
    # "disabled/off" can be an intentional stabilization mode.  Treat it as
    # neutral so the institutional radar does not advertise a camera failure
    # while visual validation is explicitly disconnected.
    if st in {"disabled", "off"}:
        return out
    bad_states = frozenset({"unavailable", "degraded", "not_configured"})
    if pr is False or (st and st in bad_states):
        sev = "warning" if st in {"degraded", "not_configured"} else "critical"
        out.append(
            {
                "code": "CAMERA_UNAVAILABLE",
                "label": "Cámara o proveedor de visión no listo para captura en vivo.",
                "severity": sev,
                "source": "camera",
            }
        )
    return out


def _snapshot_classification(symbol: str, report: dict[str, Any], cand: dict[str, Any] | None) -> str:
    if report.get("error"):
        return "non_operable"
    status = report.get("status") or {}
    if status.get("last_error") and not status.get("running"):
        return "non_operable"
    if not status.get("running"):
        return "structural_only"
    if cand is not None:
        return "fully_operable"
    # Escáner en marcha pero el ticker no está entre candidatos de este ciclo.
    return "operable_with_degradation"


def build_dashboard_summary(symbol: str, report: dict[str, Any], *, quant_ms: float | None = None) -> dict[str, Any]:
    sym = (symbol or "SPY").strip().upper() or "SPY"
    cand = pick_candidate(sym, report)
    ts = _utc_from_report(report) or datetime.now(timezone.utc).isoformat()
    classification = _snapshot_classification(sym, report, cand)
    if report.get("error"):
        logger.warning(
            "build_dashboard_summary: reporte con error de Quant (symbol=%s classification=%s err=%r)",
            sym,
            classification,
            report.get("error"),
        )
    elif cand is None and (report.get("status") or {}).get("running"):
        logger.info(
            "build_dashboard_summary: motor en marcha pero sin candidato para %s (clasificación=%s)",
            sym,
            classification,
        )
    fast_pressure = 0.0
    structural_conf = 0.0
    alignment = 0.0
    divergence = 0.0
    if cand:
        fast_pressure = _norm_0_1_from_pct(cand.get("signal_strength_pct"), scale=100.0)
        if fast_pressure == 0.0 and cand.get("selection_score") is not None:
            fast_pressure = _norm_0_1_from_pct(cand.get("selection_score"), scale=100.0)
        structural_conf = _norm_0_1_from_pct(cand.get("local_win_rate_pct"), scale=100.0)
        try:
            alignment = max(0.0, min(1.0, float(cand.get("macd_hist") or 0.0)))
        except (TypeError, ValueError):
            alignment = 0.0
        try:
            divergence = max(0.0, min(1.0, abs(float(cand.get("bb_pct") or 0.0))))
        except (TypeError, ValueError):
            divergence = 0.0

    status = report.get("status") or {}
    summary = report.get("summary") or {}
    order_flow = report.get("order_flow_system") or {}

    gate_latest: dict[str, Any] | None = None
    if cand:
        gate_latest = {
            "decision": "accepted",
            "reason": f"Candidato Quant ({cand.get('strategy_label') or cand.get('strategy_key') or 'setup'})",
            "symbol": sym,
            "timestamp": ts,
        }

    result: dict[str, Any] = {
        "symbol": sym,
        "last_update": ts,
        "stream_available": True,
        "transport": {
            "sse": True,
            "stub": False,
            "quant": True,
            "quant_scanner_ms": quant_ms,
        },
        "quant": {
            "connected": True,
            "scanner_running": bool(status.get("running")),
            "cycle_count": int(status.get("cycle_count") or summary.get("cycle_count") or 0),
            "current_symbol": status.get("current_symbol"),
            "current_step": status.get("current_step"),
        },
        "radar": {
            "signal": {
                "timestamp": ts,
                "bias": _bias_from_candidate(cand),
                "meta": {
                    "snapshot_classification": classification,
                    "fast_pressure_score": round(float(fast_pressure), 4),
                    "structural_confidence_score": round(float(structural_conf), 4),
                    "fast_structural_alignment": round(float(alignment), 4),
                    "fast_structural_divergence_score": round(float(divergence), 4),
                    "horizon_conflict": bool(cand.get("vix_context", {}).get("gate") == "caution") if cand else False,
                    "cross_horizon_alignment": str(cand.get("timeframe") or "-") if cand else "-",
                },
            }
        },
        "decision_gate": {
            "recent": [],
            "latest": gate_latest,
        },
        "camera_context": {
            "provider": "Atlas Code-Quant (sensor vía servicio Quant)",
            "status": "La vista en vivo depende del proveedor de visión configurado en Quant, no de este stub.",
            "provider_ready": bool(order_flow.get("provider_ready")),
            "last_capture": None,
            "presence_score": None,
            "activity_level": None,
        },
        "provider_health_summary": {
            "providers_checked": 2,
            "degraded_count": 0 if status.get("running") else 1,
        },
    }
    result["degradations_active"] = compute_degradations_active(
        result["transport"], result["provider_health_summary"], result["camera_context"]
    )
    return result


def build_providers_payload(report: dict[str, Any]) -> dict[str, Any]:
    status = report.get("status") or {}
    ofs = report.get("order_flow_system") or {}
    running = bool(status.get("running"))
    of_ready = bool(ofs.get("provider_ready"))
    last_ms = status.get("last_cycle_ms")
    try:
        lat = int(float(last_ms)) if last_ms is not None else 0
    except (TypeError, ValueError):
        lat = 0
    err = status.get("last_error") or ofs.get("provider_error") or ""
    return {
        "providers": [
            {
                "provider": "opportunity_scanner",
                "name": "OpportunityScanner (Atlas Code-Quant)",
                "is_ready": running,
                "ready": running,
                "stale_indicator": False,
                "latency_ms": lat,
                "p95_latency_ms": lat,
                "active_fallback_indicator": False,
                "circuit_open_indicator": bool(report.get("error")),
                "consecutive_errors": 1 if report.get("error") else 0,
                "availability_ratio": 1.0 if running else 0.0,
                "last_error_type": str(err)[:120] if err else "",
                "ui_status_hint": "quant" if running else "degradado",
            },
            {
                "provider": "order_flow_surface",
                "name": "Superficie opciones / order flow (Quant)",
                "is_ready": of_ready,
                "ready": of_ready,
                "stale_indicator": False,
                "latency_ms": 0,
                "p95_latency_ms": 0,
                "active_fallback_indicator": not of_ready and running,
                "circuit_open_indicator": False,
                "consecutive_errors": 0,
                "availability_ratio": 1.0 if of_ready else 0.35,
                "last_error_type": str(ofs.get("provider_error") or "")[:120],
                "ui_status_hint": "quant" if of_ready else "fallback",
            },
        ]
    }


def build_decisions_recent(report: dict[str, Any], limit: int = 40) -> dict[str, Any]:
    ts = _utc_from_report(report)
    rows: list[dict[str, Any]] = []
    for rej in (report.get("rejections") or [])[:limit]:
        if not isinstance(rej, dict):
            continue
        rows.append(
            {
                "evaluation": {
                    "symbol": rej.get("symbol"),
                    "timeframe": rej.get("timeframe"),
                    "snapshot_classification": "non_operable",
                    "fast_pressure_score": _norm_0_1_from_pct(rej.get("selection_score"), scale=100.0),
                    "structural_confidence_score": _norm_0_1_from_pct(rej.get("local_win_rate_pct"), scale=100.0),
                    "decision": "rejected",
                    "reason": "; ".join([str(x) for x in (rej.get("reasons") or [])])[:220],
                    "timestamp": ts,
                }
            }
        )
    for act in list(reversed(report.get("activity") or []))[: max(0, limit - len(rows))]:
        if not isinstance(act, dict):
            continue
        rows.append(
            {
                "evaluation": {
                    "symbol": act.get("symbol") or "-",
                    "timeframe": "-",
                    "snapshot_classification": "structural_only",
                    "fast_pressure_score": None,
                    "structural_confidence_score": None,
                    "decision": "caution" if act.get("level") == "warn" else "bypassed",
                    "reason": str(act.get("message") or "")[:220],
                    "timestamp": act.get("timestamp") or ts,
                }
            }
        )
    return {"recent": rows[:limit]}


def build_decisions_stats(report: dict[str, Any]) -> dict[str, Any]:
    cands = [c for c in (report.get("candidates") or []) if isinstance(c, dict)]
    rejs = [r for r in (report.get("rejections") or []) if isinstance(r, dict)]
    fast_vals: list[float] = []
    struct_vals: list[float] = []
    for item in cands + rejs:
        fast_vals.append(_norm_0_1_from_pct(item.get("signal_strength_pct") or item.get("selection_score"), scale=100.0))
        struct_vals.append(_norm_0_1_from_pct(item.get("local_win_rate_pct"), scale=100.0))
    avg_fast = sum(fast_vals) / len(fast_vals) if fast_vals else None
    avg_struct = sum(struct_vals) / len(struct_vals) if struct_vals else None
    return {
        "stats": {
            "by_decision": {"accepted": len(cands), "rejected": len(rejs)},
            "avg_fast_pressure_score": round(avg_fast, 4) if avg_fast is not None else None,
            "avg_structural_confidence_score": round(avg_struct, 4) if avg_struct is not None else None,
        }
    }


def build_dealer(symbol: str, report: dict[str, Any]) -> dict[str, Any]:
    cand = pick_candidate(symbol, report)
    ts = _utc_from_report(report)
    iv = float(cand.get("iv_rank") or 0.0) / 100.0 if cand else None
    skew = float(cand.get("iv_hv_ratio") or 0.0) if cand else None
    return {
        "symbol": (symbol or "").strip().upper() or "SPY",
        "timestamp": ts,
        "gamma_flip_level": "-",
        "dealer_skew_score": round(skew, 4) if skew is not None else None,
        "call_wall": "-",
        "put_wall": "-",
        "acceleration_zone_score": round(iv, 4) if iv is not None else None,
    }


def build_fast(symbol: str, report: dict[str, Any]) -> dict[str, Any]:
    cand = pick_candidate(symbol, report)
    ts = _utc_from_report(report)
    fp = None
    risk = None
    bias = None
    if cand:
        fp = round(_norm_0_1_from_pct(cand.get("signal_strength_pct"), scale=100.0), 4)
        if fp == 0.0:
            fp = round(_norm_0_1_from_pct(cand.get("selection_score"), scale=100.0), 4)
        risk = round(_norm_0_1_from_pct(cand.get("rsi"), scale=100.0), 4) if cand.get("rsi") is not None else None
        try:
            bias = round(float(cand.get("volume_ratio") or 0.0), 4)
        except (TypeError, ValueError):
            bias = None
    return {
        "symbol": (symbol or "").strip().upper() or "SPY",
        "timestamp": ts,
        "fast_pressure_score": fp,
        "fast_risk_score": risk,
        "fast_directional_bias_score": bias,
    }


def build_structural(symbol: str, report: dict[str, Any]) -> dict[str, Any]:
    cand = pick_candidate(symbol, report)
    ts = _utc_from_report(report)
    sc = None
    bull = None
    bear = None
    if cand:
        sc = round(_norm_0_1_from_pct(cand.get("local_win_rate_pct"), scale=100.0), 4)
        bull = round(_norm_0_1_from_pct(cand.get("local_avg_win_pct"), scale=100.0), 4) if cand.get("local_avg_win_pct") is not None else None
        bear = round(_norm_0_1_from_pct(cand.get("local_expectancy_pct"), scale=100.0), 4) if cand.get("local_expectancy_pct") is not None else None
    return {
        "symbol": (symbol or "").strip().upper() or "SPY",
        "timestamp": ts,
        "structural_confidence_score": sc,
        "structural_bullish_score": bull,
        "structural_bearish_score": bear,
    }


def build_political(symbol: str, report: dict[str, Any]) -> dict[str, Any]:
    cand = pick_candidate(symbol, report)
    ts = _utc_from_report(report)
    agg = None
    if cand:
        agg = round(_norm_0_1_from_pct(cand.get("selection_score"), scale=100.0), 4)
    return {
        "symbol": (symbol or "").strip().upper() or "SPY",
        "timestamp": ts,
        "aggregate_signal_score": agg,
    }
