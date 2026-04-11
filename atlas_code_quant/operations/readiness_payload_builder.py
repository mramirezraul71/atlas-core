"""Construcción de payloads de readiness reutilizable por API y notificaciones."""
from __future__ import annotations

from typing import Any

from config.settings import settings as global_settings


def quant_readiness_flags(st: Any | None = None) -> dict[str, object]:
    s = st or global_settings
    return {
        "lightweight_startup": bool(getattr(s, "lightweight_startup", False)),
        "chart_auto_open_enabled": bool(getattr(s, "chart_auto_open_enabled", False)),
        "chart_verify_after_open": bool(getattr(s, "chart_verify_after_open", True)),
        "regime_hysteresis_enabled": bool(getattr(s, "regime_hysteresis_enabled", True)),
        "vision_ocr_crop_margin": float(getattr(s, "vision_ocr_crop_margin", 0.0)),
        "context_price_cycle_enabled": bool(getattr(s, "context_price_cycle_enabled", True)),
        "context_cycle_soft_gate": bool(getattr(s, "context_cycle_soft_gate", False)),
        "default_vision_provider_configured": bool(str(getattr(s, "default_vision_provider", "") or "").strip()),
        "startup_chart_warmup_enabled": bool(getattr(s, "startup_chart_warmup_enabled", False)),
        "chart_provider_default": str(getattr(s, "chart_provider_default", "tradingview") or "tradingview"),
    }


def build_readiness_fast_payload(
    *,
    operation_center: Any,
    vision_service: Any,
    st: Any | None = None,
) -> dict[str, object]:
    """Misma semántica que el endpoint /operation/readiness (modo fast)."""
    from operations.chart_plan_builder import chart_plan_probe_ok
    from operations.readiness_eval import evaluate_operational_readiness, startup_warmup_gate_satisfied

    s = st or global_settings
    chart = operation_center.chart_execution.status()
    vision_status = vision_service.status_for_gate()
    auto_open = bool(getattr(s, "chart_auto_open_enabled", False))
    warmup_on = bool(getattr(s, "startup_chart_warmup_enabled", False))
    chart_plan_needed = auto_open or warmup_on
    probe_detail = ""
    if chart_plan_needed:
        symbols = list(getattr(s, "startup_chart_warmup_symbols", []) or [])
        probe_sym = symbols[0] if symbols else "SPY"
        tf = str(getattr(s, "startup_chart_warmup_timeframe", "1h") or "1h")
        pv = str(getattr(s, "chart_provider_default", "tradingview") or "tradingview")
        chart_plan_buildable, probe_detail = chart_plan_probe_ok(probe_sym, tf, pv)
    else:
        chart_plan_buildable = True
    warm_ok = startup_warmup_gate_satisfied(
        chart,
        startup_chart_warmup_enabled=warmup_on,
        chart_auto_open_enabled=auto_open,
    )
    ready, reasons = evaluate_operational_readiness(
        chart_execution=chart,
        vision_provider_ready=bool(vision_status.get("provider_ready")),
        chart_auto_open_enabled=auto_open,
        chart_plan_buildable=chart_plan_buildable,
        startup_chart_warmup_enabled=warmup_on,
        startup_warmup_satisfied=warm_ok,
        visual_pipeline_ok=None,
    )
    return {
        "ready": ready,
        "reasons_not_ready": reasons,
        "readiness_mode": "fast",
        "chart_execution": chart,
        "vision_status": vision_status,
        "vision_diagnose": None,
        "chart_plan_probe": {
            "required": chart_plan_needed,
            "ok": chart_plan_buildable,
            "detail": probe_detail or None,
        },
        "startup_warmup_gate": {
            "required": warmup_on and auto_open,
            "satisfied": warm_ok,
        },
        "visual_pipeline": {
            "omitted": True,
            "detail": "Usa GET /operation/readiness/diagnostic para diagnose() + VisualPipeline.status().",
        },
        "quant_flags": quant_readiness_flags(s),
    }
