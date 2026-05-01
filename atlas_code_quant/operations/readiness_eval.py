"""Evaluación semántica de readiness operativo (gate), separada del transporte HTTP."""
from __future__ import annotations

from typing import Any


def startup_warmup_gate_satisfied(
    chart_execution: dict[str, Any],
    *,
    startup_chart_warmup_enabled: bool,
    chart_auto_open_enabled: bool,
) -> bool:
    """Si warmup+auto_open están activos, exige evidencia en last_payload (apertura o cooldown)."""
    if not (startup_chart_warmup_enabled and chart_auto_open_enabled):
        return True
    lp = chart_execution.get("last_payload")
    if not isinstance(lp, dict) or not lp:
        return False
    if lp.get("open_ok") is True:
        return True
    st = str(lp.get("execution_state") or "")
    return st in {"opened", "cooldown_reuse"}


def evaluate_operational_readiness(
    *,
    chart_execution: dict[str, Any],
    vision_provider_ready: bool,
    chart_auto_open_enabled: bool,
    chart_plan_buildable: bool = True,
    startup_chart_warmup_enabled: bool = False,
    startup_warmup_satisfied: bool = True,
    visual_pipeline_ok: bool | None = None,
) -> tuple[bool, list[str]]:
    """Devuelve (listo_para_operar_paper_visual, razones_si_no).

    Reglas:
    - Si auto-open de charts está activo, debe haber navegador resuelto.
    - Si aplica chart_plan al flujo (auto-open o warmup), el plan debe ser generable (probe).
    - Si warmup de arranque + auto-open: evidencia de misión de gráficos en last_payload.
    - El proveedor de visión activo debe reportar provider_ready.
    - Modo diagnostic: si visual_pipeline_ok es False, no listo (p. ej. VisualPipeline no inicializado).
    """
    reasons: list[str] = []
    if chart_auto_open_enabled and not bool(chart_execution.get("browser_available")):
        reasons.append(
            "chart_auto_open_enabled=true pero browser_available=false; instala Chrome/Edge o revisa chart_launcher."
        )
    if not chart_plan_buildable:
        reasons.append(
            "chart_plan no generable o URLs inválidas; revisa QUANT_CHART_PROVIDER, símbolos de warmup y chart_plan_builder."
        )
    if startup_chart_warmup_enabled and chart_auto_open_enabled and not startup_warmup_satisfied:
        reasons.append(
            "QUANT_STARTUP_CHART_WARMUP=true con auto-open: aún no hay evidencia de apertura de gráficos "
            "(last_payload); espera al warmup de arranque o abre gráficos manualmente vía misión."
        )
    if not vision_provider_ready:
        reasons.append(
            "vision provider_ready=false; revisa POST /operation/vision/provider o arranque (QUANT_DEFAULT_VISION_PROVIDER)."
        )
    if visual_pipeline_ok is False:
        reasons.append(
            "visual_pipeline no operativo; revisa GET /operation/readiness/diagnostic y /vision/chart/status."
        )
    return (len(reasons) == 0, reasons)


def readiness_http_body_ok(body: Any) -> bool:
    """True si la API declaró éxito semántico (no solo HTTP 200)."""
    if not isinstance(body, dict):
        return False
    if not body.get("ok"):
        return False
    data = body.get("data")
    if not isinstance(data, dict):
        return False
    return data.get("ready") is True
