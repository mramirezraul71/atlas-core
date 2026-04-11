"""Evaluación semántica de readiness operativo (gate), separada del transporte HTTP."""
from __future__ import annotations

from typing import Any


def evaluate_operational_readiness(
    *,
    chart_execution: dict[str, Any],
    vision_provider_ready: bool,
    chart_auto_open_enabled: bool,
) -> tuple[bool, list[str]]:
    """Devuelve (listo_para_operar_paper_visual, razones_si_no).

    Reglas mínimas:
    - Si auto-open de charts está activo, debe haber navegador resuelto.
    - El proveedor de visión activo debe reportar provider_ready (p. ej. Nexus arriba si direct_nexus).
    """
    reasons: list[str] = []
    if chart_auto_open_enabled and not bool(chart_execution.get("browser_available")):
        reasons.append(
            "chart_auto_open_enabled=true pero browser_available=false; instala Chrome/Edge o revisa chart_launcher."
        )
    if not vision_provider_ready:
        reasons.append(
            "vision provider_ready=false; revisa POST /operation/vision/provider o arranque (QUANT_DEFAULT_VISION_PROVIDER)."
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
