"""Conexión en arranque: proveedor de visión persistido + apertura opcional de gráficos (mismo chart_plan que el selector)."""
from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger("quant.startup_visual_connect")

_SUPPORTED_VISION = frozenset({"off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360"})


def apply_startup_camera_autoconfigure(*, vision_service: Any, settings: Any) -> dict[str, Any]:
    """Si ENABLE_CAMERA=true y no hay QUANT_DEFAULT_VISION_PROVIDER, activa Insta360 solo con evidencia (RTMP o USB).

    No bloquea el proceso si falla; no sobrescribe un proveedor explícito en entorno.
    """
    import os

    out: dict[str, Any] = {}
    if not bool(getattr(settings, "enable_camera", False)):
        out["applied"] = False
        out["reason"] = "ENABLE_CAMERA=false"
        return out
    if str(getattr(settings, "default_vision_provider", "") or "").strip().lower():
        out["applied"] = False
        out["reason"] = "QUANT_DEFAULT_VISION_PROVIDER set"
        return out
    rtmp = (os.getenv("INSTA360_RTMP_URL") or "").strip()
    try:
        diag = vision_service.diagnose()
    except Exception as exc:
        out["applied"] = False
        out["error"] = str(exc)
        logger.warning("Startup camera autoconfigure: diagnose failed: %s", exc)
        return out
    current = str(diag.get("provider") or "").strip().lower()
    if current == "insta360":
        out["applied"] = False
        out["reason"] = "already_insta360"
        return out
    if rtmp:
        try:
            vision_service.update(provider="insta360", notes="startup: ENABLE_CAMERA auto (INSTA360_RTMP_URL)")
            out["applied"] = True
            out["provider"] = "insta360"
            return out
        except Exception as exc:
            out["applied"] = False
            out["error"] = str(exc)
            logger.warning("Startup camera autoconfigure: insta360 RTMP apply failed: %s", exc)
            return out
    insta_chk = next((c for c in (diag.get("checks") or []) if isinstance(c, dict) and c.get("name") == "insta360"), {})
    if bool(insta_chk.get("reachable")):
        try:
            vision_service.update(provider="insta360", notes="startup: ENABLE_CAMERA auto (USB/pipeline)")
            out["applied"] = True
            out["provider"] = "insta360"
            return out
        except Exception as exc:
            out["applied"] = False
            out["error"] = str(exc)
            logger.warning("Startup camera autoconfigure: insta360 USB apply failed: %s", exc)
            return out
    out["applied"] = False
    out["reason"] = "no_insta360_evidence"
    return out


def apply_startup_visual_connections(
    *,
    vision_service: Any,
    operation_center: Any,
    settings: Any,
) -> dict[str, Any]:
    """Ejecutar en hilo de fondo tras cargar settings (p. ej. desde FastAPI startup)."""
    out: dict[str, Any] = {}

    prov = str(getattr(settings, "default_vision_provider", "") or "").strip().lower()
    if prov and prov in _SUPPORTED_VISION:
        try:
            vision_service.update(provider=prov, notes="startup: QUANT_DEFAULT_VISION_PROVIDER")
            out["vision_provider_applied"] = prov
        except Exception as exc:
            out["vision_provider_error"] = str(exc)
            logger.warning("Startup vision provider apply failed: %s", exc)
    else:
        out["vision_provider_applied"] = None

    warmup = bool(getattr(settings, "startup_chart_warmup_enabled", False))
    auto_open = bool(getattr(settings, "chart_auto_open_enabled", False))
    if warmup and auto_open:
        symbols = list(getattr(settings, "startup_chart_warmup_symbols", []) or [])
        tf = str(getattr(settings, "startup_chart_warmup_timeframe", "1h") or "1h")
        chart_pv = str(getattr(settings, "chart_provider_default", "tradingview") or "tradingview")
        try:
            try:
                from atlas_code_quant.operations.chart_plan_builder import build_selector_chart_plan
            except ModuleNotFoundError:
                from operations.chart_plan_builder import build_selector_chart_plan
        except Exception as exc:
            out["chart_warmup_error"] = f"import chart_plan_builder: {exc}"
            logger.exception("Chart warmup: fallo import chart_plan_builder (revisar PYTHONPATH/cwd)")
            return out

        chart_exec = operation_center.chart_execution
        results: list[dict[str, Any]] = []
        for sym in symbols:
            sym_u = str(sym or "").strip().upper()
            if not sym_u:
                continue
            try:
                cp = build_selector_chart_plan(sym_u, tf, None, chart_pv)
                payload = chart_exec.ensure_chart_mission(
                    chart_plan=cp,
                    camera_plan={},
                    symbol=sym_u,
                )
                results.append(
                    {
                        "symbol": sym_u,
                        "execution_state": payload.get("execution_state"),
                        "open_ok": payload.get("open_ok"),
                        "open_mode": payload.get("open_mode"),
                    }
                )
            except Exception as exc:
                logger.warning("Chart warmup %s: %s", sym_u, exc)
                results.append({"symbol": sym_u, "error": str(exc)})
        out["chart_warmup"] = results
    elif warmup and not auto_open:
        out["chart_warmup_skipped"] = "QUANT_STARTUP_CHART_WARMUP=true but QUANT_CHART_AUTO_OPEN_ENABLED=false"
    else:
        out["chart_warmup"] = None

    return out
