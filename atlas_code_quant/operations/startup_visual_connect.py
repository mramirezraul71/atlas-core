"""Conexión en arranque: proveedor de visión persistido + apertura opcional de gráficos (mismo chart_plan que el selector)."""
from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger("quant.startup_visual_connect")

_SUPPORTED_VISION = frozenset({"off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360"})


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
            from selector.strategy_selector import _chart_plan
        except Exception as exc:
            out["chart_warmup_error"] = f"import _chart_plan: {exc}"
            logger.exception("Chart warmup import failed")
            return out

        chart_exec = operation_center.chart_execution
        results: list[dict[str, Any]] = []
        for sym in symbols:
            sym_u = str(sym or "").strip().upper()
            if not sym_u:
                continue
            try:
                cp = _chart_plan(sym_u, tf, None, chart_pv)
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
