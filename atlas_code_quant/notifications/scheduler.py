"""Ventanas horarias premarket/EoD + pulso intradía ligero."""
from __future__ import annotations

import asyncio
import logging
from datetime import datetime
from typing import Any
from zoneinfo import ZoneInfo

from config.settings import settings as global_settings

logger = logging.getLogger("quant.notifications.scheduler")

_state: dict[str, Any] = {"premarket_day": None, "eod_day": None, "last_intraday_fp": None, "task": None}


def _parse_hhmm(raw: str) -> tuple[int, int]:
    parts = (raw or "08:00").strip().split(":")
    try:
        h = max(0, min(23, int(parts[0])))
        m = max(0, min(59, int(parts[1]) if len(parts) > 1 else 0))
        return h, m
    except Exception:
        return 8, 0


def _is_trade_exit_event(ev: Any) -> bool:
    if getattr(ev, "category", None) != "trade":
        return False
    meta = getattr(ev, "metadata", None) or {}
    if bool(meta.get("is_exit")):
        return True
    if str(meta.get("trade_stage") or "").strip().lower() == "exit":
        return True
    return "SALIDA" in str(getattr(ev, "title", "") or "")


def _direction_bias(raw: Any) -> str:
    v = str(raw or "").strip().lower()
    if v in {"alcista", "bullish", "long", "up", "compra"}:
        return "alcista"
    if v in {"bajista", "bearish", "short", "down", "venta"}:
        return "bajista"
    return "neutral"


def _safe_float(raw: Any, default: float = 0.0) -> float:
    try:
        return float(raw)
    except Exception:
        return default


def _strategy_recommendation(candidate: dict[str, Any], urgency: str | None = None) -> str:
    direction = _direction_bias(candidate.get("direction"))
    method_key = str(candidate.get("strategy_key") or candidate.get("method") or candidate.get("primary_method") or "").strip()
    method_label = str(candidate.get("strategy_label") or candidate.get("method_label") or method_key or "setup técnico").strip()
    family = str(candidate.get("strategy_family") or "").strip().lower()
    urgency = str(urgency or "").strip().upper()

    if urgency == "ALTA":
        execution_hint = "Si confirma trigger de entrada, se puede ejecutar sin demorar."
    elif urgency == "BAJA":
        execution_hint = "Priorizar paciencia: esperar confirmación adicional o reducir tamaño."
    else:
        execution_hint = "Aplicar confirmación estándar y mantener gestión de riesgo disciplinada."

    if direction == "alcista":
        base = f"Estrategia sugerida: enfoque LONG con {method_label}"
        if family in {"trend", "momentum", "breakout"}:
            return f"{base}, buscando continuidad/pullback con confirmación. {execution_hint}"
        return f"{base}, validando confirmación antes de ampliar tamaño. {execution_hint}"
    if direction == "bajista":
        base = f"Estrategia sugerida: enfoque SHORT/PUT con {method_label}"
        if family in {"trend", "momentum", "breakout"}:
            return f"{base}, priorizando rupturas fallidas o rechazos en resistencia. {execution_hint}"
        return f"{base}, con ejecución defensiva y control de riesgo. {execution_hint}"
    return (
        f"Estrategia sugerida: sesgo neutral con {method_label}; "
        f"esperar ruptura confirmada o usar tamaño reducido. {execution_hint}"
    )


def _tactical_plan(candidate: dict[str, Any], urgency: str) -> str:
    direction = _direction_bias(candidate.get("direction"))
    urgency = str(urgency or "").strip().upper()
    price = _safe_float(candidate.get("price"), 0.0)
    predicted_move_pct = abs(_safe_float(candidate.get("predicted_move_pct"), 0.0))
    avg_loss_pct = abs(_safe_float(candidate.get("local_avg_loss_pct"), 0.0))

    if price <= 0:
        return (
            "Plan táctico: Entrada en trigger del setup | Invalidación al romper estructura "
            "opuesta | Objetivo en primer tramo de continuación."
        )

    if predicted_move_pct <= 0:
        predicted_move_pct = 1.2
    if avg_loss_pct <= 0:
        avg_loss_pct = 0.8

    # Ajuste de agresividad por urgencia.
    if urgency == "ALTA":
        target_factor = 0.75
        risk_factor = 1.00
    elif urgency == "BAJA":
        target_factor = 0.45
        risk_factor = 0.80
    else:
        target_factor = 0.60
        risk_factor = 0.90

    objective_pct = predicted_move_pct * target_factor
    invalidation_pct = avg_loss_pct * risk_factor

    if direction == "alcista":
        entry = price
        invalidation = price * (1.0 - invalidation_pct / 100.0)
        target = price * (1.0 + objective_pct / 100.0)
    elif direction == "bajista":
        entry = price
        invalidation = price * (1.0 + invalidation_pct / 100.0)
        target = price * (1.0 - objective_pct / 100.0)
    else:
        entry = price
        invalidation = price * (1.0 - (invalidation_pct * 0.7) / 100.0)
        target = price * (1.0 + (objective_pct * 0.7) / 100.0)

    risk = abs(entry - invalidation)
    reward = abs(target - entry)
    rr = (reward / risk) if risk > 0 else 0.0

    return (
        f"Plan táctico: Entrada ≈ {entry:.2f} | Invalidación ≈ {invalidation:.2f} "
        f"| Objetivo ≈ {target:.2f} | R:R ≈ {rr:.2f}"
    )


def _ic_alignment_text(orchestrator: dict[str, Any], method_key: str) -> str:
    method_key = str(method_key or "").strip()
    ic_by_method = orchestrator.get("last_ic_by_method") or {}
    method_ic = ic_by_method.get(method_key) if method_key else None
    if not isinstance(method_ic, dict):
        return (
            "IC (Information Coefficient) mide si el método anticipa la dirección futura. "
            "No hay muestra reciente para este setup; tratar señal como táctica y usar tamaño conservador."
        )
    try:
        ic_val = float(method_ic.get("ic") or 0.0)
    except Exception:
        ic_val = 0.0
    try:
        n_obs = int(method_ic.get("n") or 0)
    except Exception:
        n_obs = 0
    if n_obs < 10:
        status = "aún no concluyente"
        action = "validar manualmente contexto y no escalar tamaño agresivo."
    elif ic_val >= 0.10:
        status = "fuerte y favorable"
        action = "permite ejecutar con confianza táctica si el resto del contexto acompaña."
    elif ic_val >= 0.05:
        status = "positivo"
        action = "apoya la señal, pero mantener disciplina de confirmación y riesgo."
    elif ic_val <= -0.05:
        status = "negativo"
        action = "contradice la señal; conviene reducir tamaño o evitar entrada."
    else:
        status = "cercano a neutral"
        action = "el edge estadístico es débil; priorizar confirmaciones extra."
    return f"Alineación con IC ({method_key}): IC={ic_val:+.3f} con n={n_obs}, {status}; {action}"


def _urgency_level(candidate: dict[str, Any], selection_score: float) -> tuple[str, str]:
    direction = _direction_bias(candidate.get("direction"))
    confirmation = candidate.get("confirmation") if isinstance(candidate.get("confirmation"), dict) else {}
    higher_dir = _direction_bias(confirmation.get("direction"))
    higher_conf = float(confirmation.get("confidence_pct") or 0.0)
    aligned_context = direction in {"alcista", "bajista"} and higher_dir == direction and higher_conf >= 60.0

    if selection_score >= 85.0 and aligned_context:
        return "ALTA", "score alto y contexto superior alineado"
    if selection_score >= 75.0:
        return "MEDIA", "señal válida, pero requiere confirmación de ejecución"
    return "BAJA", "convicción limitada; conviene esperar mejor confirmación"


async def _scheduler_tick() -> None:
    from notifications.briefing_service import get_operational_briefing_service

    s = global_settings
    if not s.notify_enabled:
        return
    if s.notify_paper_only and not s.paper_trading:
        return
    try:
        tz = ZoneInfo(s.notify_tz or "America/New_York")
    except Exception:
        tz = ZoneInfo("America/New_York")
    now = datetime.now(tz)
    today = now.date().isoformat()
    ph, pm = _parse_hhmm(s.notify_premarket_hhmm)
    eh, em = _parse_hhmm(s.notify_eod_hhmm)

    if s.notify_premarket and now.hour == ph and now.minute >= pm and _state["premarket_day"] != today:
        _state["premarket_day"] = today
        try:
            await get_operational_briefing_service().run_premarket()
            logger.info("[notify] premarket briefing enviado")
        except Exception:
            logger.exception("[notify] premarket fallo")

    if s.notify_eod and now.hour == eh and now.minute >= em and _state["eod_day"] != today:
        _state["eod_day"] = today
        try:
            await get_operational_briefing_service().run_eod()
            logger.info("[notify] EoD briefing enviado")
        except Exception:
            logger.exception("[notify] EoD fallo")

    if s.notify_intraday:
        try:
            svc = get_operational_briefing_service()
            ctx = svc.peek_context()
            opps = (ctx.get("scanner") or {}).get("candidates") or []
            if opps:
                best = max(opps, key=lambda c: float(c.get("selection_score") or 0))
                sc = float(best.get("selection_score") or 0)
                sym = str(best.get("symbol") or best.get("underlying") or "")
                min_gap = float(getattr(s, "notify_intraday_min_interval_sec", 300.0))
                import time as _time

                gap_ok = _time.time() - float(_state.get("last_intraday_ts") or 0.0) >= min_gap
                if gap_ok and sc >= float(getattr(s, "scanner_min_selection_score", 65)) + 5:
                    fp = f"{sym}:{sc:.1f}"
                    if _state["last_intraday_fp"] != fp:
                        _state["last_intraday_fp"] = fp
                        _state["last_intraday_ts"] = _time.time()
                        direction = _direction_bias(best.get("direction"))
                        method_key = str(best.get("strategy_key") or best.get("method") or best.get("primary_method") or "").strip()
                        method_label = str(best.get("strategy_label") or best.get("method_label") or method_key or "setup técnico").strip()
                        confirmation = best.get("confirmation") if isinstance(best.get("confirmation"), dict) else {}
                        higher_tf = str(confirmation.get("higher_timeframe") or "N/D").strip()
                        higher_dir = _direction_bias(confirmation.get("direction"))
                        higher_conf = float(confirmation.get("confidence_pct") or 0.0)
                        urgency, urgency_reason = _urgency_level(best, sc)
                        await svc.emit_intraday(
                            "Scanner — alta convicción",
                            [
                                "Qué veo:",
                                (
                                    f"{sym} con score {sc:.1f}/100, sesgo {direction}, "
                                    f"método {method_label}."
                                ),
                                (
                                    f"Contexto superior ({higher_tf}): sesgo {higher_dir} "
                                    f"con {higher_conf:.1f}% de confianza."
                                ),
                                f"Nivel de urgencia: {urgency} ({urgency_reason}).",
                                "Qué significa:",
                                _ic_alignment_text(ctx.get('learning_orchestrator') or {}, method_key),
                                "Qué haría ahora:",
                                _strategy_recommendation(best, urgency=urgency),
                                _tactical_plan(best, urgency),
                            ],
                        )
        except Exception:
            logger.debug("[notify] intraday pulse skip", exc_info=True)


async def notification_scheduler_loop() -> None:
    s = global_settings
    poll = max(15, int(getattr(s, "notify_scheduler_poll_sec", 45)))
    logger.info("[notify] scheduler loop poll=%ss", poll)
    while True:
        try:
            await _scheduler_tick()
        except asyncio.CancelledError:
            break
        except Exception:
            logger.exception("[notify] scheduler tick")
        await asyncio.sleep(poll)


def start_notification_scheduler() -> asyncio.Task | None:
    if _state.get("task") and not _state["task"].done():
        return _state["task"]
    task = asyncio.create_task(notification_scheduler_loop(), name="quant-notify-scheduler")
    _state["task"] = task
    return task


def attach_exit_intelligence_bridge() -> None:
    from operations.alert_dispatcher import AlertEvent, get_alert_dispatcher
    from notifications.briefing_service import get_operational_briefing_service

    from config.settings import settings

    dispatcher = get_alert_dispatcher()
    prev = dispatcher._on_alert

    def chain(ev: AlertEvent) -> None:
        if prev:
            try:
                prev(ev)
            except Exception:
                pass
        if not settings.notify_enabled or not settings.notify_exit_intelligence:
            return
        if not _is_trade_exit_event(ev):
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return

        async def _go() -> None:
            try:
                svc = get_operational_briefing_service()
            except Exception:
                return
            sym = ev.symbol or "?"
            meta = ev.metadata or {}
            lines = [
                f"Evento: {ev.title}",
                ev.body.replace("<b>", "").replace("</b>", "").replace("<code>", "").replace("</code>", "")[:500],
                f"Meta: {meta}",
                "Revisar si el cierre siguió plan (TP/time stop) vs reactivo.",
                "Registrar post-mortem en journal para alimentar aprendizaje.",
            ]
            await svc.emit_exit_intelligence(symbol=sym, title_suffix="Análisis de salida", body_lines=lines)

        loop.create_task(_go())

    dispatcher._on_alert = chain  # type: ignore[assignment]
