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
                        await svc.emit_intraday(
                            "Scanner — alta convicción",
                            [
                                f"Candidato destacado {sym} score={sc:.1f}",
                                "Revisar alineación con IC y contexto antes de tamaño.",
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
        if ev.category != "trade" or "SALIDA" not in (ev.title or ""):
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
