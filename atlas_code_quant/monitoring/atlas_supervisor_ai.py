"""atlas_supervisor_ai.py — Supervisor AI de ATLAS-Quant.

Extiende AdvancedMonitor con revision autonoma cada 2 minutos:

  1. build_monitor_summary() — snapshot de posiciones + PnL + grecos
  2. ai_review() — lee logs recientes (scanner→signal→vision→submit)
     y evalua 4 reglas de calidad:

     R1. score_trend_declining   — score medio cayendo 3+ ciclos
     R2. full_tier_low_winrate   — FULL tier con winrate <55% ultimas 20 ops
     R3. mtf_misaligned          — MTF <0.70 pero signal_score alto (>0.65)
     R4. visual_low_submit       — visual_conf <0.70 pero orden enviada

  3. Alerta Telegram: "Supervisor: PAUSAR XOP, motivo: Y"
  4. Opcional (auto_force_paper=True): ProductionGuard.force_paper_mode()

Uso::

    supervisor = AtlasSupervisorAI(
        strategy_tracker=tracker,
        guard=ProductionGuard(),
        dispatcher=get_alert_dispatcher(),
        auto_force_paper=False,
    )
    supervisor.start()     # lanza thread de fondo
    supervisor.stop()      # detiene limpiamente

Hook en ATLASQuantCore.setup() — ver docstring de ese metodo.
"""
from __future__ import annotations

import logging
import re
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

logger = logging.getLogger("atlas.supervisor.ai")

# ── Constantes de analisis ─────────────────────────────────────────────────────
_SCORE_WINDOW        = 10    # ciclos para detectar tendencia bajista
_SCORE_DECLINE_MIN   = 3     # minimo de ciclos consecutivos en declive
_WIN_RATE_OPS        = 20    # ultimas N operaciones para winrate check
_WIN_RATE_THRESHOLD  = 0.55  # alerta si winrate < 55%
_MTF_LOW_THRESHOLD   = 0.70  # MTF "desalineado" si < 0.70
_MTF_SIGNAL_HIGH     = 0.65  # signal_score "alto" si > 0.65
_VIS_CONF_THRESHOLD  = 0.70  # visual_conf "baja" si < 0.70
_LOG_READ_LINES      = 600   # cuantas lineas leer del log
_REVIEW_INTERVAL_S   = 120.0 # cada 2 minutos

# ── Regex para parsear log del autonomous_loop ─────────────────────────────────
# Formato: "HH:MM:SS [INFO] ..." producido por RotatingFileHandler
_RE_SCANNER = re.compile(
    r"SCANNER (?P<sym>[A-Z0-9.]+) \| scanner_score=(?P<sc>[0-9.]+) dir=(?P<dir>\w+)"
)
_RE_SIGNAL  = re.compile(
    r"SIGNAL\s+(?P<sym>[A-Z0-9.]+) \| score=(?P<score>[0-9.]+) tier=(?P<tier>\w+)"
    r".*?motif=(?P<motif>[0-9.]+) tin=(?P<tin>[0-9.]+) mtf=(?P<mtf>[0-9.]+)"
    r" regime=(?P<regime>[0-9.]+)"
)
_RE_VISION  = re.compile(
    r"VISION\s+(?P<sym>[A-Z0-9.]+) \| (?P<ok>visual_ok|visual_BLOCKED)"
    r" conf=(?P<conf>[0-9.]+) color=(?P<color>\w+)"
)
_RE_SUBMIT  = re.compile(
    r"SUBMIT\s+(?P<sym>[A-Z0-9.]+) \| id=(?P<id>\S+)"
)
_RE_SKIP    = re.compile(
    r"SKIP\s+(?P<sym>[A-Z0-9.]+) \|"
)
_RE_BLOCKED = re.compile(
    r"BLOCKED\s+(?P<sym>[A-Z0-9.]+) \|"
)


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class CycleEvent:
    """Evento de un simbolo en un ciclo de evaluacion."""
    ts_str:       str   = ""
    symbol:       str   = ""
    scanner_score: float = 0.0
    signal_score:  float = 0.0
    signal_tier:   str   = ""
    motif:         float = 0.5
    tin:           float = 0.5
    mtf:           float = 0.5
    regime:        float = 0.5
    visual_conf:   float = 0.0
    visual_ok:     bool  = False
    visual_color:  str   = ""
    submitted:     bool  = False
    skipped:       bool  = False
    blocked:       bool  = False


@dataclass
class SupervisorAlert:
    """Alerta generada por AtlasSupervisorAI."""
    level:   str   # "WARNING" | "CRITICAL"
    rule:    str   # "score_trend_declining" | "full_tier_low_winrate" | ...
    symbol:  str   # simbolo afectado ("*" = sistema)
    message: str   # mensaje legible para Telegram
    action:  str   # "MONITOR" | "PAUSAR" | "PAPER_MODE"
    data:    dict = field(default_factory=dict)

    def telegram_text(self) -> str:
        icon = "🚨" if self.level == "CRITICAL" else "⚠️"
        action_str = f" → Accion: {self.action}" if self.action != "MONITOR" else ""
        return f"{icon} Supervisor ATLAS | {self.rule}\n{self.message}{action_str}"


# ── Parser de logs ─────────────────────────────────────────────────────────────

def _parse_log_events(log_path: Path, n_lines: int = _LOG_READ_LINES) -> list[CycleEvent]:
    """Lee las ultimas n_lines del log y parsea eventos SCANNER/SIGNAL/VISION/SUBMIT."""
    if not log_path.exists():
        return []

    try:
        with open(log_path, "r", encoding="utf-8", errors="replace") as f:
            lines = f.readlines()
    except Exception as exc:
        logger.debug("parse_log_events: %s", exc)
        return []

    lines = lines[-n_lines:]

    # Estado acumulado por simbolo (ventana deslizante por bloque)
    pending: dict[str, CycleEvent] = {}
    completed: list[CycleEvent] = []

    for line in lines:
        ts_str = line[:8]  # "HH:MM:SS"

        m = _RE_SCANNER.search(line)
        if m:
            sym = m.group("sym")
            ev = CycleEvent(
                ts_str=ts_str,
                symbol=sym,
                scanner_score=float(m.group("sc")),
            )
            pending[sym] = ev
            continue

        m = _RE_SIGNAL.search(line)
        if m:
            sym = m.group("sym")
            ev = pending.get(sym) or CycleEvent(ts_str=ts_str, symbol=sym)
            ev.signal_score = float(m.group("score"))
            ev.signal_tier  = m.group("tier")
            ev.motif        = float(m.group("motif"))
            ev.tin          = float(m.group("tin"))
            ev.mtf          = float(m.group("mtf"))
            ev.regime       = float(m.group("regime"))
            pending[sym] = ev
            continue

        m = _RE_VISION.search(line)
        if m:
            sym = m.group("sym")
            ev = pending.get(sym) or CycleEvent(ts_str=ts_str, symbol=sym)
            ev.visual_conf  = float(m.group("conf"))
            ev.visual_ok    = m.group("ok") == "visual_ok"
            ev.visual_color = m.group("color")
            pending[sym] = ev
            continue

        m = _RE_SUBMIT.search(line)
        if m:
            sym = m.group("sym")
            ev = pending.pop(sym, CycleEvent(ts_str=ts_str, symbol=sym))
            ev.submitted = True
            completed.append(ev)
            continue

        m = _RE_SKIP.search(line)
        if m:
            sym = m.group("sym")
            ev = pending.pop(sym, CycleEvent(ts_str=ts_str, symbol=sym))
            ev.skipped = True
            completed.append(ev)
            continue

        m = _RE_BLOCKED.search(line)
        if m:
            sym = m.group("sym")
            ev = pending.pop(sym, CycleEvent(ts_str=ts_str, symbol=sym))
            ev.blocked = True
            completed.append(ev)
            continue

    # Volcar pendientes (ciclo en progreso)
    for ev in pending.values():
        if ev.signal_score > 0:
            completed.append(ev)

    return completed


# ── Reglas de analisis ─────────────────────────────────────────────────────────

def _check_score_trend_declining(events: list[CycleEvent]) -> list[SupervisorAlert]:
    """R1: score medio cayendo 3+ ciclos consecutivos."""
    alerts: list[SupervisorAlert] = []
    # Agrupar por simbolo
    by_sym: dict[str, list[float]] = defaultdict(list)
    for ev in events:
        if ev.signal_score > 0:
            by_sym[ev.symbol].append(ev.signal_score)

    for sym, scores in by_sym.items():
        if len(scores) < _SCORE_DECLINE_MIN + 1:
            continue
        recent = scores[-(_SCORE_WINDOW):]
        # Contar declives consecutivos al final
        consecutive = 0
        for i in range(len(recent) - 1, 0, -1):
            if recent[i] < recent[i - 1]:
                consecutive += 1
            else:
                break
        if consecutive >= _SCORE_DECLINE_MIN:
            first = round(recent[-consecutive - 1], 3)
            last  = round(recent[-1], 3)
            drop  = round(first - last, 3)
            alerts.append(SupervisorAlert(
                level="WARNING",
                rule="score_trend_declining",
                symbol=sym,
                message=(
                    f"Supervisor: MONITOREAR {sym} — signal_score cayendo "
                    f"{consecutive} ciclos consecutivos ({first}→{last}, -{drop})"
                ),
                action="MONITOR",
                data={"consecutive_decline": consecutive, "from": first, "to": last, "drop": drop},
            ))
    return alerts


def _check_full_tier_low_winrate(
    events: list[CycleEvent],
    monitor_summary: dict | None = None,
) -> list[SupervisorAlert]:
    """R2: FULL tier con winrate efectivo <55% ultimas 20 operaciones.

    Usa PnL del monitor_summary si disponible; si no, usa ratio submit/blocked
    como proxy (submit=exito parcial, blocked=fallo).
    """
    alerts: list[SupervisorAlert] = []
    by_sym: dict[str, list[CycleEvent]] = defaultdict(list)
    for ev in events:
        if ev.signal_tier == "FULL":
            by_sym[ev.symbol].append(ev)

    for sym, sym_events in by_sym.items():
        last_n = sym_events[-_WIN_RATE_OPS:]
        if len(last_n) < 5:
            continue

        # Intentar usar P&L real del monitor_summary
        win_rate: float | None = None
        if monitor_summary:
            groups = monitor_summary.get("strategies") or monitor_summary.get("groups") or []
            for g in groups:
                if str(g.get("underlying") or g.get("symbol") or "") == sym:
                    prob = g.get("win_probability") or g.get("win_rate_pct")
                    if prob is not None:
                        win_rate = float(prob) / 100.0 if float(prob) > 1.0 else float(prob)
                        break

        # Fallback: proxy submit/(submit+blocked)
        if win_rate is None:
            submitted = sum(1 for e in last_n if e.submitted)
            total = sum(1 for e in last_n if e.submitted or e.blocked)
            if total < 3:
                continue
            win_rate = submitted / total

        if win_rate < _WIN_RATE_THRESHOLD:
            alerts.append(SupervisorAlert(
                level="CRITICAL",
                rule="full_tier_low_winrate",
                symbol=sym,
                message=(
                    f"Supervisor: PAUSAR {sym} — FULL tier pero winrate "
                    f"{win_rate*100:.1f}% < {_WIN_RATE_THRESHOLD*100:.0f}% "
                    f"(ultimas {len(last_n)} ops)"
                ),
                action="PAUSAR",
                data={"win_rate": round(win_rate, 3), "n_ops": len(last_n), "tier": "FULL"},
            ))
    return alerts


def _check_mtf_misaligned(events: list[CycleEvent]) -> list[SupervisorAlert]:
    """R3: MTF <0.70 pero signal_score >0.65 — coherencia multi-TF cuestionable."""
    alerts: list[SupervisorAlert] = []
    flagged: dict[str, list[tuple[float, float]]] = defaultdict(list)

    for ev in events:
        if ev.signal_score > _MTF_SIGNAL_HIGH and ev.mtf < _MTF_LOW_THRESHOLD and ev.submitted:
            flagged[ev.symbol].append((ev.signal_score, ev.mtf))

    for sym, cases in flagged.items():
        if len(cases) < 2:
            continue
        avg_mtf   = round(sum(c[1] for c in cases) / len(cases), 3)
        avg_score = round(sum(c[0] for c in cases) / len(cases), 3)
        alerts.append(SupervisorAlert(
            level="WARNING",
            rule="mtf_misaligned_high_signal",
            symbol=sym,
            message=(
                f"Supervisor: MONITOREAR {sym} — signal_score alto ({avg_score}) "
                f"con MTF bajo ({avg_mtf} < {_MTF_LOW_THRESHOLD}) en {len(cases)} ops. "
                f"Coherencia multi-TF cuestionable."
            ),
            action="MONITOR",
            data={"avg_signal_score": avg_score, "avg_mtf": avg_mtf, "cases": len(cases)},
        ))
    return alerts


def _check_visual_low_submit(events: list[CycleEvent]) -> list[SupervisorAlert]:
    """R4: visual_conf <0.70 pero orden enviada — validacion visual debil."""
    alerts: list[SupervisorAlert] = []
    flagged: dict[str, list[float]] = defaultdict(list)

    for ev in events:
        if ev.submitted and ev.visual_conf > 0 and ev.visual_conf < _VIS_CONF_THRESHOLD:
            flagged[ev.symbol].append(ev.visual_conf)

    for sym, confs in flagged.items():
        if len(confs) < 2:
            continue
        avg_conf = round(sum(confs) / len(confs), 3)
        alerts.append(SupervisorAlert(
            level="WARNING",
            rule="visual_conf_low_submit",
            symbol=sym,
            message=(
                f"Supervisor: MONITOREAR {sym} — ordenes enviadas con visual_conf "
                f"baja ({avg_conf} < {_VIS_CONF_THRESHOLD}) en {len(confs)} ocasiones. "
                f"Considera subir --min-visual-conf."
            ),
            action="MONITOR",
            data={"avg_visual_conf": avg_conf, "cases": len(confs)},
        ))
    return alerts


# ── Clase principal ────────────────────────────────────────────────────────────

class AtlasSupervisorAI:
    """Supervisor AI que revisa ATLAS-Quant cada 2 minutos.

    Uso::

        supervisor = AtlasSupervisorAI(
            strategy_tracker=_STRATEGY_TRACKER,
            guard=ProductionGuard(),
            dispatcher=get_alert_dispatcher(),
            log_file=Path("logs/atlas_live_loop.log"),
            auto_force_paper=False,   # True = accion automatica critica
        )
        supervisor.start()
    """

    def __init__(
        self,
        strategy_tracker=None,   # StrategyTracker — para build_summary()
        guard=None,               # ProductionGuard — para force_paper_mode()
        dispatcher=None,          # AlertDispatcher — para alertas Telegram
        telegram_alerts=None,     # TelegramAlerts (alternativa al dispatcher)
        log_file: Path | None = None,
        review_interval_sec: float = _REVIEW_INTERVAL_S,
        auto_force_paper: bool = False,
        account_scope: str = "paper",
    ) -> None:
        self._tracker    = strategy_tracker
        self._guard      = guard
        self._dispatcher = dispatcher
        self._telegram   = telegram_alerts
        self._log_file   = log_file or _default_log_path()
        self._interval   = review_interval_sec
        self._auto_paper = auto_force_paper
        self._scope      = account_scope

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._cycle_count = 0
        self._last_alerts: list[SupervisorAlert] = []
        self._alert_history: list[SupervisorAlert] = []   # max 200
        self._alert_cooldown: dict[str, float] = {}       # rule+sym → last_sent ts

        logger.info(
            "AtlasSupervisorAI init | interval=%.0fs auto_paper=%s log=%s",
            self._interval, self._auto_paper, self._log_file,
        )

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            logger.warning("AtlasSupervisorAI ya esta corriendo")
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="AtlasSupervisorAI",
            daemon=True,
        )
        self._thread.start()
        logger.info("AtlasSupervisorAI iniciado (daemon thread)")

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=10)
        logger.info("AtlasSupervisorAI detenido tras %d ciclos", self._cycle_count)

    def _run(self) -> None:
        logger.info("AtlasSupervisorAI loop arrancado")
        while not self._stop_event.is_set():
            t0 = time.monotonic()
            try:
                alerts = self.ai_review()
                elapsed = round((time.monotonic() - t0) * 1000)
                if alerts:
                    logger.info(
                        "SupervisorAI ciclo %d: %d alertas en %dms",
                        self._cycle_count, len(alerts), elapsed,
                    )
                else:
                    logger.debug(
                        "SupervisorAI ciclo %d: OK en %dms", self._cycle_count, elapsed
                    )
            except Exception as exc:
                logger.exception("AtlasSupervisorAI._run error: %s", exc)
            self._stop_event.wait(self._interval)

    # ── Revision AI principal ─────────────────────────────────────────────────

    def ai_review(self) -> list[SupervisorAlert]:
        """Revisa estado del sistema y devuelve alertas generadas.

        Pasos:
          1. build_monitor_summary() — snapshot de posiciones/PnL
          2. Parsear logs recientes
          3. Evaluar 4 reglas
          4. Enviar alertas Telegram
          5. Si auto_force_paper y alerta CRITICAL→PAPER_MODE: activar
        """
        self._cycle_count += 1

        # ── 1. Monitor summary ────────────────────────────────────────────────
        monitor_summary: dict | None = None
        if self._tracker is not None:
            try:
                monitor_summary = self._tracker.build_summary(account_scope=self._scope)
                logger.debug(
                    "SupervisorAI monitor_summary: %d estrategias",
                    len(monitor_summary.get("strategies") or []),
                )
            except Exception as exc:
                logger.warning("SupervisorAI build_summary error: %s", exc)

        # ── 2. Parsear logs ───────────────────────────────────────────────────
        events = _parse_log_events(self._log_file, n_lines=_LOG_READ_LINES)
        logger.debug("SupervisorAI: %d eventos parseados del log", len(events))

        # ── 3. Evaluar reglas ─────────────────────────────────────────────────
        alerts: list[SupervisorAlert] = []
        alerts += _check_score_trend_declining(events)
        alerts += _check_full_tier_low_winrate(events, monitor_summary)
        alerts += _check_mtf_misaligned(events)
        alerts += _check_visual_low_submit(events)

        self._last_alerts = alerts

        # ── 4. Despachar alertas ──────────────────────────────────────────────
        for alert in alerts:
            self._dispatch_alert(alert)

        # ── 5. Auto force_paper en alertas criticas ───────────────────────────
        if self._auto_paper and self._guard is not None:
            critical_paper = [a for a in alerts if a.action == "PAPER_MODE"]
            if critical_paper:
                reasons = "; ".join(a.rule for a in critical_paper)
                logger.critical(
                    "AtlasSupervisorAI: force_paper_mode activado — %s", reasons
                )
                try:
                    self._guard.force_paper_mode()
                except Exception as exc:
                    logger.error("force_paper_mode fallido: %s", exc)

        # Historial (max 200)
        self._alert_history.extend(alerts)
        if len(self._alert_history) > 200:
            self._alert_history = self._alert_history[-200:]

        return alerts

    # ── Despacho de alertas ───────────────────────────────────────────────────

    def _dispatch_alert(self, alert: SupervisorAlert) -> None:
        """Envia alerta por Telegram con rate-limiting (una vez por 10 min por regla+sym)."""
        key = f"{alert.rule}:{alert.symbol}"
        now = time.time()
        cooldown = 600.0 if alert.level == "WARNING" else 300.0
        last = self._alert_cooldown.get(key, 0.0)
        if now - last < cooldown:
            logger.debug("SupervisorAI alert cooldown: %s", key)
            return

        self._alert_cooldown[key] = now
        text = alert.telegram_text()
        logger.warning("SupervisorAI ALERT [%s] %s", alert.level, alert.message)

        # Via AlertDispatcher (preferido)
        if self._dispatcher is not None:
            try:
                import asyncio
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    asyncio.ensure_future(
                        self._dispatcher.system_error(
                            component="supervisor_ai",
                            error=alert.message,
                            critical=(alert.level == "CRITICAL"),
                        )
                    )
                else:
                    loop.run_until_complete(
                        self._dispatcher.system_error(
                            component="supervisor_ai",
                            error=alert.message,
                            critical=(alert.level == "CRITICAL"),
                        )
                    )
                return
            except Exception as exc:
                logger.debug("dispatcher send error: %s", exc)

        # Via TelegramAlerts (directo)
        if self._telegram is not None:
            try:
                self._telegram.send(text, urgent=(alert.level == "CRITICAL"))
                return
            except Exception as exc:
                logger.debug("telegram send error: %s", exc)

        # Fallback: envio directo via env vars
        _send_telegram_direct(text)

    # ── Estado / introspeccion ────────────────────────────────────────────────

    def status(self) -> dict[str, Any]:
        return {
            "running": bool(self._thread and self._thread.is_alive()),
            "cycle_count": self._cycle_count,
            "review_interval_sec": self._interval,
            "auto_force_paper": self._auto_paper,
            "log_file": str(self._log_file),
            "log_exists": self._log_file.exists(),
            "last_alerts": [
                {"level": a.level, "rule": a.rule, "symbol": a.symbol, "action": a.action}
                for a in self._last_alerts
            ],
            "alert_history_count": len(self._alert_history),
        }

    def last_review_summary(self) -> dict[str, Any]:
        """Resumen de la ultima revision para API/dashboard."""
        by_rule: dict[str, list[str]] = defaultdict(list)
        for a in self._last_alerts:
            by_rule[a.rule].append(a.symbol)
        return {
            "cycle": self._cycle_count,
            "alerts_count": len(self._last_alerts),
            "by_rule": dict(by_rule),
            "critical_count": sum(1 for a in self._last_alerts if a.level == "CRITICAL"),
            "messages": [a.message for a in self._last_alerts],
        }


# ── Helpers ────────────────────────────────────────────────────────────────────

def _default_log_path() -> Path:
    return Path(__file__).resolve().parents[2] / "logs" / "atlas_live_loop.log"


def _send_telegram_direct(text: str) -> None:
    """Envio directo a Telegram sin dependencias — usa env vars."""
    import os
    from urllib.request import Request, urlopen
    from urllib.error import URLError
    token   = os.getenv("TELEGRAM_BOT_TOKEN", "")
    chat_id = os.getenv("TELEGRAM_ADMIN_CHAT_ID", "")
    if not token or not chat_id:
        logger.debug("TELEGRAM_BOT_TOKEN o TELEGRAM_ADMIN_CHAT_ID no configurados")
        return
    import json as _json
    url  = f"https://api.telegram.org/bot{token}/sendMessage"
    body = _json.dumps({"chat_id": chat_id, "text": text, "parse_mode": "HTML"}).encode()
    try:
        req = Request(url, data=body, headers={"Content-Type": "application/json"})
        with urlopen(req, timeout=8) as resp:
            logger.debug("Telegram supervisor enviado: %s", resp.status)
    except URLError as exc:
        logger.warning("Telegram directo fallido: %s", exc)
    except Exception as exc:
        logger.debug("Telegram directo error: %s", exc)


# ── Singleton global para uso en API ──────────────────────────────────────────

_SUPERVISOR_INSTANCE: AtlasSupervisorAI | None = None


def get_supervisor(
    strategy_tracker=None,
    guard=None,
    dispatcher=None,
    telegram_alerts=None,
    auto_force_paper: bool = False,
    **kwargs,
) -> AtlasSupervisorAI:
    """Retorna la instancia singleton del supervisor (creandola si no existe)."""
    global _SUPERVISOR_INSTANCE
    if _SUPERVISOR_INSTANCE is None:
        _SUPERVISOR_INSTANCE = AtlasSupervisorAI(
            strategy_tracker=strategy_tracker,
            guard=guard,
            dispatcher=dispatcher,
            telegram_alerts=telegram_alerts,
            auto_force_paper=auto_force_paper,
            **kwargs,
        )
    return _SUPERVISOR_INSTANCE
