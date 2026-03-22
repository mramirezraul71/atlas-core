# ATLAS-Quant — Módulo 9B: Telegram Alerts
"""Alertas Telegram en tiempo real para eventos críticos del robot.

Usa python-telegram-bot (async) con cola de mensajes para no bloquear
el loop de trading. Fallback a log si el token no está configurado.

Mensajes implementados:
  - Orden ejecutada / bloqueada
  - Drawdown / Kelly reducido / Circuit breaker
  - Emergency stop
  - Sharpe y métricas de sesión
  - Self-healing (reparación completada / fallida)
  - Activación de modo LIVE
  - Daily summary (al cierre del mercado)
"""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from typing import Optional

logger = logging.getLogger("atlas.production.telegram")

# ── Configuración ─────────────────────────────────────────────────────────────
_TOKEN   = os.getenv("TELEGRAM_BOT_TOKEN", "").strip()
_CHAT_ID = os.getenv("TELEGRAM_ADMIN_CHAT_ID", os.getenv("TELEGRAM_CHAT_ID", "")).strip()
_ENABLED = bool(_TOKEN and _CHAT_ID)

# ── Imports opcionales ────────────────────────────────────────────────────────
try:
    import requests as _requests
    _REQ_OK = True
except ImportError:
    _REQ_OK = False
    logger.warning("requests no disponible — Telegram deshabilitado")


class TelegramAlerts:
    """Cliente de alertas Telegram no bloqueante para ATLAS-Quant.

    Uso::

        tg = TelegramAlerts()
        tg.start()
        tg.alert_order_executed("AAPL", 238.45, 10, 1.2)
        tg.alert_drawdown(4.8)
        tg.stop()

    También expone método estático para uso desde otros módulos::

        TelegramAlerts.send_static("Mensaje urgente")
    """

    # ── Instancia global (singleton ligero) ───────────────────────────────────
    _instance: Optional["TelegramAlerts"] = None

    def __init__(
        self,
        token: str = _TOKEN,
        chat_id: str = _CHAT_ID,
        voice=None,
    ) -> None:
        self.token   = token
        self.chat_id = chat_id
        self.voice   = voice
        self.enabled = bool(token and chat_id and _REQ_OK)

        self._queue: queue.Queue[Optional[str]] = queue.Queue(maxsize=50)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._sent_count = 0
        self._fail_count = 0

        TelegramAlerts._instance = self

        if not self.enabled:
            logger.warning(
                "TelegramAlerts deshabilitado — "
                "configura TELEGRAM_BOT_TOKEN y TELEGRAM_CHAT_ID"
            )

    # ── Ciclo de vida ─────────────────────────────────────────────────────────

    def start(self) -> None:
        self._running = True
        self._thread  = threading.Thread(
            target=self._worker, daemon=True, name="atlas-telegram"
        )
        self._thread.start()
        logger.info("TelegramAlerts iniciado (chat_id=%s enabled=%s)",
                    self.chat_id[:6] + "…" if self.chat_id else "N/A", self.enabled)

    def stop(self) -> None:
        self._running = False
        self._queue.put(None)
        if self._thread:
            self._thread.join(timeout=5.0)

    def _worker(self) -> None:
        while self._running:
            item = self._queue.get()
            if item is None:
                break
            self._send_now(item)

    def _send_now(self, text: str) -> bool:
        if not self.enabled:
            logger.info("TG[deshabilitado]: %s", text[:80])
            return False
        url = f"https://api.telegram.org/bot{self.token}/sendMessage"
        try:
            resp = _requests.post(url, json={
                "chat_id":    self.chat_id,
                "text":       text,
                "parse_mode": "HTML",
            }, timeout=8)
            if resp.ok:
                self._sent_count += 1
                return True
            else:
                self._fail_count += 1
                logger.warning("TG error %d: %s", resp.status_code, resp.text[:100])
                return False
        except Exception as exc:
            self._fail_count += 1
            logger.debug("TG send error: %s", exc)
            return False

    # ── API pública ───────────────────────────────────────────────────────────

    def send(self, text: str, urgent: bool = False) -> None:
        """Encola mensaje. Prefija 🚨 si urgente."""
        prefix = "🚨 " if urgent else ""
        full   = f"{prefix}<b>ATLAS-Quant</b>\n{text}"
        try:
            self._queue.put_nowait(full)
        except queue.Full:
            logger.debug("TG queue llena — mensaje descartado")

    # ── Método estático (para uso desde otros módulos sin instancia) ──────────

    @classmethod
    def send_static(cls, text: str, urgent: bool = False) -> None:
        if cls._instance is not None and cls._instance._running:
            cls._instance.send(text, urgent=urgent)
        elif _ENABLED and _REQ_OK:
            # Envío síncrono directo si no hay instancia activa
            try:
                prefix = "🚨 " if urgent else ""
                _requests.post(
                    f"https://api.telegram.org/bot{_TOKEN}/sendMessage",
                    json={"chat_id": _CHAT_ID,
                          "text": f"{prefix}<b>ATLAS-Quant</b>\n{text}",
                          "parse_mode": "HTML"},
                    timeout=5
                )
            except Exception:
                pass

    # ── Alertas semánticas ────────────────────────────────────────────────────

    def alert_live_activated(self) -> None:
        self.send(
            "✅ <b>Modo LIVE activado</b>\n"
            "Trading real con dinero real en progreso.\n"
            f"<i>{_ts()}</i>",
            urgent=True
        )
        if self.voice:
            self.voice.announce_live_activated()

    def alert_order_executed(
        self,
        symbol: str,
        price: float,
        qty: int,
        kelly_pct: float,
        side: str = "BUY",
        order_id: str = "",
    ) -> None:
        emoji = "🟢" if side.upper() == "BUY" else "🔴"
        self.send(
            f"{emoji} <b>Orden ejecutada</b>\n"
            f"Símbolo : <code>{symbol}</code>\n"
            f"Lado    : {side}\n"
            f"Precio  : ${price:,.2f}\n"
            f"Cantidad: {qty} acciones\n"
            f"Kelly   : {kelly_pct:.1f}%\n"
            f"ID      : {order_id or 'paper'}\n"
            f"<i>{_ts()}</i>"
        )

    def alert_order_blocked(self, symbol: str, reason: str) -> None:
        self.send(
            f"⛔ <b>Orden bloqueada</b>\n"
            f"Símbolo: <code>{symbol}</code>\n"
            f"Razón  : {reason}\n"
            f"<i>{_ts()}</i>"
        )

    def alert_drawdown(self, pct: float, kelly_fraction: float | None = None) -> None:
        urgent = pct > 6.0
        lines  = [
            f"📉 <b>Drawdown {pct:.1f}%</b>",
        ]
        if kelly_fraction is not None:
            lines.append(f"Kelly reducido a {kelly_fraction*100:.0f}%")
        lines.append(f"<i>{_ts()}</i>")
        self.send("\n".join(lines), urgent=urgent)
        if self.voice:
            self.voice.announce_drawdown(pct, critical=urgent)

    def alert_circuit_breaker(self, pause_min: int = 30) -> None:
        self.send(
            f"⚡ <b>Circuit Breaker ACTIVADO</b>\n"
            f"Pausa de {pause_min} minutos.\n"
            f"<i>{_ts()}</i>",
            urgent=True
        )
        if self.voice:
            self.voice.announce_circuit_breaker(pause_min)

    def alert_emergency_stop(self) -> None:
        self.send(
            "🛑 <b>EMERGENCY STOP</b>\n"
            "Todas las operaciones suspendidas inmediatamente.\n"
            f"<i>{_ts()}</i>",
            urgent=True
        )
        if self.voice:
            self.voice.announce_emergency_stop()

    def alert_sharpe(self, sharpe: float, drawdown: float, equity: float) -> None:
        self.send(
            f"📊 <b>Métricas de sesión</b>\n"
            f"Sharpe    : {sharpe:.2f}\n"
            f"Drawdown  : {drawdown:.1f}%\n"
            f"Equity    : ${equity:,.2f}\n"
            f"<i>{_ts()}</i>"
        )

    def alert_repair_start(self, subsystem: str, eta_s: int) -> None:
        self.send(
            f"🔧 <b>Reparando {subsystem}</b>\n"
            f"ETA: ~{eta_s}s\n"
            f"<i>{_ts()}</i>"
        )
        if self.voice:
            self.voice.announce_repair(subsystem, eta_s)

    def alert_repair_done(self, subsystem: str, success: bool) -> None:
        if success:
            self.send(f"✅ <b>{subsystem}</b> reparado correctamente. <i>{_ts()}</i>")
            if self.voice:
                self.voice.announce_repair_done(subsystem)
        else:
            self.send(
                f"❌ <b>Reparación fallida: {subsystem}</b>\n"
                "Intervención manual requerida.\n"
                f"<i>{_ts()}</i>",
                urgent=True
            )
            if self.voice:
                self.voice.announce_repair_failed(subsystem)

    def alert_daily_summary(
        self,
        trades: int,
        pnl: float,
        win_rate: float,
        sharpe: float,
    ) -> None:
        emoji = "🟢" if pnl >= 0 else "🔴"
        self.send(
            f"{emoji} <b>Resumen del día</b>\n"
            f"Operaciones: {trades}\n"
            f"PnL diario : ${pnl:+,.2f}\n"
            f"Win rate   : {win_rate:.1f}%\n"
            f"Sharpe     : {sharpe:.2f}\n"
            f"<i>{_ts()}</i>"
        )

    # ── Callback compatible con SignalExecutor ────────────────────────────────

    def make_execution_callback(self):
        """Retorna función callback para llamar desde SignalExecutor tras ejecución."""
        def on_execution(result) -> None:
            if result.status in ("submitted", "simulated"):
                # Estimar Kelly % (qty * price / equity approximado)
                kelly_pct = 0.0
                try:
                    if hasattr(result, "fill_price") and result.quantity > 0:
                        kelly_pct = result.fill_price * result.quantity / 100_000 * 100
                except Exception:
                    pass
                self.alert_order_executed(
                    symbol    = result.signal_symbol,
                    price     = result.fill_price,
                    qty       = result.quantity,
                    kelly_pct = kelly_pct,
                    side      = result.signal_type,
                    order_id  = result.order_id,
                )
            elif result.status == "blocked":
                self.alert_order_blocked(result.signal_symbol, result.error)
        return on_execution

    def stats(self) -> dict:
        return {
            "enabled":    self.enabled,
            "sent":       self._sent_count,
            "failed":     self._fail_count,
            "queue_size": self._queue.qsize(),
        }


def _ts() -> str:
    import datetime
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
