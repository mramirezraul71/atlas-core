"""Atlas Code-Quant — Dispatcher de alertas operacionales.

Envía notificaciones en tiempo real a Telegram y WhatsApp cuando:
  - Se genera una señal de trading (BUY/SELL)
  - Se ejecuta un trade (entrada/salida)
  - Drawdown crítico superado (>20%)
  - Circuit breaker activado
  - Cámara obstruida o feed visual degradado (Grok criterio)
  - Retraining del modelo RL completado
  - Error crítico del sistema

Reutiliza TelegramBridge de Atlas Core (modules/humanoid/comms/)
y WhatsApp via ATLAS HTTP API (/api/whatsapp/send).

Grok/xAI criterio: "Monitoreo: Telegram + alertas de cámara obstruida."

Uso::
    dispatcher = get_alert_dispatcher()
    await dispatcher.signal(symbol="BTC/USDT", action="BUY", price=65000, confidence=0.78)
    await dispatcher.trade_executed(symbol="BTC/USDT", side="long", entry=65000, size=0.1)
    await dispatcher.drawdown_critical(drawdown_pct=22.5)
    await dispatcher.camera_obstruction(reason="low_brightness")
"""
from __future__ import annotations

import asyncio
import logging
import os
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable

logger = logging.getLogger("quant.operations.alerts")

# ── Nivel de alerta ───────────────────────────────────────────────────────────
LEVEL_INFO     = "INFO"
LEVEL_WARNING  = "WARNING"
LEVEL_CRITICAL = "CRITICAL"

# ── Rate limit de alertas (evita spam) ───────────────────────────────────────
_ALERT_COOLDOWN: dict[str, float] = {}
_DEFAULT_COOLDOWN_S = 60.0   # 1 minuto entre alertas del mismo tipo
_CRITICAL_COOLDOWN_S = 30.0  # Críticos se repiten antes


@dataclass
class AlertEvent:
    """Evento de alerta a despachar."""
    level: str
    category: str       # signal | trade | drawdown | circuit_breaker | camera | retraining | error
    title: str
    body: str
    symbol: str = ""
    timestamp: datetime = field(default_factory=lambda: datetime.now(tz=timezone.utc))
    metadata: dict = field(default_factory=dict)
    body_plain: str | None = None  # WhatsApp u otros canales sin HTML; si None se usa body sin tags

    def format_telegram(self) -> str:
        icon = {"INFO": "📊", "WARNING": "⚠️", "CRITICAL": "🚨"}.get(self.level, "ℹ️")
        ts = self.timestamp.strftime("%H:%M:%S UTC")
        lines = [
            f"{icon} <b>[ATLAS Quant] {self.title}</b>",
            f"<code>{ts}</code>",
            "",
            self.body,
        ]
        if self.symbol:
            lines.insert(2, f"Símbolo: <code>{self.symbol}</code>")
        if self.metadata:
            meta_str = " | ".join(f"{k}={v}" for k, v in self.metadata.items() if v is not None)
            if meta_str:
                lines.append(f"\n<i>{meta_str}</i>")
        return "\n".join(lines)

    def format_whatsapp(self) -> str:
        icon = {"INFO": "📊", "WARNING": "⚠️", "CRITICAL": "🚨"}.get(self.level, "ℹ️")
        ts = self.timestamp.strftime("%H:%M UTC")
        raw = (self.body_plain if self.body_plain is not None else self.body).strip()
        if self.body_plain is None and "<" in raw and ">" in raw:
            import re
            raw = re.sub(r"<[^>]+>", "", raw)
            raw = raw.replace("&nbsp;", " ").strip()
        lines = [f"{icon} *[ATLAS Quant] {self.title}*", f"_{ts}_", "", raw]
        if self.symbol:
            lines.insert(2, f"Símbolo: {self.symbol}")
        return "\n".join(lines)


class AlertDispatcher:
    """Dispatcher central de alertas del sistema de trading.

    Args:
        telegram_enabled: Enviar alertas por Telegram.
        whatsapp_enabled: Enviar alertas por WhatsApp (via ATLAS API).
        atlas_api_url: URL del ATLAS HTTP API (para WhatsApp proxy).
        min_level: Nivel mínimo de alerta a despachar ("INFO", "WARNING", "CRITICAL").
        cooldown_s: Segundos mínimos entre alertas del mismo tipo.
        on_alert: Callback opcional llamado en cada alerta (para logging/dashboard).
    """

    def __init__(
        self,
        telegram_enabled: bool = True,
        whatsapp_enabled: bool = True,
        atlas_api_url: str = "http://127.0.0.1:8791",
        min_level: str = LEVEL_INFO,
        cooldown_s: float = _DEFAULT_COOLDOWN_S,
        on_alert: Callable | None = None,
    ) -> None:
        self.telegram_enabled  = telegram_enabled and bool(os.getenv("TELEGRAM_BOT_TOKEN") or os.getenv("TELEGRAM_TOKEN"))
        self.whatsapp_enabled  = whatsapp_enabled and os.getenv("WHATSAPP_ENABLED", "").lower() not in ("", "false", "0", "no")
        self.atlas_api_url     = atlas_api_url.rstrip("/")
        self.min_level         = min_level
        self.cooldown_s        = cooldown_s
        self._on_alert         = on_alert
        self._sent_count       = 0
        self._error_count      = 0
        self._last_alerts: dict[str, float] = {}
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=200)
        self._worker_task: asyncio.Task | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    async def start(self) -> None:
        """Inicia el worker de despacho en background."""
        if self._worker_task is None or self._worker_task.done():
            self._worker_task = asyncio.create_task(self._dispatch_worker())
            logger.info("[AlertDispatcher] Iniciado — telegram=%s whatsapp=%s",
                        self.telegram_enabled, self.whatsapp_enabled)

    async def stop(self) -> None:
        if self._worker_task and not self._worker_task.done():
            self._worker_task.cancel()

    # ── API de eventos de trading ─────────────────────────────────────────────

    async def signal(
        self,
        symbol: str,
        action: str,
        price: float,
        confidence: float,
        strategy: str = "",
        atr: float | None = None,
    ) -> None:
        """Alerta de señal generada por el sistema."""
        icon = "🟢" if action == "BUY" else "🔴"
        body_lines = [
            f"{icon} <b>{action}</b> @ ${price:,.4f}",
            f"Confianza: {confidence:.1%}",
        ]
        if strategy:
            body_lines.append(f"Estrategia: {strategy}")
        if atr:
            sl = price - 1.5 * atr if action == "BUY" else price + 1.5 * atr
            tp = price + 3.0 * atr if action == "BUY" else price - 3.0 * atr
            body_lines.append(f"SL: ${sl:,.4f} | TP: ${tp:,.4f}")

        await self._enqueue(AlertEvent(
            level=LEVEL_INFO, category="signal",
            title=f"Señal {action} — {symbol}",
            body="\n".join(body_lines),
            symbol=symbol,
            metadata={"conf": f"{confidence:.2f}", "strat": strategy},
        ))

    async def trade_executed(
        self,
        symbol: str,
        side: str,
        entry_price: float | None = None,
        exit_price: float | None = None,
        size: float = 0.0,
        pnl: float | None = None,
        log_return: float | None = None,
        reason: str = "",
    ) -> None:
        """Alerta de trade ejecutado (apertura o cierre)."""
        is_open = entry_price is not None and exit_price is None
        action_str = f"ENTRADA {side.upper()}" if is_open else "SALIDA"
        price_str = f"${entry_price:,.4f}" if is_open else f"${exit_price:,.4f}"

        body_lines = [f"💼 {action_str} @ {price_str}"]
        if size:
            body_lines.append(f"Tamaño: {size:.6f}")
        if pnl is not None:
            pnl_icon = "✅" if pnl >= 0 else "❌"
            body_lines.append(f"{pnl_icon} PnL: ${pnl:+.4f}")
        if log_return is not None:
            body_lines.append(f"Log-return: {log_return:+.4f}")
        if reason:
            body_lines.append(f"Razón: {reason}")

        level = LEVEL_INFO if (pnl is None or pnl >= 0) else LEVEL_WARNING
        await self._enqueue(AlertEvent(
            level=level, category="trade",
            title=f"Trade {action_str} — {symbol}",
            body="\n".join(body_lines),
            symbol=symbol,
            metadata={
                "pnl": f"{pnl:+.4f}" if pnl is not None else None,
                "trade_stage": "open" if is_open else "exit",
                "is_exit": not is_open,
                "side": side,
                "reason": reason or None,
                "entry_price": entry_price,
                "exit_price": exit_price,
            },
        ))

    async def drawdown_critical(
        self,
        drawdown_pct: float,
        equity: float | None = None,
        circuit_breaker: bool = False,
    ) -> None:
        """Alerta de drawdown crítico o circuit breaker activado."""
        title = "Circuit Breaker ACTIVADO" if circuit_breaker else f"Drawdown crítico: {drawdown_pct:.1f}%"
        body_lines = [
            f"⛔ Drawdown actual: <b>{drawdown_pct:.2f}%</b>",
            "Trading PAUSADO automáticamente" if circuit_breaker else f"Límite: 20%",
        ]
        if equity:
            body_lines.append(f"Equity actual: ${equity:,.2f}")
        body_lines.append("Acción requerida: revisar posiciones en dashboard")

        await self._enqueue(AlertEvent(
            level=LEVEL_CRITICAL, category="circuit_breaker" if circuit_breaker else "drawdown",
            title=title, body="\n".join(body_lines),
            metadata={"dd_pct": f"{drawdown_pct:.2f}", "cb": str(circuit_breaker)},
        ), cooldown_s=_CRITICAL_COOLDOWN_S)

    async def camera_obstruction(
        self,
        reason: str = "",
        brightness: float | None = None,
        confidence: float | None = None,
    ) -> None:
        """Alerta de cámara obstruida o degradada — trading entra en modo seguro.
        Grok criterio: 'Si el robot detecta pantalla oscura o baja calidad → modo seguro.'
        """
        body_lines = ["📷 Feed visual DEGRADADO — modo seguro activado"]
        if reason:
            body_lines.append(f"Causa: {reason}")
        if brightness is not None:
            body_lines.append(f"Brillo detectado: {brightness:.1f}/255")
        if confidence is not None:
            body_lines.append(f"Confianza OCR: {confidence:.1%}")
        body_lines.append("Trading visual suspendido hasta restauración")

        await self._enqueue(AlertEvent(
            level=LEVEL_WARNING, category="camera",
            title="Cámara obstruida — modo seguro",
            body="\n".join(body_lines),
            metadata={"reason": reason, "brightness": brightness},
        ), cooldown_s=120.0)

    async def retraining_completed(
        self,
        symbol: str,
        sharpe: float,
        geometric_return_pct: float,
        model_path: str = "",
        elapsed_s: float = 0.0,
    ) -> None:
        """Alerta de retraining RL completado."""
        icon = "✅" if sharpe > 1.0 else "⚠️"
        body_lines = [
            f"{icon} Modelo PPO reentrenado para {symbol}",
            f"Sharpe test: <b>{sharpe:.3f}</b>",
            f"Retorno geométrico: <b>{geometric_return_pct:+.2f}%</b>",
        ]
        if elapsed_s:
            body_lines.append(f"Tiempo: {elapsed_s:.0f}s")
        if model_path:
            body_lines.append(f"Modelo: <code>{model_path.split('/')[-1]}</code>")

        await self._enqueue(AlertEvent(
            level=LEVEL_INFO, category="retraining",
            title=f"Retraining completado — {symbol}",
            body="\n".join(body_lines),
            symbol=symbol,
            metadata={"sharpe": f"{sharpe:.3f}", "ret_pct": f"{geometric_return_pct:+.2f}"},
        ))

    async def system_error(self, component: str, error: str, critical: bool = False) -> None:
        """Alerta de error crítico del sistema."""
        level = LEVEL_CRITICAL if critical else LEVEL_WARNING
        await self._enqueue(AlertEvent(
            level=level, category="error",
            title=f"Error en {component}",
            body=f"🔧 <code>{error[:300]}</code>",
            metadata={"component": component},
        ), cooldown_s=_CRITICAL_COOLDOWN_S if critical else _DEFAULT_COOLDOWN_S)

    def status(self) -> dict:
        return {
            "telegram_enabled": self.telegram_enabled,
            "whatsapp_enabled": self.whatsapp_enabled,
            "sent_count":       self._sent_count,
            "error_count":      self._error_count,
            "queue_size":       self._queue.qsize(),
            "worker_alive":     bool(self._worker_task and not self._worker_task.done()),
        }

    # ── Worker async ──────────────────────────────────────────────────────────

    async def _enqueue(self, event: AlertEvent, cooldown_s: float | None = None) -> None:
        """Encola una alerta respetando rate limit por categoría."""
        lvl_order = {LEVEL_INFO: 0, LEVEL_WARNING: 1, LEVEL_CRITICAL: 2}
        if lvl_order.get(event.level, 0) < lvl_order.get(self.min_level, 0):
            return

        key = f"{event.category}:{event.symbol}"
        now = time.time()
        cd = cooldown_s if cooldown_s is not None else self.cooldown_s
        last = self._last_alerts.get(key, 0.0)
        if now - last < cd:
            logger.debug("[Alerts] Cooldown activo para %s (%.0fs restantes)", key, cd - (now - last))
            return

        self._last_alerts[key] = now
        if self._on_alert:
            try:
                self._on_alert(event)
            except Exception:
                pass

        if not self._queue.full():
            await self._queue.put(event)
        else:
            logger.warning("[Alerts] Cola llena — alerta descartada: %s", event.title)

    async def _dispatch_worker(self) -> None:
        """Worker que procesa la cola y despacha por todos los canales."""
        logger.info("[AlertDispatcher] Worker iniciado")
        while True:
            try:
                event: AlertEvent = await asyncio.wait_for(self._queue.get(), timeout=5.0)
                await self._send_all(event)
                self._queue.task_done()
            except asyncio.TimeoutError:
                continue
            except asyncio.CancelledError:
                break
            except Exception as exc:
                logger.error("[AlertDispatcher] Error en worker: %s", exc)
                self._error_count += 1

    async def _send_all(self, event: AlertEvent) -> None:
        """Despacha el evento a todos los canales habilitados."""
        tasks = []
        if self.telegram_enabled:
            tasks.append(self._send_telegram(event))
        if self.whatsapp_enabled:
            tasks.append(self._send_whatsapp(event))

        if tasks:
            results = await asyncio.gather(*tasks, return_exceptions=True)
            ok = sum(1 for r in results if r is True)
            if ok:
                self._sent_count += 1
                logger.info("[Alerts] ✓ %s → %s", event.category, event.title)
            else:
                self._error_count += 1

    async def _send_telegram(self, event: AlertEvent) -> bool:
        """Envía alerta por Telegram usando TelegramBridge de Atlas Core."""
        try:
            from modules.humanoid.comms.telegram_bridge import TelegramBridge, _allowed_chat_ids
            bridge = TelegramBridge()
            chat_ids = _allowed_chat_ids()
            if not chat_ids:
                return False
            text = event.format_telegram()
            for chat_id in chat_ids:
                bridge.send(chat_id, text, parse_html=True)
            return True
        except ImportError:
            # Fallback directo sin dependencia de Atlas Core
            return await self._send_telegram_direct(event)
        except Exception as exc:
            logger.warning("[Alerts] Telegram error: %s", exc)
            return False

    async def _send_telegram_direct(self, event: AlertEvent) -> bool:
        """Fallback: envío directo a Telegram API sin TelegramBridge."""
        import os
        import urllib.request
        import urllib.parse
        import json

        token = (os.getenv("TELEGRAM_BOT_TOKEN") or os.getenv("TELEGRAM_TOKEN", "")).strip()
        chat_id = (os.getenv("TELEGRAM_CHAT_ID") or os.getenv("TELEGRAM_ADMIN_CHAT_ID", "")).strip()
        if not token or not chat_id:
            return False
        try:
            payload = json.dumps({
                "chat_id":    chat_id,
                "text":       event.format_telegram(),
                "parse_mode": "HTML",
            }).encode()
            url = f"https://api.telegram.org/bot{token}/sendMessage"
            req = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=8) as resp:
                return resp.status == 200
        except Exception as exc:
            logger.debug("[Alerts] Telegram direct error: %s", exc)
            return False

    async def _send_whatsapp(self, event: AlertEvent) -> bool:
        """Envía alerta por WhatsApp vía el endpoint operativo real de Atlas."""
        try:
            import httpx
            payload = {
                "text": event.format_whatsapp(),
            }
            async with httpx.AsyncClient(timeout=8.0) as client:
                resp = await client.post(
                    f"{self.atlas_api_url}/api/comms/whatsapp/send",
                    json=payload,
                    headers={"x-api-key": os.getenv("ATLAS_API_KEY", "atlas-internal")},
                )
                return resp.status_code == 200
        except Exception as exc:
            logger.debug("[Alerts] WhatsApp error: %s", exc)
            return False

    async def operational_briefing(
        self,
        *,
        title: str,
        body_html: str,
        body_plain: str,
        category: str = "briefing",
        level: str = LEVEL_INFO,
        symbol: str = "",
        metadata: dict | None = None,
        cooldown_s: float | None = None,
        channels: set[str] | None = None,
    ) -> None:
        """Briefings ejecutivos (pre-market / EoD): HTML en Telegram, texto plano en WhatsApp.

        channels: subconjunto de {"telegram","whatsapp"}; None = ambos si están habilitados en el dispatcher.
        """
        event = AlertEvent(
            level=level,
            category=category,
            title=title,
            body=body_html,
            body_plain=body_plain,
            symbol=symbol,
            metadata=metadata or {},
        )
        lvl_order = {LEVEL_INFO: 0, LEVEL_WARNING: 1, LEVEL_CRITICAL: 2}
        if lvl_order.get(event.level, 0) < lvl_order.get(self.min_level, 0):
            return
        cd = cooldown_s if cooldown_s is not None else self.cooldown_s
        key = f"{event.category}:{event.symbol or '__briefing__'}"
        now = time.time()
        last = self._last_alerts.get(key, 0.0)
        if now - last < cd:
            logger.debug("[Alerts] briefing cooldown %s", key)
            return
        self._last_alerts[key] = now
        if self._on_alert:
            try:
                self._on_alert(event)
            except Exception:
                pass
        await self._send_briefing_to_channels(event, channels)

    async def _send_briefing_to_channels(self, event: AlertEvent, channels: set[str] | None) -> None:
        want = channels or {"telegram", "whatsapp"}
        tasks = []
        if "telegram" in want and self.telegram_enabled:
            tasks.append(self._send_telegram(event))
        if "whatsapp" in want and self.whatsapp_enabled:
            tasks.append(self._send_whatsapp(event))
        if not tasks:
            return
        results = await asyncio.gather(*tasks, return_exceptions=True)
        ok = sum(1 for r in results if r is True)
        if ok:
            self._sent_count += 1
        else:
            self._error_count += 1


# ── Singleton global ──────────────────────────────────────────────────────────
_dispatcher: AlertDispatcher | None = None


def get_alert_dispatcher(
    atlas_api_url: str = "http://127.0.0.1:8791",
    min_level: str = LEVEL_INFO,
) -> AlertDispatcher:
    """Retorna la instancia global del AlertDispatcher (lazy singleton)."""
    global _dispatcher
    if _dispatcher is None:
        _dispatcher = AlertDispatcher(
            atlas_api_url=atlas_api_url,
            min_level=min_level,
        )
    return _dispatcher
