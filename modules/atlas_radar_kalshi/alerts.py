from __future__ import annotations

import asyncio
import os
import time
from dataclasses import dataclass, field
from typing import Optional

import httpx

from .utils.logger import get_logger


@dataclass
class AlertEngine:
    """Alertas operativas con cooldown y múltiples canales."""

    log_dir: str
    log_level: str = "INFO"
    min_hedge_rate_1h: float = 0.60
    cooldown_s: int = 300
    webhook_url: Optional[str] = None
    telegram_bot_token: Optional[str] = None
    telegram_chat_id: Optional[str] = None
    _last_sent_by_key: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        self.log = get_logger("alerts", self.log_dir, self.log_level)
        if self.webhook_url is None:
            self.webhook_url = os.getenv("RADAR_ALERT_WEBHOOK_URL") or None
        if self.telegram_bot_token is None:
            self.telegram_bot_token = os.getenv("TELEGRAM_BOT_TOKEN") or None
        if self.telegram_chat_id is None:
            self.telegram_chat_id = os.getenv("TELEGRAM_CHAT_ID") or None

    def should_send(self, key: str) -> bool:
        now = time.time()
        last = self._last_sent_by_key.get(key, 0.0)
        if now - last < self.cooldown_s:
            return False
        self._last_sent_by_key[key] = now
        return True

    async def evaluate_and_alert(
        self,
        *,
        degraded: bool,
        hedge_rate_1h: float,
        total_1h: int,
    ) -> list[dict]:
        sent: list[dict] = []
        if degraded and self.should_send("degraded"):
            msg = "ALERTA RADAR: estado degraded=true en pipeline de eventos."
            sent.extend(await self._dispatch("degraded", msg))
        if total_1h >= 5 and hedge_rate_1h < self.min_hedge_rate_1h and self.should_send("hedge_rate_1h"):
            msg = (
                f"ALERTA RADAR: hedge_rate_1h bajo ({hedge_rate_1h:.2%}) "
                f"con total={total_1h}. Revisar liquidez/latencia/venues."
            )
            sent.extend(await self._dispatch("hedge_rate_1h", msg))
        return sent

    async def _dispatch(self, key: str, message: str) -> list[dict]:
        out: list[dict] = []
        if self.webhook_url:
            ok = await self._send_webhook(message)
            out.append({"channel": "webhook", "key": key, "ok": ok})
        if self.telegram_bot_token and self.telegram_chat_id:
            ok = await self._send_telegram(message)
            out.append({"channel": "telegram", "key": key, "ok": ok})
        if not out:
            self.log.warning("No channels configured for alert: %s", message)
        return out

    async def _send_webhook(self, message: str) -> bool:
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                r = await client.post(self.webhook_url, json={"text": message})
                return r.is_success
        except Exception as exc:
            self.log.warning("Webhook alert failed: %s", exc)
            return False

    async def _send_telegram(self, message: str) -> bool:
        url = f"https://api.telegram.org/bot{self.telegram_bot_token}/sendMessage"
        payload = {"chat_id": self.telegram_chat_id, "text": message}
        try:
            async with httpx.AsyncClient(timeout=10) as client:
                r = await client.post(url, data=payload)
                return r.is_success
        except Exception as exc:
            self.log.warning("Telegram alert failed: %s", exc)
            return False

