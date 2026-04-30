"""
lotto_quant.signals.alert_engine
================================

Multi-channel alert dispatcher.

Channels
--------
    - Telegram bot (if ATLAS_TELEGRAM_TOKEN + chat id are set)
    - Console log (always)
    - DuckDB persistence (always)

Optional: Use the local LLM to humanize the alert text. If the model is
unreachable we fall back to a deterministic template.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Optional

import httpx

from .. import config
from ..llm import LocalAIClient, LocalAIError
from ..llm.prompts import SIGNAL_NARRATION_SYSTEM, SIGNAL_NARRATION_USER
from ..models.ev_calculator import EVResult
from ..data.database import LottoQuantDB

logger = logging.getLogger(__name__)


class AlertEngine:
    """Dispatches alerts to Telegram / console with optional LLM narration."""

    def __init__(
        self,
        db: Optional[LottoQuantDB] = None,
        bot_token: Optional[str] = None,
        chat_id: Optional[str] = None,
        use_llm: bool = config.LOCAL_AI_ENABLED,
    ):
        self.db = db
        self.bot_token = bot_token or config.TELEGRAM_BOT_TOKEN
        self.chat_id = chat_id or config.TELEGRAM_CHAT_ID
        self.use_llm = use_llm
        self.telegram_enabled = bool(self.bot_token and self.chat_id)

    # ── narration ─────────────────────────────────────────────────
    async def narrate(self, ev: EVResult) -> str:
        deterministic = self._fallback_message(ev)
        if not self.use_llm:
            return deterministic
        try:
            async with LocalAIClient() as ai:
                healthy = await ai.health()
                if not healthy:
                    return deterministic
                resp = await ai.complete(
                    SIGNAL_NARRATION_USER.format(
                        game_name=ev.game_name,
                        ticket_price=ev.ticket_price,
                        ev_per_dollar=ev.ev_per_dollar,
                        depletion_ratio=ev.depletion_ratio,
                        major_prize_retention=ev.major_prize_retention,
                        anomaly_score=ev.anomaly_score,
                        signal_strength=ev.signal_strength,
                    ),
                    system=SIGNAL_NARRATION_SYSTEM,
                    temperature=0.2,
                    max_tokens=256,
                )
                return resp.text.strip() or deterministic
        except LocalAIError as e:
            logger.warning("LLM narration failed: %s — using deterministic template", e)
            return deterministic

    @staticmethod
    def _fallback_message(ev: EVResult) -> str:
        circle = {"STRONG": "🟢", "MODERATE": "🟡", "WEAK": "🟠"}.get(
            ev.signal_strength, "⚪"
        )
        return (
            f"{circle} ATLAS LOTTO-QUANT • {ev.signal_strength}\n"
            f"{ev.game_name} (${ev.ticket_price:.2f})  "
            f"EV/$={ev.ev_per_dollar:+.4f}  "
            f"Depl={ev.depletion_ratio:.0%}  "
            f"MajorRem={ev.major_prize_retention:.0%}  "
            f"AnomalyScore={ev.anomaly_score:.2f}\n"
            f"⚠ High-variance asset — consult Kelly allocator before action."
        )

    # ── dispatch ──────────────────────────────────────────────────
    async def dispatch(
        self,
        ev: EVResult,
        signal_type: str = "STALE_PRIZE",
        recommended_position: float = 0.0,
    ) -> dict:
        message = await self.narrate(ev)

        # Console
        if config.ENABLE_CONSOLE_ALERTS:
            logger.info("ALERT [%s | %s]: %s", signal_type, ev.signal_strength, message)
            print(f"\n=== Atlas Lotto-Quant ALERT ===\n{message}\n", flush=True)

        # Telegram
        delivered_telegram = False
        if self.telegram_enabled:
            delivered_telegram = await self._send_telegram(message)

        # Persist
        if self.db is not None:
            self.db.insert_alert(
                game_id=ev.game_id,
                message=message,
                channel=("telegram" if delivered_telegram else "console"),
                delivered=delivered_telegram or config.ENABLE_CONSOLE_ALERTS,
            )

        return {
            "message": message,
            "delivered_telegram": delivered_telegram,
            "delivered_console": config.ENABLE_CONSOLE_ALERTS,
        }

    async def _send_telegram(self, message: str) -> bool:
        url = f"https://api.telegram.org/bot{self.bot_token}/sendMessage"
        try:
            async with httpx.AsyncClient(timeout=15) as client:
                r = await client.post(
                    url,
                    json={
                        "chat_id": self.chat_id,
                        "text": message,
                        "disable_web_page_preview": True,
                    },
                )
                r.raise_for_status()
            return True
        except httpx.HTTPError as e:
            logger.error("Telegram send failed: %s", e)
            return False

    # ── free-form text (used by execution events, not just signals) ───
    async def notify_text(
        self,
        message: str,
        *,
        game_id: str = "EXEC",
    ) -> dict:
        """Send arbitrary text via Telegram + console + alert_log."""
        if config.ENABLE_CONSOLE_ALERTS:
            logger.info("NOTIFY: %s", message)
            print(f"\n=== Atlas Lotto-Quant ===\n{message}\n", flush=True)
        delivered_telegram = False
        if self.telegram_enabled:
            delivered_telegram = await self._send_telegram(message)
        if self.db is not None:
            try:
                self.db.insert_alert(
                    game_id=game_id,
                    message=message,
                    channel=("telegram" if delivered_telegram else "console"),
                    delivered=delivered_telegram or config.ENABLE_CONSOLE_ALERTS,
                )
            except Exception as e:  # pragma: no cover
                logger.debug("alert_log insert failed: %s", e)
        return {
            "delivered_telegram": delivered_telegram,
            "delivered_console": config.ENABLE_CONSOLE_ALERTS,
        }


# ── helpers for sync callers ──────────────────────────────────────
def dispatch_sync(ev: EVResult, **kwargs) -> dict:
    engine = AlertEngine(**kwargs)
    return asyncio.run(engine.dispatch(ev))


def notify_text_sync(message: str, *, game_id: str = "EXEC", **kwargs) -> dict:
    engine = AlertEngine(**kwargs)
    return asyncio.run(engine.notify_text(message, game_id=game_id))


def make_default_notifier(db: Optional[LottoQuantDB] = None):
    """
    Build the default execution-event notifier.

    Returns a callable `(message, game_id='EXEC') -> bool` that fires
    Telegram + console + alert_log. Tests can replace this with a fake
    callable to capture emitted messages.
    """
    def _notify(message: str, game_id: str = "EXEC") -> bool:
        try:
            res = notify_text_sync(message, game_id=game_id, db=db)
            return bool(res.get("delivered_telegram") or res.get("delivered_console"))
        except Exception as e:  # pragma: no cover
            logger.warning("Notifier failed: %s", e)
            return False
    return _notify
