"""Telegram bridge: stub for future Telegram integration."""
from __future__ import annotations

from typing import Any, Dict


class TelegramBridge:
    """Bridge to Telegram bot. Stub until real integration."""

    def send(self, chat_id: str, text: str) -> Dict[str, Any]:
        return {"ok": False, "error": "TelegramBridge stub"}

    def health_check(self) -> Dict[str, Any]:
        return {"ok": True, "message": "stub", "details": {}}
