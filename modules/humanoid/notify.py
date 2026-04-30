"""
Notification helpers (Telegram, etc.).

This module exists to provide a stable import path for legacy callers:
`from modules.humanoid.notify import send_telegram`
"""

from __future__ import annotations

import asyncio
import os
from pathlib import Path
from typing import Optional


def _cached_chat_id() -> Optional[str]:
    # Explicit env first
    raw = (
        os.getenv("TELEGRAM_CHAT_ID") or os.getenv("TELEGRAM_ADMIN_CHAT_ID") or ""
    ).strip()
    if raw:
        return raw
    # Common cache file (used by ops_bus)
    p = Path(
        os.getenv(
            "TELEGRAM_CHAT_ID_CACHE_PATH", r"C:\ATLAS_PUSH\logs\telegram_chat_id.txt"
        )
    )
    try:
        if p.is_file():
            v = p.read_text(encoding="utf-8", errors="ignore").strip()
            return v or None
    except Exception:
        return None
    return None


async def send_telegram(message: str) -> bool:
    """
    Best-effort Telegram notification.
    - Returns False if not configured (token/chat_id missing) or on error.
    """
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
    except Exception:
        return False

    chat_id = _cached_chat_id()
    if not chat_id:
        return False

    msg = (message or "").strip()
    if not msg:
        return False

    # TelegramBridge is sync; run in a worker thread.
    try:
        out = await asyncio.to_thread(TelegramBridge().send, chat_id, msg[:3500])
        return bool(out and out.get("ok"))
    except Exception:
        return False
