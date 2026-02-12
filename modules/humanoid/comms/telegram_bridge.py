"""Telegram bridge: send messages, approval inline buttons (Aprobar/Rechazar), allowlist, rate limit."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

TELEGRAM_API = "https://api.telegram.org/bot"
_RATE: Dict[str, float] = {}
_RATE_WINDOW = 10.0
_RATE_MAX = 5


def _allowed_chat_ids() -> List[str]:
    raw = os.getenv("TELEGRAM_ALLOWED_CHAT_IDS", "") or os.getenv("TELEGRAM_CHAT_ID", "")
    return [x.strip() for x in raw.replace(",", " ").split() if x.strip()]


def _rate_limit(key: str) -> bool:
    now = time.time()
    recent = [t for t in _RATE.values() if now - t < _RATE_WINDOW]
    if len(recent) >= _RATE_MAX:
        return False
    _RATE[key] = now
    return True


def _token() -> Optional[str]:
    return (os.getenv("TELEGRAM_BOT_TOKEN", "") or os.getenv("TELEGRAM_TOKEN", "") or "").strip() or None


class TelegramBridge:
    """Bridge to Telegram bot: send, approval inline keyboard, allowlist, rate limit."""

    def send(self, chat_id: str, text: str) -> Dict[str, Any]:
        token = _token()
        if not token:
            return {"ok": False, "error": "TELEGRAM_BOT_TOKEN not set"}
        allowed = _allowed_chat_ids()
        if allowed and chat_id not in allowed:
            return {"ok": False, "error": "chat_id not allowed"}
        if not _rate_limit(f"send:{chat_id}"):
            return {"ok": False, "error": "rate limit"}
        try:
            import urllib.request
            import json
            url = f"{TELEGRAM_API}{token}/sendMessage"
            body = {"chat_id": chat_id, "text": text, "parse_mode": "HTML"}
            data = json.dumps(body).encode("utf-8")
            req = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=15) as r:
                out = json.loads(r.read().decode())
            return {"ok": out.get("ok", False), "result": out.get("result")}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def send_approval_inline(self, chat_id: str, approval_id: str, action: str, risk: str = "high") -> Dict[str, Any]:
        """Send message with inline buttons: Aprobar / Rechazar. callback_data: approve:ID / reject:ID."""
        token = _token()
        if not token:
            return {"ok": False, "error": "TELEGRAM_BOT_TOKEN not set"}
        allowed = _allowed_chat_ids()
        if allowed and chat_id not in allowed:
            return {"ok": False, "error": "chat_id not allowed"}
        if not _rate_limit(f"approval:{chat_id}"):
            return {"ok": False, "error": "rate limit"}
        text = f"[ATLAS] Aprobación <b>{risk.upper()}</b>: {action} (id={approval_id})"
        inline_keyboard = {
            "inline_keyboard": [
                [{"text": "Aprobar", "callback_data": f"approve:{approval_id}"}, {"text": "Rechazar", "callback_data": f"reject:{approval_id}"}]
            ]
        }
        try:
            import urllib.request
            import json
            url = f"{TELEGRAM_API}{token}/sendMessage"
            body = {"chat_id": chat_id, "text": text, "parse_mode": "HTML", "reply_markup": inline_keyboard}
            data = json.dumps(body).encode("utf-8")
            req = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=15) as r:
                out = json.loads(r.read().decode())
            return {"ok": out.get("ok", False), "result": out.get("result")}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def handle_callback_data(self, callback_data: str, chat_id: str, resolved_by: str = "telegram") -> Dict[str, Any]:
        """Parse callback_data (approve:ID or reject:ID), validate chat_id, then approve/reject. Returns {ok, action, id, error}."""
        allowed = _allowed_chat_ids()
        if allowed and chat_id not in allowed:
            return {"ok": False, "action": None, "id": None, "error": "chat_id not allowed"}
        if not _rate_limit(f"cb:{chat_id}:{callback_data}"):
            return {"ok": False, "action": None, "id": None, "error": "rate limit"}
        if callback_data.startswith("approve:"):
            aid = callback_data[8:].strip()
            if not aid:
                return {"ok": False, "action": "approve", "id": None, "error": "missing id"}
            try:
                from modules.humanoid.approvals import approve
                out = approve(aid, resolved_by=resolved_by, owner_session_token=None)
                return {"ok": out.get("ok"), "action": "approve", "id": aid, "error": out.get("error")}
            except Exception as e:
                return {"ok": False, "action": "approve", "id": aid, "error": str(e)}
        if callback_data.startswith("reject:"):
            aid = callback_data[7:].strip()
            if not aid:
                return {"ok": False, "action": "reject", "id": None, "error": "missing id"}
            try:
                from modules.humanoid.approvals import reject
                out = reject(aid, resolved_by=resolved_by)
                return {"ok": out.get("ok"), "action": "reject", "id": aid, "error": None}
            except Exception as e:
                return {"ok": False, "action": "reject", "id": aid, "error": str(e)}
        return {"ok": False, "action": None, "id": None, "error": "unknown callback_data"}

    def health_check(self) -> Dict[str, Any]:
        return {"ok": True, "message": "bridge ready", "details": {"token_set": _token() is not None, "allowed_chats": len(_allowed_chat_ids())}}
