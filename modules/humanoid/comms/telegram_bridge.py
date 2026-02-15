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
    raw = (os.getenv("TELEGRAM_ALLOWED_CHAT_IDS", "") or os.getenv("TELEGRAM_CHAT_ID", "") or "").strip()
    chat_ids = [x.strip() for x in raw.replace(",", " ").split() if x.strip()]
    if chat_ids:
        return chat_ids
    # Fallbacks comunes (Bóveda)
    admin = (os.getenv("TELEGRAM_ADMIN_CHAT_ID") or "").strip()
    if admin:
        return [admin]
    allowed_users = (os.getenv("ALLOWED_USERS") or "").strip()
    if allowed_users and allowed_users.isdigit():
        return [allowed_users]
    return []


def _rate_limit(key: str) -> bool:
    now = time.time()
    recent = [t for t in _RATE.values() if now - t < _RATE_WINDOW]
    if len(recent) >= _RATE_MAX:
        return False
    _RATE[key] = now
    return True


def _token() -> Optional[str]:
    try:
        from modules.humanoid.config.vault import load_vault_env

        load_vault_env(override=False)
    except Exception:
        pass
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

    def discover_chat_id(self, limit: int = 10) -> Dict[str, Any]:
        """
        Descubre un chat_id reciente usando getUpdates.
        Retorna {ok, chat_id, error}.
        Requisito: el usuario debe haber enviado al menos un mensaje al bot.
        """
        token = _token()
        if not token:
            return {"ok": False, "chat_id": "", "error": "TELEGRAM_BOT_TOKEN not set"}
        try:
            import urllib.request
            import json

            url = f"{TELEGRAM_API}{token}/getUpdates?limit={int(limit or 10)}"
            req = urllib.request.Request(url, method="GET")
            with urllib.request.urlopen(req, timeout=15) as r:
                data = json.loads(r.read().decode("utf-8", errors="replace"))
            if not data.get("ok"):
                return {"ok": False, "chat_id": "", "error": "getUpdates not ok"}
            results = data.get("result") or []
            # Buscar chat_id en message o callback_query
            chat_id = ""
            for upd in reversed(results):
                try:
                    if upd.get("message") and upd["message"].get("chat") and upd["message"]["chat"].get("id") is not None:
                        chat_id = str(upd["message"]["chat"]["id"])
                        break
                    cq = upd.get("callback_query") or {}
                    msg = cq.get("message") or {}
                    chat = msg.get("chat") or {}
                    if chat.get("id") is not None:
                        chat_id = str(chat["id"])
                        break
                except Exception:
                    continue
            if not chat_id:
                return {"ok": False, "chat_id": "", "error": "no_chat_id_found_in_updates"}
            return {"ok": True, "chat_id": chat_id, "error": None}
        except Exception as e:
            return {"ok": False, "chat_id": "", "error": str(e)}

    def send_photo(self, chat_id: str, photo_path: str, caption: str = "") -> Dict[str, Any]:
        """Send a photo (screenshot evidence) to Telegram."""
        token = _token()
        if not token:
            return {"ok": False, "error": "TELEGRAM_BOT_TOKEN not set"}
        allowed = _allowed_chat_ids()
        if allowed and chat_id not in allowed:
            return {"ok": False, "error": "chat_id not allowed"}
        if not _rate_limit(f"photo:{chat_id}"):
            return {"ok": False, "error": "rate limit"}
        try:
            from pathlib import Path
            p = Path(photo_path)
            if not p.is_file():
                return {"ok": False, "error": "photo_not_found"}
            import urllib.request
            import uuid

            url = f"{TELEGRAM_API}{token}/sendPhoto"
            boundary = "----atlas" + uuid.uuid4().hex
            # Multipart form-data
            def _part(name: str, value: str) -> bytes:
                return (
                    f"--{boundary}\r\n"
                    f'Content-Disposition: form-data; name="{name}"\r\n\r\n'
                    f"{value}\r\n"
                ).encode("utf-8")

            header = (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="photo"; filename="{p.name}"\r\n'
                f"Content-Type: image/png\r\n\r\n"
            ).encode("utf-8")
            tail = f"\r\n--{boundary}--\r\n".encode("utf-8")

            body = b"".join(
                [
                    _part("chat_id", chat_id),
                    _part("caption", (caption or "")[:900]),
                    _part("parse_mode", "HTML"),
                    header,
                    p.read_bytes(),
                    tail,
                ]
            )
            req = urllib.request.Request(
                url,
                data=body,
                method="POST",
                headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
            )
            with urllib.request.urlopen(req, timeout=30) as r:
                import json
                out = json.loads(r.read().decode("utf-8", errors="replace"))
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
        def _human_action(a: str) -> str:
            x = (a or "").strip().lower()
            return {
                "architect_apply": "Aplicar cambios propuestos por el Arquitecto",
                "update_apply": "Aplicar una actualización del sistema",
            }.get(x, (a or "Ejecutar una acción").strip())

        risk_h = (risk or "high").strip().lower()
        risk_h = {"low": "BAJO", "medium": "MEDIO", "med": "MEDIO", "high": "ALTO", "critical": "CRÍTICO"}.get(risk_h, risk_h.upper())
        action_h = _human_action(action)
        text = (
            "<b>ATLAS</b>\n"
            f"Necesito tu aprobación para: <b>{action_h}</b>\n"
            f"Riesgo: <b>{risk_h}</b>\n"
            f"ID: <code>{approval_id}</code>"
        )
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
        """Parse callback_data (approve:ID or reject:ID), validate chat_id, then approve/reject.

        Returns: {ok, action, id, status, error}
        """
        allowed = _allowed_chat_ids()
        if allowed and chat_id not in allowed:
            return {"ok": False, "action": None, "id": None, "status": None, "error": "chat_id not allowed"}
        if not _rate_limit(f"cb:{chat_id}:{callback_data}"):
            return {"ok": False, "action": None, "id": None, "status": None, "error": "rate limit"}
        if callback_data.startswith("approve:"):
            aid = callback_data[8:].strip()
            if not aid:
                return {"ok": False, "action": "approve", "id": None, "status": "missing_id", "error": "missing id"}
            try:
                from modules.humanoid.approvals import approve
                out = approve(aid, resolved_by=resolved_by, owner_session_token=None)
                return {
                    "ok": out.get("ok"),
                    "action": "approve",
                    "id": aid,
                    "status": out.get("status"),
                    "error": out.get("error"),
                    "executed": out.get("executed"),
                    "execution": out.get("execution"),
                }
            except Exception as e:
                return {"ok": False, "action": "approve", "id": aid, "status": "exception", "error": str(e)}
        if callback_data.startswith("reject:"):
            aid = callback_data[7:].strip()
            if not aid:
                return {"ok": False, "action": "reject", "id": None, "status": "missing_id", "error": "missing id"}
            try:
                from modules.humanoid.approvals import reject
                out = reject(aid, resolved_by=resolved_by)
                return {"ok": out.get("ok"), "action": "reject", "id": aid, "status": out.get("status"), "error": out.get("error")}
            except Exception as e:
                return {"ok": False, "action": "reject", "id": aid, "status": "exception", "error": str(e)}
        return {"ok": False, "action": None, "id": None, "status": "unknown", "error": "unknown callback_data"}

    def health_check(self) -> Dict[str, Any]:
        return {"ok": True, "message": "bridge ready", "details": {"token_set": _token() is not None, "allowed_chats": len(_allowed_chat_ids())}}
