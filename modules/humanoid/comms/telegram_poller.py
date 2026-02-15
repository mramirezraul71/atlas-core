"""Telegram polling loop (no webhook).

Purpose:
 - Receive inline approval callbacks (approve:ID / reject:ID)
 - Allow remote approvals from Telegram while away from PC

Enable:
  TELEGRAM_POLLING_ENABLED=true
Requires:
  TELEGRAM_BOT_TOKEN
  TELEGRAM_ALLOWED_CHAT_IDS or TELEGRAM_CHAT_ID
"""
from __future__ import annotations

import json
import os
import threading
import time
import urllib.request
from typing import Any, Dict, Optional

from .telegram_bridge import TelegramBridge, TELEGRAM_API


_thread: Optional[threading.Thread] = None
_stop = threading.Event()


def _token() -> Optional[str]:
    return (os.getenv("TELEGRAM_BOT_TOKEN", "") or os.getenv("TELEGRAM_TOKEN", "") or "").strip() or None


def start_polling() -> bool:
    global _thread
    if _thread and _thread.is_alive():
        return True
    if (os.getenv("TELEGRAM_POLLING_ENABLED", "false") or "").strip().lower() not in ("1", "true", "yes"):
        return False
    if not _token():
        return False
    _stop.clear()
    _thread = threading.Thread(target=_loop, daemon=True)
    _thread.start()
    return True


def stop_polling() -> None:
    _stop.set()


def _api_get_updates(offset: int, timeout_s: int = 20) -> Dict[str, Any]:
    token = _token()
    if not token:
        return {"ok": False, "result": []}
    url = f"{TELEGRAM_API}{token}/getUpdates?offset={int(offset)}&timeout={int(timeout_s)}"
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout_s + 5) as r:
        return json.loads(r.read().decode("utf-8", errors="replace"))


def _api_answer_callback(callback_query_id: str, text: str = "") -> None:
    token = _token()
    if not token or not callback_query_id:
        return
    url = f"{TELEGRAM_API}{token}/answerCallbackQuery"
    body = json.dumps({"callback_query_id": callback_query_id, "text": (text or "")[:180]}).encode("utf-8")
    req = urllib.request.Request(url, data=body, method="POST", headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=10).read()
    except Exception:
        pass


def _loop() -> None:
    bridge = TelegramBridge()
    offset = 0
    # Load last offset from env file? Keep in-memory.
    while not _stop.is_set():
        try:
            out = _api_get_updates(offset=offset, timeout_s=20)
            if not out.get("ok"):
                time.sleep(2)
                continue
            for upd in out.get("result") or []:
                try:
                    update_id = int(upd.get("update_id") or 0)
                    offset = max(offset, update_id + 1)
                except Exception:
                    pass
                # Callback query (inline buttons)
                cq = upd.get("callback_query") or {}
                if cq:
                    data = (cq.get("data") or "").strip()
                    chat_id = str(((cq.get("message") or {}).get("chat") or {}).get("id") or "")
                    cqid = (cq.get("id") or "").strip()
                    if data and chat_id:
                        r = bridge.handle_callback_data(data, chat_id, resolved_by="telegram")
                        _api_answer_callback(cqid, text=("OK" if r.get("ok") else (r.get("error") or "error")))
                        # Also send a confirmation message to the chat
                        try:
                            bridge.send(chat_id, f"[ATLAS] {r.get('action')} {r.get('id')} => {'OK' if r.get('ok') else 'FAIL'}")
                        except Exception:
                            pass
                    continue
                # Text commands (optional): /approve <id>, /reject <id>
                msg = upd.get("message") or {}
                text = (msg.get("text") or "").strip()
                chat_id = str(((msg.get("chat") or {}).get("id") or ""))
                if text and chat_id:
                    lower = text.lower()
                    if lower.startswith("/approve ") or lower.startswith("aprobar "):
                        aid = text.split(maxsplit=1)[1].strip() if len(text.split(maxsplit=1)) > 1 else ""
                        r = bridge.handle_callback_data(f"approve:{aid}", chat_id, resolved_by="telegram")
                        bridge.send(chat_id, f"[ATLAS] approve {aid} => {'OK' if r.get('ok') else 'FAIL'}")
                    elif lower.startswith("/reject ") or lower.startswith("rechazar "):
                        aid = text.split(maxsplit=1)[1].strip() if len(text.split(maxsplit=1)) > 1 else ""
                        r = bridge.handle_callback_data(f"reject:{aid}", chat_id, resolved_by="telegram")
                        bridge.send(chat_id, f"[ATLAS] reject {aid} => {'OK' if r.get('ok') else 'FAIL'}")
        except Exception:
            time.sleep(2)

