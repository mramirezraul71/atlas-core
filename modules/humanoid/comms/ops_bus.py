"""OPS Bus: comunicación permanente de operaciones y movimientos.

Emite eventos por 3 canales:
- Audio PC (TTS)
- Telegram (mensajes + evidencia si hay screenshot)
- WhatsApp (opcional, Twilio)

Objetivo: que ATLAS "hable" y reporte al detalle cada paso.
"""
from __future__ import annotations

import os
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, Optional

_RECENT: Deque[Dict[str, Any]] = deque(maxlen=200)
_LOG_PATH = Path(os.getenv("OPS_LOG_PATH", r"C:\ATLAS_PUSH\logs\ops_bus.log"))


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat()


def _bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "y", "on")


def _telegram_chat_id() -> str:
    raw = (os.getenv("TELEGRAM_ALLOWED_CHAT_IDS") or os.getenv("TELEGRAM_CHAT_ID", "") or "").strip()
    chat_ids = [x.strip() for x in raw.replace(",", " ").split() if x.strip()]
    return chat_ids[0] if chat_ids else ""


def _append_log(line: str) -> None:
    try:
        _LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line.rstrip() + "\n")
    except Exception:
        pass


def recent(limit: int = 50) -> list:
    xs = list(_RECENT)
    return xs[-max(1, int(limit)) :]


def status() -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "ok": True,
        "audio_enabled": _bool("OPS_AUDIO_ENABLED", True),
        "telegram_enabled": _bool("OPS_TELEGRAM_ENABLED", True),
        "whatsapp_enabled": _bool("OPS_WHATSAPP_ENABLED", False),
        "recent_count": len(_RECENT),
        "log_path": str(_LOG_PATH),
    }
    try:
        from modules.humanoid.voice import tts

        out["tts_available"] = tts.is_available()
        out["tts_missing"] = tts.get_missing_deps()
    except Exception:
        out["tts_available"] = False
        out["tts_missing"] = []
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge

        out["telegram_health"] = TelegramBridge().health_check()
        out["telegram_chat_id"] = _telegram_chat_id()
    except Exception as e:
        out["telegram_health"] = {"ok": False, "error": str(e)}
        out["telegram_chat_id"] = ""
    try:
        from modules.humanoid.comms.whatsapp_bridge import status as wa_status

        out["whatsapp_health"] = wa_status()
    except Exception as e:
        out["whatsapp_health"] = {"enabled": False, "error": str(e)}
    return out


def emit(
    subsystem: str,
    message: str,
    level: str = "info",
    data: Optional[Dict[str, Any]] = None,
    evidence_path: str = "",
) -> Dict[str, Any]:
    """Emite un evento (no lanza)."""
    lvl = (level or "info").strip().lower()
    if lvl == "medium":
        lvl = "med"
    if lvl == "warn":
        lvl = "med"
    if lvl not in ("info", "low", "med", "high", "critical"):
        lvl = "info"
    msg = (message or "").strip()
    if not msg:
        return {"ok": True, "skipped": "empty"}

    ev = {
        "ts": _ts(),
        "subsystem": (subsystem or "ops").strip(),
        "level": lvl,
        "message": msg[:1200],
        "data": data or {},
        "evidence_path": evidence_path or "",
    }
    _RECENT.append(ev)
    _append_log(f"{ev['ts']} [{ev['level']}] {ev['subsystem']}: {ev['message']}")

    # Audio PC
    if _bool("OPS_AUDIO_ENABLED", True):
        try:
            from modules.humanoid.voice.tts import speak

            speak_text = msg
            if len(speak_text) > 240:
                speak_text = speak_text[:240] + "..."
            speak(speak_text)
        except Exception:
            pass

    # Telegram (mensaje + evidencia si hay)
    if _bool("OPS_TELEGRAM_ENABLED", True):
        chat_id = _telegram_chat_id()
        if chat_id:
            try:
                from modules.humanoid.comms.telegram_bridge import TelegramBridge

                bridge = TelegramBridge()
                header = f"[ATLAS OPS] <b>{ev['subsystem'].upper()}</b> <code>{ev['level']}</code>\n"
                text = header + ev["message"]
                if evidence_path and str(evidence_path).lower().endswith((".png", ".jpg", ".jpeg", ".webp")):
                    bridge.send_photo(chat_id, str(evidence_path), caption=text[:900])
                else:
                    bridge.send(chat_id, text)
            except Exception:
                pass

    # WhatsApp (Twilio opcional)
    if _bool("OPS_WHATSAPP_ENABLED", False):
        try:
            from modules.humanoid.comms.whatsapp_bridge import send_text

            send_text(f"[ATLAS OPS] {ev['subsystem']} {ev['level']}: {ev['message']}")
        except Exception:
            pass

    return {"ok": True}

