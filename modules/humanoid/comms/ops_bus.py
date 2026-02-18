"""OPS Bus: comunicación permanente de operaciones y movimientos.

Emite eventos por 3 canales:
- Audio PC (TTS)
- Telegram (mensajes + evidencia si hay screenshot)
- WhatsApp (opcional, Twilio)

Objetivo: que ATLAS "hable" y reporte al detalle cada paso.
"""
from __future__ import annotations

import os
import re
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, Optional

_RECENT: Deque[Dict[str, Any]] = deque(maxlen=200)
_LOG_PATH = Path(os.getenv("OPS_LOG_PATH", r"C:\ATLAS_PUSH\logs\ops_bus.log"))
_TG_CHAT_CACHE: Optional[str] = None
_TG_CHAT_CACHE_PATH = Path(os.getenv("TELEGRAM_CHAT_ID_CACHE_PATH", r"C:\ATLAS_PUSH\logs\telegram_chat_id.txt"))
_AUDIO_LAST: Dict[str, float] = {}  # message_key -> ts (dedupe)
_DEDUP_LAST: Dict[str, float] = {}  # key -> last_ts
_RATE_1M: Dict[str, Deque[float]] = {}  # subsystem -> timestamps (sec)

_RE_SESSION_STARTED = re.compile(r"(listo para trabajar).*(sesión iniciada)", re.IGNORECASE)


def _ts() -> str:
    return datetime.now(timezone.utc).isoformat()


def _bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "y", "on")


def _telegram_chat_id() -> str:
    try:
        from modules.humanoid.config.vault import load_vault_env

        load_vault_env(override=False)
    except Exception:
        pass
    raw = (os.getenv("TELEGRAM_ALLOWED_CHAT_IDS") or os.getenv("TELEGRAM_CHAT_ID", "") or "").strip()
    chat_ids = [x.strip() for x in raw.replace(",", " ").split() if x.strip()]
    if chat_ids:
        return chat_ids[0]
    # Fallbacks comunes de la Bóveda (credenciales.txt)
    # - TELEGRAM_ADMIN_CHAT_ID suele ser el chat del owner/admin
    # - ALLOWED_USERS a veces contiene el chat_id numérico
    admin = (os.getenv("TELEGRAM_ADMIN_CHAT_ID") or "").strip()
    if admin:
        return admin
    allowed_users = (os.getenv("ALLOWED_USERS") or "").strip()
    # si es numérico, tratarlo como chat_id
    if allowed_users and allowed_users.isdigit():
        return allowed_users
    global _TG_CHAT_CACHE
    if _TG_CHAT_CACHE:
        return _TG_CHAT_CACHE
    # Intentar leer cache en disco
    try:
        if _TG_CHAT_CACHE_PATH.is_file():
            v = (_TG_CHAT_CACHE_PATH.read_text(encoding="utf-8", errors="ignore") or "").strip()
            if v:
                _TG_CHAT_CACHE = v
                return v
    except Exception:
        pass
    # Descubrir automáticamente (getUpdates)
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge

        out = TelegramBridge().discover_chat_id(limit=10)
        if out.get("ok") and out.get("chat_id"):
            v = str(out["chat_id"]).strip()
            if v:
                _TG_CHAT_CACHE = v
                try:
                    _TG_CHAT_CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
                    _TG_CHAT_CACHE_PATH.write_text(v, encoding="utf-8")
                except Exception:
                    pass
                return v
    except Exception:
        pass
    return ""


def _append_log(line: str) -> None:
    try:
        _LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(line.rstrip() + "\n")
    except Exception:
        pass


_RE_WIN_PATH = re.compile(r"\b[A-Za-z]:\\[^\s<>\"']+")
_RE_UUIDISH = re.compile(r"\b[0-9a-f]{8,}(?:-[0-9a-f]{3,})+\b", re.IGNORECASE)
_RE_HEX_LONG = re.compile(r"\b[0-9a-f]{12,}\b", re.IGNORECASE)
_RE_URL = re.compile(r"\bhttps?://[^\s<>\"']+\b", re.IGNORECASE)
_RE_BRACKETS_TAG = re.compile(r"^\[[A-ZÁÉÍÓÚÜÑ _-]{3,30}\]\s*")


def _subsystem_human(subsystem: str) -> str:
    s = (subsystem or "ops").strip().lower()
    return {
        "approval": "Aprobaciones",
        "dashboard": "Panel",
        "vision": "Visión",
        "nexus": "Cuerpo (NEXUS)",
        "architect": "Arquitecto",
        "ops": "Sistema",
    }.get(s, (subsystem or "Sistema").strip())


def _humanize_text(text: str) -> str:
    """
    Convierte texto técnico a mensaje humano:
    - oculta rutas, URLs, hashes largos
    - reduce IDs (sin borrar completamente si aportan algo)
    - quita símbolos técnicos repetitivos
    """
    t = (text or "").strip()
    if not t:
        return ""
    # Quitar tags tipo [CODE], [DEBUG] al inicio
    t = _RE_BRACKETS_TAG.sub("", t).strip()
    # Reemplazos
    # Evitar símbolos raros (flechas/emoji) y mantener texto copiable.
    t = t.replace("=>", " resultado ").replace("::", ":").replace("`", "")
    t = _RE_WIN_PATH.sub("ruta local", t)
    t = _RE_URL.sub("enlace", t)
    # UUIDs / ids: acortar
    t = _RE_UUIDISH.sub("ID", t)
    t = _RE_HEX_LONG.sub("ID", t)
    # Compactar espacios
    t = re.sub(r"\s+", " ", t).strip()
    return t


def _human_message(subsystem: str, msg: str, level: str, data: Optional[Dict[str, Any]] = None) -> str:
    low = (msg or "").strip().lower()
    sub = (subsystem or "").strip().lower()
    if "aprobación pendiente" in low:
        return "Hay una aprobación pendiente. Te la envié por Telegram para que la resuelvas."
    if "aprobación aprobada" in low:
        return "Aprobación registrada como aprobada."
    if "aprobación rechazada" in low:
        return "Aprobación registrada como rechazada."
    if "perdió conexión" in low or "sin conexión" in low:
        return "El panel perdio conexion. Estoy reintentando automaticamente."
    # Humanización agresiva para eventos de sistema: no leer códigos, ids, rutas.
    if sub in ("ops", "dashboard", "nexus", "architect", "approval"):
        if "error" in low or "exception" in low or "traceback" in low or "failed" in low or "fall" in low:
            # Si el mensaje incluye "Accion:", extraer y describirlo de forma humana.
            try:
                key = "acción:" if "acción:" in low else ("accion:" if "accion:" in low else "")
                i = low.find(key) if key else -1
                if i >= 0 and key:
                    action = msg[i + len(key) :].strip()
                    if action:
                        action = _humanize_text(action)
                        action = action.rstrip().rstrip(".")
                        return f"Detecte un problema. Accion: {action}. Estoy corrigiendolo automaticamente."
            except Exception:
                pass
            return "Detecte un problema. Estoy corrigiendolo automaticamente y te mantengo informado."
    return _humanize_text(msg)


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
    # Asegura carga de Bóveda antes de leer TELEGRAM_* / rutas.
    try:
        from modules.humanoid.config.vault import load_vault_env

        load_vault_env(override=False)
    except Exception:
        pass
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

    msg_human = _human_message(subsystem, msg, lvl, data)
    if not msg_human:
        msg_human = msg

    # Anti-spam: dedupe + rate-limit SOLO para niveles info/low.
    # No silenciamos med/high/critical para no perder señales operativas.
    if lvl in ("info", "low"):
        now = time.time()
        sub_key = (subsystem or "ops").strip().lower() or "ops"
        text_key = (msg_human or msg).strip().lower()

        # Ventana de dedupe: por defecto 20s; para "sesión iniciada" ampliar.
        try:
            base_window = float(os.getenv("OPS_DEDUP_WINDOW_SEC", "20") or "20")
        except Exception:
            base_window = 20.0
        window = 300.0 if _RE_SESSION_STARTED.search(text_key) else base_window
        key = f"{sub_key}|{lvl}|{text_key[:220]}"
        last = float(_DEDUP_LAST.get(key) or 0.0)
        if now - last < window:
            return {"ok": True, "skipped": "dedupe"}
        _DEDUP_LAST[key] = now

        # Rate-limit por subsystem (1 minuto): por defecto 30 eventos/min para info/low.
        try:
            per_min = int(os.getenv("OPS_RATE_LIMIT_PER_MIN", "30") or "30")
        except Exception:
            per_min = 30
        if per_min > 0:
            dq = _RATE_1M.get(sub_key)
            if dq is None:
                dq = deque(maxlen=500)
                _RATE_1M[sub_key] = dq
            cutoff = now - 60.0
            while dq and dq[0] < cutoff:
                dq.popleft()
            if len(dq) >= per_min:
                return {"ok": True, "skipped": "rate_limited"}
            dq.append(now)

    ev = {
        "ts": _ts(),
        "subsystem": (subsystem or "ops").strip(),
        "level": lvl,
        "message": msg[:1200],
        "message_human": msg_human[:1200],
        "data": data or {},
        "evidence_path": evidence_path or "",
    }
    _RECENT.append(ev)
    _append_log(f"{ev['ts']} [{ev['level']}] {ev['subsystem']}: {ev['message']}")

    # Audio PC - Solo para niveles med, high, critical (evitar spam de eventos rutinarios)
    if _bool("OPS_AUDIO_ENABLED", True) and lvl in ("med", "high", "critical"):
        try:
            from modules.humanoid.voice.tts import speak

            speak_text = msg_human
            if len(speak_text) > 240:
                speak_text = speak_text[:240] + "..."
            # Dedupe: evita repetir 2-3 veces el mismo evento en pocos segundos
            import time as _time
            key = (speak_text or "").strip().lower()[:200]
            now = _time.time()
            last = float(_AUDIO_LAST.get(key) or 0.0)
            if now - last >= 8.0:
                _AUDIO_LAST[key] = now
                speak(speak_text)
        except Exception:
            pass

    # Telegram (mensaje + evidencia si hay)
    # Solo enviar a Telegram si el nivel es med, high o critical (evitar spam de info/low)
    if _bool("OPS_TELEGRAM_ENABLED", True) and lvl in ("med", "high", "critical"):
        chat_id = _telegram_chat_id()
        if chat_id:
            try:
                from modules.humanoid.comms.telegram_bridge import TelegramBridge

                bridge = TelegramBridge()
                header = f"<b>ATLAS</b> — {_subsystem_human(ev['subsystem'])}\n"
                text = header + (ev.get("message_human") or ev["message"])
                if evidence_path and str(evidence_path).lower().endswith((".png", ".jpg", ".jpeg", ".webp")):
                    bridge.send_photo(chat_id, str(evidence_path), caption=text[:900])
                else:
                    bridge.send(chat_id, text)
            except Exception:
                pass

    # WhatsApp (Twilio opcional) - Solo para niveles altos
    # WhatsApp se reserva para alertas críticas únicamente
    if _bool("OPS_WHATSAPP_ENABLED", False) and lvl in ("high", "critical"):
        try:
            from modules.humanoid.comms.whatsapp_bridge import send_text

            send_text(f"[ATLAS OPS] {ev['subsystem']} {ev['level']}: {ev['message']}")
        except Exception:
            pass

    return {"ok": True}

