"""Voice approval: 'aprobar <id>' -> confirmación 'confirmar <id>'. Audit con transcript hash."""
from __future__ import annotations

import hashlib
import os
import re
from typing import Any, Dict, Optional, Tuple

_PENDING: Dict[str, str] = {}  # approval_id -> "await_confirm"


def _normalize_id(text: str) -> Optional[str]:
    m = re.search(r"[a-f0-9]{8,12}", text, re.I)
    return m.group(0).lower() if m else None


def handle_voice_command(transcript: str, device_source: Optional[str] = None) -> Tuple[str, bool]:
    """
    Procesa comando de voz. transcript en minúsculas/normalizado.
    Returns: (response_text, did_approve).
    Flujo: "aprobar X" -> pide "confirmar X"; "confirmar X" -> ejecuta approve(X).
    """
    if not transcript or not transcript.strip():
        return "No escuché nada.", False
    text = transcript.strip().lower()
    # Confirmación explícita
    if text.startswith("confirmar "):
        aid = _normalize_id(text[9:].strip())
        if not aid:
            return "Dime el número de aprobación para confirmar.", False
        if aid not in _PENDING:
            return f"No hay solicitud pendiente para {aid}. Di primero: aprobar {aid}.", False
        del _PENDING[aid]
        try:
            from modules.humanoid.approvals import approve
            out = approve(aid, resolved_by="voice", owner_session_token=None)
            if not out.get("ok"):
                return f"No se pudo aprobar {aid}: " + (out.get("error") or "rechazado"), False
            try:
                from modules.humanoid.audit import get_audit_logger
                th = hashlib.sha256(transcript.encode("utf-8")).hexdigest()[:16]
                get_audit_logger().log_event("owner", "voice", "approve", True, 0, None, {"approval_id": aid, "transcript_hash": th, "device": device_source or "voice"}, None)
            except Exception:
                pass
            return f"Aprobado {aid}.", True
        except Exception as e:
            return f"Error al aprobar: {e}. Prueba por UI o Telegram.", False
    # Solicitud inicial
    if text.startswith("aprobar "):
        aid = _normalize_id(text[7:].strip())
        if not aid:
            return "Dime el número de aprobación. Ejemplo: aprobar abc123.", False
        from modules.humanoid.approvals import get
        item = get(aid)
        if not item:
            return f"No existe la aprobación {aid}.", False
        if item.get("status") != "pending":
            return f"La aprobación {aid} ya está resuelta.", False
        _PENDING[aid] = "await_confirm"
        return f"Confirma aprobar {aid}. Di: confirmar {aid}.", False
    return "", False


def voice_approval_enabled() -> bool:
    v = os.getenv("VOICE_APPROVALS_ENABLED") or os.getenv("OWNER_VOICE_CONFIRM", "")
    return v.strip().lower() in ("1", "true", "yes")
