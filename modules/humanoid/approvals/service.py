"""Approval service: create, list, approve, reject. Audit + optional memory link."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional, Tuple

from .gate import requires_approval, risk_level, requires_2fa_for_risk


def _bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "y", "on")


def _human_risk(risk: str) -> str:
    r = (risk or "").strip().lower()
    return {"low": "BAJO", "medium": "MEDIO", "med": "MEDIO", "high": "ALTO", "critical": "CRÍTICO"}.get(r, r.upper() or "MEDIO")


def _summarize_action(action: str, payload: Dict[str, Any]) -> Tuple[str, str]:
    """
    Retorna (summary_text, audio_text) en español, sin detalles técnicos excesivos.
    """
    a = (action or "").strip()
    low = a.lower()
    p = payload or {}

    # POT execution
    if low.startswith("pot_execute:") or low.startswith("pot_execute"):
        pot_id = (a.split(":", 1)[1].strip() if ":" in a else (p.get("pot_id") or "")).strip() or "POT"
        pot_name = ""
        try:
            from modules.humanoid.quality import get_pot
            pot = get_pot(pot_id)
            if pot:
                pot_name = (pot.name or "").strip()
        except Exception:
            pot_name = ""
        title = f"Ejecutar procedimiento: {pot_id}" + (f" — {pot_name}" if pot_name else "")
        why = (p.get("reason") or p.get("trigger") or p.get("check_id") or "").strip()
        if why:
            return (f"{title}\nMotivo: {why}", "Tienes una aprobación pendiente para ejecutar un procedimiento.")
        return (title, "Tienes una aprobación pendiente para ejecutar un procedimiento.")

    # Shell exec
    if low == "shell_exec" or "shell_exec" in low:
        cmd = (p.get("command") or p.get("cmd") or "").strip()
        if cmd:
            short = cmd if len(cmd) <= 140 else cmd[:140] + "..."
            return (f"Ejecutar comando en PC:\n{short}", "Tienes una aprobación pendiente para ejecutar un comando.")
        return ("Ejecutar un comando en PC (detalle no disponible).", "Tienes una aprobación pendiente para ejecutar un comando.")

    # Screen action destructive
    if low == "screen_act_destructive":
        ra = (p.get("requested_action") or "acción de pantalla").strip()
        hint = (p.get("confirm_text") or p.get("expected_window") or p.get("expected_process") or "").strip()
        if hint:
            return (f"Acción en pantalla (destructiva): {ra}\nConfirmación esperada: {hint}", "Tienes una aprobación pendiente para una acción en pantalla.")
        return (f"Acción en pantalla (destructiva): {ra}", "Tienes una aprobación pendiente para una acción en pantalla.")

    # Generic fallback
    return (f"Ejecutar acción: {a or 'aprobación'}", "Tienes una aprobación pendiente en ATLAS.")


def _approval_human_message(item: Dict[str, Any]) -> Tuple[str, str]:
    """
    Retorna (text_for_chat, text_for_audio) claros para el Owner.
    """
    risk = _human_risk((item.get("risk") or "medium").strip())
    aid = (item.get("id") or "").strip()
    expires = (item.get("expires_at") or "").strip()
    action = (item.get("action") or "").strip()
    payload = item.get("payload") or {}

    summary, audio = _summarize_action(action, payload)
    lines = [
        "<b>ATLAS</b> — <b>APROBACIÓN PENDIENTE</b>",
        "",
        f"<b>Qué se quiere hacer</b>",
        summary,
        "",
        f"<b>Riesgo</b>: <b>{risk}</b>",
    ]
    if expires:
        lines.append(f"<b>Expira</b>: {str(expires)[:19].replace('T',' ')}")
    if aid:
        lines.append(f"<b>ID</b>: <code>{aid}</code>")
    lines.append("")
    lines.append("Decisión: <b>Aprobar</b> o <b>Rechazar</b>.")
    return ("\n".join(lines), audio)


def _notify_audio_approval_pending(item: Dict[str, Any]) -> None:
    try:
        if not _bool("APPROVALS_AUDIO_ENABLED", True):
            return
        if not _bool("OPS_AUDIO_ENABLED", True):
            return
        from modules.humanoid.voice.tts import speak
        _text, audio = _approval_human_message(item)
        speak((audio or "Tienes una aprobación pendiente. Revisa Telegram.")[:200])
    except Exception:
        pass


def _notify_whatsapp_approval_pending(item: Dict[str, Any]) -> None:
    try:
        enabled = _bool("APPROVALS_WHATSAPP_ENABLED", True)
        if not enabled:
            return
        from modules.humanoid.comms.whatsapp_bridge import send_text
        text, _audio = _approval_human_message(item)
        # WhatsApp no soporta HTML: limpiar tags básicos.
        clean = (
            text.replace("<b>", "")
            .replace("</b>", "")
            .replace("<code>", "")
            .replace("</code>", "")
        )
        clean += "\n\nAprobación se confirma por Telegram (botones) o por el panel cuando esté integrado."
        send_text(clean[:1500])
    except Exception:
        pass


def _notify_whatsapp_approval_resolved(item: Optional[Dict[str, Any]], aid: str, status: str) -> None:
    try:
        enabled = _bool("APPROVALS_WHATSAPP_ENABLED", True)
        if not enabled:
            return
        from modules.humanoid.comms.whatsapp_bridge import send_text
        st = (status or "").strip().lower()
        if st == "approved":
            msg = "Aprobación ejecutada: APROBADA."
        elif st == "rejected":
            msg = "Aprobación ejecutada: RECHAZADA."
        else:
            msg = "Aprobación: estado actualizado."
        action = ((item or {}).get("action") or "").strip()
        risk = _human_risk(((item or {}).get("risk") or "medium").strip())
        send_text(f"ATLAS\n{msg}\nAcción: {action or '—'}\nRiesgo: {risk}\nID: {aid}"[:1200])
    except Exception:
        pass


def _notify_telegram_approval_pending(item: Dict[str, Any]) -> None:
    """Push a Telegram for ANY high/critical approval (owner away from dashboard)."""
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        from modules.humanoid.comms.ops_bus import _telegram_chat_id  # type: ignore

        # Enabled by default if token+chat_id exist; can be disabled explicitly.
        enabled = (os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "true") or "").strip().lower()
        if enabled not in ("1", "true", "yes"):
            return
        chat_id = (_telegram_chat_id() or "").strip()
        if not chat_id:
            return
        bridge = TelegramBridge()
        risk = (item.get("risk") or "high").strip().lower()
        action = item.get("action") or "approval"
        aid = item.get("id") or ""
        # 1) Mensaje humano entendible
        text, _audio = _approval_human_message(item)
        bridge.send(chat_id, text[:3500])
        # 2) Botones inline (más rápido desde móvil).
        bridge.send_approval_inline(chat_id, aid, action, risk=risk)
    except Exception:
        pass


def _notify_telegram_approval_resolved(item: Optional[Dict[str, Any]], aid: str, status: str, resolved_by: str) -> None:
    try:
        from modules.humanoid.comms.telegram_bridge import TelegramBridge
        from modules.humanoid.comms.ops_bus import _telegram_chat_id  # type: ignore

        enabled = (os.getenv("TELEGRAM_APPROVALS_ENABLED") or os.getenv("TELEGRAM_ENABLED", "true") or "").strip().lower()
        if enabled not in ("1", "true", "yes"):
            return
        chat_id = (_telegram_chat_id() or "").strip()
        if not chat_id:
            return
        st = (status or "").strip().lower()
        if st == "approved":
            msg = "Aprobación ejecutada: APROBADA."
        elif st == "rejected":
            msg = "Aprobación ejecutada: RECHAZADA."
        else:
            msg = "Aprobación: estado actualizado."
        TelegramBridge().send(chat_id, f"<b>ATLAS</b>\n{msg}\nID: <code>{aid}</code>")
    except Exception:
        pass
from .store import create as store_create, list_items, get, approve as store_approve, reject as store_reject


def _log_to_autonomy_timeline(event: str, kind: str, result: str) -> None:
    """Registra evento en autonomy_timeline para el dashboard."""
    try:
        import sqlite3
        from pathlib import Path
        db_path = Path(__file__).resolve().parent.parent.parent.parent / "logs" / "autonomy.db"
        db_path.parent.mkdir(parents=True, exist_ok=True)
        conn = sqlite3.connect(str(db_path), timeout=5)
        conn.execute(
            "CREATE TABLE IF NOT EXISTS autonomy_timeline (id INTEGER PRIMARY KEY AUTOINCREMENT, ts TEXT, event TEXT, kind TEXT, result TEXT)"
        )
        conn.execute(
            "INSERT INTO autonomy_timeline(ts, event, kind, result) VALUES(datetime('now'), ?, ?, ?)",
            (event[:500], kind[:50], result[:500])
        )
        conn.commit()
        conn.close()
    except Exception:
        pass


def create(action: str, payload: Dict[str, Any], job_id: Optional[str] = None, run_id: Optional[int] = None, origin_node_id: Optional[str] = None) -> Dict[str, Any]:
    """Enqueue item if it requires approval. Returns {ok, approval_id?, error}."""
    if not requires_approval(action, payload):
        return {"ok": True, "approval_id": None, "error": None}
    try:
        # Permitir override de riesgo desde payload (p. ej. dispatcher determina riesgo por severidad de POT).
        override_risk = (payload.get("risk") if isinstance(payload, dict) else None) or ""
        override_risk = (str(override_risk).strip().lower() if override_risk else "")
        risk = override_risk if override_risk in ("low", "medium", "high", "critical") else risk_level(action, payload)

        # Dedupe en origen: si ya existe pending equivalente vigente, reutilizarla.
        try:
            from .chain import compute_request_hash
            from .store import find_pending_equivalent

            req_hash = compute_request_hash(payload or {})
            existing = find_pending_equivalent(action=action, risk=risk, request_hash=req_hash)
            if existing and not bool(existing.get("expired")):
                return {
                    "ok": True,
                    "approval_id": existing.get("id"),
                    "error": None,
                    "duplicated": True,
                }
        except Exception:
            pass

        node_id = origin_node_id or (payload.get("origin_node_id") if isinstance(payload, dict) else None) or os.getenv("CLUSTER_NODE_ID")
        item = store_create(action=action, payload=payload, risk=risk, job_id=job_id, run_id=run_id, requires_2fa=requires_2fa_for_risk(risk), origin_node_id=node_id)
        try:
            from modules.humanoid.audit import get_audit_logger
            get_audit_logger().log_event("approvals", "system", "create", True, 0, None, {"id": item["id"], "action": action, "risk": risk}, None)
        except Exception:
            pass
        # Autonomía: TODA aprobación debe llegar a Telegram (owner puede estar fuera del dashboard).
        _notify_telegram_approval_pending(item)
        _notify_whatsapp_approval_pending(item)
        _notify_audio_approval_pending(item)
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            # Log interno (no spam a canales): low no manda Telegram/WhatsApp ni audio desde ops_bus.
            lvl = "low"
            text, _audio = _approval_human_message(item)
            ops_emit(
                "approval",
                text.replace("<b>", "").replace("</b>", "").replace("<code>", "").replace("</code>", "")[:800],
                level=lvl,
                data={"id": item.get("id"), "action": action, "risk": risk},
            )
        except Exception:
            pass
        # Registrar en autonomy_timeline para dashboard
        try:
            _log_to_autonomy_timeline(f"Aprobación creada: {action}", "approval", f"pending:{item.get('id')}")
        except Exception:
            pass
        return {"ok": True, "approval_id": item["id"], "error": None}
    except Exception as e:
        return {"ok": False, "approval_id": None, "error": str(e)}


def list_pending(limit: int = 50, risk: Optional[str] = None) -> List[Dict[str, Any]]:
    # Reaper liviano: evita backlog zombie de approvals vencidas.
    try:
        from .store import expire_pending
        expire_pending(limit=max(100, int(limit or 50) * 4))
    except Exception:
        pass

    items = list_items(status="pending", risk=risk, limit=limit * 2)
    alive = [it for it in items if not bool(it.get("expired"))]
    return alive[: max(1, int(limit or 50))]


def list_all(limit: int = 50, status: Optional[str] = None, risk: Optional[str] = None) -> List[Dict[str, Any]]:
    return list_items(status=status, risk=risk, limit=limit)


def approve(
    aid: str,
    resolved_by: str = "api",
    owner_session_token: Optional[str] = None,
    signature: Optional[str] = None,
    approved_via: str = "api",
    confirm_token: Optional[str] = None,
) -> Dict[str, Any]:
    from modules.humanoid.owner.gate import check_owner_gate
    from .replay import consume_nonce
    from .ttl import is_expired
    item = get(aid)
    if item:
        # Idempotencia: si ya está resuelta, no fallar (evita re-aprobar en Telegram)
        st0 = (item.get("status") or "").strip().lower()
        if st0 == "approved":
            return {"ok": True, "id": aid, "status": "already_approved", "error": None}
        if st0 == "rejected":
            return {"ok": False, "id": aid, "status": "already_rejected", "error": "Ya fue rechazada."}
        risk = (item.get("risk") or "medium").strip().lower()
        allow, err = check_owner_gate(risk, owner_session_token, action=None)
        if not allow:
            return {"ok": False, "id": aid, "status": "owner_session_required", "error": err or "X-Owner-Session required"}
        if is_expired(item.get("expires_at")):
            return {"ok": False, "id": aid, "status": "expired", "error": "Approval expirado. Rechaza y regenera el plan."}
        if confirm_token and not consume_nonce(confirm_token):
            return {"ok": False, "id": aid, "status": "replay_or_invalid", "error": "Invalid or reused confirm_token (nonce)"}
    ok = store_approve(aid, resolved_by=resolved_by, signature=signature)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "approve", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    try:
        from modules.humanoid.metalearn.collector import record_approval_resolved
        record_approval_resolved(aid, item.get("action"), item.get("risk"), "approved", item)
    except Exception:
        pass
    # Si existía item pero no aprobó, distinguir expiración vs no encontrado.
    if not ok and item:
        try:
            from .ttl import is_expired
            if is_expired(item.get("expires_at")):
                return {"ok": False, "id": aid, "status": "expired", "error": "Approval expirado. Rechaza y regenera el plan."}
        except Exception:
            pass
        return {"ok": False, "id": aid, "status": "failed", "error": "No se pudo aprobar (estado inválido o condición de seguridad)."}
    if ok:
        _notify_telegram_approval_resolved(item, aid, "approved", resolved_by=resolved_by)
        _notify_whatsapp_approval_resolved(item, aid, "approved")
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("approval", f"Aprobación APROBADA: id={aid} action={(item or {}).get('action')}", level="info", data={"id": aid, "status": "approved"})
        except Exception:
            pass
        # Registrar en autonomy_timeline
        try:
            _log_to_autonomy_timeline(f"Aprobación aprobada: {(item or {}).get('action')}", "approval", f"approved:{aid}")
        except Exception:
            pass
        # Ejecutar acción aprobada (si hay ejecutor). No bloquea el approve.
        exec_out = None
        try:
            from modules.humanoid.approvals.executor import execute_approved
            exec_out = execute_approved(item or {}, approval_id=aid, resolved_by=resolved_by)
            try:
                from modules.humanoid.comms.ops_bus import emit as ops_emit
                lvl = "info" if exec_out.get("ok") else "med"
                ops_emit(
                    "approval",
                    "Acción aprobada ejecutada." if exec_out.get("ok") else "Aprobación aprobada, pero la ejecución falló.",
                    level=lvl,
                    data={"id": aid, "action": (item or {}).get("action"), "exec_ok": exec_out.get("ok"), "error": exec_out.get("error")},
                )
            except Exception:
                pass
        except Exception:
            exec_out = None
        return {
            "ok": ok,
            "id": aid,
            "status": "approved",
            "error": None,
            "executed": bool(exec_out is not None),
            "execution": exec_out,
        }
    return {"ok": ok, "id": aid, "status": "approved" if ok else "not_found", "error": None if ok else "not_found"}


def reject(aid: str, resolved_by: str = "api") -> Dict[str, Any]:
    item = get(aid)
    if item:
        # Idempotencia: si ya está resuelta, no fallar (evita re-rechazar en Telegram)
        st0 = (item.get("status") or "").strip().lower()
        if st0 == "rejected":
            return {"ok": True, "id": aid, "status": "already_rejected"}
        if st0 == "approved":
            return {"ok": False, "id": aid, "status": "already_approved"}
    ok = store_reject(aid, resolved_by=resolved_by)
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("approvals", resolved_by, "reject", ok, 0, None, {"id": aid}, None)
    except Exception:
        pass
    try:
        from modules.humanoid.metalearn.collector import record_approval_resolved
        record_approval_resolved(aid, (item or {}).get("action"), (item or {}).get("risk"), "rejected", item or {})
    except Exception:
        pass
    if ok:
        _notify_telegram_approval_resolved(item, aid, "rejected", resolved_by=resolved_by)
        _notify_whatsapp_approval_resolved(item, aid, "rejected")
        try:
            from modules.humanoid.comms.ops_bus import emit as ops_emit
            ops_emit("approval", f"Aprobación RECHAZADA: id={aid} action={(item or {}).get('action')}", level="info", data={"id": aid, "status": "rejected"})
        except Exception:
            pass
        # Registrar en autonomy_timeline
        try:
            _log_to_autonomy_timeline(f"Aprobación rechazada: {(item or {}).get('action')}", "approval", f"rejected:{aid}")
        except Exception:
            pass
    return {"ok": ok, "id": aid, "status": "rejected" if ok else "not_found"}
