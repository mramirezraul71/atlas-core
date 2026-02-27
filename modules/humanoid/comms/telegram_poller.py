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
import re
import threading
import time
import urllib.request
import urllib.parse
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from .telegram_bridge import TelegramBridge, TELEGRAM_API


_thread: Optional[threading.Thread] = None
_stop = threading.Event()

_INBOX_DIR = Path(os.getenv("TELEGRAM_INBOX_DIR", r"C:\ATLAS_PUSH\logs\telegram_inbox"))


def _token() -> Optional[str]:
    try:
        from modules.humanoid.config.vault import load_vault_env

        load_vault_env(override=False)
    except Exception:
        pass
    return (os.getenv("TELEGRAM_BOT_TOKEN", "") or os.getenv("TELEGRAM_TOKEN", "") or "").strip() or None


def _bool(name: str, default: bool) -> bool:
    v = (os.getenv(name) or "").strip().lower()
    if not v:
        return default
    return v in ("1", "true", "yes", "y", "on")


def _download_telegram_file(file_id: str, *, suffix: str = ".bin") -> Optional[str]:
    """
    Descarga un archivo de Telegram (photo/document) al inbox local.
    Retorna ruta local o None.
    """
    token = _token()
    if not token or not file_id:
        return None
    try:
        # getFile -> file_path
        q = urllib.parse.quote(str(file_id))
        meta_url = f"{TELEGRAM_API}{token}/getFile?file_id={q}"
        with urllib.request.urlopen(meta_url, timeout=20) as r:
            meta = json.loads(r.read().decode("utf-8", errors="replace"))
        if not meta.get("ok"):
            return None
        fp = ((meta.get("result") or {}).get("file_path") or "").strip()
        if not fp:
            return None
        dl_url = f"https://api.telegram.org/file/bot{token}/{fp}"
        _INBOX_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        ext = Path(fp).suffix or suffix
        out = _INBOX_DIR / f"inbox_{ts}_{str(file_id)[-10:]}{ext}"
        with urllib.request.urlopen(dl_url, timeout=30) as r:
            out.write_bytes(r.read())
        return str(out)
    except Exception:
        return None


def _try_ocr_image(path: str) -> str:
    try:
        from modules.humanoid.screen.ocr import run_ocr
        b = Path(path).read_bytes()
        text, err = run_ocr(image_bytes=b)
        if err:
            return ""
        return (text or "").strip()
    except Exception:
        return ""


def _handle_inbox_media(msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """
    Si el usuario envía una foto/documento al bot, guardarlo como evidencia local.
    Retorna dict con {saved_path, kind, caption, ocr_text?} o None si no aplica.
    """
    if not _bool("TELEGRAM_INBOX_ENABLED", True):
        return None
    if not msg:
        return None
    caption = str(msg.get("caption") or msg.get("text") or "").strip()
    photos = msg.get("photo") or []
    doc = msg.get("document") or None
    file_id = ""
    kind = ""
    ext_hint = ".bin"
    if photos:
        kind = "photo"
        file_id = str((photos[-1] or {}).get("file_id") or "").strip()
        ext_hint = ".jpg"
    elif doc:
        kind = "document"
        file_id = str((doc or {}).get("file_id") or "").strip()
        fname = str((doc or {}).get("file_name") or "").strip()
        if fname:
            ext_hint = Path(fname).suffix or ".bin"
    if not file_id:
        return None
    saved = _download_telegram_file(file_id, suffix=ext_hint)
    if not saved:
        return {"saved_path": "", "kind": kind, "caption": caption, "error": "download_failed"}
    ocr_text = ""
    if kind in ("photo", "document") and (saved.lower().endswith((".png", ".jpg", ".jpeg", ".webp"))):
        ocr_text = _try_ocr_image(saved)
        # Persist OCR (si hay) al lado, para auditoría rápida sin abrir imagen.
        if ocr_text:
            try:
                p = Path(saved)
                p.with_suffix(p.suffix + ".ocr.txt").write_text(ocr_text[:20000], encoding="utf-8", errors="ignore")
            except Exception:
                pass
    return {"saved_path": saved, "kind": kind, "caption": caption, "ocr_text": ocr_text}


def start_polling() -> bool:
    global _thread
    if _thread and _thread.is_alive():
        return True
    # Default ON si hay token; se puede desactivar explícitamente con TELEGRAM_POLLING_ENABLED=false
    if (os.getenv("TELEGRAM_POLLING_ENABLED", "true") or "").strip().lower() not in ("1", "true", "yes"):
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


def _api_edit_message_text(chat_id: str, message_id: int, text: str, *, remove_keyboard: bool = True) -> None:
    """Edit the original inline-approval message to reflect final status and optionally remove buttons."""
    token = _token()
    if not token or not chat_id or not message_id:
        return
    url = f"{TELEGRAM_API}{token}/editMessageText"
    body: Dict[str, Any] = {
        "chat_id": chat_id,
        "message_id": int(message_id),
        "text": (text or "")[:3500],
        "parse_mode": "HTML",
        "disable_web_page_preview": True,
    }
    if remove_keyboard:
        body["reply_markup"] = {"inline_keyboard": []}
    data = json.dumps(body).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST", headers={"Content-Type": "application/json"})
    try:
        urllib.request.urlopen(req, timeout=10).read()
    except Exception:
        pass


def _should_remove_keyboard(status: str) -> bool:
    st = (status or "").strip().lower()
    return st in ("approved", "rejected", "already_approved", "already_rejected", "expired", "not_found")


def _format_status_message(action: str, aid: str, status: str, err: str = "", execution: Optional[Dict[str, Any]] = None) -> str:
    a = (action or "").strip().lower()
    st = (status or "").strip().lower()
    eid = (aid or "").strip()
    err_s = (err or "").strip()

    if st in ("approved", "already_approved"):
        lines = [f"<b>ATLAS</b>", "Aprobación: <b>APROBADA</b>", f"ID: <code>{eid}</code>"]
        ex = execution or {}
        if ex:
            ex_ok = ex.get("ok")
            ms = ex.get("ms")
            if ex_ok is True:
                lines.append("Ejecución: <b>OK</b>")
            elif ex_ok is False:
                lines.append("Ejecución: <b>FALLÓ</b>")
            if ms is not None:
                try:
                    lines.append(f"Tiempo: <b>{int(ms)}ms</b>")
                except Exception:
                    pass
            # Evidencia (paths locales)
            try:
                res = ex.get("result") or {}
                ev = res.get("evidence") or {}
                b = ev.get("before")
                a2 = ev.get("after")
                if b or a2:
                    lines.append("Evidencia:")
                    if b:
                        lines.append(f"- before: <code>{str(b)[-220:]}</code>")
                    if a2:
                        lines.append(f"- after: <code>{str(a2)[-220:]}</code>")
            except Exception:
                pass
            if ex.get("error"):
                lines.append(f"Detalle: {str(ex.get('error'))[:160]}")
        return "\n".join(lines)[:3500]
    if st in ("rejected", "already_rejected"):
        return f"<b>ATLAS</b>\nAprobación: <b>RECHAZADA</b>\nID: <code>{eid}</code>"
    if st == "expired":
        return f"<b>ATLAS</b>\nAprobación: <b>EXPIRADA</b>\nID: <code>{eid}</code>"
    if st == "owner_session_required":
        return f"<b>ATLAS</b>\nSeguridad: requiere sesión del Owner.\nID: <code>{eid}</code>"
    if st == "not_found":
        return f"<b>ATLAS</b>\nEsta aprobación ya no está disponible.\nID: <code>{eid}</code>"
    if err_s:
        return f"<b>ATLAS</b>\nNo se pudo completar la acción.\nDetalle: {err_s[:160]}"
    return "<b>ATLAS</b>\nNo se pudo completar la acción."


def _handle_doctor_command(bridge: TelegramBridge, chat_id: str) -> None:
    """Ejecuta diagnóstico completo del sistema y envía reporte a Telegram."""
    lines = ["<b>🩺 ATLAS DOCTOR</b>\n"]
    # 1) Watchdog status
    try:
        from modules.humanoid.watchdog.engine import watchdog_status
        ws = watchdog_status()
        lines.append(f"<b>Watchdog:</b> {'🟢 Activo' if ws.get('running') else '🔴 Inactivo'}")
        alerts = ws.get("last_alerts") or []
        if alerts:
            lines.append(f"  ⚠️ {len(alerts)} alerta(s):")
            for a in alerts[:5]:
                lines.append(f"  · {a.get('rule', '?')}: {a.get('message', a.get('error', ''))[:80]}")
        else:
            lines.append("  ✅ Sin alertas")
    except Exception as e:
        lines.append(f"<b>Watchdog:</b> ❌ Error: {str(e)[:80]}")
    # 2) Scheduler status
    try:
        from modules.humanoid.scheduler.engine import is_scheduler_running, get_scheduler_db
        running = is_scheduler_running()
        lines.append(f"\n<b>Scheduler:</b> {'🟢 Activo' if running else '🔴 Detenido'}")
        db = get_scheduler_db()
        jobs = db.list_jobs()
        stale = [j for j in jobs if (j.get("status") or "").lower() == "running"]
        failed = [j for j in jobs if (j.get("status") or "").lower() == "failed"]
        if stale:
            lines.append(f"  ⚠️ {len(stale)} job(s) en 'running' (posible cuelgue)")
            for j in stale[:3]:
                lines.append(f"  · {j.get('name', j.get('id', '?'))[:40]}")
        if failed:
            lines.append(f"  ❌ {len(failed)} job(s) fallido(s)")
            for j in failed[:3]:
                err = (j.get("last_error") or "")[:60]
                lines.append(f"  · {j.get('name', j.get('id', '?'))[:30]}: {err}")
        if not stale and not failed:
            lines.append("  ✅ Todos los jobs OK")
    except Exception as e:
        lines.append(f"\n<b>Scheduler:</b> ❌ Error: {str(e)[:80]}")
    # 3) Servicios críticos
    try:
        from modules.humanoid.watchdog.rules import check_critical_services
        svc_alerts = check_critical_services()
        if svc_alerts:
            lines.append("\n<b>Servicios:</b>")
            for sa in svc_alerts:
                lines.append(f"  🔴 {sa.get('service', '?')}: {sa.get('error', f'HTTP {sa.get(\"status\", \"?\")}')[:60]}")
        else:
            lines.append("\n<b>Servicios:</b> ✅ adapter + robot OK")
    except Exception as e:
        lines.append(f"\n<b>Servicios:</b> ❌ {str(e)[:80]}")
    # 4) WorldState
    try:
        from modules.humanoid.vision.world_state import load_latest_world_state
        ws_data = load_latest_world_state()
        ws_ok = ws_data.get("ok", False)
        ws_src = ws_data.get("source", "?")
        ws_err = ws_data.get("error", "")
        if ws_ok:
            lines.append(f"\n<b>WorldState:</b> ✅ OK (src={ws_src})")
        else:
            lines.append(f"\n<b>WorldState:</b> 🔴 Error: {ws_err[:60]}")
    except Exception as e:
        lines.append(f"\n<b>WorldState:</b> ❌ {str(e)[:80]}")
    bridge.send(chat_id, "\n".join(lines))


def _handle_fix_command(bridge: TelegramBridge, chat_id: str, text: str) -> None:
    """Auto-reparación: resetear jobs colgados, reiniciar scheduler si caído."""
    parts = text.strip().split()
    target = parts[1].lower() if len(parts) > 1 else "all"
    lines = ["<b>🔧 ATLAS FIX</b>\n"]
    fixed_count = 0
    # Fix stale jobs
    if target in ("all", "jobs", "scheduler"):
        try:
            from modules.humanoid.scheduler.engine import get_scheduler_db
            db = get_scheduler_db()
            jobs = db.list_jobs()
            stale = [j for j in jobs if (j.get("status") or "").lower() == "running"]
            for j in stale:
                db.set_queued(j["id"])
                lines.append(f"  ✅ Job '{j.get('name', j['id'])[:30]}' → queued")
                fixed_count += 1
            failed = [j for j in jobs if (j.get("status") or "").lower() == "failed"]
            for j in failed:
                db.set_queued(j["id"])
                lines.append(f"  ✅ Job '{j.get('name', j['id'])[:30]}' (failed) → queued")
                fixed_count += 1
        except Exception as e:
            lines.append(f"  ❌ Error reseteando jobs: {str(e)[:80]}")
    # Fix scheduler
    if target in ("all", "scheduler"):
        try:
            from modules.humanoid.scheduler.engine import is_scheduler_running
            if not is_scheduler_running():
                from modules.humanoid.healing import restart_scheduler
                r = restart_scheduler()
                if r.get("ok"):
                    lines.append("  ✅ Scheduler reiniciado")
                    fixed_count += 1
                else:
                    lines.append(f"  ⚠️ Scheduler: {r.get('error', 'no arrancó')[:60]}")
            else:
                lines.append("  ℹ️ Scheduler ya estaba activo")
        except Exception as e:
            lines.append(f"  ❌ Error reiniciando scheduler: {str(e)[:80]}")
    if fixed_count == 0:
        lines.append("  ℹ️ Nada que reparar, todo parece OK")
    else:
        lines.append(f"\n<b>Total reparaciones:</b> {fixed_count}")
    bridge.send(chat_id, "\n".join(lines))


def _handle_cameras_command(bridge: TelegramBridge, chat_id: str) -> None:
    """Envía snapshot de cámaras activas y estado."""
    lines = ["<b>📸 ATLAS CAMERAS</b>\n"]
    try:
        import urllib.request as _ur
        adapter_url = os.getenv("ATLAS_ADAPTER_URL", "http://127.0.0.1:8000").rstrip("/")
        for idx in range(3):
            try:
                url = f"{adapter_url}/cuerpo/vision/snapshot?source=camera&index={idx}&jpeg_quality=60"
                req = _ur.Request(url, method="GET")
                with _ur.urlopen(req, timeout=5) as resp:
                    content_type = resp.headers.get("Content-Type", "")
                    if "image" in content_type and resp.status == 200:
                        lines.append(f"  🟢 Cámara {idx}: activa")
                    else:
                        lines.append(f"  🔴 Cámara {idx}: sin imagen")
            except Exception:
                lines.append(f"  ⚫ Cámara {idx}: no disponible")
    except Exception as e:
        lines.append(f"  ❌ Error: {str(e)[:80]}")
    bridge.send(chat_id, "\n".join(lines))


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
                # Mensajes con media (fotos/documentos): archivar a inbox local para auditoría.
                try:
                    msg_any = upd.get("message") or upd.get("edited_message") or upd.get("channel_post") or {}
                    if msg_any and (msg_any.get("photo") or msg_any.get("document")):
                        inbox = _handle_inbox_media(msg_any)
                        if inbox and inbox.get("saved_path"):
                            try:
                                from modules.humanoid.comms.ops_bus import emit as ops_emit
                                cap = (inbox.get("caption") or "").strip()
                                ocr = (inbox.get("ocr_text") or "").strip()
                                msg_h = "Telegram: evidencia recibida y guardada."
                                if cap:
                                    msg_h += " Nota: " + cap[:120]
                                if ocr and ("github" in ocr.lower() or "login" in ocr.lower() or "security" in ocr.lower() or "intento" in ocr.lower()):
                                    msg_h += " OCR: posible aviso de GitHub."
                                ops_emit("telegram", msg_h, level="info", data={"kind": inbox.get("kind"), "caption": cap[:300]}, evidence_path=str(inbox.get("saved_path") or ""))
                            except Exception:
                                pass
                except Exception:
                    pass
                # Callback query (inline buttons)
                cq = upd.get("callback_query") or {}
                if cq:
                    data = (cq.get("data") or "").strip()
                    msg_obj = cq.get("message") or {}
                    chat_id = str(((msg_obj.get("chat") or {}).get("id") or ""))
                    message_id = int(msg_obj.get("message_id") or 0)
                    cqid = (cq.get("id") or "").strip()
                    if data and chat_id:
                        r = bridge.handle_callback_data(data, chat_id, resolved_by="telegram")
                        # Respuesta instantánea (toast)
                        _api_answer_callback(cqid, text=("OK" if r.get("ok") else (r.get("status") or r.get("error") or "error")))
                        # Editar el mensaje original: actualizar estado y remover botones si ya se resolvió.
                        try:
                            st = str(r.get("status") or "")
                            txt = _format_status_message(
                                str(r.get("action") or ""),
                                str(r.get("id") or ""),
                                st,
                                str(r.get("error") or ""),
                                r.get("execution"),
                            )
                            _api_edit_message_text(chat_id, message_id, txt, remove_keyboard=_should_remove_keyboard(st))
                        except Exception:
                            pass
                    continue
                # Text commands (optional): /approve <id>, /reject <id>, /doctor, /fix, /cameras
                msg = upd.get("message") or {}
                text = (msg.get("text") or "").strip()
                chat_id = str(((msg.get("chat") or {}).get("id") or ""))
                if text and chat_id:
                    lower = text.lower()
                    if lower.startswith("/approve ") or lower.startswith("aprobar "):
                        aid = text.split(maxsplit=1)[1].strip() if len(text.split(maxsplit=1)) > 1 else ""
                        r = bridge.handle_callback_data(f"approve:{aid}", chat_id, resolved_by="telegram")
                        bridge.send(chat_id, _format_status_message("approve", aid, str(r.get("status") or ""), str(r.get("error") or ""), r.get("execution")))
                    elif lower.startswith("/reject ") or lower.startswith("rechazar "):
                        aid = text.split(maxsplit=1)[1].strip() if len(text.split(maxsplit=1)) > 1 else ""
                        r = bridge.handle_callback_data(f"reject:{aid}", chat_id, resolved_by="telegram")
                        bridge.send(chat_id, _format_status_message("reject", aid, str(r.get("status") or ""), str(r.get("error") or "")))
                    elif lower == "/doctor" or lower == "/diagnostico":
                        _handle_doctor_command(bridge, chat_id)
                    elif lower.startswith("/fix"):
                        _handle_fix_command(bridge, chat_id, text)
                    elif lower == "/cameras" or lower == "/camaras":
                        _handle_cameras_command(bridge, chat_id)
        except Exception:
            time.sleep(2)

