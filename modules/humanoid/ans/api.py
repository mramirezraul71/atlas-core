"""ANS API: status, incidents, report, run-now, live stream, self-model."""
from __future__ import annotations

from fastapi import APIRouter
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from typing import Optional
import json
import time
import re
from datetime import datetime

router = APIRouter(prefix="/ans", tags=["ANS"])

_RE_SESSION_STARTED = re.compile(r"(listo para trabajar).*(sesión iniciada)", re.IGNORECASE)


def _ts_to_epoch(ts: str) -> float:
    t = (ts or "").strip()
    if not t:
        return 0.0
    try:
        # ISO 8601 (con o sin timezone)
        dt = datetime.fromisoformat(t.replace("Z", "+00:00"))
        return dt.timestamp()
    except Exception:
        return 0.0


@router.get("/status")
def ans_status():
    from .engine import get_ans_status
    return get_ans_status()


@router.get("/incidents")
def ans_incidents(status: Optional[str] = None, limit: int = 50):
    try:
        from .incident import get_incidents
        items = get_incidents(status=status, limit=limit)
        return {"ok": True, "data": items}
    except Exception as e:
        return {"ok": False, "data": [], "error": str(e)}


@router.post("/resolve-incidents")
def ans_resolve_incidents(check_id: Optional[str] = None):
    """Resuelve todos los incidentes abiertos (stale). Opcional: check_id para solo ese check (ej. deps_health)."""
    try:
        from .incident import resolve_all_open
        count = resolve_all_open(check_id=check_id)
        return {"ok": True, "resolved": count}
    except Exception as e:
        return {"ok": False, "resolved": 0, "error": str(e)}


@router.get("/bitacora")
def ans_bitacora(limit: int = 50):
    """Bitácora Central: todos los eventos del sistema incluyendo evolución, comunicación, incidentes.
    
    Fuentes:
    - Incidentes ANS (problemas detectados y acciones)
    - Evolución (tríada PyPI/GitHub/HF)
    - Comunicaciones (Telegram, WhatsApp, Audio)
    - Sistema (startup, servicios, etc.)
    """
    try:
        from .incident import get_incidents
        from .evolution_bitacora import get_evolution_entries
        from modules.humanoid.comms import ops_bus
        from modules.humanoid.comms.hub import recent as comms_recent
        
        entries = []
        seen_key: set = set()
        
        # 1. Incidentes ANS
        items = get_incidents(status=None, limit=limit * 2)
        for inc in items:
            prob = (inc.get("message") or inc.get("check_id") or "?")[:120]
            cid = inc.get("check_id") or ""
            key = (cid, prob)
            if key in seen_key:
                continue
            seen_key.add(key)
            actions = inc.get("actions_taken", [])
            if actions:
                for a in actions:
                    accion = a.get("heal_id", "—")
                    ok = a.get("ok", False)
                    entries.append({
                        "timestamp": inc.get("created_at"),
                        "message": f"{prob} - {accion}: {(a.get('message') or '')[:80]}",
                        "level": "success" if ok else "error",
                        "source": "incident",
                        "icon": "🔧" if ok else "❌",
                        "category": "Incidente",
                    })
            else:
                is_open = inc.get("status") == "open"
                entries.append({
                    "timestamp": inc.get("created_at"),
                    "message": prob,
                    "level": "warning" if is_open else "info",
                    "source": "incident",
                    "icon": "⚠️" if is_open else "📋",
                    "category": "Incidente",
                })
        
        # 2. Entradas de evolución y comunicación
        for ev in get_evolution_entries(limit=limit * 2):
            src = ev.get("source", "evolution")
            msg = ev.get("message", "")
            
            # Determinar icono y categoría según source y mensaje
            icon = "⚙️"
            if src in ("telegram", "whatsapp", "comms", "audio"):
                icon = "📱" if src == "whatsapp" else ("📨" if src == "telegram" else ("🔊" if src == "audio" else "📡"))
            elif src == "evolution":
                icon = "🔄"
            elif src == "repo_monitor":
                icon = "📦"
            elif src in ("startup", "services", "system"):
                icon = "🚀"
            elif src == "nervous":
                icon = "⚡"
            elif not ev.get("ok", True):
                icon = "❌"
            
            # Categoría legible
            categoria = {
                "repo_monitor": "Repositorio",
                "evolution": "Evolución",
                "telegram": "Telegram",
                "whatsapp": "WhatsApp",
                "comms": "Comunicación",
                "audio": "Audio",
                "startup": "Inicio",
                "services": "Servicios",
                "system": "Sistema",
                "nervous": "Sistema Nervioso",
                "test": "Prueba",
            }.get(src, src.capitalize())
            
            entries.append({
                "timestamp": ev.get("timestamp", ""),
                "message": msg[:200],
                "level": "success" if ev.get("ok", True) else "error",
                "source": src,
                "icon": icon,
                "category": categoria,
            })

        # 3. OPS bus (eventos operativos internos) — fuente real de muchos subsistemas
        # Nota: ops_bus mantiene un buffer en memoria y log en disco. Aquí solo exponemos el buffer reciente.
        try:
            for ev in (ops_bus.recent(limit=limit * 4) or []):
                ts = (ev.get("ts") or "").strip()
                msg = (ev.get("message_human") or ev.get("message") or "").strip()
                if not msg:
                    continue
                src = (ev.get("subsystem") or "ops").strip().lower() or "ops"
                lvl = (ev.get("level") or "info").strip().lower()
                # Mapear nivel OPS → nivel UI
                if lvl in ("critical", "high"):
                    level = "error"
                    icon = "🛑" if lvl == "critical" else "⚠️"
                elif lvl in ("med", "warn", "warning"):
                    level = "warning"
                    icon = "⚠️"
                else:
                    level = "info"
                    icon = "🧾"
                key = (ts[:19], src, msg[:120])
                if key in seen_key:
                    continue
                seen_key.add(key)
                entries.append({
                    "timestamp": ts,
                    "message": msg[:240],
                    "level": level,
                    "source": src,
                    "icon": icon,
                    "category": "OPS",
                })
        except Exception:
            pass

        # 4. CommsHub (historial unificado de mensajes/alertas)
        try:
            for m in (comms_recent(limit=limit * 4) or []):
                ts = (m.get("timestamp") or "").strip()
                msg = (m.get("content") or "").strip()
                if not msg:
                    continue
                sub = (m.get("subsystem") or "comms").strip().lower() or "comms"
                lvl = (m.get("level") or "info").strip().lower()
                if lvl in ("critical",):
                    level = "error"
                    icon = "🛑"
                elif lvl in ("high", "warning"):
                    level = "warning"
                    icon = "⚠️"
                elif lvl in ("debug",):
                    level = "info"
                    icon = "🧪"
                else:
                    level = "info"
                    icon = "📡"
                key = (ts[:19], sub, msg[:120])
                if key in seen_key:
                    continue
                seen_key.add(key)
                entries.append({
                    "timestamp": ts,
                    "message": msg[:240],
                    "level": level,
                    "source": sub,
                    "icon": icon,
                    "category": "Comms",
                })
        except Exception:
            pass
        
        # Ordenar por timestamp (más reciente primero)
        entries.sort(key=lambda x: x.get("timestamp") or "", reverse=True)

        # Dedupe defensivo: evitar que spam de INFO tape el resto.
        # No tocamos warning/error (señales), solo "info/success" rutinarios.
        now = time.time()
        out = []
        last_by_key: dict[tuple[str, str], float] = {}

        for e in entries:
            lvl = (e.get("level") or "info").strip().lower()
            if lvl not in ("info", "success"):
                out.append(e)
                if len(out) >= limit:
                    break
                continue

            src = (e.get("source") or "system").strip().lower()
            msg = (e.get("message") or "").strip()
            if not msg:
                continue

            msg_key = msg.lower()[:220]
            key = (src, msg_key)

            ts_epoch = _ts_to_epoch(e.get("timestamp") or "") or now
            window = 300.0 if _RE_SESSION_STARTED.search(msg) else 20.0
            last = float(last_by_key.get(key) or 0.0)
            if last and abs(ts_epoch - last) < window:
                continue
            last_by_key[key] = ts_epoch

            out.append(e)
            if len(out) >= limit:
                break

        return {"ok": True, "entries": out, "total": len(entries)}
    except Exception as e:
        return {"ok": False, "entries": [], "error": str(e)}


class EvolutionLogBody(BaseModel):
    message: str = ""
    ok: Optional[bool] = True
    source: Optional[str] = None  # evolution | repo_monitor | ...


@router.post("/evolution-log")
def ans_evolution_log(body: EvolutionLogBody):
    """Registro industrial: daemon EVOLUTION, repo_monitor y otros envían cada paso a la Bitácora ANS."""
    try:
        from .evolution_bitacora import append_evolution_log
        from .live_stream import emit
        msg = (body.message or "").strip()
        ok = body.ok if body.ok is not None else True
        source = (body.source or "evolution").strip() or "evolution"
        if msg:
            append_evolution_log(msg, ok=ok, source=source)
            emit("evolution_log", message=msg, ok=ok)
        return {"ok": True, "logged": bool(msg)}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/report/latest")
def ans_report_latest():
    try:
        from .reporter import get_latest_report
        path = get_latest_report()
        return {"ok": True, "path": path}
    except Exception as e:
        return {"ok": False, "path": "", "error": str(e)}


class RunNowBody(BaseModel):
    mode: Optional[str] = None


@router.get("/live/recent")
def ans_live_recent(limit: int = 100):
    """Últimos eventos del stream (polling). Estilo Cursor."""
    try:
        from .live_stream import get_recent
        return {"ok": True, "events": get_recent(limit=limit)}
    except Exception as e:
        return {"ok": False, "events": [], "error": str(e)}


@router.get("/live")
async def ans_live_stream():
    """SSE: señales en vivo de lo que ejecuta el ANS. Estilo Cursor."""
    async def gen():
        from .live_stream import stream_events
        async for ev in stream_events():
            yield f"data: {json.dumps(ev, ensure_ascii=False)}\n\n"
    return StreamingResponse(
        gen(),
        media_type="text/event-stream",
        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
    )


@router.get("/scanner-status")
def ans_scanner_status():
    """Último scan (MakePlay) y últimas descargas pip."""
    try:
        from modules.humanoid.comms.scanner_store import get_last_scan, get_last_downloads
        return {
            "ok": True,
            "last_scan": get_last_scan(),
            "last_downloads": get_last_downloads(limit=20),
        }
    except Exception as e:
        return {"ok": False, "last_scan": {}, "last_downloads": [], "error": str(e)}


@router.get("/self-model")
def ans_self_model():
    """Autoconocimiento: anatomía del sistema, checks, heals, dependencias."""
    try:
        from modules.humanoid.self_model import get_manifest
        return {"ok": True, "manifest": get_manifest()}
    except Exception as e:
        return {"ok": False, "manifest": None, "error": str(e)}


@router.post("/run-now")
def ans_run_now(body: Optional[RunNowBody] = None):
    try:
        from modules.humanoid.policy import ActorContext, get_policy_engine
        ctx = ActorContext(actor="api", role="owner")
        decision = get_policy_engine().can(ctx, "ans", "ans_run_now", None)
        if not decision.allow:
            return {"ok": False, "error": decision.reason}
        from .engine import run_ans_cycle
        mode = body.mode if body else None
        r = run_ans_cycle(mode=mode, timeout_sec=60)
        return r
    except Exception as e:
        return {"ok": False, "error": str(e)}


# ─────────────────────────────────────────────────────────────────────────────
# Workshop Central: API para el Taller de Reparaciones
# ─────────────────────────────────────────────────────────────────────────────

class WorkshopRunBody(BaseModel):
    mode: Optional[str] = "incidents"  # full | incidents | maintenance
    limit: Optional[int] = 50
    require_approval_heavy: Optional[bool] = True


@router.get("/workshop/status")
def workshop_status():
    """Estado del Workshop Central: directorios, conteos, último reporte."""
    import os
    from pathlib import Path
    try:
        repo_root = Path(os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or Path(__file__).resolve().parent.parent.parent.parent)
        workshop_root = repo_root / "logs" / "workshop"
        inbox = list((workshop_root / "inbox").glob("*.json")) if (workshop_root / "inbox").exists() else []
        working = list((workshop_root / "working").glob("*.json")) if (workshop_root / "working").exists() else []
        resolved = list((workshop_root / "resolved").glob("*.json")) if (workshop_root / "resolved").exists() else []
        failed = list((workshop_root / "failed").glob("*.json")) if (workshop_root / "failed").exists() else []
        reports = sorted((workshop_root / "reports").glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True) if (workshop_root / "reports").exists() else []
        last_report = None
        if reports:
            try:
                last_report = json.loads(reports[0].read_text(encoding="utf-8"))
            except Exception:
                last_report = {"path": str(reports[0])}
        return {
            "ok": True,
            "inbox_count": len(inbox),
            "working_count": len(working),
            "resolved_count": len(resolved),
            "failed_count": len(failed),
            "reports_count": len(reports),
            "last_report": last_report,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/workshop/tickets")
def workshop_tickets(tray: str = "inbox", limit: int = 50):
    """Listar tickets del Workshop en una bandeja (inbox, working, resolved, failed)."""
    import os
    from pathlib import Path
    try:
        repo_root = Path(os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or Path(__file__).resolve().parent.parent.parent.parent)
        tray_dir = repo_root / "logs" / "workshop" / tray
        if not tray_dir.exists():
            return {"ok": True, "tickets": [], "count": 0, "tray": tray}
        files = sorted(tray_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
        tickets = []
        for f in files:
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                tickets.append({
                    "filename": f.name,
                    "state": data.get("state"),
                    "incident_id": (data.get("incident") or {}).get("id"),
                    "check_id": (data.get("incident") or {}).get("check_id"),
                    "created_at": data.get("ticket_created_at"),
                    "last_processed_at": data.get("last_processed_at"),
                })
            except Exception:
                tickets.append({"filename": f.name, "error": "parse_failed"})
        return {"ok": True, "tickets": tickets, "count": len(tickets), "tray": tray}
    except Exception as e:
        return {"ok": False, "tickets": [], "error": str(e)}


@router.post("/workshop/run-now")
def workshop_run_now(body: Optional[WorkshopRunBody] = None):
    """Ejecutar ciclo del Workshop ahora (modo: incidents, maintenance, full)."""
    import subprocess
    import sys
    import os
    from pathlib import Path
    try:
        repo_root = Path(os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or Path(__file__).resolve().parent.parent.parent.parent)
        script = repo_root / "scripts" / "atlas_central_workshop.py"
        if not script.is_file():
            return {"ok": False, "error": "script not found: scripts/atlas_central_workshop.py"}
        mode = (body.mode or "incidents").strip().lower() if body else "incidents"
        limit = int(body.limit or 50) if body else 50
        require_approval = body.require_approval_heavy if body else True
        argv = [sys.executable, str(script), "--mode", mode, "--limit", str(limit)]
        if require_approval:
            argv.append("--require-approval-heavy")
        r = subprocess.run(
            argv,
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            timeout=300,
            env={**os.environ, "ATLAS_REPO_PATH": str(repo_root), "ATLAS_PUSH_ROOT": str(repo_root)},
        )
        return {
            "ok": r.returncode in (0, 2),
            "exit_code": r.returncode,
            "mode": mode,
            "stdout": (r.stdout or "")[-2000:],
            "stderr": (r.stderr or "")[-500:],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "workshop timeout 300s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.get("/workshop/reports")
def workshop_reports(limit: int = 20):
    """Listar reportes generados por el Workshop."""
    import os
    from pathlib import Path
    try:
        repo_root = Path(os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or Path(__file__).resolve().parent.parent.parent.parent)
        reports_dir = repo_root / "logs" / "workshop" / "reports"
        if not reports_dir.exists():
            return {"ok": True, "reports": [], "count": 0}
        files = sorted(reports_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)[:limit]
        reports = []
        for f in files:
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                reports.append({
                    "filename": f.name,
                    "started_at": data.get("started_at"),
                    "ended_at": data.get("ended_at"),
                    "mode": data.get("mode"),
                    "overall_ok": data.get("overall_ok"),
                    "result": data.get("result"),
                })
            except Exception:
                reports.append({"filename": f.name, "error": "parse_failed"})
        return {"ok": True, "reports": reports, "count": len(reports)}
    except Exception as e:
        return {"ok": False, "reports": [], "error": str(e)}


# ============================================================================
# COMUNICACIONES - Estado completo del sistema de comunicación
# ============================================================================

@router.get("/comms/status")
def ans_comms_status():
    """Estado completo del sistema de comunicación: Telegram, WhatsApp (WAHA), Audio.
    
    Incluye:
    - Estado de cada canal
    - Configuración de WAHA
    - URL del Dashboard de WAHA
    - Estadísticas de mensajes
    """
    import os
    import urllib.request
    
    result = {
        "ok": True,
        "channels": {},
        "waha": {
            "url": None,
            "dashboard_url": None,
            "status": "unknown",
            "authenticated": False,
            "session_name": None,
        },
        "telegram": {
            "enabled": False,
            "token_configured": False,
            "chat_id_configured": False,
        },
        "audio": {
            "enabled": False,
            "engine": None,
        },
    }
    
    # WhatsApp (WAHA)
    waha_url = os.getenv("WAHA_API_URL", "http://localhost:3010")
    waha_key = os.getenv("WAHA_API_KEY", "atlas123")
    result["waha"]["url"] = waha_url
    result["waha"]["dashboard_url"] = waha_url  # WAHA dashboard está en la misma URL
    
    # Formato para el frontend
    result["whatsapp"] = {
        "connected": False,
        "session": "default",
        "phone": None,
    }
    
    try:
        headers = {"X-Api-Key": waha_key}
        req = urllib.request.Request(f"{waha_url}/api/sessions/default", headers=headers)
        with urllib.request.urlopen(req, timeout=5) as r:
            data = json.loads(r.read().decode())
            result["waha"]["status"] = data.get("status", "unknown")
            result["waha"]["authenticated"] = data.get("status") == "WORKING"
            result["waha"]["session_name"] = data.get("name", "default")
            result["whatsapp"]["connected"] = data.get("status") == "WORKING"
            result["whatsapp"]["session"] = data.get("name", "default")
            me = data.get("me", {})
            if me:
                phone = me.get("id", "").split("@")[0]
                result["waha"]["phone"] = phone
                result["waha"]["name"] = me.get("pushName", "")
                result["whatsapp"]["phone"] = phone
    except Exception as e:
        result["waha"]["status"] = "error"
        result["waha"]["error"] = str(e)[:100]
    
    # Telegram
    telegram_token = os.getenv("TELEGRAM_BOT_TOKEN") or os.getenv("TG_BOT_TOKEN")
    telegram_chat = os.getenv("TELEGRAM_CHAT_ID") or os.getenv("TG_CHAT_ID")
    result["telegram"]["enabled"] = bool(telegram_token)
    result["telegram"]["token_configured"] = bool(telegram_token)
    result["telegram"]["chat_id_configured"] = bool(telegram_chat)
    result["telegram"]["configured"] = bool(telegram_token)
    result["telegram"]["has_token"] = bool(telegram_token)
    result["telegram"]["chat_id"] = telegram_chat[:6] + "..." if telegram_chat else None
    
    # Audio
    try:
        import pyttsx3
        # Verificar que realmente puede inicializar
        engine = pyttsx3.init()
        result["audio"]["enabled"] = True
        result["audio"]["available"] = True
        result["audio"]["engine"] = "pyttsx3"
    except Exception:
        result["audio"]["enabled"] = False
        result["audio"]["available"] = False
        result["audio"]["engine"] = None
    
    # CommsHub status
    try:
        from modules.humanoid.comms.hub import get_hub
        hub = get_hub()
        if hub:
            hub_health = hub.health()
            result["channels"] = hub_health.get("channels", {})
    except Exception:
        pass
    
    return result


@router.get("/comms/waha/qr")
def ans_comms_waha_qr():
    """Obtener QR de WAHA para vincular WhatsApp."""
    import os
    import urllib.request
    import base64
    
    waha_url = os.getenv("WAHA_API_URL", "http://localhost:3010")
    waha_key = os.getenv("WAHA_API_KEY", "atlas123")
    
    try:
        headers = {"X-Api-Key": waha_key}
        req = urllib.request.Request(f"{waha_url}/api/default/auth/qr", headers=headers)
        with urllib.request.urlopen(req, timeout=10) as r:
            qr_data = r.read()
            # Devolver como base64 para mostrar en el dashboard
            qr_b64 = base64.b64encode(qr_data).decode()
            return {"ok": True, "qr_base64": qr_b64, "content_type": "image/png"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/comms/waha/restart")
def ans_comms_waha_restart():
    """Reiniciar sesión de WAHA."""
    import os
    import urllib.request
    
    waha_url = os.getenv("WAHA_API_URL", "http://localhost:3010")
    waha_key = os.getenv("WAHA_API_KEY", "atlas123")
    
    try:
        headers = {"X-Api-Key": waha_key, "Content-Type": "application/json"}
        # Stop session
        req = urllib.request.Request(f"{waha_url}/api/sessions/default/stop", data=b"{}", headers=headers, method="POST")
        try:
            urllib.request.urlopen(req, timeout=5)
        except Exception:
            pass
        
        # Start session
        req = urllib.request.Request(f"{waha_url}/api/sessions/default/start", data=b"{}", headers=headers, method="POST")
        with urllib.request.urlopen(req, timeout=10) as r:
            return {"ok": True, "message": "Session restarted"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/comms/test")
def ans_comms_test(channel: str = "all", message: str = "Mensaje de prueba desde ATLAS"):
    """Enviar mensaje de prueba a un canal específico o todos.
    
    Canales: telegram, whatsapp, audio, all
    """
    results = {}
    
    if channel in ("telegram", "all"):
        try:
            from modules.humanoid.comms.telegram_bridge import send
            ok = send(message)
            results["telegram"] = {"ok": ok}
        except Exception as e:
            results["telegram"] = {"ok": False, "error": str(e)}
    
    if channel in ("whatsapp", "all"):
        try:
            from modules.humanoid.comms.whatsapp_bridge import send_text
            result = send_text(message)
            results["whatsapp"] = result
        except Exception as e:
            results["whatsapp"] = {"ok": False, "error": str(e)}
    
    if channel in ("audio", "all"):
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.setProperty('rate', 150)
            engine.say(message)
            engine.runAndWait()
            results["audio"] = {"ok": True}
        except Exception as e:
            results["audio"] = {"ok": False, "error": str(e)}
    
    return {"ok": all(r.get("ok", False) for r in results.values()), "results": results}


# ============================================================================
# VOICE ASSISTANT - Control de interacción por voz
# ============================================================================

# Variable global para el proceso del asistente de voz
_voice_process = None

@router.get("/voice/status")
def ans_voice_status():
    """Estado del asistente de voz interactivo."""
    global _voice_process
    import subprocess
    
    running = False
    if _voice_process is not None:
        running = _voice_process.poll() is None
    
    return {
        "ok": True,
        "running": running,
        "pid": _voice_process.pid if _voice_process and running else None,
        "available": True,
        "features": ["wake_word", "stt", "tts", "llm_integration"]
    }


@router.post("/voice/start")
def ans_voice_start():
    """Inicia el asistente de voz en background."""
    global _voice_process
    import subprocess
    import os
    
    # Verificar si ya está corriendo
    if _voice_process is not None and _voice_process.poll() is None:
        return {"ok": True, "message": "Voice assistant already running", "pid": _voice_process.pid}
    
    try:
        repo_root = os.getenv("ATLAS_PUSH_ROOT") or os.getenv("ATLAS_REPO_PATH") or "C:\\ATLAS_PUSH"
        script_path = os.path.join(repo_root, "start_voice.py")
        
        _voice_process = subprocess.Popen(
            ["python", script_path],
            cwd=repo_root,
            creationflags=subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0
        )
        
        return {"ok": True, "message": "Voice assistant started", "pid": _voice_process.pid}
    except Exception as e:
        return {"ok": False, "error": str(e)}


@router.post("/voice/stop")
def ans_voice_stop():
    """Detiene el asistente de voz."""
    global _voice_process
    
    if _voice_process is None:
        return {"ok": True, "message": "Voice assistant not running"}
    
    try:
        _voice_process.terminate()
        _voice_process.wait(timeout=5)
        _voice_process = None
        return {"ok": True, "message": "Voice assistant stopped"}
    except Exception as e:
        try:
            _voice_process.kill()
            _voice_process = None
        except Exception:
            pass
        return {"ok": False, "error": str(e)}


@router.post("/voice/speak")
def ans_voice_speak(text: str = ""):
    """Hace que ATLAS hable un texto específico."""
    if not text:
        return {"ok": False, "error": "text is required"}
    
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.setProperty('rate', 140)
        engine.say(text)
        engine.runAndWait()
        return {"ok": True, "spoken": text}
    except Exception as e:
        return {"ok": False, "error": str(e)}
