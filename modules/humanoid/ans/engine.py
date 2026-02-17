"""ANS engine: run checks -> detect issues -> plan -> heal -> verify -> report."""
from __future__ import annotations

import os
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

# Heals que Atlas puede ejecutar automáticamente sin supervisión
SAFE_HEALS = {
    # Básicos
    "clear_stale_locks", "restart_scheduler", "rotate_logs",
    # Modelos/Router
    "fallback_models", "tune_router",
    # Servicios
    "restart_nexus_services", "restart_camera_service", "restart_robot_backend",
    # Infraestructura
    "retry_gateway_bootstrap", "mark_node_offline", "regenerate_support_bundle",
    # Dependencias
    "install_optional_deps", "install_tesseract",
    # Docker/WAHA
    "restart_waha", "restart_docker_container",
}

# Mapeo check→heal por defecto cuando suggested_heals está vacío (evita "0 actions" con issues)
CHECK_DEFAULT_HEALS: Dict[str, List[str]] = {
    "api_health": ["clear_stale_locks", "restart_scheduler"],
    "deploy_health": ["restart_scheduler"],
    "ui_health": ["clear_stale_locks"],
    "router_health": ["fallback_models", "tune_router"],
    # Servicios NEXUS/Robot
    "nexus_services_health": ["restart_nexus_services"],
    "robot_camera_health": ["restart_nexus_services", "restart_camera_service"],
    "nervous_health": ["restart_nexus_services"],
    "camera_service": ["restart_nexus_services"],
}


def _env(name: str, default: str) -> str:
    v = os.getenv(name)
    return (v or "").strip() or default


def _env_bool(name: str, default: bool) -> bool:
    v = _env(name, "true" if default else "false").lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_int(name: str, default: int) -> int:
    try:
        return int(_env(name, str(default)) or default)
    except Exception:
        return default


def _ans_bypass_governance() -> bool:
    """True si ANS puede ejecutar heals sin aprobación (modo agresivo)."""
    mode = _env("SYSTEM_MODE", "observe").lower()
    force = _env("ANS_GROWTH_FORCE", "false").lower() in ("1", "true", "yes")
    if mode == "aggressive" or force:
        return True
    try:
        from modules.humanoid.governance.state import get_mode
        return get_mode() == "growth"
    except Exception:
        return False


def run_ans_cycle(mode: str = None, timeout_sec: int = 30) -> Dict[str, Any]:
    if not _env_bool("ANS_ENABLED", True):
        return {"ok": True, "enabled": False, "message": "ANS disabled"}
    mode = mode or _env("ANS_MODE", "auto")
    if mode == "observe_only":
        return _run_observe(timeout_sec)
    return _run_full(timeout_sec)


def _run_observe(timeout_sec: int) -> Dict[str, Any]:
    t0 = time.perf_counter()
    from modules.humanoid.ans.checks import _register_all
    _register_all()
    from modules.humanoid.ans.registry import run_check, get_checks
    results = []
    for cid in list(get_checks().keys())[:14]:
        if time.perf_counter() - t0 > timeout_sec:
            break
        try:
            r = run_check(cid)
            results.append(r)
        except Exception as e:
            results.append({"ok": False, "check_id": cid, "message": str(e)})
    ms = int((time.perf_counter() - t0) * 1000)
    issues = [r for r in results if not r.get("ok")]
    return {"ok": True, "mode": "observe_only", "results": results, "issues_count": len(issues), "ms": ms}


def _run_full(timeout_sec: int) -> Dict[str, Any]:
    t0 = time.perf_counter()
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("cycle_start", message="Iniciando ciclo ANS")
    except Exception:
        pass
    from modules.humanoid.ans.checks import _register_all
    _register_all()
    import modules.humanoid.ans.heals  # ensure heals registered
    from modules.humanoid.ans.registry import run_check, run_heal, get_checks
    from modules.humanoid.ans.incident import create_incident, resolve_incident, add_action, get_incidents
    from modules.humanoid.ans.limits import can_auto_action, record_action
    from modules.humanoid.ans.evidence import capture_evidence
    from modules.humanoid.ans.reporter import write_report, notify_telegram
    from modules.humanoid.ans.brain_link import generate_summary
    from modules.humanoid.policy import ActorContext, get_policy_engine

    results: List[Dict] = []
    incidents_created: List[str] = []
    actions_taken: List[Dict] = []
    max_checks = _env_int("ANS_MAX_CHECKS", 25)
    all_ids = list(get_checks().keys())
    check_ids = all_ids[:max(1, min(max_checks, 50))]
    # Forzar checks críticos aunque queden fuera del corte (sensibilidad industrial).
    for critical_id in ("nervous_health", "nexus_services_health", "robot_camera_health"):
        if critical_id in all_ids and critical_id not in check_ids:
            check_ids.append(critical_id)
    max_workers = min(8, len(check_ids), _env_int("ANS_PARALLEL_WORKERS", 6))

    # Fase 1: ejecutar checks en paralelo (acelera ~3-4x vs secuencial)
    def _run_one(cid: str) -> tuple[str, Dict]:
        try:
            from modules.humanoid.ans.live_stream import emit
            emit("check_start", check_id=cid)
        except Exception:
            pass
        try:
            r = run_check(cid)
            return (cid, r)
        except Exception as e:
            return (cid, {"ok": False, "check_id": cid, "message": str(e)})

    result_by_id: Dict[str, Dict] = {}
    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        futures = {ex.submit(_run_one, cid): cid for cid in check_ids}
        for future in as_completed(futures):
            if time.perf_counter() - t0 > timeout_sec:
                break
            cid, r = future.result()
            result_by_id[cid] = r
            try:
                from modules.humanoid.ans.live_stream import emit
                emit("check_end", check_id=cid, ok=r.get("ok"), message=r.get("message", "")[:80])
            except Exception:
                pass

    # Fase 2: procesar resultados secuencialmente (incidentes + heals)
    for cid in check_ids:
        try:
            r = result_by_id.get(cid)
            if r is None:
                continue
            results.append(r)
            if not r.get("ok"):
                fp = f"{cid}:{r.get('message', '')[:50]}"
                evidence = capture_evidence(cid, r.get("details", {}))
                inc_id = create_incident(cid, fp, r.get("severity", "med"), r.get("message", ""), evidence, r.get("suggested_heals", []))
                incidents_created.append(inc_id)
                human_msg = (r.get("details") or {}).get("human_intervention_required")
                if human_msg:
                    try:
                        from modules.humanoid.ans.live_stream import emit
                        emit("alert", check_id=cid, message=human_msg, details={"human_intervention": True})
                    except Exception:
                        pass
                    try:
                        notify_telegram(human_msg, "med")
                    except Exception:
                        pass
                try:
                    from modules.humanoid.ans.live_stream import emit
                    emit("incident", check_id=cid, message=r.get("message", "")[:80], details={"inc_id": inc_id, "severity": r.get("severity")})
                except Exception:
                    pass
                heals = list(r.get("suggested_heals") or [])
                if not heals:
                    heals = [h for h in (CHECK_DEFAULT_HEALS.get(cid) or []) if h in SAFE_HEALS]
                if not heals:
                    omit_msg = human_msg or "sin heal disponible para este check"
                    add_action(inc_id, "(omitido)", False, omit_msg)
                    try:
                        from modules.humanoid.ans.live_stream import emit
                        emit("skip", check_id=cid, message=omit_msg[:80])
                    except Exception:
                        pass
                for heal_id in heals:
                    if heal_id not in SAFE_HEALS:
                        add_action(inc_id, "(omitido)", False, f"heal no seguro: {heal_id}")
                        try:
                            from modules.humanoid.ans.live_stream import emit
                            emit("skip", check_id=cid, heal_id=heal_id, message="heal no seguro")
                        except Exception:
                            pass
                        continue
                    allowed, reason = can_auto_action(heal_id)
                    if not allowed:
                        # No registrar cooldown como acción para evitar spam en bitácora
                        # Solo emitir al live_stream para debug
                        try:
                            from modules.humanoid.ans.live_stream import emit
                            emit("skip", check_id=cid, heal_id=heal_id, message=f"límite: {reason}")
                        except Exception:
                            pass
                        continue
                    try:
                        from modules.humanoid.governance.gates import decide
                        d = decide("ans_heal")
                        if d.blocked_by_emergency:
                            from modules.humanoid.governance.audit import audit_blocked
                            audit_blocked("ans_heal", "emergency_stop_block")
                            add_action(inc_id, "(omitido)", False, "emergency_stop activo")
                            continue
                        if not _ans_bypass_governance() and d.needs_approval and not d.allow:
                            add_action(inc_id, "(omitido)", False, "modo governed - cambiar a GROWTH para auto-ejecutar")
                            try:
                                from modules.humanoid.ans.live_stream import emit
                                emit("skip", check_id=cid, heal_id=heal_id, message="governed: requiere GROWTH")
                            except Exception:
                                pass
                            continue
                    except Exception:
                        pass
                    try:
                        policy_ok = get_policy_engine().can(ActorContext(actor="ans", role="system"), "ans", "ans_autofix", heal_id).allow
                        if not policy_ok:
                            add_action(inc_id, "(omitido)", False, f"policy bloqueó: {heal_id}")
                            try:
                                from modules.humanoid.ans.live_stream import emit
                                emit("skip", check_id=cid, heal_id=heal_id, message="policy bloqueó")
                            except Exception:
                                pass
                            continue
                    except Exception:
                        policy_ok = True
                    try:
                        from modules.humanoid.ans.live_stream import emit
                        emit("heal_attempt", check_id=cid, heal_id=heal_id)
                    except Exception:
                        pass
                    hr = run_heal(heal_id)
                    try:
                        from modules.humanoid.ans.live_stream import emit
                        emit("heal_end", check_id=cid, heal_id=heal_id, ok=hr.get("ok"), message=(hr.get("message") or "")[:80])
                    except Exception:
                        pass
                    record_action()
                    add_action(inc_id, heal_id, hr.get("ok", False), hr.get("message", ""))
                    actions_taken.append({"heal_id": heal_id, "ok": hr.get("ok"), "message": hr.get("message")})
                    if hr.get("ok"):
                        resolve_incident(inc_id, [{"heal_id": heal_id, "ok": True, "message": hr.get("message")}])
                        break
        except Exception as e:
            results.append({"ok": False, "check_id": cid, "message": str(e)})

    open_incidents = [i for i in get_incidents(status="open") if i["id"] in incidents_created]
    for inc in open_incidents[:5]:
        if inc.get("severity") in ("high", "critical"):
            notify_telegram(f"ANS: {inc.get('check_id')} - {inc.get('message')}", inc.get("severity", "med"))

    summary = generate_summary(open_incidents, actions_taken)
    # MTTR (autonomía): incluir estadística de recuperación reciente en el reporte.
    try:
        from modules.humanoid.ans.mttr import format_stats_human
        summary = (summary or "").strip() + ("\n" if summary else "") + format_stats_human(hours=int(os.getenv("ANS_MTTR_HOURS", "24") or 24))
    except Exception:
        pass
    diagnosis = {}
    try:
        from modules.humanoid.brain.ans_diagnoser import diagnose_with_self_model
        from modules.humanoid.self_model import get_manifest
        diagnosis = diagnose_with_self_model(open_incidents, actions_taken, get_manifest())
    except Exception:
        pass
    report_path = write_report(open_incidents, actions_taken, summary, diagnosis=diagnosis)
    try:
        from modules.humanoid.ans.live_stream import emit
        emit("cycle_end", message=f"issues={len([r for r in results if not r.get('ok')])} actions={len(actions_taken)} ms={int((time.perf_counter()-t0)*1000)}")
    except Exception:
        pass

    ms = int((time.perf_counter() - t0) * 1000)
    return {
        "ok": True,
        "mode": _env("ANS_MODE", "auto"),
        "results": results,
        "issues_count": len([r for r in results if not r.get("ok")]),
        "actions_taken": actions_taken,
        "incidents_created": len(incidents_created),
        "report_path": report_path,
        "diagnosis": diagnosis,
        "ms": ms,
    }


def get_ans_status() -> Dict[str, Any]:
    enabled = _env_bool("ANS_ENABLED", True)
    mode = _env("ANS_MODE", "auto")
    interval = _env_int("ANS_INTERVAL_SECONDS", 30)
    try:
        from modules.humanoid.ans.incident import get_incidents
        open_inc = get_incidents(status="open", limit=20)
        closed_inc = get_incidents(status="resolved", limit=5)
    except Exception:
        open_inc = []
        closed_inc = []
    try:
        from modules.humanoid.ans.reporter import get_latest_report
        report_path = get_latest_report()
    except Exception:
        report_path = ""
    return {
        "ok": True,
        "enabled": enabled,
        "mode": mode,
        "interval_seconds": interval,
        "incidents_open": len(open_inc),
        "incidents_closed": len(closed_inc),
        "report_path": report_path,
    }
