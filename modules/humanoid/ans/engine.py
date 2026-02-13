"""ANS engine: run checks -> detect issues -> plan -> heal -> verify -> report."""
from __future__ import annotations

import os
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

SAFE_HEALS = {"clear_stale_locks", "restart_scheduler", "fallback_models", "tune_router", "rotate_logs", "retry_gateway_bootstrap", "mark_node_offline", "regenerate_support_bundle"}


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
    from modules.humanoid.ans.checks import _register_all
    _register_all()
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

    for cid in list(get_checks().keys())[:14]:
        if time.perf_counter() - t0 > timeout_sec:
            break
        try:
            r = run_check(cid)
            results.append(r)
            if not r.get("ok"):
                fp = f"{cid}:{r.get('message', '')[:50]}"
                evidence = capture_evidence(cid, r.get("details", {}))
                inc_id = create_incident(cid, fp, r.get("severity", "med"), r.get("message", ""), evidence, r.get("suggested_heals", []))
                incidents_created.append(inc_id)
                heals = r.get("suggested_heals", [])
                for heal_id in heals:
                    if heal_id not in SAFE_HEALS:
                        continue
                    allowed, reason = can_auto_action(heal_id)
                    if not allowed:
                        continue
                    try:
                        from modules.humanoid.governance.gates import decide
                        d = decide("ans_heal")
                        if d.blocked_by_emergency:
                            from modules.humanoid.governance.audit import audit_blocked
                            audit_blocked("ans_heal", "emergency_stop_block")
                            continue
                        if d.needs_approval and not d.allow:
                            continue
                    except Exception:
                        pass
                    try:
                        policy_ok = get_policy_engine().can(ActorContext(actor="ans", role="system"), "ans", "ans_autofix", heal_id).allow
                        if not policy_ok:
                            continue
                    except Exception:
                        policy_ok = True
                    hr = run_heal(heal_id)
                    record_action()
                    add_action(inc_id, heal_id, hr.get("ok", False), hr.get("message", ""))
                    actions_taken.append({"heal_id": heal_id, "ok": hr.get("ok"), "message": hr.get("message")})
                    if hr.get("ok"):
                        resolve_incident(inc_id, [{"heal_id": heal_id, "ok": True, "message": hr.get("message")}])
                        break
        except Exception as e:
            results.append({"ok": False, "check_id": "engine", "message": str(e)})

    open_incidents = [i for i in get_incidents(status="open") if i["id"] in incidents_created]
    for inc in open_incidents[:5]:
        if inc.get("severity") in ("high", "critical"):
            notify_telegram(f"ANS: {inc.get('check_id')} - {inc.get('message')}", inc.get("severity", "med"))

    summary = generate_summary(open_incidents, actions_taken)
    report_path = write_report(open_incidents, actions_taken, summary)

    ms = int((time.perf_counter() - t0) * 1000)
    return {
        "ok": True,
        "mode": _env("ANS_MODE", "auto"),
        "results": results,
        "issues_count": len([r for r in results if not r.get("ok")]),
        "actions_taken": actions_taken,
        "incidents_created": len(incidents_created),
        "report_path": report_path,
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
