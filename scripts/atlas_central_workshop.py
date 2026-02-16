#!/usr/bin/env python3
"""
ATLAS Central Workshop
----------------------
Central repair workflow for incidents and maintenance.

Concept:
- Receives incidents from ANS (/ans/incidents).
- Moves each incident through workshop trays:
  inbox -> working -> resolved|failed
- Applies the corresponding runbook by module/check.
- Verifies system health after each repair.
- Produces JSON + Markdown report with evidence.

Usage examples:
  python scripts/atlas_central_workshop.py
  python scripts/atlas_central_workshop.py --mode incidents
  python scripts/atlas_central_workshop.py --mode maintenance
  python scripts/atlas_central_workshop.py --limit 30
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib import error, parse, request


REPO_ROOT = Path(__file__).resolve().parent.parent
WORKSHOP_ROOT = REPO_ROOT / "logs" / "workshop"
INBOX_DIR = WORKSHOP_ROOT / "inbox"
WORKING_DIR = WORKSHOP_ROOT / "working"
RESOLVED_DIR = WORKSHOP_ROOT / "resolved"
FAILED_DIR = WORKSHOP_ROOT / "failed"
REPORTS_DIR = WORKSHOP_ROOT / "reports"
MAINTENANCE_DIR = WORKSHOP_ROOT / "maintenance"
APPROVAL_STATE_FILE = WORKSHOP_ROOT / "approval_state.json"

DEFAULT_PUSH_BASE = os.getenv("ATLAS_PUSH_BASE_URL", "http://127.0.0.1:8791").rstrip("/")
DEFAULT_ROBOT_BASE = os.getenv("ATLAS_ROBOT_BASE_URL", "http://127.0.0.1:8002").rstrip("/")


@dataclass
class StepResult:
    name: str
    ok: bool
    detail: str
    elapsed_ms: int


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _safe_slug(text: str) -> str:
    out = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in (text or "x"))
    out = out.strip("_")
    return out[:80] or "x"


def _ensure_dirs() -> None:
    for d in (INBOX_DIR, WORKING_DIR, RESOLVED_DIR, FAILED_DIR, REPORTS_DIR, MAINTENANCE_DIR):
        d.mkdir(parents=True, exist_ok=True)


def _load_approval_state() -> Dict[str, Any]:
    try:
        if APPROVAL_STATE_FILE.is_file():
            return json.loads(APPROVAL_STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        pass
    return {"consumed": [], "last_request_ts": "", "last_request_id": ""}


def _save_approval_state(state: Dict[str, Any]) -> None:
    try:
        APPROVAL_STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        APPROVAL_STATE_FILE.write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass


def _approval_gate_for_heavy(mode: str, require_approval_heavy: bool, approval_cooldown_seconds: int = 900) -> Dict[str, Any]:
    """
    Gate for heavy operations (maintenance/full):
    - If approval is not required: granted immediately.
    - If required:
      * If there is an approved unconsumed request => granted.
      * Else if pending exists => wait.
      * Else create a new pending approval (with cooldown to avoid spam).
    """
    heavy = mode in ("full", "maintenance")
    if not heavy or not require_approval_heavy:
        return {"granted": True, "reason": "not_required"}

    try:
        from modules.humanoid.approvals import list_pending, list_all, create
    except Exception as e:
        return {"granted": False, "reason": f"approval_module_unavailable: {e}", "pending": False}

    marker = {"domain": "workshop", "operation": "maintenance", "source": "scheduler"}
    state = _load_approval_state()
    consumed = set(state.get("consumed") or [])

    approved = list_all(limit=50, status="approved")
    for item in approved:
        payload = item.get("payload") or {}
        if payload.get("domain") == "workshop" and payload.get("operation") == "maintenance":
            aid = str(item.get("id") or "")
            if aid and aid not in consumed:
                consumed.add(aid)
                state["consumed"] = sorted(consumed)[-300:]
                _save_approval_state(state)
                return {"granted": True, "reason": "approved", "approval_id": aid}

    pending = list_pending(limit=50)
    for item in pending:
        payload = item.get("payload") or {}
        if payload.get("domain") == "workshop" and payload.get("operation") == "maintenance":
            return {"granted": False, "reason": "pending", "pending": True, "approval_id": item.get("id")}

    now = time.time()
    try:
        last_ts = state.get("last_request_ts") or ""
        last_epoch = datetime.fromisoformat(last_ts.replace("Z", "+00:00")).timestamp() if last_ts else 0.0
    except Exception:
        last_epoch = 0.0
    if now - last_epoch < max(60, int(approval_cooldown_seconds or 900)):
        return {
            "granted": False,
            "reason": "cooldown",
            "pending": True,
            "approval_id": state.get("last_request_id"),
        }

    payload = {
        **marker,
        "mode": mode,
        "intent": "central_workshop_heavy_run",
        "details": "Ejecutar mantenimiento/reparaciones pesadas del Taller Central",
        "risk": "high",
    }
    out = create(action="execute", payload=payload, job_id="workshop_cycle")
    if out.get("ok") and out.get("approval_id"):
        state["last_request_ts"] = _now_iso()
        state["last_request_id"] = out.get("approval_id")
        _save_approval_state(state)
        return {"granted": False, "reason": "created_pending", "pending": True, "approval_id": out.get("approval_id")}
    return {"granted": False, "reason": out.get("error") or "approval_create_failed", "pending": False}


def _http_json(method: str, url: str, payload: Optional[Dict[str, Any]] = None, timeout: int = 20) -> Tuple[bool, int, Dict[str, Any], str]:
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = request.Request(url, data=data, method=method.upper(), headers=headers)
    try:
        with request.urlopen(req, timeout=timeout) as r:
            raw = r.read().decode("utf-8", "replace")
            try:
                body = json.loads(raw) if raw else {}
            except Exception:
                return False, int(r.status), {}, f"invalid_json: {raw[:200]}"
            return True, int(r.status), body, ""
    except error.HTTPError as e:
        raw = e.read().decode("utf-8", "replace")
        try:
            body = json.loads(raw) if raw else {}
        except Exception:
            body = {"raw": raw[:200]}
        return False, int(e.code), body, f"http_{e.code}"
    except Exception as e:
        return False, 0, {}, f"{type(e).__name__}: {e}"


def _http_status(url: str, timeout: int = 10) -> int:
    req = request.Request(url, method="GET")
    try:
        with request.urlopen(req, timeout=timeout) as r:
            return int(r.status)
    except Exception:
        return 0


def _run_cmd(name: str, cmd: List[str], cwd: Optional[Path] = None, timeout_s: int = 90) -> StepResult:
    t0 = time.time()
    try:
        p = subprocess.run(
            cmd,
            cwd=str(cwd or REPO_ROOT),
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
        detail = ((p.stdout or "") + "\n" + (p.stderr or "")).strip()
        ok = p.returncode == 0
        elapsed = int((time.time() - t0) * 1000)
        return StepResult(name=name, ok=ok, detail=detail[:4000], elapsed_ms=elapsed)
    except Exception as e:
        elapsed = int((time.time() - t0) * 1000)
        return StepResult(name=name, ok=False, detail=f"{type(e).__name__}: {e}", elapsed_ms=elapsed)


def _fetch_open_incidents(push_base: str, limit: int) -> List[Dict[str, Any]]:
    qs = parse.urlencode({"status": "open", "limit": int(limit)})
    ok, _, body, _ = _http_json("GET", f"{push_base}/ans/incidents?{qs}", timeout=20)
    if not ok:
        return []
    data = body.get("data")
    return data if isinstance(data, list) else []


def _incident_ticket_name(incident: Dict[str, Any]) -> str:
    iid = str(incident.get("id") or "noid")
    chk = _safe_slug(str(incident.get("check_id") or "check"))
    return f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{iid}_{chk}.json"


def _write_json(path: Path, obj: Dict[str, Any]) -> None:
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding="utf-8")


def _ingest_incidents(push_base: str, limit: int) -> int:
    incidents = _fetch_open_incidents(push_base=push_base, limit=limit)
    added = 0
    existing_ids = set()
    for d in INBOX_DIR.glob("*.json"):
        try:
            j = json.loads(d.read_text(encoding="utf-8"))
            existing_ids.add(str(j.get("incident", {}).get("id") or ""))
        except Exception:
            continue
    for inc in incidents:
        iid = str(inc.get("id") or "")
        if not iid or iid in existing_ids:
            continue
        ticket = {
            "ticket_created_at": _now_iso(),
            "state": "new",
            "incident": inc,
            "history": [],
        }
        _write_json(INBOX_DIR / _incident_ticket_name(inc), ticket)
        added += 1
    return added


def _select_runbook(check_id: str, message: str) -> str:
    """Selecciona el runbook/POT apropiado para el incidente."""
    # Intentar usar el sistema de POTs del módulo de Calidad
    try:
        from modules.humanoid.quality import get_pot_by_incident
        pot = get_pot_by_incident(check_id=check_id, message=message)
        if pot:
            return pot.id
    except ImportError:
        pass  # Fallback a lógica legacy
    
    # Lógica legacy de selección
    cid = (check_id or "").lower()
    msg = (message or "").lower()
    if "camera" in cid or "camera" in msg:
        return "camera_repair"
    if "nexus" in cid or "robot" in cid or "gateway" in cid:
        return "services_repair"
    if "deps" in cid or "llm" in cid:
        return "dependency_repair"
    if "disk" in cid or "storage" in cid:
        return "disk_maintenance"
    if "api" in cid or "health" in cid:
        return "api_repair"
    return "generic_repair"


def _verify_baseline(push_base: str, robot_base: str) -> Dict[str, Any]:
    return {
        "push_ui": _http_status(f"{push_base}/ui", timeout=8),
        "push_health": _http_status(f"{push_base}/health", timeout=8),
        "robot_health": _http_status(f"{robot_base}/api/health", timeout=8),
    }


def _runbook_camera_repair() -> List[StepResult]:
    return [
        _run_cmd("camera_autorepair", [sys.executable, str(REPO_ROOT / "scripts" / "atlas_camera_autorepair.py")], timeout_s=240),
    ]


def _runbook_services_repair() -> List[StepResult]:
    return [
        _run_cmd(
            "restart_robot",
            ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "robot"],
            timeout_s=90,
        ),
        _run_cmd(
            "restart_push",
            ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "push"],
            timeout_s=120,
        ),
    ]


def _runbook_dependency_repair() -> List[StepResult]:
    return [
        _run_cmd("ans_run_now", [sys.executable, "-c", "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8791/ans/run-now')"], timeout_s=75),
        _run_cmd("repo_hygiene", [sys.executable, str(REPO_ROOT / "scripts" / "repo_hygiene.py")], timeout_s=150),
    ]


def _runbook_disk_maintenance() -> List[StepResult]:
    return [
        _run_cmd(
            "clear_project_cache",
            ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "robot", "-ClearCache"],
            timeout_s=180,
        ),
    ]


def _runbook_api_repair() -> List[StepResult]:
    return [
        _run_cmd(
            "restart_push",
            ["powershell", "-ExecutionPolicy", "Bypass", "-File", str(REPO_ROOT / "scripts" / "restart_service_clean.ps1"), "-Service", "push"],
            timeout_s=120,
        ),
    ]


def _runbook_generic_repair() -> List[StepResult]:
    return [
        _run_cmd("ans_run_now", [sys.executable, "-c", "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8791/ans/run-now')"], timeout_s=75),
    ]


def _execute_runbook(runbook_id: str) -> List[StepResult]:
    """Ejecuta el runbook/POT especificado."""
    # Intentar usar el sistema de POTs del módulo de Calidad
    try:
        from modules.humanoid.quality import get_pot, execute_pot
        pot = get_pot(runbook_id)
        if pot:
            # Ejecutar POT y convertir resultado a StepResults
            result = execute_pot(pot, context={"source": "workshop"})
            steps = []
            for sr in result.step_results:
                steps.append(StepResult(
                    name=sr.step_name,
                    ok=sr.ok,
                    detail=sr.output or sr.error or "",
                    elapsed_ms=sr.elapsed_ms,
                ))
            return steps
    except ImportError:
        pass  # Fallback a lógica legacy
    
    # Lógica legacy de ejecución
    if runbook_id == "camera_repair":
        return _runbook_camera_repair()
    if runbook_id == "services_repair":
        return _runbook_services_repair()
    if runbook_id == "dependency_repair":
        return _runbook_dependency_repair()
    if runbook_id == "disk_maintenance":
        return _runbook_disk_maintenance()
    if runbook_id == "api_repair":
        return _runbook_api_repair()
    return _runbook_generic_repair()


def _process_ticket(path: Path, push_base: str, robot_base: str) -> Dict[str, Any]:
    raw = json.loads(path.read_text(encoding="utf-8"))
    inc = raw.get("incident") or {}
    check_id = str(inc.get("check_id") or "")
    message = str(inc.get("message") or "")
    runbook = _select_runbook(check_id=check_id, message=message)

    working = WORKING_DIR / path.name
    shutil.move(str(path), str(working))

    before = _verify_baseline(push_base=push_base, robot_base=robot_base)
    steps = _execute_runbook(runbook_id=runbook)
    after = _verify_baseline(push_base=push_base, robot_base=robot_base)
    ok_steps = all(s.ok for s in steps) if steps else False
    ok_verify = after["push_ui"] == 200 and after["push_health"] == 200 and after["robot_health"] == 200
    ok_final = bool(ok_steps and ok_verify)

    result = {
        "ticket": path.name,
        "incident_id": inc.get("id"),
        "check_id": check_id,
        "runbook": runbook,
        "before": before,
        "after": after,
        "ok_steps": ok_steps,
        "ok_verify": ok_verify,
        "ok_final": ok_final,
        "steps": [s.__dict__ for s in steps],
        "processed_at": _now_iso(),
    }

    raw["state"] = "resolved" if ok_final else "failed"
    raw.setdefault("history", []).append(result)
    raw["last_processed_at"] = _now_iso()
    _write_json(working, raw)

    dst = (RESOLVED_DIR if ok_final else FAILED_DIR) / working.name
    shutil.move(str(working), str(dst))
    return result


def _run_incident_cycle(push_base: str, robot_base: str, limit: int) -> Dict[str, Any]:
    ingested = _ingest_incidents(push_base=push_base, limit=limit)
    processed: List[Dict[str, Any]] = []
    for p in sorted(INBOX_DIR.glob("*.json")):
        try:
            processed.append(_process_ticket(p, push_base=push_base, robot_base=robot_base))
        except Exception as e:
            processed.append({"ticket": p.name, "ok_final": False, "error": f"{type(e).__name__}: {e}"})
    return {
        "ingested": ingested,
        "processed_count": len(processed),
        "processed": processed,
    }


def _run_maintenance(push_base: str, robot_base: str) -> Dict[str, Any]:
    tasks: List[Dict[str, Any]] = []

    baseline = _verify_baseline(push_base=push_base, robot_base=robot_base)
    tasks.append({"name": "baseline_health", "ok": baseline["push_ui"] == 200 and baseline["robot_health"] == 200, "detail": baseline})

    r = _run_cmd("camera_autorepair", [sys.executable, str(REPO_ROOT / "scripts" / "atlas_camera_autorepair.py")], timeout_s=240)
    tasks.append({"name": r.name, "ok": r.ok, "detail": r.detail, "elapsed_ms": r.elapsed_ms})

    r2 = _run_cmd("check_nexus_ports", [sys.executable, str(REPO_ROOT / "scripts" / "check_nexus_ports.py")], timeout_s=60)
    tasks.append({"name": r2.name, "ok": r2.ok, "detail": r2.detail, "elapsed_ms": r2.elapsed_ms})

    end_health = _verify_baseline(push_base=push_base, robot_base=robot_base)
    tasks.append({"name": "post_health", "ok": end_health["push_ui"] == 200 and end_health["robot_health"] == 200, "detail": end_health})

    ok = all(bool(t.get("ok")) for t in tasks)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    maintenance_path = MAINTENANCE_DIR / f"maintenance_{stamp}.json"
    _write_json(
        maintenance_path,
        {
            "started_at": _now_iso(),
            "tasks": tasks,
            "ok": ok,
        },
    )
    return {"ok": ok, "tasks": tasks, "log_path": str(maintenance_path)}


def _write_report(report: Dict[str, Any]) -> Tuple[str, str]:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = REPORTS_DIR / f"workshop_report_{stamp}.json"
    md_path = REPORTS_DIR / f"workshop_report_{stamp}.md"
    _write_json(json_path, report)

    lines = [
        "# ATLAS Workshop Report",
        "",
        f"- started_at: {report.get('started_at')}",
        f"- ended_at: {report.get('ended_at')}",
        f"- mode: {report.get('mode')}",
        f"- overall_ok: {report.get('overall_ok')}",
        "",
        "## Incident Cycle",
        "",
        f"- ingested: {report.get('incident_cycle', {}).get('ingested', 0)}",
        f"- processed_count: {report.get('incident_cycle', {}).get('processed_count', 0)}",
        "",
        "## Maintenance",
        "",
        f"- ok: {report.get('maintenance', {}).get('ok')}",
        "",
    ]
    md_path.write_text("\n".join(lines), encoding="utf-8")
    return str(json_path), str(md_path)


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS Central Workshop")
    parser.add_argument("--mode", choices=["full", "incidents", "maintenance"], default="full")
    parser.add_argument("--limit", type=int, default=50, help="Max open incidents to ingest from ANS")
    parser.add_argument("--push-base", default=DEFAULT_PUSH_BASE)
    parser.add_argument("--robot-base", default=DEFAULT_ROBOT_BASE)
    parser.add_argument(
        "--require-approval-heavy",
        action="store_true",
        help="Require owner approval for heavy modes (maintenance/full).",
    )
    parser.add_argument(
        "--approval-cooldown-seconds",
        type=int,
        default=int(os.getenv("WORKSHOP_APPROVAL_COOLDOWN_SECONDS", "900") or 900),
        help="Minimum seconds between new approval requests.",
    )
    args = parser.parse_args()

    _ensure_dirs()
    report: Dict[str, Any] = {
        "started_at": _now_iso(),
        "mode": args.mode,
        "push_base": args.push_base,
        "robot_base": args.robot_base,
        "require_approval_heavy": bool(args.require_approval_heavy),
        "incident_cycle": {},
        "maintenance": {},
        "approval": {},
    }

    gate = _approval_gate_for_heavy(
        mode=args.mode,
        require_approval_heavy=bool(args.require_approval_heavy),
        approval_cooldown_seconds=int(args.approval_cooldown_seconds or 900),
    )
    report["approval"] = gate
    if not gate.get("granted", False) and args.mode in ("full", "maintenance"):
        report["ended_at"] = _now_iso()
        report["overall_ok"] = True
        report["result"] = "awaiting_approval"
        json_report, md_report = _write_report(report)
        print("WORKSHOP_DONE:", "AWAITING_APPROVAL")
        print("MODE:", args.mode)
        print("APPROVAL_REASON:", gate.get("reason"))
        print("APPROVAL_ID:", gate.get("approval_id"))
        print("REPORT_JSON:", json_report)
        print("REPORT_MD:", md_report)
        return 0

    if args.mode in ("full", "incidents"):
        report["incident_cycle"] = _run_incident_cycle(push_base=args.push_base, robot_base=args.robot_base, limit=args.limit)
    if args.mode in ("full", "maintenance"):
        report["maintenance"] = _run_maintenance(push_base=args.push_base, robot_base=args.robot_base)

    report["ended_at"] = _now_iso()
    incident_ok = True
    if report.get("incident_cycle", {}).get("processed"):
        incident_ok = all(bool(x.get("ok_final", False)) for x in report["incident_cycle"]["processed"])
    maintenance_ok = report.get("maintenance", {}).get("ok", True)
    report["overall_ok"] = bool(incident_ok and maintenance_ok)

    json_report, md_report = _write_report(report)
    print("WORKSHOP_DONE:", "OK" if report["overall_ok"] else "PARTIAL")
    print("MODE:", args.mode)
    print("REPORT_JSON:", json_report)
    print("REPORT_MD:", md_report)
    print("INCIDENTS_INGESTED:", report.get("incident_cycle", {}).get("ingested", 0))
    print("INCIDENTS_PROCESSED:", report.get("incident_cycle", {}).get("processed_count", 0))
    return 0 if report["overall_ok"] else 2


if __name__ == "__main__":
    sys.exit(main())

