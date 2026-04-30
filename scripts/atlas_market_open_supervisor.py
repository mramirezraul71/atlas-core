from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional
from zoneinfo import ZoneInfo

import requests

BASE_PUSH = os.getenv("ATLAS_PUSH_BASE", "http://127.0.0.1:8791")
BASE_NEXUS = os.getenv("ATLAS_NEXUS_BASE", "http://127.0.0.1:8000")
BASE_ROBOT = os.getenv("ATLAS_ROBOT_BASE", "http://127.0.0.1:8002")
BASE_QUANT = os.getenv("ATLAS_QUANT_BASE", "http://127.0.0.1:8795")
REPO_ROOT = Path(os.getenv("ATLAS_BASE", Path(__file__).resolve().parents[1]))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
NY_TZ = ZoneInfo("America/New_York")
REQUEST_TIMEOUT = 6.0


@dataclass
class CheckResult:
    name: str
    ok: bool
    severity: str
    summary: str
    details: Dict[str, Any]


def _now_ny() -> datetime:
    return datetime.now(NY_TZ)


def _http_json(url: str, method: str = "GET", payload: Optional[Dict[str, Any]] = None, timeout: float = REQUEST_TIMEOUT) -> Dict[str, Any]:
    try:
        if method.upper() == "POST":
            response = requests.post(url, json=payload or {}, timeout=timeout)
        else:
            response = requests.get(url, timeout=timeout)
        try:
            data = response.json()
        except Exception:
            data = {"raw_text": response.text[:1000]}
        if isinstance(data, dict):
            data.setdefault("_status_code", response.status_code)
            return data
        return {"data": data, "_status_code": response.status_code}
    except Exception as exc:
        return {"_error": str(exc)}


def _read_json_file(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _fault_manager_snapshot() -> Dict[str, Any]:
    latest = _read_json_file(REPO_ROOT / "state" / "atlas_fault_manager_latest.json")
    snapshot = _read_json_file(REPO_ROOT / "state" / "atlas_fault_snapshot.json")
    if latest or snapshot:
        return {"latest": latest, "snapshot": snapshot}
    return {"_error": "fault manager state files unavailable"}


def _push_check() -> CheckResult:
    health = _http_json(f"{BASE_PUSH}/health")
    status = _http_json(f"{BASE_PUSH}/status")
    if "_error" in health and "_error" in status:
        return CheckResult(
            "push",
            False,
            "critical",
            f"PUSH no responde ({health.get('_error') or status.get('_error')})",
            {"health": health, "status": status},
        )
    return CheckResult(
        "push",
        True,
        "ok",
        f"PUSH OK en {BASE_PUSH}",
        {"health": health, "status": status},
    )


def _nexus_check() -> CheckResult:
    health = _http_json(f"{BASE_NEXUS}/health")
    root = _http_json(f"{BASE_NEXUS}/")
    if "_error" in health and "_error" in root:
        return CheckResult(
            "nexus",
            False,
            "critical",
            f"NEXUS no responde ({health.get('_error') or root.get('_error')})",
            {"health": health, "root": root},
        )
    return CheckResult(
        "nexus",
        True,
        "ok",
        f"NEXUS OK en {BASE_NEXUS}",
        {"health": health, "root": root},
    )


def _robot_check() -> CheckResult:
    health = _http_json(f"{BASE_ROBOT}/api/health")
    status = _http_json(f"{BASE_ROBOT}/status")
    if "_error" in health and "_error" in status:
        return CheckResult(
            "robot",
            False,
            "critical",
            f"Robot no responde ({health.get('_error') or status.get('_error')})",
            {"health": health, "status": status},
        )
    actuators = None
    if isinstance(status.get("data"), dict):
        actuators = status["data"].get("actuators")
    severity = "warning" if actuators is False else "ok"
    summary = "Robot OK"
    if actuators is False:
        summary = "Robot online con actuadores desactivados"
    return CheckResult(
        "robot",
        True,
        severity,
        summary,
        {"health": health, "status": status},
    )


def _quant_check() -> CheckResult:
    health = _http_json(f"{BASE_QUANT}/health")
    loop = _http_json(f"{BASE_QUANT}/operation/loop/status")
    paper = _http_json(f"{BASE_QUANT}/paper/account")
    supervisor = _http_json(f"{BASE_QUANT}/supervisor/status")
    op_status = _http_json(f"{BASE_QUANT}/operation/status")
    if "_error" in health or int(health.get("_status_code") or 0) >= 500:
        return CheckResult(
            "quant",
            False,
            "critical",
            f"Quant no responde ({health.get('_error')})",
            {"health": health, "loop": loop, "paper": paper, "supervisor": supervisor, "operation_status": op_status},
        )
    severity = "ok"
    notes: List[str] = []
    loop_status = int(loop.get("_status_code") or 0)
    paper_status = int(paper.get("_status_code") or 0)
    op_status_code = int(op_status.get("_status_code") or 0)
    supervisor_status = int(supervisor.get("_status_code") or 0)
    loop_data = loop.get("data", {}) if isinstance(loop.get("data"), dict) else {}
    if loop_status == 401:
        notes.append("loop protegido por auth")
    elif loop_data and not loop_data.get("active", False):
        severity = "warning"
        notes.append("loop inactivo")
    if paper_status == 401:
        notes.append("paper account protegido por auth")
    op_payload = op_status.get("data", {}) if isinstance(op_status.get("data"), dict) else op_status
    config = op_payload.get("config", {}) if isinstance(op_payload, dict) else {}
    if op_status_code == 401:
        notes.append("operation status protegido por auth")
    elif config and config.get("kill_switch_active"):
        severity = "warning"
        notes.append("kill switch activo")
    if supervisor_status == 404:
        notes.append("supervisor sin endpoint dedicado")
    account = paper.get("data", {}) if isinstance(paper.get("data"), dict) else {}
    equity = account.get("total_equity", account.get("initial_capital"))
    summary = "Quant OK"
    if equity is not None:
        summary += f" | paper equity ${equity:,.0f}"
    if notes:
        summary += " | " + ", ".join(notes)
    return CheckResult(
        "quant",
        True,
        severity,
        summary,
        {"health": health, "loop": loop, "paper": paper, "supervisor": supervisor, "operation_status": op_status},
    )


def _scheduler_check() -> CheckResult:
    jobs = _http_json(f"{BASE_PUSH}/scheduler/jobs")
    if "_error" in jobs:
        return CheckResult(
            "scheduler",
            False,
            "critical",
            f"Scheduler no responde ({jobs.get('_error')})",
            {"jobs": jobs},
        )
    payload = jobs.get("data", {}) if isinstance(jobs.get("data"), dict) else jobs
    counts = payload.get("counts", {}) if isinstance(payload, dict) else {}
    queued = counts.get("queued")
    running = counts.get("running")
    failed = counts.get("failed")
    severity = "warning" if (failed or 0) > 0 else "ok"
    summary = f"Scheduler OK | queued={queued} running={running} failed={failed}"
    return CheckResult("scheduler", True, severity, summary, {"jobs": jobs})


def _fault_check() -> CheckResult:
    snapshot = _fault_manager_snapshot()
    if "_error" in snapshot:
        return CheckResult(
            "fault_manager",
            False,
            "warning",
            f"Fault Manager sin snapshot ({snapshot.get('_error')})",
            {"snapshot": snapshot},
        )
    latest = snapshot.get("latest", {}) if isinstance(snapshot.get("latest"), dict) else {}
    latest_after = latest.get("after", {}) if isinstance(latest.get("after"), dict) else {}
    counts = latest_after.get("severity_counts", {}) if isinstance(latest_after.get("severity_counts"), dict) else {}
    if not counts:
        snap = snapshot.get("snapshot", {}) if isinstance(snapshot.get("snapshot"), dict) else {}
        counts = snap.get("severity_counts", {}) if isinstance(snap.get("severity_counts"), dict) else {}
    degraded = int(counts.get("degraded") or 0)
    critical = int(counts.get("critical") or 0)
    emergency = int(counts.get("emergency") or 0)
    severity = "ok"
    if emergency > 0 or critical > 0:
        severity = "critical"
    elif degraded > 0:
        severity = "warning"
    summary = f"Fault Manager | degraded={degraded} critical={critical} emergency={emergency}"
    return CheckResult("fault_manager", True, severity, summary, {"snapshot": snapshot})


def _overall_severity(results: List[CheckResult]) -> str:
    weights = {"ok": 0, "warning": 1, "degraded": 1, "critical": 2, "emergency": 3}
    highest = max((weights.get(r.severity, 0) for r in results), default=0)
    reverse = {0: "ok", 1: "warning", 2: "critical", 3: "emergency"}
    return reverse.get(highest, "warning")


def _format_message(label: str, results: List[CheckResult], overall: str) -> str:
    now_local = _now_ny().strftime("%Y-%m-%d %H:%M:%S %Z")
    header = f"[ATLAS OPEN SUPERVISOR][{label}] {now_local} | severity={overall}"
    lines = [header]
    for result in results:
        icon = "OK" if result.severity == "ok" else result.severity.upper()
        lines.append(f"- {result.name}: {icon} | {result.summary}")
    return "\n".join(lines)


def _emit_bitacora(message: str, ok: bool, source: str) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log

        append_evolution_log(message=message, ok=ok, source=source)
    except Exception:
        pass


def _persist_report(report: Dict[str, Any]) -> None:
    try:
        state_dir = REPO_ROOT / "state"
        logs_dir = REPO_ROOT / "logs"
        state_dir.mkdir(parents=True, exist_ok=True)
        logs_dir.mkdir(parents=True, exist_ok=True)
        latest_path = state_dir / "atlas_market_open_supervisor_latest.json"
        history_path = logs_dir / "atlas_market_open_supervisor_runs.jsonl"
        latest_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        with history_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(report, ensure_ascii=False) + "\n")
    except Exception:
        pass


def _raw_telegram_send(text: str) -> tuple[bool, str]:
    try:
        from modules.humanoid.config.vault import load_vault_env
        from modules.humanoid.notify import _cached_chat_id

        load_vault_env()
        token = (
            os.getenv("TELEGRAM_BOT_TOKEN")
            or os.getenv("TELEGRAM_TOKEN")
            or ""
        ).strip()
        chat_id = (_cached_chat_id() or "").strip()
        if not token:
            return False, "telegram token missing"
        if not chat_id:
            return False, "telegram chat_id missing"
        body = json.dumps(
            {
                "chat_id": chat_id,
                "text": (text or "")[:3500],
                "disable_web_page_preview": True,
            }
        ).encode("utf-8")
        req = urllib.request.Request(
            f"https://api.telegram.org/bot{token}/sendMessage",
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=20) as response:
            payload = json.loads(response.read().decode("utf-8", errors="replace"))
        return bool(payload.get("ok")), "" if payload.get("ok") else str(payload)
    except Exception as exc:
        return False, str(exc)


def _emit_telegram_sync(message: str) -> tuple[bool, Optional[str]]:
    try:
        from modules.humanoid.notify import send_telegram
        from modules.humanoid.notify import _cached_chat_id
        from modules.humanoid.comms.telegram_bridge import TelegramBridge

        for _ in range(2):
            if bool(asyncio.run(send_telegram(message))):
                return True, None
            time.sleep(1.0)
        chat_id = _cached_chat_id()
        if chat_id:
            out = TelegramBridge().send(chat_id, message[:3500], parse_html=False)
            if bool(out and out.get("ok")):
                return True, None
            bridge_err = str((out or {}).get("error") or "telegram bridge send failed")
        else:
            bridge_err = "telegram chat_id unavailable"
        ok, raw_err = _raw_telegram_send(message)
        if ok:
            return True, None
        return False, raw_err or bridge_err
    except Exception as exc:
        ok, raw_err = _raw_telegram_send(message)
        if ok:
            return True, None
        return False, raw_err or str(exc)


def run_supervision(label: str, emit_bitacora: bool, emit_telegram: bool) -> Dict[str, Any]:
    results = [
        _push_check(),
        _nexus_check(),
        _robot_check(),
        _quant_check(),
        _scheduler_check(),
        _fault_check(),
    ]
    overall = _overall_severity(results)
    message = _format_message(label=label, results=results, overall=overall)
    if emit_bitacora:
        _emit_bitacora(
            message=message,
            ok=overall not in {"critical", "emergency"},
            source="market_open_supervisor",
        )
    telegram_ok = None
    telegram_error = None
    if emit_telegram:
        telegram_ok, telegram_error = _emit_telegram_sync(message)
    report = {
        "ok": overall not in {"critical", "emergency"},
        "label": label,
        "generated_at": _now_ny().isoformat(),
        "overall_severity": overall,
        "message": message,
        "telegram_ok": telegram_ok,
        "telegram_error": telegram_error,
        "results": [
            {
                "name": r.name,
                "ok": r.ok,
                "severity": r.severity,
                "summary": r.summary,
                "details": r.details,
            }
            for r in results
        ],
    }
    _persist_report(report)
    return report


def _scheduler_job_name(slot_label: str, date_str: str) -> str:
    return f"atlas_market_open_supervisor_native_{date_str}_{slot_label}"


def _list_scheduler_jobs() -> List[Dict[str, Any]]:
    payload = _http_json(f"{BASE_PUSH}/scheduler/jobs")
    if "_error" in payload:
        raise RuntimeError(payload["_error"])
    data = payload.get("data")
    if isinstance(data, list):
        return data
    if isinstance(data, dict):
        jobs = data.get("jobs", [])
        return jobs if isinstance(jobs, list) else []
    return []


def _create_scheduler_job(name: str, run_at_iso: str, slot_label: str) -> Dict[str, Any]:
    payload = {
        "name": name,
        "kind": "market_open_supervisor",
        "payload": {
            "label": slot_label,
            "emit_bitacora": True,
            "emit_telegram": True,
        },
        "run_at": run_at_iso,
        "max_retries": 1,
        "backoff_seconds": 15,
    }
    result = _http_json(f"{BASE_PUSH}/scheduler/job/create", method="POST", payload=payload, timeout=10)
    if "_error" in result:
        raise RuntimeError(result["_error"])
    return result


def schedule_today() -> Dict[str, Any]:
    now_local = _now_ny()
    date_str = now_local.strftime("%Y-%m-%d")
    slots = [
        ("0900", now_local.replace(hour=9, minute=0, second=0, microsecond=0)),
        ("0929", now_local.replace(hour=9, minute=29, second=0, microsecond=0)),
    ]
    jobs = _list_scheduler_jobs()
    existing_names = {str(job.get("name") or "") for job in jobs}
    created: List[Dict[str, Any]] = []
    skipped: List[str] = []
    for slot_label, slot_dt in slots:
        name = _scheduler_job_name(slot_label=slot_label, date_str=date_str)
        if name in existing_names:
            skipped.append(name)
            continue
        run_at_utc = slot_dt.astimezone(timezone.utc).isoformat()
        result = _create_scheduler_job(
            name=name,
            run_at_iso=run_at_utc,
            slot_label=slot_label,
        )
        created.append({"name": name, "run_at": run_at_utc, "result": result})
    refreshed = _list_scheduler_jobs()
    matched = [
        job for job in refreshed if str(job.get("name") or "").startswith(f"atlas_market_open_supervisor_native_{date_str}_")
    ]
    return {
        "ok": True,
        "date": date_str,
        "created": created,
        "skipped": skipped,
        "registered_jobs": matched,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS market-open supervisor")
    parser.add_argument("--mode", choices=["run", "schedule-today"], default="run")
    parser.add_argument("--label", default="manual")
    parser.add_argument("--emit-bitacora", action="store_true")
    parser.add_argument("--emit-telegram", action="store_true")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    if args.mode == "schedule-today":
        result = schedule_today()
    else:
        result = run_supervision(
            label=args.label,
            emit_bitacora=args.emit_bitacora,
            emit_telegram=args.emit_telegram,
        )

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(result.get("message") or json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
