#!/usr/bin/env python3
"""
ATLAS Fault Snapshot

Normaliza varias fuentes de diagnóstico ya existentes en ATLAS hacia el contrato
del Fault Manager:

- ANS checks registrados
- autonomous.health_monitor.HealthAggregator
- autonomous.health_monitor.ServiceHealth

Produce:
- state/atlas_fault_snapshot.json
- state/atlas_fault_events_latest.json

Y emite un resumen a Bitacora / Telegram según severidad.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import uuid
import urllib.request
from dataclasses import asdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List

import psutil

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autonomous.health_monitor import HealthAggregator, ServiceHealth
from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.ans.evolution_bitacora import get_evolution_entries
from modules.humanoid.ans.registry import get_checks
from modules.humanoid.notify import send_telegram
from modules.humanoid.scheduler.db import SchedulerDB

# trigger check registration side effects
import modules.humanoid.ans.checks  # noqa: F401


STATE_DIR = REPO_ROOT / "state"
SNAPSHOT_PATH = STATE_DIR / "atlas_fault_snapshot.json"
EVENTS_PATH = STATE_DIR / "atlas_fault_events_latest.json"

CHECK_DOMAIN_MAP = {
    "api_health": "gateway_core",
    "gateway_health": "gateway_core",
    "router_health": "gateway_core",
    "ui_health": "workspace_core",
    "scheduler_health": "operations_core",
    "deploy_health": "operations_core",
    "deps_health": "operations_core",
    "disk_health": "runtime_core",
    "logs_health": "runtime_core",
    "evolution_health": "autonomy_core",
    "nervous_health": "autonomy_core",
    "memory_health": "memory_core",
    "audit_health": "runtime_core",
    "llm_health": "cognitive_core",
    "nexus_services_health": "cognitive_core",
    "robot_camera_health": "robotics_core",
    "cluster_health": "integration_core",
    "cge_health": "cognitive_core",
}

SERVICE_DOMAIN_MAP = {
    "push": "gateway_core",
    "nexus": "cognitive_core",
    "robot": "robotics_core",
}

SCHEDULER_DOMAIN_MAP = {
    "nervous_cycle": "autonomy_core",
    "ans_cycle": "autonomy_core",
    "makeplay_scanner": "integration_core",
    "repo_hygiene_cycle": "operations_core",
    "repo_monitor_cycle": "operations_core",
    "repo_monitor_after_fix": "operations_core",
}

FATAL_SCHEDULER_ERROR_SNIPPETS = (
    "No module named",
    "not found",
    "strict: no matching allow rule",
)
STALE_FAILED_DAYS = 14


def _http_json(url: str, timeout: float = 5.0) -> Dict[str, Any] | None:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def _scheduler_jobs_payload() -> Dict[str, Any]:
    payload = _http_json("http://127.0.0.1:8791/scheduler/jobs", timeout=6.0)
    if isinstance(payload, dict):
        return {"source": "http", "jobs": payload.get("data") or []}
    try:
        jobs = SchedulerDB().list_jobs(limit=1000) or []
        return {"source": "db_fallback", "jobs": jobs}
    except Exception:
        return {"source": "unavailable", "jobs": []}


def _latest_scheduler_run(job_id: str) -> Dict[str, Any] | None:
    try:
        runs = SchedulerDB().get_runs(job_id, limit=1) or []
        return runs[0] if runs else None
    except Exception:
        return None


def _is_recent_semantic_completion(job: Dict[str, Any], run: Dict[str, Any] | None) -> bool:
    if not run:
        return False
    if str(job.get("kind") or "").strip().lower() != "nervous_cycle":
        return False
    if str((run or {}).get("error") or "").strip():
        return False
    result = (run or {}).get("result") or {}
    if not isinstance(result, dict):
        return False
    if "score" not in result or "points" not in result:
        return False
    # Para nervous_cycle, esto ya prueba que el ciclo terminó semánticamente y
    # generó salida estructurada. Si el job quedó en failed sin error textual,
    # es una desincronización de estado del scheduler, no un crash real.
    return True


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _parse_iso(ts: Any) -> datetime | None:
    raw = str(ts or "").strip()
    if not raw:
        return None
    try:
        if raw.endswith("Z"):
            raw = raw[:-1] + "+00:00"
        parsed = datetime.fromisoformat(raw)
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=timezone.utc)
        return parsed.astimezone(timezone.utc)
    except Exception:
        return None


def _is_stale_failed_scheduler_job(job: Dict[str, Any]) -> bool:
    if str(job.get("kind") or "").strip().lower() == "nervous_cycle":
        return False
    if str(job.get("status") or "").strip().lower() != "failed":
        return False
    if job.get("next_run_ts"):
        return False
    ts = _parse_iso(job.get("updated_ts")) or _parse_iso(job.get("last_run_ts"))
    if ts is None:
        return False
    return (datetime.now(timezone.utc) - ts) >= timedelta(days=STALE_FAILED_DAYS)


def _severity_from_check(raw: str, ok: bool) -> str:
    sev = (raw or "").strip().lower()
    if ok:
        return "info"
    if sev in {"low", "leve"}:
        return "warning"
    if sev in {"med", "medium", "moderada", "moderado"}:
        return "degraded"
    if sev in {"high", "alta", "severa", "severe"}:
        return "critical"
    return "critical"


def _recoverability(suggested_heals: List[str]) -> str:
    if not suggested_heals:
        return "manual_required"
    if len(suggested_heals) == 1:
        return "auto"
    return "bounded_auto"


def _build_event(
    *,
    component_id: str,
    domain: str,
    severity: str,
    fault_code: str,
    symptom: str,
    evidence: List[str] | None = None,
    recoverability: str = "manual_required",
    status: str = "detected",
    recommended_action: str = "",
    dependencies: List[str] | None = None,
    telegram_required: bool = False,
    metadata: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    return {
        "event_id": uuid.uuid4().hex[:16],
        "timestamp": _now_iso(),
        "component_id": component_id,
        "domain": domain,
        "severity": severity,
        "fault_code": fault_code,
        "symptom": symptom,
        "evidence": evidence or [],
        "recoverability": recoverability,
        "status": status,
        "recommended_action": recommended_action,
        "dependencies": dependencies or [],
        "snapshot_refs": [],
        "bitacora_required": True,
        "telegram_required": telegram_required,
        "metadata": metadata or {},
    }


def _run_ans_checks() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    for check_id, fn in sorted(get_checks().items()):
        try:
            result = fn() or {}
        except Exception as exc:
            result = {
                "ok": False,
                "check_id": check_id,
                "message": str(exc),
                "details": {"error": str(exc)},
                "severity": "high",
                "suggested_heals": [],
            }
        ok = bool(result.get("ok", False))
        raw_sev = result.get("severity", "low")
        severity = _severity_from_check(raw_sev, ok)
        if ok:
            continue
        suggested_heals = list(result.get("suggested_heals") or [])
        event = _build_event(
            component_id=result.get("check_id", check_id),
            domain=CHECK_DOMAIN_MAP.get(check_id, "unclassified"),
            severity=severity,
            fault_code=f"check.{check_id}",
            symptom=result.get("message", "check failed"),
            evidence=[json.dumps(result.get("details", {}), ensure_ascii=False)[:500]],
            recoverability=_recoverability(suggested_heals),
            recommended_action=", ".join(suggested_heals),
            telegram_required=severity in {"critical", "emergency"},
            metadata={"source": "ans_check", "raw": result},
        )
        events.append(event)
    return events


def _run_service_health() -> List[Dict[str, Any]]:
    svc = ServiceHealth()
    statuses = svc.get_all_services_status()
    events: List[Dict[str, Any]] = []
    for name, status in statuses.items():
        if status.online:
            continue
        severity = "critical" if name in {"push", "nexus"} else "degraded"
        event = _build_event(
            component_id=f"service.{name}",
            domain=SERVICE_DOMAIN_MAP.get(name, "unclassified"),
            severity=severity,
            fault_code="service.offline",
            symptom=f"{name} offline or unhealthy",
            evidence=[
                f"url={status.url}",
                f"latency_ms={status.latency_ms}",
                f"status_code={status.status_code}",
                f"error={status.error}",
            ],
            recoverability="bounded_auto",
            recommended_action="restart_service, verify port, recheck health endpoint",
            telegram_required=severity in {"critical", "emergency"},
            metadata={"source": "service_health", "status": asdict(status)},
        )
        events.append(event)
    return events


def _run_gateway_binding_health() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    listeners: List[Dict[str, Any]] = []
    try:
        for conn in psutil.net_connections(kind="tcp"):
            laddr = getattr(conn, "laddr", None)
            if not laddr:
                continue
            if int(getattr(laddr, "port", 0) or 0) != 8791:
                continue
            if str(getattr(conn, "status", "")).upper() != "LISTEN":
                continue
            listeners.append(
                {
                    "address": str(getattr(laddr, "ip", "") or ""),
                    "port": int(getattr(laddr, "port", 0) or 0),
                    "pid": int(conn.pid or 0),
                }
            )
    except Exception as exc:
        events.append(
            _build_event(
                component_id="gateway.push_listener",
                domain="gateway_core",
                severity="warning",
                fault_code="gateway.listener_snapshot_unavailable",
                symptom=f"Could not inspect local listeners on 8791: {exc}",
                recoverability="manual_required",
                recommended_action="inspect netstat / process table for PUSH listener ownership",
                metadata={"source": "listener_probe", "error": str(exc)},
            )
        )
        return events

    unique_pids = sorted({item["pid"] for item in listeners if item["pid"] > 0})
    unique_addresses = sorted({item["address"] for item in listeners if item["address"]})
    if len(unique_pids) <= 1:
        return events

    severity = "critical" if "0.0.0.0" in unique_addresses and "127.0.0.1" in unique_addresses else "degraded"
    events.append(
        _build_event(
            component_id="gateway.push_listener",
            domain="gateway_core",
            severity=severity,
            fault_code="gateway.listener_conflict_8791",
            symptom="Multiple processes are listening on port 8791 with different bindings",
            evidence=[
                f"listener_count={len(listeners)}",
                f"pids={','.join(str(pid) for pid in unique_pids)}",
                f"addresses={','.join(unique_addresses)}",
            ],
            recoverability="manual_required",
            recommended_action="converge PUSH to a single listener, preferring 127.0.0.1 for the local control plane",
            telegram_required=severity == "critical",
            metadata={"source": "listener_probe", "listeners": listeners},
        )
    )
    return events


def _run_global_health() -> List[Dict[str, Any]]:
    agg = HealthAggregator()
    report = agg.get_global_health()
    events: List[Dict[str, Any]] = []
    if report.score < 70:
        severity = "degraded" if report.score >= 50 else "critical"
        events.append(
            _build_event(
                component_id="global.health",
                domain="runtime_core",
                severity=severity,
                fault_code="global.health_score_low",
                symptom=f"Global health score low: {report.score}",
                evidence=[
                    f"system_score={report.system_score}",
                    f"services_score={report.services_score}",
                    *report.recommendations[:5],
                ],
                recoverability="bounded_auto",
                recommended_action="review recommendations and subsystem bottlenecks",
                telegram_required=severity == "critical",
                metadata={
                    "source": "health_aggregator",
                    "score": report.score,
                    "recommendations": report.recommendations,
                    "components": report.components,
                },
            )
        )
    for anomaly in report.anomalies:
        sev = (anomaly.severity or "").lower()
        severity = (
            "critical"
            if sev in {"severa", "severe"}
            else "degraded"
            if sev in {"moderada", "moderado", "medium"}
            else "warning"
        )
        events.append(
            _build_event(
                component_id="global.anomaly",
                domain="runtime_core",
                severity=severity,
                fault_code="global.anomaly_detected",
                symptom="System anomaly detected",
                evidence=[
                    f"severity={anomaly.severity}",
                    f"score={anomaly.score}",
                    f"metrics={','.join(anomaly.metrics_affected)}",
                ],
                recoverability="manual_required",
                recommended_action="inspect anomaly context and correlated subsystems",
                telegram_required=severity == "critical",
                metadata={"source": "health_aggregator", "anomaly": asdict(anomaly)},
            )
        )
    return events


def _run_supervisor_status() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    try:
        from atlas_adapter.supervisor_daemon import get_supervisor_daemon

        status = get_supervisor_daemon().status()
        data = (status or {}).get("data") or {}
        interval_sec = int(data.get("interval_sec") or 30)
        recent_supervisor_entry = None
        try:
            for entry in get_evolution_entries(limit=30) or []:
                if str(entry.get("source") or "").strip().lower() == "supervisor":
                    recent_supervisor_entry = entry
                    break
        except Exception:
            recent_supervisor_entry = None

        external_activity = False
        last_external_ts = None
        if recent_supervisor_entry:
            last_external_ts = _parse_iso(recent_supervisor_entry.get("timestamp"))
            if last_external_ts is not None:
                external_activity = (
                    (datetime.now(timezone.utc) - last_external_ts).total_seconds()
                    <= max(interval_sec * 3, 600)
                )

        if data.get("enabled") and not data.get("running"):
            if external_activity:
                events.append(
                    _build_event(
                        component_id="supervisor.daemon",
                        domain="operations_core",
                        severity="warning",
                        fault_code="supervisor.process_mismatch",
                        symptom="Supervisor active recently, but not in current HTTP process",
                        evidence=[
                            f"interval_sec={data.get('interval_sec')}",
                            f"last_severity={data.get('last_severity')}",
                            f"last_tick_ts={data.get('last_tick_ts')}",
                            f"recent_external_ts={recent_supervisor_entry.get('timestamp')}",
                        ],
                        recoverability="manual_required",
                        recommended_action="converge PUSH processes or restart runtime so supervisor ownership is explicit",
                        metadata={
                            "source": "supervisor_daemon",
                            "status": status,
                            "recent_entry": recent_supervisor_entry,
                        },
                    )
                )
            else:
                events.append(
                    _build_event(
                        component_id="supervisor.daemon",
                        domain="operations_core",
                        severity="degraded",
                        fault_code="supervisor.not_running",
                        symptom="Supervisor daemon enabled but not running",
                        evidence=[
                            f"interval_sec={data.get('interval_sec')}",
                            f"last_severity={data.get('last_severity')}",
                            f"last_tick_ts={data.get('last_tick_ts')}",
                        ],
                        recoverability="bounded_auto",
                        recommended_action="restart push and verify supervisor lock",
                        metadata={"source": "supervisor_daemon", "status": status},
                    )
                )
    except Exception as exc:
        events.append(
            _build_event(
                component_id="supervisor.daemon",
                domain="operations_core",
                severity="warning",
                fault_code="supervisor.status_unavailable",
                symptom=f"Could not read supervisor daemon status: {exc}",
                recoverability="manual_required",
                recommended_action="inspect supervisor daemon import/runtime state",
                metadata={"source": "supervisor_daemon", "error": str(exc)},
            )
        )
    return events


def _run_comms_health() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    try:
        from modules.humanoid.ans.api import _build_comms_status

        status = _build_comms_status(deep=False)
    except Exception:
        status = _http_json("http://127.0.0.1:8791/ans/comms/status", timeout=5.0) or {}

    waha = status.get("waha") or {}
    whatsapp = status.get("whatsapp") or {}
    waha_status = str(waha.get("status") or "").strip().upper()
    connected = bool(whatsapp.get("connected", False))
    if waha_status in {"UNREACHABLE", "ERROR", ""}:
        events.append(
            _build_event(
                component_id="comms.whatsapp",
                domain="integration_core",
                severity="degraded",
                fault_code="comms.whatsapp_unavailable",
                symptom="WhatsApp / WAHA unavailable or unauthenticated",
                evidence=[
                    f"waha_status={waha.get('status')}",
                    f"connected={whatsapp.get('connected')}",
                    f"url={waha.get('url')}",
                ],
                recoverability="bounded_auto",
                recommended_action="ensure WAHA runtime is up and retry session bootstrap",
                metadata={"source": "comms_status", "status": status},
            )
        )
    elif not connected or waha_status not in {"WORKING"}:
        events.append(
            _build_event(
                component_id="comms.whatsapp",
                domain="integration_core",
                severity="warning",
                fault_code="comms.whatsapp_auth_pending",
                symptom="WhatsApp runtime reachable but session is not authenticated yet",
                evidence=[
                    f"waha_status={waha.get('status')}",
                    f"connected={whatsapp.get('connected')}",
                    f"url={waha.get('url')}",
                ],
                recoverability="manual_required",
                recommended_action="scan WAHA QR or request pairing code to authenticate session",
                metadata={"source": "comms_status", "status": status},
            )
        )
    return events


def _run_robot_autonomy_health() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    safe_status = _http_json("http://127.0.0.1:8002/api/safe-mode/status", timeout=5.0)
    if safe_status and safe_status.get("safe_mode_active"):
        events.append(
            _build_event(
                component_id="robot.safe_mode",
                domain="robotics_core",
                severity="degraded",
                fault_code="robot.safe_mode_active",
                symptom="Robot safe mode active",
                evidence=[
                    f"seconds_since_heartbeat={safe_status.get('seconds_since_heartbeat')}",
                    f"reconnect_attempts={safe_status.get('reconnect_attempts')}",
                ],
                recoverability="manual_required",
                recommended_action="restore control heartbeat and inspect robot control plane",
                metadata={"source": "robot.safe_mode", "status": safe_status},
            )
        )

    watchdog_status = _http_json("http://127.0.0.1:8002/api/watchdog/status", timeout=5.0)
    if watchdog_status:
        services = watchdog_status.get("services") or {}
        bad = [name for name, ok in services.items() if not ok]
        if not watchdog_status.get("running") or bad:
            severity = "critical" if bad else "degraded"
            events.append(
                _build_event(
                    component_id="robot.watchdog",
                    domain="robotics_core",
                    severity=severity,
                    fault_code="robot.watchdog_issue",
                    symptom="Robot watchdog reports service degradation",
                    evidence=[
                        f"running={watchdog_status.get('running')}",
                        f"failed_services={','.join(bad) if bad else 'none'}",
                        f"restarts={json.dumps(watchdog_status.get('restarts') or {}, ensure_ascii=False)}",
                    ],
                    recoverability="bounded_auto",
                    recommended_action="restart robot backend and verify watchdog services",
                    telegram_required=severity == "critical",
                    metadata={"source": "robot.watchdog", "status": watchdog_status},
                )
            )
    return events


def _run_scheduler_runtime_health() -> List[Dict[str, Any]]:
    events: List[Dict[str, Any]] = []
    payload = _scheduler_jobs_payload()
    jobs = payload.get("jobs") or []
    if not isinstance(jobs, list):
        return events

    for job in jobs[:100]:
        if not isinstance(job, dict):
            continue
        if not bool(job.get("enabled", True)):
            continue

        status = str(job.get("status") or "").strip().lower()
        if status != "failed":
            continue

        kind = str(job.get("kind") or job.get("name") or "unknown")
        job_id = str(job.get("id") or "")
        retries = int(job.get("retries") or 0)
        max_retries = int(job.get("max_retries") or 0)
        next_run_ts = job.get("next_run_ts")
        last_error = str(job.get("last_error") or "")
        updated_ts = str(job.get("updated_ts") or "")
        latest_run = _latest_scheduler_run(job_id)

        fault_code = "scheduler.job_failed_recurrent"
        severity = "degraded"
        recoverability = "manual_required"
        recommended_action = "inspect scheduler job state and root cause"

        fatal_stale = bool(last_error and any(token in last_error for token in FATAL_SCHEDULER_ERROR_SNIPPETS))
        stale_failed = _is_stale_failed_scheduler_job(job)
        if fatal_stale:
            fault_code = "scheduler.job_legacy_broken"
            severity = "warning"
            recoverability = "bounded_auto"
            recommended_action = "pause obsolete broken scheduler job to remove persistent noise"
        elif stale_failed:
            fault_code = "scheduler.job_stale_failed"
            severity = "warning"
            recoverability = "bounded_auto"
            recommended_action = "pause stale failed scheduler job so it no longer counts as active runtime degradation"

        if next_run_ts in (None, "", "null"):
            fault_code = "scheduler.job_stuck"
            recommended_action = "rerun stuck scheduler job and verify it leaves failed state"
            if fatal_stale:
                fault_code = "scheduler.job_legacy_broken"
                severity = "warning"
                recoverability = "bounded_auto"
                recommended_action = "pause obsolete broken scheduler job to remove persistent noise"
            elif stale_failed:
                fault_code = "scheduler.job_stale_failed"
                severity = "warning"
                recoverability = "bounded_auto"
                recommended_action = "pause stale failed scheduler job so it no longer counts as active runtime degradation"

        if _is_recent_semantic_completion(job, latest_run):
            fault_code = "scheduler.job_state_desync"
            severity = "warning"
            recoverability = "bounded_auto"
            recommended_action = "repair scheduler state from latest semantically completed nervous_cycle run"

        if kind == "nervous_cycle":
            severity = "critical" if retries >= max(max_retries, 10) else "degraded"
            recoverability = "bounded_auto"
            recommended_action = (
                "rerun nervous_cycle job and inspect degraded nervous signals"
            )
            if fault_code == "scheduler.job_state_desync":
                severity = "warning"
                recommended_action = "repair scheduler state from latest semantically completed nervous_cycle run"
        elif retries > max(max_retries, 3):
            severity = "critical"

        evidence = [
            f"job_id={job_id}",
            f"status={status}",
            f"retries={retries}",
            f"max_retries={max_retries}",
            f"next_run_ts={next_run_ts}",
            f"last_run_ts={job.get('last_run_ts')}",
            f"updated_ts={updated_ts}",
        ]
        if last_error:
            evidence.append(f"last_error={last_error[:240]}")

        events.append(
            _build_event(
                component_id=f"scheduler.job.{kind}",
                domain=SCHEDULER_DOMAIN_MAP.get(kind, "operations_core"),
                severity=severity,
                fault_code=fault_code,
                symptom=f"Scheduler job '{kind}' remains in failed state",
                evidence=evidence,
                recoverability=recoverability,
                recommended_action=recommended_action,
                telegram_required=severity == "critical",
                metadata={
                    "source": "scheduler_runtime",
                    "job": job,
                    "jobs_source": payload.get("source"),
                    "latest_run": latest_run,
                },
            )
        )

    return events


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _emit_bitacora(message: str, ok: bool) -> None:
    append_evolution_log(message=message, ok=ok, source="fault_snapshot")


async def _emit_telegram(message: str) -> bool:
    return await send_telegram(message)


def _notify_critical(message: str) -> None:
    try:
        asyncio.run(_emit_telegram(message))
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS normalized fault snapshot")
    parser.add_argument("--json", action="store_true", help="Print JSON summary")
    parser.add_argument("--no-emit", action="store_true", help="Skip bitacora/telegram")
    args = parser.parse_args()

    events = []
    events.extend(_run_ans_checks())
    events.extend(_run_service_health())
    events.extend(_run_gateway_binding_health())
    events.extend(_run_global_health())
    events.extend(_run_supervisor_status())
    events.extend(_run_comms_health())
    events.extend(_run_robot_autonomy_health())
    events.extend(_run_scheduler_runtime_health())

    severity_counts: Dict[str, int] = {}
    for sev in ("info", "warning", "degraded", "critical", "emergency"):
        severity_counts[sev] = sum(1 for event in events if event["severity"] == sev)

    snapshot = {
        "generated_at": _now_iso(),
        "event_count": len(events),
        "severity_counts": severity_counts,
        "events": events,
    }
    _write_json(SNAPSHOT_PATH, snapshot)
    _write_json(EVENTS_PATH, {"generated_at": snapshot["generated_at"], "events": events})

    if not args.no_emit:
        message = (
            f"[FAULT_SNAPSHOT] {len(events)} eventos normalizados | "
            f"warning={severity_counts['warning']} degraded={severity_counts['degraded']} "
            f"critical={severity_counts['critical']} emergency={severity_counts['emergency']}"
        )
        _emit_bitacora(message, ok=severity_counts["critical"] == 0 and severity_counts["emergency"] == 0)
        if severity_counts["critical"] or severity_counts["emergency"]:
            _notify_critical(f"ATLAS FAULT SNAPSHOT CRITICAL\n{message}")

    if args.json:
        print(json.dumps({k: snapshot[k] for k in ("generated_at", "event_count", "severity_counts")}, indent=2, ensure_ascii=False))
    else:
        print(json.dumps({k: snapshot[k] for k in ("generated_at", "event_count", "severity_counts")}, indent=2, ensure_ascii=False))
        print(f"Snapshot: {SNAPSHOT_PATH}")
        print(f"Events  : {EVENTS_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
