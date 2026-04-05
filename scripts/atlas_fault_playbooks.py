#!/usr/bin/env python3
"""
ATLAS Fault Playbooks

Planificador y ejecutor de playbooks seguros por dominio/fault_code.
Se apoya en RecoveryStrategies y en las convenciones ya existentes del repo.
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, List
from urllib.error import URLError
from urllib.request import Request, urlopen

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from autonomous.resilience.survival_mode import SurvivalMode
from autonomous.self_healing.recovery_strategies import RecoveryStrategies
from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.notify import send_telegram
from modules.humanoid.scheduler.db import SchedulerDB

STATE_PATH = REPO_ROOT / "state" / "atlas_fault_playbook_state.json"
CONTROL_PLANE_PATH = REPO_ROOT / "state" / "atlas_control_plane_state.json"
DEFAULT_BACKOFF_SEC = 300

SERVICE_HEALTH_URLS = {
    "push": "http://127.0.0.1:8791/health",
    "nexus": "http://127.0.0.1:8000/health",
    "robot": "http://127.0.0.1:8002/api/health",
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _read_state() -> Dict[str, Any]:
    if not STATE_PATH.exists():
        return {"last_run": {}, "updated_at": _now_iso()}
    try:
        return json.loads(STATE_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {"last_run": {}, "updated_at": _now_iso()}


def _write_state(state: Dict[str, Any]) -> None:
    STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    state["updated_at"] = _now_iso()
    STATE_PATH.write_text(
        json.dumps(state, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


def _write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _emit_bitacora(message: str, ok: bool) -> None:
    append_evolution_log(message=message, ok=ok, source="fault_playbook")


def _emit_telegram(message: str, severity: str) -> bool:
    try:
        return bool(asyncio.run(send_telegram(message)))
    except Exception:
        return False


def _http_ok(url: str, timeout_sec: int = 5) -> bool:
    try:
        with urlopen(url, timeout=timeout_sec) as resp:
            code = getattr(resp, "status", None) or resp.getcode()
            return int(code) == 200
    except (URLError, OSError, ValueError):
        return False


def _http_json(url: str, timeout_sec: int = 5) -> Dict[str, Any] | None:
    try:
        with urlopen(url, timeout=timeout_sec) as resp:
            return json.loads(resp.read().decode())
    except (URLError, OSError, ValueError, json.JSONDecodeError):
        return None


def _post_ok(url: str, timeout_sec: int = 10, headers: Dict[str, str] | None = None) -> bool:
    try:
        req = Request(
            url,
            data=b"{}",
            headers=headers or {"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(req, timeout=timeout_sec) as resp:
            code = getattr(resp, "status", None) or resp.getcode()
            return int(code) == 200
    except (URLError, OSError, ValueError):
        return False


def _post_json(url: str, payload: Dict[str, Any], timeout_sec: int = 10) -> bool:
    try:
        req = Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(req, timeout=timeout_sec) as resp:
            code = getattr(resp, "status", None) or resp.getcode()
            return int(code) == 200
    except (URLError, OSError, ValueError):
        return False


def _run_command(args: List[str], timeout_sec: int = 30) -> Dict[str, Any]:
    try:
        cp = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout_sec,
            check=False,
        )
        return {
            "ok": cp.returncode == 0,
            "code": int(cp.returncode),
            "stdout": str(cp.stdout or "")[-600:],
            "stderr": str(cp.stderr or "")[-600:],
        }
    except Exception as exc:
        return {"ok": False, "code": -1, "stdout": "", "stderr": str(exc)}


def _waha_status() -> Dict[str, Any]:
    return _http_json("http://127.0.0.1:8791/api/comms/whatsapp/status", timeout_sec=6) or {}


def _wait_for_docker(timeout_sec: int = 120) -> Dict[str, Any]:
    deadline = time.time() + max(15, timeout_sec)
    last = {"ok": False, "stderr": "docker not ready"}
    while time.time() < deadline:
        last = _run_command(["docker", "info"], timeout_sec=12)
        if last.get("ok"):
            return last
        time.sleep(5)
    return last


def _wait_for_waha_http(timeout_sec: int = 45) -> Dict[str, Any]:
    deadline = time.time() + max(10, timeout_sec)
    last_status: Dict[str, Any] = {}
    while time.time() < deadline:
        last_status = _waha_status()
        provider_status = (last_status.get("provider_status") or {})
        if provider_status.get("ok") or "not_found" in str(provider_status.get("status") or "").lower():
            return last_status
        error_txt = str(provider_status.get("error") or "").lower()
        if "10061" not in error_txt and "failed to establish" not in error_txt and "connection refused" not in error_txt:
            return last_status
        time.sleep(3)
    return last_status


def _ensure_waha_runtime() -> Dict[str, Any]:
    status = _waha_status()
    provider_status = (status.get("provider_status") or {})
    if provider_status.get("ok"):
        return {"ok": True, "status": status, "reason": "already_reachable"}

    if not shutil.which("docker"):
        return {"ok": False, "reason": "docker_cli_missing"}

    docker_info = _run_command(["docker", "info"], timeout_sec=12)
    if not docker_info.get("ok"):
        docker_desktop = r"C:\Program Files\Docker\Docker\Docker Desktop.exe"
        if os.path.exists(docker_desktop):
            try:
                subprocess.Popen([docker_desktop], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as exc:
                return {"ok": False, "reason": "docker_desktop_start_failed", "error": str(exc)}
            docker_info = _wait_for_docker(timeout_sec=120)
        if not docker_info.get("ok"):
            return {"ok": False, "reason": "docker_daemon_unavailable", "docker": docker_info}

    exists = _run_command(
        ["docker", "ps", "-a", "--filter", "name=^/waha$", "--format", "{{.Names}}|{{.Status}}"],
        timeout_sec=15,
    )
    exists_out = str(exists.get("stdout") or "")
    if "waha" in exists_out:
        started = _run_command(["docker", "start", "waha"], timeout_sec=30)
        if not started.get("ok"):
            return {"ok": False, "reason": "docker_start_waha_failed", "docker": started}
    else:
        created = _run_command(
            [
                "docker",
                "run",
                "-d",
                "-p",
                "3010:3000",
                "--name",
                "waha",
                "--restart",
                "unless-stopped",
                "devlikeapro/waha",
            ],
            timeout_sec=60,
        )
        if not created.get("ok"):
            return {"ok": False, "reason": "docker_run_waha_failed", "docker": created}

    status = _wait_for_waha_http(timeout_sec=45)
    provider_status = (status.get("provider_status") or {})
    return {
        "ok": bool(provider_status.get("ok")),
        "reason": "waha_runtime_ready" if provider_status.get("ok") else "waha_runtime_not_ready",
        "status": status,
    }


def _scheduler_job_status(job_id: str) -> Dict[str, Any] | None:
    status = _http_json("http://127.0.0.1:8791/scheduler/jobs", timeout_sec=5)
    jobs = (status or {}).get("data") or []
    for job in jobs if isinstance(jobs, list) else []:
        if str((job or {}).get("id") or "") == str(job_id or ""):
            return job
    return None


def _check_backoff(signature: str, backoff_sec: int = DEFAULT_BACKOFF_SEC) -> bool:
    state = _read_state()
    item = (state.get("last_run") or {}).get(signature) or {}
    last_ts = float(item.get("ts") or 0.0)
    return (time.time() - last_ts) < backoff_sec


def _mark_run(signature: str, payload: Dict[str, Any]) -> None:
    state = _read_state()
    last_run = state.setdefault("last_run", {})
    payload = dict(payload)
    payload["ts"] = time.time()
    last_run[signature] = payload
    _write_state(state)


def _service_from_event(event: Dict[str, Any]) -> str | None:
    component_id = str(event.get("component_id") or "")
    if component_id.startswith("service."):
        return component_id.split(".", 1)[1]
    if component_id in {"robot_camera_health"}:
        return "robot"
    if component_id in {"nexus_services_health"}:
        return "nexus"
    return None


def _plan_actions(event: Dict[str, Any]) -> List[Dict[str, Any]]:
    component_id = str(event.get("component_id") or "")
    fault_code = str(event.get("fault_code") or "")
    severity = str(event.get("severity") or "warning")
    domain = str(event.get("domain") or "unclassified")
    service_name = _service_from_event(event)

    if fault_code == "service.offline" and service_name in SERVICE_HEALTH_URLS:
        return [
            {
                "type": "restart_service",
                "service": service_name,
                "verify_http_url": SERVICE_HEALTH_URLS[service_name],
                "verify_after_sec": 6 if service_name == "robot" else 8,
            }
        ]

    if component_id in {"robot_camera_health"} or "camera" in component_id:
        return [
            {
                "type": "activate_fallback",
                "component": "vision",
            },
            {
                "type": "restart_service",
                "service": "robot",
                "verify_http_url": SERVICE_HEALTH_URLS["robot"],
                "verify_after_sec": 6,
            },
        ]

    if fault_code == "comms.whatsapp_unavailable":
        return [
            {
                "type": "ensure_waha_runtime",
                "verify_http_url": "http://127.0.0.1:8791/api/comms/whatsapp/status",
                "verify_after_sec": 4,
            },
            {
                "type": "start_waha_session",
                "verify_http_url": "http://127.0.0.1:8791/api/comms/whatsapp/status",
                "verify_after_sec": 4,
            }
        ]

    if fault_code == "supervisor.not_running":
        return [
            {
                "type": "restart_service",
                "service": "push",
                "verify_http_url": SERVICE_HEALTH_URLS["push"],
                "verify_after_sec": 8,
            }
        ]

    if fault_code == "robot.watchdog_issue":
        return [
            {
                "type": "restart_service",
                "service": "robot",
                "verify_http_url": SERVICE_HEALTH_URLS["robot"],
                "verify_after_sec": 6,
            }
        ]

    if component_id in {"router_health", "llm_health"} or domain == "cognitive_core":
        if "llm" in component_id or "router" in component_id:
            return [
                {
                    "type": "activate_fallback",
                    "component": "llm_router",
                }
            ]

    if fault_code == "global.health_score_low" and severity in {"critical", "emergency"}:
        return [
            {
                "type": "enter_degraded_mode",
                "reason": f"fault_manager:{fault_code}",
            }
        ]

    if component_id == "nexus_services_health":
        return [
            {
                "type": "restart_service",
                "service": "nexus",
                "verify_http_url": SERVICE_HEALTH_URLS["nexus"],
                "verify_after_sec": 8,
            }
        ]

    if fault_code == "scheduler.job_stuck":
        job_data = ((event.get("metadata") or {}).get("job") or {})
        job_id = str(job_data.get("id") or "")
        job_kind = str(job_data.get("kind") or "")
        last_error = str(job_data.get("last_error") or "")
        if job_id and job_kind == "nervous_cycle" and not last_error:
            return [
                {
                    "type": "rerun_scheduler_job",
                    "job_id": job_id,
                    "verify_after_sec": 3,
                }
            ]

    if fault_code in {"scheduler.job_legacy_broken", "scheduler.job_stale_failed"}:
        job_data = ((event.get("metadata") or {}).get("job") or {})
        job_id = str(job_data.get("id") or "")
        if job_id:
            return [
                {
                    "type": "pause_scheduler_job",
                    "job_id": job_id,
                    "verify_after_sec": 1,
                }
            ]

    if fault_code == "scheduler.job_state_desync":
        job_data = ((event.get("metadata") or {}).get("job") or {})
        job_id = str(job_data.get("id") or "")
        if job_id:
            return [
                {
                    "type": "repair_scheduler_job_state",
                    "job_id": job_id,
                    "verify_after_sec": 0,
                }
            ]

    if fault_code == "gateway.listener_conflict_8791":
        return [
            {
                "type": "publish_control_plane_override",
                "preferred_endpoint": "http://127.0.0.1:8791",
                "verify_http_url": "http://127.0.0.1:8791/health",
                "verify_after_sec": 0,
            }
        ]

    return []


def _execution_class(actions: List[Dict[str, Any]]) -> str:
    disruptive_types = {"restart_service"}
    if any((action.get("type") in disruptive_types) for action in actions):
        return "disruptive"
    return "non_disruptive"


def plan_playbook_for_event(event: Dict[str, Any]) -> Dict[str, Any] | None:
    actions = _plan_actions(event)
    if not actions:
        return None
    component_id = str(event.get("component_id") or "")
    fault_code = str(event.get("fault_code") or "")
    severity = str(event.get("severity") or "warning")
    signature = f"{component_id}|{fault_code}|{severity}"
    return {
        "playbook_id": f"playbook.{component_id or 'event'}",
        "signature": signature,
        "component_id": component_id,
        "fault_code": fault_code,
        "domain": event.get("domain"),
        "severity": severity,
        "safe": True,
        "execution_class": _execution_class(actions),
        "source_event": {
            "component_id": component_id,
            "fault_code": fault_code,
            "severity": severity,
            "symptom": event.get("symptom"),
        },
        "actions": actions,
    }


def execute_playbook(
    playbook: Dict[str, Any],
    *,
    emit: bool = True,
    backoff_sec: int = DEFAULT_BACKOFF_SEC,
) -> Dict[str, Any]:
    signature = str(playbook.get("signature") or "")
    if signature and _check_backoff(signature, backoff_sec=backoff_sec):
        return {
            "ok": False,
            "skipped": True,
            "reason": "backoff_active",
            "playbook_id": playbook.get("playbook_id"),
            "signature": signature,
            "results": [],
        }

    strategies = RecoveryStrategies()
    results: List[Dict[str, Any]] = []
    overall_ok = True
    survival_toggled = False

    for action in playbook.get("actions", []):
        action_type = action.get("type")
        result: Dict[str, Any] = {"type": action_type, "ok": False}
        try:
            if action_type == "restart_service":
                service = str(action.get("service") or "")
                result["service"] = service
                result["ok"] = bool(strategies.restart_service(service))
                wait_sec = int(action.get("verify_after_sec") or 0)
                verify_url = str(action.get("verify_http_url") or "")
                if result["ok"] and wait_sec > 0:
                    time.sleep(wait_sec)
                if verify_url:
                    result["verified"] = _http_ok(verify_url)
                    result["ok"] = bool(result["ok"] and result["verified"])
            elif action_type == "activate_fallback":
                component = str(action.get("component") or "")
                result["component"] = component
                result["ok"] = bool(strategies.activate_fallback(component))
            elif action_type == "enter_degraded_mode":
                reason = str(action.get("reason") or "fault_manager")
                result["reason"] = reason
                result["ok"] = bool(strategies.enter_degraded_mode())
                result["survival_active"] = SurvivalMode.is_in_survival()
                survival_toggled = survival_toggled or result["survival_active"]
            elif action_type == "restart_waha_session":
                result["ok"] = _post_ok(
                    "http://127.0.0.1:8791/ans/comms/waha/restart",
                    timeout_sec=10,
                )
                wait_sec = int(action.get("verify_after_sec") or 0)
                verify_url = str(action.get("verify_http_url") or "")
                if result["ok"] and wait_sec > 0:
                    time.sleep(wait_sec)
                if verify_url:
                    status = _http_json(verify_url, timeout_sec=5)
                    waha = (status or {}).get("waha") or {}
                    result["verified"] = bool(
                        status and waha.get("status") not in {"unreachable", "error", None}
                    )
                    result["verification_status"] = waha.get("status")
                    result["ok"] = bool(result["ok"] and result["verified"])
            elif action_type == "ensure_waha_runtime":
                runtime_result = _ensure_waha_runtime()
                result.update(runtime_result)
                result["ok"] = bool(runtime_result.get("ok"))
                status = runtime_result.get("status") or {}
                provider_status = (status.get("provider_status") or {})
                result["verification_status"] = provider_status.get("status") or provider_status.get("error")
            elif action_type == "start_waha_session":
                result["ok"] = _post_ok(
                    "http://127.0.0.1:8791/api/comms/whatsapp/start-session",
                    timeout_sec=15,
                )
                wait_sec = int(action.get("verify_after_sec") or 0)
                if wait_sec > 0:
                    time.sleep(wait_sec)
                status = _waha_status()
                provider_status = (status or {}).get("provider_status") or {}
                provider_state = str(provider_status.get("status") or "")
                result["verified"] = bool(
                    provider_status.get("ok")
                    or provider_state.upper() in {"SCAN_QR", "SCAN_QR_CODE", "STARTING", "WORKING"}
                )
                result["verification_status"] = provider_state or provider_status.get("error")
                result["ok"] = bool(result["ok"] and result["verified"])
            elif action_type == "rerun_scheduler_job":
                job_id = str(action.get("job_id") or "")
                result["job_id"] = job_id
                result["ok"] = _post_json(
                    "http://127.0.0.1:8791/scheduler/job/run-now",
                    {"job_id": job_id},
                    timeout_sec=10,
                )
                wait_sec = int(action.get("verify_after_sec") or 0)
                if result["ok"] and wait_sec > 0:
                    time.sleep(wait_sec)
                if job_id:
                    job_status = _scheduler_job_status(job_id)
                    result["job_status"] = (
                        (job_status or {}).get("status") if job_status else None
                    )
                    result["verified"] = bool(
                        job_status
                        and str(job_status.get("status") or "").lower()
                        in {"queued", "running", "success"}
                    )
                    result["ok"] = bool(result["ok"] and result["verified"])
            elif action_type == "pause_scheduler_job":
                job_id = str(action.get("job_id") or "")
                result["job_id"] = job_id
                result["ok"] = _post_json(
                    "http://127.0.0.1:8791/scheduler/job/pause",
                    {"job_id": job_id},
                    timeout_sec=10,
                )
                wait_sec = int(action.get("verify_after_sec") or 0)
                if result["ok"] and wait_sec > 0:
                    time.sleep(wait_sec)
                if job_id:
                    job_status = _scheduler_job_status(job_id)
                    result["job_status"] = (
                        (job_status or {}).get("status") if job_status else None
                    )
                    result["verified"] = bool(
                        job_status
                        and str(job_status.get("status") or "").lower() == "paused"
                    )
                    result["ok"] = bool(result["ok"] and result["verified"])
            elif action_type == "repair_scheduler_job_state":
                job_id = str(action.get("job_id") or "")
                result["job_id"] = job_id
                db = SchedulerDB()
                job_status = db.get_job(job_id)
                if job_status:
                    interval = int(job_status.get("interval_seconds") or 0)
                    next_run_ts = None
                    if interval > 0:
                        next_run_ts = (
                            datetime.now(timezone.utc) + timedelta(seconds=interval)
                        ).isoformat()
                    db.set_finished(job_id, True, None, next_run_ts, 0)
                    if next_run_ts:
                        db.set_queued(job_id, next_run_ts)
                    repaired = db.get_job(job_id)
                    result["job_status"] = (repaired or {}).get("status")
                    result["next_run_ts"] = (repaired or {}).get("next_run_ts")
                    result["verified"] = bool(
                        repaired
                        and str(repaired.get("status") or "").lower() in {"queued", "success"}
                    )
                    result["ok"] = bool(result["verified"])
                else:
                    result["reason"] = "job_not_found"
            elif action_type == "publish_control_plane_override":
                preferred_endpoint = str(action.get("preferred_endpoint") or "http://127.0.0.1:8791")
                verify_url = str(action.get("verify_http_url") or f"{preferred_endpoint}/health")
                health = _http_json(verify_url, timeout_sec=4) or {}
                preferred_ok = bool(health.get("ok"))
                payload = {
                    "generated_at": _now_iso(),
                    "preferred_endpoint": preferred_endpoint,
                    "health_url": verify_url,
                    "preferred_ok": preferred_ok,
                    "status": "conflict_mitigated" if preferred_ok else "conflict_unhealthy",
                    "operator_action_required": True,
                    "recommended_operator_action": (
                        "Use 127.0.0.1:8791 for dashboard/API and retire the extra 0.0.0.0 listener with elevated privileges."
                    ),
                    "source": "fault_playbook.gateway.listener_conflict_8791",
                }
                _write_json(CONTROL_PLANE_PATH, payload)
                result["preferred_endpoint"] = preferred_endpoint
                result["verified"] = preferred_ok
                result["status"] = payload["status"]
                result["ok"] = preferred_ok
            else:
                result["reason"] = "unsupported_action"
        except Exception as exc:
            result["error"] = str(exc)
            result["ok"] = False

        overall_ok = overall_ok and bool(result.get("ok"))
        results.append(result)

    if survival_toggled:
        try:
            SurvivalMode().exit_survival_mode()
            results.append({"type": "exit_degraded_mode", "ok": True})
        except Exception as exc:
            results.append(
                {"type": "exit_degraded_mode", "ok": False, "error": str(exc)}
            )
            overall_ok = False

    report = {
        "executed_at": _now_iso(),
        "ok": overall_ok,
        "skipped": False,
        "playbook_id": playbook.get("playbook_id"),
        "signature": signature,
        "results": results,
    }

    if signature:
        _mark_run(
            signature,
            {
                "playbook_id": playbook.get("playbook_id"),
                "ok": overall_ok,
                "component_id": playbook.get("component_id"),
                "fault_code": playbook.get("fault_code"),
            },
        )

    if emit:
        message = (
            f"[FAULT_PLAYBOOK] {playbook.get('playbook_id')} "
            f"ok={overall_ok} actions={len(results)} component={playbook.get('component_id')}"
        )
        _emit_bitacora(message, ok=overall_ok)
        severity = str(playbook.get("severity") or "warning")
        if severity in {"critical", "emergency"}:
            _emit_telegram(message, severity=severity)

    return report


def selftest(*, emit: bool = False) -> Dict[str, Any]:
    fallback_event = {
        "component_id": "llm_health",
        "domain": "cognitive_core",
        "severity": "degraded",
        "fault_code": "check.llm_health",
        "symptom": "llm route unstable",
    }
    critical_event = {
        "component_id": "global.health",
        "domain": "runtime_core",
        "severity": "critical",
        "fault_code": "global.health_score_low",
        "symptom": "synthetic critical test",
    }
    fallback_playbook = plan_playbook_for_event(fallback_event)
    degrade_playbook = plan_playbook_for_event(critical_event)
    return {
        "tested_at": _now_iso(),
        "fallback": execute_playbook(fallback_playbook, emit=emit)
        if fallback_playbook
        else None,
        "degrade": execute_playbook(degrade_playbook, emit=emit, backoff_sec=0)
        if degrade_playbook
        else None,
        "survival_active_after": SurvivalMode.is_in_survival(),
    }


if __name__ == "__main__":
    print(json.dumps(selftest(emit=False), indent=2, ensure_ascii=False))
