"""Blue-green local: launch staging, smoke, health with retries, promote (restart on ACTIVE) or rollback. Policy + audit."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, List, Optional

from .healthcheck import run_health_verbose
from .process_manager import start_instance, stop_by_pid, wait_until_healthy
from .switcher import get_deploy_state, persist_state

_log = __import__("logging").getLogger("humanoid.deploy.bluegreen")

TIMEOUT_START_SEC = 30


def _env_bool(name: str, default: bool) -> bool:
    v = os.getenv(name, "true" if default else "false").strip().lower()
    return v in ("1", "true", "yes", "y", "on")


def _env_int(name: str, default: int) -> int:
    try:
        return int(os.getenv(name, str(default)) or default)
    except (TypeError, ValueError):
        return default


def _audit(action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("deploy", "bluegreen", "bluegreen", action, ok, ms, error, payload, None)
    except Exception:
        pass


def run_bluegreen_flow(actor: Any = None, dry_run: bool = False, ref: Optional[str] = None) -> Dict[str, Any]:
    """
    Option 1: ACTIVE always 8791. 1) Launch STAGING on STAGING_PORT, 2) Smoke vs STAGING, 3) Health vs STAGING
    (DEPLOY_HEALTH_RETRIES, DEPLOY_HEALTH_MIN_SCORE), 4) If OK: promote = restart service on 8791 (net stop/start);
    if fail: stop STAGING, rollback audit.
    """
    t0 = time.perf_counter()
    deploy_mode = os.getenv("DEPLOY_MODE", "").strip().lower()
    if deploy_mode != "bluegreen":
        return {"ok": False, "data": None, "ms": int((time.perf_counter() - t0) * 1000), "error": "DEPLOY_MODE != bluegreen"}

    state = get_deploy_state()
    active_port = state.get("active_port", _env_int("ACTIVE_PORT", 8791))
    staging_port = state.get("staging_port", _env_int("STAGING_PORT", 8792))
    health_retries = _env_int("DEPLOY_HEALTH_RETRIES", 3)
    health_retry_sec = _env_int("DEPLOY_HEALTH_RETRY_SECONDS", 2)
    min_score = _env_int("DEPLOY_HEALTH_MIN_SCORE", 75)
    auto_switch = state.get("auto_switch_on_health", _env_bool("AUTO_SWITCH_ON_HEALTH", True))
    steps: List[Dict[str, Any]] = []
    staging_pid: Optional[int] = None

    _audit("deploy_start", True, {"ref": ref}, None, 0)

    # 1) Launch staging (process_manager, PID tracking)
    launch = start_instance(staging_port, "staging")
    staging_pid = launch.get("pid")
    steps.append({"step": "staging_started", "ok": launch.get("ok"), "pid": staging_pid, "port": staging_port, "error": launch.get("error")})
    if not launch.get("ok"):
        _audit("staging_started", False, {"steps": steps}, launch.get("error"), int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps}, "ms": int((time.perf_counter() - t0) * 1000), "error": launch.get("error")}
    state["staging_pid"] = staging_pid
    persist_state(state)
    _audit("staging_started", True, {"pid": staging_pid, "port": staging_port}, None, 0)

    base_staging = f"http://127.0.0.1:{staging_port}"
    if not wait_until_healthy(base_staging, timeout_sec=TIMEOUT_START_SEC):
        stop_by_pid(staging_pid)
        state["staging_pid"] = None
        state["last_deploy"] = {"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "ref": ref, "result": "rollback", "error": "staging did not become healthy"}
        persist_state(state)
        _audit("rollback", False, {"steps": steps}, "staging unhealthy", int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps, "rollback": "staging unhealthy"}, "ms": int((time.perf_counter() - t0) * 1000), "error": "staging unhealthy"}

    # 2) Smoke vs STAGING
    try:
        from modules.humanoid.update.smoke_runner import run_smoke
        smoke_result = run_smoke(port=staging_port, timeout_sec=120)
        smoke_ok = smoke_result.get("ok", False)
    except Exception as e:
        smoke_ok = False
        smoke_result = {"error": str(e)}
    steps.append({"step": "smoke_result", "ok": smoke_ok, "error": smoke_result.get("error")})
    _audit("smoke_result", smoke_ok, {"ok": smoke_ok, "error": smoke_result.get("error")}, smoke_result.get("error"), smoke_result.get("ms", 0))
    if not smoke_ok:
        stop_by_pid(staging_pid)
        state["staging_pid"] = None
        state["mode"] = "single"
        state["last_deploy"] = {"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "ref": ref, "result": "rollback", "error": smoke_result.get("error")}
        persist_state(state)
        _audit("rollback", False, {"steps": steps}, "smoke failed", int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps, "rollback": "smoke failed"}, "ms": int((time.perf_counter() - t0) * 1000), "error": "smoke failed"}

    # 3) Health vs STAGING with retries and min score
    health_ok = False
    last_health: Dict[str, Any] = {}
    for i in range(health_retries):
        time.sleep(health_retry_sec)
        h = run_health_verbose(base_url=base_staging, active_port=staging_port)
        last_health = h
        score = h.get("score", 0)
        steps.append({"step": f"health_{i+1}", "ok": score >= min_score, "score": score, "checks": h.get("checks")})
        if score >= min_score:
            health_ok = True
            break
    _audit("health_scores", health_ok, {"steps": steps, "last_health": last_health}, None if health_ok else "score < min", 0)
    if not health_ok:
        stop_by_pid(staging_pid)
        state["staging_pid"] = None
        state["mode"] = "single"
        state["last_deploy"] = {"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "ref": ref, "result": "rollback", "error": f"health score < {min_score}"}
        persist_state(state)
        _audit("rollback", False, {"steps": steps}, f"health < {min_score}", int((time.perf_counter() - t0) * 1000))
        return {"ok": False, "data": {"steps": steps, "rollback": "health failed"}, "ms": int((time.perf_counter() - t0) * 1000), "error": "health check failed"}

    if dry_run:
        stop_by_pid(staging_pid)
        state["staging_pid"] = None
        persist_state(state)
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("deploy", True, {"steps": steps, "dry_run": True}, None, ms)
        return {"ok": True, "data": {"steps": steps, "switched": False, "dry_run": True}, "ms": ms, "error": None}

    if not auto_switch:
        stop_by_pid(staging_pid)
        state["staging_pid"] = None
        persist_state(state)
        return {"ok": True, "data": {"steps": steps, "switched": False, "message": "AUTO_SWITCH_ON_HEALTH=false"}, "ms": int((time.perf_counter() - t0) * 1000), "error": None}

    # 4) Promote: merge staging -> main so disk has new code, then restart service on ACTIVE_PORT
    stop_by_pid(staging_pid)
    state["staging_pid"] = None
    state["last_deploy"] = {"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "ref": ref, "result": "success", "error": None}
    state["last_health"] = {"score": last_health.get("score"), "checks": last_health.get("checks")}
    state["last_deploy_ts"] = state["last_deploy"]["ts"]
    state["last_deploy_ref"] = ref
    state["last_deploy_result"] = "success"
    persist_state(state)

    try:
        from modules.humanoid.update.git_manager import checkout_branch, merge_staging
        _main = os.getenv("UPDATE_BRANCH", "main").strip() or "main"
        _staging_name = os.getenv("UPDATE_STAGING_BRANCH", "staging").strip() or "staging"
        if checkout_branch(_main).get("ok") and merge_staging(_staging_name).get("ok"):
            pass  # disk now has main + staging merged
    except Exception:
        pass

    svc = os.getenv("SERVICE_NAME", "ATLAS_PUSH")
    if os.getenv("SERVICE_ENABLED", "").strip().lower() in ("1", "true", "yes"):
        try:
            import subprocess
            subprocess.run(["net", "stop", svc], capture_output=True, timeout=15)
            time.sleep(2)
            subprocess.run(["net", "start", svc], capture_output=True, timeout=15)
        except Exception as e:
            _audit("promote_success", False, {"steps": steps}, str(e), int((time.perf_counter() - t0) * 1000))
            try:
                from modules.humanoid.metalearn.collector import record_deploy
                record_deploy("fail", {"error": str(e), "steps": steps})
            except Exception:
                pass
            return {"ok": False, "data": {"steps": steps, "switched": False, "error": str(e)}, "ms": int((time.perf_counter() - t0) * 1000), "error": str(e)}
    _audit("promote_success", True, {"steps": steps, "active_port": active_port}, None, int((time.perf_counter() - t0) * 1000))
    try:
        from modules.humanoid.metalearn.collector import record_deploy
        record_deploy("ok", {"steps": steps, "active_port": active_port})
    except Exception:
        pass
    # Post-promote: GA auto-check if enabled
    _run_ga_post_update(active_port, last_health, min_score)
    return {"ok": True, "data": {"steps": steps, "switched": True}, "ms": int((time.perf_counter() - t0) * 1000), "error": None}


def _run_ga_post_update(active_port: int, last_health: Dict[str, Any], min_score: int) -> None:
    """After promote: health + GA run. If health < min, create ApprovalItem 'rollback recommended'."""
    if not _env_bool("GA_POST_UPDATE_AUTOCHECK", True):
        return
    try:
        ga_enabled = os.getenv("GOVERNED_AUTONOMY_ENABLED", "true").strip().lower() in ("1", "true", "yes")
        if not ga_enabled:
            return
        from modules.humanoid.deploy.healthcheck import run_health_verbose
        base = f"http://127.0.0.1:{active_port}"
        time.sleep(2)
        h = run_health_verbose(base_url=base, active_port=active_port)
        score = h.get("score", 0)
        ga_min = int(os.getenv("GA_POST_DEPLOY_HEALTH_MIN", "75") or 75)
        if score < ga_min:
            try:
                from modules.humanoid.approvals.store import create as approval_create
                approval_create(
                    action="ga_rollback_recommended",
                    payload={"score": score, "min": ga_min, "reason": "post-deploy health below threshold"},
                    risk="critical",
                )
                _audit("ga_post_update", False, {"score": score, "min": ga_min}, "rollback recommended", 0)
            except Exception:
                pass
        else:
            from modules.humanoid.ga.cycle import run_cycle
            run_cycle(scope="repo", mode="plan_only", max_findings=5)
    except Exception:
        pass
