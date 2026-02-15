"""Job runner: update_check, shell_command, llm_plan, custom. Timeout 20s, Policy + Audit."""
from __future__ import annotations

import json
import logging
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, Optional

RUNNER_TIMEOUT_SEC = 20
_scheduler_actor = "scheduler"

_log = logging.getLogger("humanoid.scheduler.runner")


def run_job_sync(job: Dict[str, Any]) -> Dict[str, Any]:
    """Run one job synchronously. Returns {ok, result_json, error, ms}. No update_apply."""
    t0 = time.perf_counter()
    kind = (job.get("kind") or "custom").strip().lower()
    payload = job.get("payload") or {}
    jid = job.get("id", "")
    name = job.get("name", "")

    try:
        if kind == "update_check":
            out = _run_update_check(payload)
        elif kind == "shell_command":
            out = _run_shell_command(payload)
        elif kind == "llm_plan":
            out = _run_llm_plan(payload)
        elif kind == "system_update":
            out = _run_system_update(payload)
        elif kind == "ci_improve":
            out = _run_ci_improve(payload)
        elif kind == "ga_cycle":
            out = _run_ga_cycle(payload)
        elif kind == "metalearn_cycle":
            out = _run_metalearn_cycle(payload)
        elif kind == "remote_hands":
            out = _run_remote_hands(payload)
        elif kind == "ans_cycle":
            out = _run_ans_cycle(payload)
        elif kind == "nervous_cycle":
            out = _run_nervous_cycle(payload)
        elif kind == "makeplay_scanner":
            out = _run_makeplay_scanner(payload)
        elif kind == "repo_monitor_cycle":
            out = _run_repo_monitor_cycle(payload)
        elif kind == "repo_monitor_after_fix":
            out = _run_repo_monitor_after_fix(payload)
        else:
            out = {"ok": True, "result": "no-op", "kind": kind}
    except Exception as e:
        _log.exception("Job %s run failed: %s", jid, e)
        ms = int((time.perf_counter() - t0) * 1000)
        try:
            from modules.humanoid.metalearn.collector import record_scheduler_job
            record_scheduler_job(jid, kind, "fail", ms, str(e))
        except Exception:
            pass
        return {"ok": False, "result_json": None, "error": str(e), "ms": ms}

    ms = int((time.perf_counter() - t0) * 1000)
    ok = out.get("ok", False)
    try:
        from modules.humanoid.metalearn.collector import record_scheduler_job
        record_scheduler_job(jid, kind, "ok" if ok else "fail", ms, out.get("error"))
    except Exception:
        pass
    result_json = json.dumps(out) if isinstance(out, dict) else json.dumps({"ok": ok, "raw": str(out)})
    return {"ok": ok, "result_json": result_json, "error": out.get("error"), "ms": ms}


def _run_update_check(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Update-check only (no apply). Uses humanoid update module plan()."""
    required = payload.get("required_packages") or ["fastapi", "uvicorn", "httpx", "pydantic"]
    try:
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        update_mod = k.registry.get("update")
        if not update_mod or not hasattr(update_mod, "updater"):
            return {"ok": False, "error": "update module not available"}
        plan_result = update_mod.updater.plan(required)
        return plan_result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_remote_hands(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run command via cluster dispatcher (local or best remote node). payload: {command, timeout_sec}."""
    command = (payload.get("command") or "").strip()
    if not command:
        return {"ok": False, "error": "missing command in payload"}
    try:
        from modules.humanoid.dispatch.dispatcher import run_hands
        result = run_hands(command, timeout_sec=payload.get("timeout_sec", 30), prefer_remote=payload.get("prefer_remote", True))
        return {"ok": result.get("ok", False), "data": result.get("data"), "error": result.get("error"), "correlation_id": result.get("correlation_id")}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_shell_command(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run command via Hands SafeShellExecutor + Policy. payload: {command, cwd}."""
    command = (payload.get("command") or "").strip()
    if not command:
        return {"ok": False, "error": "missing command in payload"}
    cwd = payload.get("cwd") or None
    try:
        from modules.humanoid import get_humanoid_kernel
        from modules.humanoid.policy import ActorContext, get_policy_engine
        actor = ActorContext(actor=_scheduler_actor, role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))
        decision = get_policy_engine().can(actor, "hands", "exec_command", target=command)
        if not decision.allow:
            return {"ok": False, "error": decision.reason or "policy denied"}
        k = get_humanoid_kernel()
        hands = k.registry.get("hands")
        if not hands or not hasattr(hands, "shell"):
            return {"ok": False, "error": "hands module not available"}
        result = hands.shell.run(command, cwd=cwd, timeout_sec=min(RUNNER_TIMEOUT_SEC, 60), actor=actor)
        return result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_llm_plan(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Call humanoid planner (FAST). payload: {goal}."""
    goal = (payload.get("goal") or "").strip()
    if not goal:
        return {"ok": False, "steps": [], "error": "missing goal in payload"}
    try:
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        autonomy = k.registry.get("autonomy")
        if not autonomy or not hasattr(autonomy, "planner"):
            return {"ok": False, "steps": [], "error": "autonomy module not available"}
        result = autonomy.planner.plan(goal, fast=True)
        return result
    except Exception as e:
        return {"ok": False, "steps": [], "error": str(e)}


def _run_system_update(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run git-based update engine: check only (no apply) or full apply if policy allows. payload: {action: 'check'|'apply'}."""
    action = (payload.get("action") or "check").strip().lower()
    try:
        from modules.humanoid.update.update_engine import apply as update_apply, check as update_check, update_enabled
        if not update_enabled():
            return {"ok": False, "error": "UPDATE_ENABLED=false"}
        if action == "apply":
            result = update_apply()
        else:
            result = update_check()
        return result.get("data") or result if isinstance(result.get("data"), dict) else {"ok": result.get("ok"), "data": result, "error": result.get("error")}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ci_improve(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run CI improve cycle (plan_only). payload: {scope, mode, depth, max_items}."""
    scope = (payload.get("scope") or "repo").strip()
    mode = (payload.get("mode") or "plan_only").strip()
    depth = max(1, min(5, int(payload.get("depth") or 2)))
    max_items = payload.get("max_items") or 10
    try:
        from modules.humanoid.ci import run_improve
        result = run_improve(scope=scope, mode=mode, depth=depth, max_items=max_items)
        return result.get("data") or result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ga_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run Governed Autonomy cycle. payload: {scope, mode, max_findings}."""
    scope = (payload.get("scope") or "all").strip()
    mode = (payload.get("mode") or os.getenv("GA_MODE", "plan_only")).strip()
    max_findings = payload.get("max_findings")
    try:
        from modules.humanoid.ga.cycle import run_cycle
        result = run_cycle(scope=scope, mode=mode, max_findings=max_findings)
        return result.get("data") or result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_metalearn_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run meta-learning cycle: train + tune + report."""
    try:
        from modules.humanoid.metalearn.cycle import run_cycle
        result = run_cycle()
        return result
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_ans_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run ANS (Autonomic Nervous System) cycle: checks -> heals -> report."""
    try:
        from modules.humanoid.ans.engine import run_ans_cycle
        mode = payload.get("mode") or os.getenv("ANS_MODE", "auto")
        timeout = int(payload.get("timeout_sec") or os.getenv("ANS_INTERVAL_SECONDS", "30") or "30")
        return run_ans_cycle(mode=mode, timeout_sec=min(timeout, 60))
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_nervous_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Run Sistema Nervioso: sensores -> score -> persistencia + bitácora/incidentes."""
    try:
        from modules.humanoid.nervous.engine import run_nervous_cycle
        mode = (payload.get("mode") or os.getenv("NERVOUS_MODE", "auto")).strip() or "auto"
        return run_nervous_cycle(mode=mode)
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_makeplay_scanner(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Scanner permanente: estado ATLAS -> webhook MakePlay."""
    try:
        from modules.humanoid.comms.makeplay_scanner import run_scan
        return run_scan()
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_repo_monitor_cycle(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Ejecutar un ciclo del monitor de repo (fetch + status + bitácora). Usa scripts/repo_monitor.py --cycle."""
    import subprocess
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "repo_monitor.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/repo_monitor.py not found"}
    try:
        r = subprocess.run(
            [sys.executable, str(script), "--cycle"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=120,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
                "REPO_MONITOR_CONFIG": str(root / "config" / "repo_monitor.yaml"),
            },
        )
        return {
            "ok": r.returncode == 0,
            "exit_code": r.returncode,
            "stdout": (r.stdout or "")[:500],
            "stderr": (r.stderr or "")[:300],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "repo_monitor cycle timeout 120s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}


def _run_repo_monitor_after_fix(payload: Dict[str, Any]) -> Dict[str, Any]:
    """Ejecutar commit + push del monitor de repo (actualizacion automatica al remoto)."""
    import subprocess
    root = os.getenv("ATLAS_REPO_PATH") or os.getenv("ATLAS_PUSH_ROOT")
    if not root:
        root = str(Path(__file__).resolve().parent.parent.parent.parent)
    root = Path(root).resolve()
    script = root / "scripts" / "repo_monitor.py"
    if not script.is_file():
        return {"ok": False, "error": "scripts/repo_monitor.py not found"}
    try:
        r = subprocess.run(
            [sys.executable, str(script), "--after-fix"],
            cwd=str(root),
            capture_output=True,
            text=True,
            timeout=90,
            env={
                **os.environ,
                "ATLAS_REPO_PATH": str(root),
                "ATLAS_PUSH_ROOT": str(root),
                "REPO_MONITOR_CONFIG": str(root / "config" / "repo_monitor.yaml"),
            },
        )
        return {
            "ok": r.returncode == 0,
            "exit_code": r.returncode,
            "stdout": (r.stdout or "")[:500],
            "stderr": (r.stderr or "")[:300],
        }
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": "repo_monitor after-fix timeout 90s"}
    except Exception as e:
        return {"ok": False, "error": str(e)}
