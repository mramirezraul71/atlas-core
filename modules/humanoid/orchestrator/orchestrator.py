"""Autonomy orchestrator: plan -> propose -> (approve) -> execute -> audit."""
from __future__ import annotations

import logging
import os
import time
from typing import Any, Dict, List, Optional

from .critic import suggest_fix_for_failed_step, validate_no_destructive, validate_plan_structure, validate_step_result
from .memory import load_task, new_task_id, save_task
from .models import Plan, Result, Step, Task

_log = logging.getLogger("humanoid.orchestrator")

PLAN_TIMEOUT_SEC = 15
EXECUTE_STEP_TIMEOUT_SEC = 60


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("orchestrator", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


def _get_planner() -> Any:
    try:
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        autonomy = k.registry.get("autonomy")
        if autonomy and hasattr(autonomy, "planner"):
            return autonomy.planner
    except Exception:
        pass
    return None


def _get_hands() -> Any:
    try:
        from modules.humanoid import get_humanoid_kernel
        k = get_humanoid_kernel()
        return k.registry.get("hands")
    except Exception:
        pass
    return None


# Fallback determinista cuando el LLM no responde (timeout/error)
_DECOMPOSE_FALLBACK = {
    "ok": False,
    "steps": [],
    "error": "plan_unavailable",
    "fallback": True,
    "message": "Plan no disponible. Reintente o use modo manual.",
}


def _decompose(goal: str, fast: bool = True) -> Dict[str, Any]:
    """Convert goal into steps (definition of done per step). Uses planner. Fallback determinista si LLM falla."""
    planner = _get_planner()
    if not planner:
        return {**_DECOMPOSE_FALLBACK, "error": "Planner not available"}
    try:
        r = planner.plan(goal, fast=fast)
    except Exception as e:
        _log.warning("Planner exception: %s", e)
        return {**_DECOMPOSE_FALLBACK, "error": str(e)[:200]}
    if not r.get("ok"):
        return {
            **_DECOMPOSE_FALLBACK,
            "steps": r.get("steps", []),
            "error": r.get("error") or "LLM failed",
        }
    raw = r.get("steps", [])
    steps = []
    for i, s in enumerate(raw):
        steps.append({
            "id": f"step_{i+1}",
            "description": s if isinstance(s, str) else str(s),
            "definition_of_done": None,
            "status": "pending",
        })
    return {"ok": True, "steps": steps, "raw_steps": raw, "error": None}


def run_goal(goal: str, mode: str = "plan_only", fast: bool = True) -> Dict[str, Any]:
    """
    Pipeline: Goal → Decompose → Plan → Execute → Verify → Critic → Store → Report.
    Modos: plan_only (solo plan); execute_controlled (plan + pasos solo vía execute_step con approve);
    execute / execute_auto (ejecutar todos los pasos según policy).
    Returns {ok, plan, steps, task_id, execution_log?, artifacts?, error, ms}.
    """
    t0 = time.perf_counter()
    task_id = new_task_id()
    if not goal or not str(goal).strip():
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal", False, {"task_id": task_id}, "empty goal", ms)
        return {"ok": False, "plan": None, "steps": [], "task_id": task_id, "execution_log": [], "artifacts": [], "error": "empty goal", "ms": ms}

    dec = _decompose(goal.strip(), fast=fast)
    if not dec.get("ok"):
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal", False, {"task_id": task_id}, dec.get("error"), ms)
        out = {"ok": False, "plan": None, "steps": dec.get("steps", []), "task_id": task_id, "execution_log": [], "artifacts": [], "error": dec.get("error"), "ms": ms}
        if dec.get("fallback"):
            out["fallback"] = True
            out["message"] = dec.get("message", _DECOMPOSE_FALLBACK["message"])
        return out

    steps_data = dec.get("steps", [])
    ok_plan, err_plan = validate_plan_structure({"goal": goal, "steps": [s.get("description") for s in steps_data]})
    if not ok_plan:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "plan": None, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": err_plan, "ms": ms}

    plan = {"goal": goal, "steps": steps_data, "raw_steps": dec.get("raw_steps", [])}
    execution_log = []
    artifacts = []
    thread_id = None

    save_task(task_id, goal, plan, execution_log, "planned")
    try:
        from modules.humanoid.memory_engine import store_plan as mem_store_plan, ensure_thread
        thread_id = ensure_thread(None, goal[:200])
        mem_store_plan(goal, plan, task_id=task_id, thread_id=thread_id)
    except Exception:
        pass

    if mode == "plan_only":
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal", True, {"task_id": task_id, "mode": "plan_only", "steps_count": len(steps_data)}, None, ms)
        return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": None, "ms": ms}

    if mode == "execute_controlled":
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal", True, {"task_id": task_id, "mode": "execute_controlled", "steps_count": len(steps_data)}, None, ms)
        return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": None, "ms": ms, "message": "Ejecute cada paso vía POST /agent/step/execute con approve=true"}

    # execute / execute_auto: run steps; on failure use critic and one retry with suggested fix
    MAX_RETRIES_PER_STEP = 1
    for i, step in enumerate(steps_data):
        desc = step.get("description", "")
        if not validate_no_destructive(desc):
            execution_log.append({"step_id": step.get("id"), "status": "skipped", "error": "destructive step blocked"})
            continue
        step_result = _execute_step_internal(step, task_id)
        # Replan con critic: si falló, critic sugiere revisión y reintentamos una vez
        retries = 0
        while step_result.get("status") != "success" and retries < MAX_RETRIES_PER_STEP:
            revised_desc, _hint = suggest_fix_for_failed_step(desc, step_result, goal)
            step_revised = {**step, "description": revised_desc}
            step_result = _execute_step_internal(step_revised, task_id)
            retries += 1
        execution_log.append(step_result)
        if step_result.get("artifacts"):
            artifacts.extend(step_result["artifacts"])
        try:
            from modules.humanoid.memory_engine import store_run
            store_run(task_id, step_result.get("step_id"), step_result.get("status") == "success", result=step_result.get("result"), error=step_result.get("error"))
        except Exception:
            pass

    save_task(task_id, goal, plan, execution_log, "completed")
    for art in artifacts:
        try:
            from modules.humanoid.memory_engine import store_artifact
            store_artifact(task_id, None, "run_artifact", str(art)[:512])
        except Exception:
            pass
    # Reflect: resumen incremental en memoria
    if thread_id:
        try:
            from modules.humanoid.memory_engine import add_summary
            summary = f"Goal: {goal[:100]}. Steps: {len(steps_data)}, completed: {len([e for e in execution_log if e.get('status') == 'success'])}, artifacts: {len(artifacts)}."
            add_summary(thread_id, summary)
        except Exception:
            pass
    ms = int((time.perf_counter() - t0) * 1000)
    _audit("orchestrator", "run_goal", True, {"task_id": task_id, "mode": "execute", "steps": len(steps_data)}, None, ms)
    return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": execution_log, "artifacts": artifacts, "error": None, "ms": ms}


def _execute_step_internal(step: Dict[str, Any], task_id: str) -> Dict[str, Any]:
    """Execute one step (hands/shell). Returns {step_id, status, result?, error?, artifacts?}."""
    step_id = step.get("id", "")
    desc = step.get("description", "")
    hands = _get_hands()
    if not hands or not hasattr(hands, "shell"):
        return {"step_id": step_id, "status": "failed", "error": "hands not available", "artifacts": []}
    # Only execute if description looks like a shell command (starts with allowed prefix)
    import shlex
    parts = shlex.split(desc) or [desc]
    first = (parts[0] or "").lower()
    from modules.humanoid.policy import get_policy_engine
    from modules.humanoid.policy import ActorContext
    actor = ActorContext(actor="orchestrator", role=os.getenv("POLICY_DEFAULT_ROLE", "owner"))
    decision = get_policy_engine().can(actor, "hands", "exec_command", target=desc)
    if not decision.allow:
        return {"step_id": step_id, "status": "skipped", "error": decision.reason or "policy denied", "artifacts": []}
    result = hands.shell.run(desc, timeout_sec=min(EXECUTE_STEP_TIMEOUT_SEC, 60))
    return {
        "step_id": step_id,
        "status": "success" if result.get("ok") else "failed",
        "result": result,
        "error": result.get("error"),
        "artifacts": [],
    }


def execute_step(task_id: str, step_id: str, approve: bool = False) -> Dict[str, Any]:
    """Execute a single step after approval. Returns {ok, result, artifacts, error, ms}."""
    t0 = time.perf_counter()
    if not approve:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "result": None, "artifacts": [], "error": "approve must be true", "ms": ms}
    data = load_task(task_id)
    if not data:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "result": None, "artifacts": [], "error": "task not found", "ms": ms}
    steps = data.get("plan", {}).get("steps", [])
    step = next((s for s in steps if s.get("id") == step_id), None)
    if not step:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "result": None, "artifacts": [], "error": "step not found", "ms": ms}
    out = _execute_step_internal(step, task_id)
    ms = int((time.perf_counter() - t0) * 1000)
    return {"ok": out.get("status") == "success", "result": out, "artifacts": out.get("artifacts", []), "error": out.get("error"), "ms": ms}
