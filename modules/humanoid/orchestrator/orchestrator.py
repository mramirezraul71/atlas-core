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


def _llm_decompose_fallback(goal: str, fast: bool = True) -> Dict[str, Any]:
    """Fallback: use LLM directly to decompose a goal into steps when kernel planner is unavailable."""
    try:
        import importlib
        mod = importlib.import_module("atlas_adapter.atlas_http_api")
        call_fn = getattr(mod, "_direct_model_call", None)
        if not call_fn:
            return None
        prompt = (
            f"Descompone este objetivo en 2-5 pasos concretos y ejecutables. "
            f"Responde SOLO con una lista numerada (1. ... 2. ... etc), sin explicaciones extra.\n\n"
            f"Objetivo: {goal.strip()}"
        )
        result = call_fn("auto", prompt, use_config=False, prefer_fast=fast)
        if not result.get("ok") or not result.get("output"):
            return None
        # Parse numbered list from LLM output
        import re
        lines = result["output"].strip().split("\n")
        steps = []
        for line in lines:
            line = line.strip()
            m = re.match(r"^\d+[.)\-]\s*(.+)", line)
            if m:
                steps.append(m.group(1).strip())
            elif line and len(steps) == 0 and len(lines) <= 5:
                steps.append(line)
        if steps:
            return {"ok": True, "steps": steps, "model_used": result.get("model_used")}
    except Exception as e:
        _log.warning("LLM decompose fallback failed: %s", e)
    return None


def _decompose(goal: str, fast: bool = True) -> Dict[str, Any]:
    """Convert goal into steps (definition of done per step). Uses planner. Fallback determinista si LLM falla."""
    planner = _get_planner()
    if not planner:
        # Fallback: try direct LLM decomposition
        llm_result = _llm_decompose_fallback(goal, fast=fast)
        if llm_result and llm_result.get("ok"):
            raw = llm_result["steps"]
            steps = []
            for i, s in enumerate(raw):
                steps.append({
                    "id": f"step_{i+1}",
                    "description": s if isinstance(s, str) else str(s),
                    "definition_of_done": None,
                    "status": "pending",
                })
            _log.info("Decomposed via LLM fallback: %d steps (model=%s)", len(steps), llm_result.get("model_used", "?"))
            return {"ok": True, "steps": steps, "raw_steps": raw, "error": None}
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


def _env_int(name: str, default: int, *, min_v: int = 0, max_v: int = 100) -> int:
    try:
        v = int(os.getenv(name, str(default)) or default)
        return max(min_v, min(v, max_v))
    except Exception:
        return default


def _try_heal_step(step: Dict[str, Any], step_result: Dict[str, Any], *, goal: str, task_id: str) -> Dict[str, Any]:
    """
    Intento de recuperación antes de replanear:
    - Usa HealingOrchestrator (incluye FailureMemory persistente) si está disponible.
    - Provee retry_func que re-ejecuta el paso y lanza si falla (para backoff).
    Retorna {"attempted": bool, "success": bool, "message": str}.
    """
    err = (step_result.get("error") or "")[:200]
    if not err:
        return {"attempted": False, "success": False, "message": "no_error"}
    try:
        from autonomous.self_healing.healing_orchestrator import HealingOrchestrator
    except Exception:
        return {"attempted": False, "success": False, "message": "healing_unavailable"}

    def _retry_once() -> Dict[str, Any]:
        r = _execute_step_internal(step, task_id)
        if r.get("status") != "success":
            raise Exception((r.get("error") or "step_failed")[:200])
        return r

    svc = "nexus" if ("8000" in err or "nexus" in err.lower()) else "push"
    ctx = {
        "service": svc,
        "goal": goal[:300],
        "step": (step.get("description") or "")[:500],
        "task_id": task_id,
        "retry_func": _retry_once,
    }
    try:
        orch = HealingOrchestrator()
        res = orch.handle_error(Exception(err), ctx)
        return {"attempted": True, "success": bool(res.get("success")), "message": str(res.get("message") or "")[:200], "strategy": res.get("strategy_used")}
    except Exception as e:
        return {"attempted": True, "success": False, "message": str(e)[:200]}


def _replan_goal(goal: str, *, completed: List[str], last_failed_step: str, last_error: str, fast: bool) -> Dict[str, Any]:
    """
    Replanificación con backtracking:
    - Indica al planner qué ya se completó para no repetir.
    - Provee el error y el paso fallido para proponer alternativa.
    """
    completed_s = "; ".join([c[:120] for c in completed[-12:]]) if completed else ""
    ctx_goal = (
        f"{goal.strip()}\n\n"
        f"Contexto de replanificación (no repetir lo ya hecho):\n"
        f"- Pasos completados: {completed_s or '(ninguno)'}\n"
        f"- Paso fallido: {last_failed_step[:220]}\n"
        f"- Error observado: {last_error[:220]}\n"
        f"Propón un plan alterno para lograr el objetivo evitando repetir pasos completados."
    )
    return _decompose(ctx_goal, fast=fast)


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
    MAX_RETRIES_PER_STEP = _env_int("ORCH_MAX_RETRIES_PER_STEP", 1, min_v=0, max_v=5)
    MAX_REPLANS_PER_GOAL = _env_int("ORCH_MAX_REPLANS_PER_GOAL", 2, min_v=0, max_v=5)
    replans_used = 0
    completed_descs: List[str] = []
    open_mttr_id: Optional[int] = None

    i = 0
    while i < len(steps_data):
        step = steps_data[i]
        desc = step.get("description", "")
        if not validate_no_destructive(desc):
            execution_log.append({"step_id": step.get("id"), "status": "skipped", "error": "destructive step blocked"})
            i += 1
            continue
        step_result = _execute_step_internal(step, task_id)
        # Retry con critic: si falló, critic sugiere revisión y reintentamos N veces
        retries = 0
        while step_result.get("status") != "success" and retries < MAX_RETRIES_PER_STEP:
            revised_desc, _hint = suggest_fix_for_failed_step(desc, step_result, goal)
            step_revised = {**step, "description": revised_desc}
            step_result = _execute_step_internal(step_revised, task_id)
            retries += 1

        # Si sigue fallando: intentar healing (FailureMemory + estrategias) antes de replanear
        if step_result.get("status") != "success":
            # Abrir MTTR si no hay uno abierto
            if open_mttr_id is None:
                try:
                    from modules.humanoid.ans.mttr import start as mttr_start
                    from modules.humanoid.governance.dynamic_risk import _sig as _sig_fn  # type: ignore

                    open_mttr_id = mttr_start(
                        "orchestrator",
                        _sig_fn(str(step_result.get("error") or "")),
                        {"task_id": task_id, "goal": goal[:200], "step": desc[:200]},
                    )
                except Exception:
                    open_mttr_id = None
            heal = _try_heal_step(step, step_result, goal=goal, task_id=task_id)
            if heal.get("attempted"):
                execution_log.append({"step_id": step.get("id"), "status": "healing", "result": heal})
            if heal.get("success"):
                # reintento inmediato del paso
                step_result = _execute_step_internal(step, task_id)
                if step_result.get("status") == "success" and open_mttr_id is not None:
                    try:
                        from modules.humanoid.ans.mttr import resolve as mttr_resolve

                        mttr_resolve(open_mttr_id, ok=True)
                    except Exception:
                        pass
                    open_mttr_id = None

        execution_log.append(step_result)
        if step_result.get("status") == "success":
            completed_descs.append(desc)
        if step_result.get("artifacts"):
            artifacts.extend(step_result["artifacts"])
        try:
            from modules.humanoid.memory_engine import store_run
            store_run(task_id, step_result.get("step_id"), step_result.get("status") == "success", result=step_result.get("result"), error=step_result.get("error"))
        except Exception:
            pass

        # Si falló de forma definitiva, replanear con backtracking si quedan intentos
        if step_result.get("status") != "success" and replans_used < MAX_REPLANS_PER_GOAL:
            replans_used += 1
            re = _replan_goal(goal, completed=completed_descs, last_failed_step=desc, last_error=str(step_result.get("error") or ""), fast=fast)
            if re.get("ok") and re.get("steps"):
                steps_data = re["steps"]
                plan = {"goal": goal, "steps": steps_data, "raw_steps": re.get("raw_steps", [])}
                execution_log.append({"status": "replan", "replans_used": replans_used, "reason": (step_result.get("error") or "")[:200]})
                save_task(task_id, goal, plan, execution_log, "planned")
                i = 0
                continue

        i += 1

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
    if open_mttr_id is not None:
        try:
            from modules.humanoid.ans.mttr import resolve as mttr_resolve

            mttr_resolve(open_mttr_id, ok=(len([e for e in execution_log if e.get("status") == "failed"]) == 0))
        except Exception:
            pass
    try:
        ok_steps = len([e for e in execution_log if e.get("status") == "success"])
        fail_steps = len([e for e in execution_log if e.get("status") == "failed"])
        from modules.humanoid.comms.ops_bus import emit as ops_emit
        ops_emit(
            "autonomy",
            "Ejecucion autonoma finalizada.",
            level="info" if fail_steps == 0 else "med",
            data={"task_id": task_id, "ms": ms, "replans": replans_used, "ok_steps": ok_steps, "failed_steps": fail_steps},
        )
    except Exception:
        pass
    _audit("orchestrator", "run_goal", True, {"task_id": task_id, "mode": "execute", "steps": len(steps_data), "replans_used": replans_used}, None, ms)
    return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": execution_log, "artifacts": artifacts, "error": None, "ms": ms, "replans_used": replans_used}


def run_goal_with_plan(goal: str, steps_list: List[str], mode: str = "plan_only") -> Dict[str, Any]:
    """Same as run_goal but use pre-computed steps (e.g. from multi-agent Executive). No _decompose."""
    t0 = time.perf_counter()
    task_id = new_task_id()
    if not goal or not str(goal).strip():
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal_with_plan", False, {"task_id": task_id}, "empty goal", ms)
        return {"ok": False, "plan": None, "steps": [], "task_id": task_id, "execution_log": [], "artifacts": [], "error": "empty goal", "ms": ms}
    steps_data = [{"id": f"step_{i+1}", "description": s if isinstance(s, str) else str(s), "definition_of_done": None, "status": "pending"} for i, s in enumerate(steps_list or [])]
    ok_plan, err_plan = validate_plan_structure({"goal": goal, "steps": [s.get("description") for s in steps_data]})
    if not ok_plan and steps_data:
        pass
    elif not ok_plan:
        ms = int((time.perf_counter() - t0) * 1000)
        return {"ok": False, "plan": None, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": err_plan or "invalid steps", "ms": ms}
    plan = {"goal": goal, "steps": steps_data, "raw_steps": steps_list or []}
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
        _audit("orchestrator", "run_goal_with_plan", True, {"task_id": task_id, "mode": "plan_only"}, None, ms)
        return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": None, "ms": ms}
    if mode == "execute_controlled":
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("orchestrator", "run_goal_with_plan", True, {"task_id": task_id, "mode": "execute_controlled"}, None, ms)
        return {"ok": True, "plan": plan, "steps": steps_data, "task_id": task_id, "execution_log": [], "artifacts": [], "error": None, "ms": ms, "message": "Ejecute cada paso vía POST /agent/step/execute con approve=true"}
    for i, step in enumerate(steps_data):
        desc = step.get("description", "")
        if not validate_no_destructive(desc):
            execution_log.append({"step_id": step.get("id"), "status": "skipped", "error": "destructive step blocked"})
            continue
        step_result = _execute_step_internal(step, task_id)
        retries = 0
        while step_result.get("status") != "success" and retries < 1:
            revised_desc, _ = suggest_fix_for_failed_step(desc, step_result, goal)
            step_result = _execute_step_internal({**step, "description": revised_desc}, task_id)
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
    if thread_id:
        try:
            from modules.humanoid.memory_engine import add_summary
            add_summary(thread_id, f"Goal: {goal[:100]}. Steps: {len(steps_data)}, completed: {len([e for e in execution_log if e.get('status') == 'success'])}.")
        except Exception:
            pass
    ms = int((time.perf_counter() - t0) * 1000)
    _audit("orchestrator", "run_goal_with_plan", True, {"task_id": task_id, "mode": "execute"}, None, ms)
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
