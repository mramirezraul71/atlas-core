"""Executive: orchestrates agents, decides next step. Only Executive authorizes execution."""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

from .base import BaseAgent
from .registry import get_agent_registry

_log = logging.getLogger("humanoid.agents.executive")

EXECUTIVE_TIMEOUT_SEC = 90


def _audit(module: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("agents", "system", module, action, ok, ms, error, payload, None)
    except Exception:
        pass


def _store_in_memory(goal: str, plan: Dict[str, Any], task_id: str, thread_id: Optional[str]) -> None:
    try:
        from modules.humanoid.memory_engine import ensure_thread, store_plan
        tid = ensure_thread(thread_id, goal[:200])
        store_plan(goal, plan, task_id=task_id, thread_id=tid)
    except Exception:
        pass


class ExecutiveAgent(BaseAgent):
    name = "executive"
    timeout_sec = EXECUTIVE_TIMEOUT_SEC

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """Orchestrate: Strategist -> Architect -> Engineer -> Reviewer -> Optimizer. Return plan + decision."""
        goal = (context.get("goal") or "").strip()
        depth = max(1, min(5, int(context.get("depth", 1))))
        if not goal:
            return {"ok": False, "output": {}, "error": "empty goal", "ms": 0}

        reg = get_agent_registry()
        pipeline: List[Dict[str, Any]] = []
        ctx: Dict[str, Any] = {"goal": goal}

        if depth == 1:
            # Single-agent path: use existing planner output if provided
            steps = context.get("steps") or []
            if steps:
                pipeline.append({"agent": "engineer", "output": {"steps": steps}})
                ctx["steps"] = steps
            else:
                out = self._run_chain(reg, ctx, depth=1)
                return out
        else:
            # Multi-agent chain
            strategist = reg.get("strategist")
            if strategist:
                r = strategist.run(ctx)
                pipeline.append({"agent": "strategist", "output": r.get("output")})
                if r.get("ok") and isinstance(r.get("output"), dict):
                    ctx["strategy_steps"] = r["output"].get("strategy_steps", [])

            architect = reg.get("architect")
            if architect and depth >= 2:
                r = architect.run(ctx)
                pipeline.append({"agent": "architect", "output": r.get("output")})
                if r.get("ok") and isinstance(r.get("output"), dict):
                    ctx["components"] = r["output"].get("components", [])

            engineer = reg.get("engineer")
            if engineer and depth >= 2:
                r = engineer.run(ctx)
                pipeline.append({"agent": "engineer", "output": r.get("output")})
                if r.get("ok") and isinstance(r.get("output"), dict):
                    ctx["steps"] = r["output"].get("steps", [])

            reviewer = reg.get("reviewer")
            if reviewer and depth >= 4:
                r = reviewer.run(ctx)
                pipeline.append({"agent": "reviewer", "output": r.get("output")})
                if r.get("ok") and isinstance(r.get("output"), dict):
                    approved = r["output"].get("approved", False)
                    if not approved:
                        return {
                            "ok": True,
                            "output": {
                                "decision": "replan",
                                "reason": r["output"].get("reason", "reviewer rejected"),
                                "pipeline": pipeline,
                                "steps": ctx.get("steps", []),
                                "plan": {"goal": goal, "steps": [{"id": f"s{i+1}", "description": s} for i, s in enumerate(ctx.get("steps", []))]},
                            },
                            "error": None,
                            "ms": 0,
                        }

            optimizer = reg.get("optimizer")
            if optimizer and depth >= 5:
                r = optimizer.run(ctx)
                pipeline.append({"agent": "optimizer", "output": r.get("output")})

        steps = ctx.get("steps") or []
        plan = {"goal": goal, "steps": [{"id": f"step_{i+1}", "description": s} for i, s in enumerate(steps)], "raw_steps": steps}
        return {
            "ok": True,
            "output": {
                "decision": "approve",
                "plan": plan,
                "pipeline": pipeline,
                "steps": steps,
            },
            "error": None,
            "ms": 0,
        }

    def _run_chain(self, reg: Any, ctx: Dict[str, Any], depth: int = 1) -> Dict[str, Any]:
        engineer = reg.get("engineer")
        if engineer:
            r = engineer.run(ctx)
            if r.get("ok") and isinstance(r.get("output"), dict):
                ctx["steps"] = r["output"].get("steps", [])
        steps = ctx.get("steps") or []
        plan = {"goal": ctx.get("goal", ""), "steps": [{"id": f"step_{i+1}", "description": s} for i, s in enumerate(steps)], "raw_steps": steps}
        return {"ok": True, "output": {"decision": "approve", "plan": plan, "pipeline": [], "steps": steps}, "error": None, "ms": 0}


def run_multi_agent_goal(goal: str, depth: int = 2, mode: str = "plan_only") -> Dict[str, Any]:
    """
    Entry point: Goal -> Executive -> pipeline -> plan + decision.
    Execution is NOT done here; caller (orchestrator/API) runs steps when mode allows and policy approves.
    """
    t0 = time.perf_counter()
    try:
        from modules.humanoid.orchestrator.memory import new_task_id
        task_id = new_task_id()
    except Exception:
        task_id = ""
    reg = get_agent_registry()
    executive = reg.get("executive")
    if not executive:
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("executive", "run_goal", False, None, "executive agent not registered", ms)
        return {"ok": False, "task_id": task_id, "plan": None, "steps": [], "decision": "error", "pipeline": [], "error": "executive not found", "ms": ms}
    ctx = {"goal": goal, "depth": depth, "mode": mode}
    out = executive.run(ctx)
    ms = int((time.perf_counter() - t0) * 1000)
    if not out.get("ok"):
        _audit("executive", "run_goal", False, {"task_id": task_id}, out.get("error"), ms)
        return {"ok": False, "task_id": task_id, "plan": None, "steps": [], "decision": "error", "pipeline": [], "error": out.get("error"), "ms": ms}
    output = out.get("output") or {}
    plan = output.get("plan") or {}
    steps = output.get("steps") or plan.get("steps") or []
    if isinstance(steps, list) and steps and not isinstance(steps[0], dict):
        steps = [{"id": f"step_{i+1}", "description": s} for i, s in enumerate(steps)]
    _audit("executive", "run_goal", True, {"task_id": task_id, "depth": depth, "decision": output.get("decision")}, None, ms)
    return {
        "ok": True,
        "task_id": task_id,
        "plan": plan,
        "steps": steps,
        "decision": output.get("decision", "approve"),
        "pipeline": output.get("pipeline", []),
        "error": None,
        "ms": ms,
    }


agent = ExecutiveAgent()
