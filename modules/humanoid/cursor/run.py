"""Cursor run: plan with AI router, optional step execution, evidence."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from .status import set_last_run


def cursor_run(
    goal: str,
    mode: str = "plan_only",
    context: Optional[Dict[str, Any]] = None,
    prefer_free: bool = True,
    allow_paid: bool = False,
) -> Dict[str, Any]:
    """
    Orchestrate: build plan (AI router), optionally execute steps, return summary + evidence.
    mode: plan_only | controlled | auto
    """
    t0 = time.perf_counter()
    context = context or {}
    steps: List[Dict[str, Any]] = []
    summary = ""
    evidence: List[str] = []
    next_actions: List[str] = []
    approval_id: Optional[str] = None
    model_routing: Dict[str, Any] = {}
    error: Optional[str] = None

    try:
        # 1) Build plan using AI layer (FAST/REASON) or humanoid planner
        plan_prompt = f"Break down into 3-5 concrete steps to achieve: {goal}. Output only a short numbered list, one step per line."
        try:
            from modules.humanoid.ai.router import route_and_run
            out, decision, meta = route_and_run(plan_prompt, intent_hint="reason", prefer_free=prefer_free)
            model_routing = {
                "provider_id": decision.provider_id,
                "model_key": decision.model_key,
                "route": decision.route,
                "reason": decision.reason,
                "latency_ms": meta.get("latency_ms", 0),
            }
            raw = (out or "").strip()
            for line in raw.split("\n"):
                line = line.strip()
                if not line:
                    continue
                if line[0].isdigit() or line.startswith("-") or line.startswith("*"):
                    steps.append({"description": line.lstrip("0123456789.-)* "), "status": "pending"})
                else:
                    steps.append({"description": line, "status": "pending"})
            if not steps and raw:
                steps = [{"description": raw[:200], "status": "pending"}]
        except Exception as e:
            try:
                from modules.humanoid import get_humanoid_kernel
                k = get_humanoid_kernel()
                autonomy = k.registry.get("autonomy")
                if autonomy and hasattr(autonomy, "planner"):
                    result = autonomy.planner.plan(goal, fast=True)
                    if result.get("ok") and result.get("steps"):
                        steps = [{"description": s.get("description", str(s)), "status": "pending"} for s in result["steps"]]
                        model_routing = {"source": "humanoid_planner", "reason": "fallback"}
            except Exception:
                pass
            if not steps:
                steps = [{"description": goal, "status": "pending"}]
                error = str(e)

        summary = f"Plan: {len(steps)} steps. Mode={mode}."
        if mode == "plan_only":
            next_actions = ["Use POST /cursor/step/execute to run a step", "Or set mode=controlled/auto for execution"]
        else:
            next_actions = ["Execution not yet implemented for controlled/auto; use plan_only or /agent/goal"]

        ms = int((time.perf_counter() - t0) * 1000)
        data = {
            "goal": goal,
            "mode": mode,
            "steps": steps,
            "summary": summary,
            "evidence": evidence,
            "next_actions": next_actions,
            "model_routing": model_routing,
            "approval_id": approval_id,
            "ms": ms,
        }
        set_last_run(data)
        return {"ok": error is None, "data": data, "ms": ms, "error": error}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        set_last_run({"goal": goal, "mode": mode, "error": str(e), "ms": ms})
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}
