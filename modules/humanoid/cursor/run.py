"""Cursor run: plan with AI router, optional step execution, evidence. mode=auto = Cursor Super Mode."""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from .status import set_last_run


def _cursor_auto_execute(steps: List[Dict[str, Any]], goal: str) -> Dict[str, Any]:
    """Cursor Super Mode: ejecuta steps sin approval. Crea módulos, edita código, ejecuta tools, navega pantalla."""
    executed: List[Dict[str, Any]] = []
    evidence: List[str] = []
    for i, s in enumerate(steps):
        desc = (s.get("description") or "").lower()
        if not desc:
            continue
        r: Dict[str, Any] = {"step": i, "description": s.get("description"), "ok": False, "action": None}
        try:
            if "crear módulo" in desc or "create module" in desc:
                from modules.humanoid.selfprog import create_module
                name = desc.split()[-1][:30] if desc.split() else "new_module"
                res = create_module(name)
                r["ok"] = res.get("ok", False)
                r["action"] = "create_module"
            elif "instalar" in desc or "install" in desc:
                from modules.humanoid.selfprog import install_dependency
                pkg = desc.replace("instalar", "").replace("install", "").strip().split()[0] if desc.split() else ""
                if pkg:
                    res = install_dependency(pkg)
                    r["ok"] = res.get("ok", False)
                    r["action"] = "install_dependency"
            elif "click" in desc or "clic" in desc:
                from modules.humanoid.hands_eyes import execute_action, locate_element
                loc = locate_element(desc.split("en")[-1].strip()[:50] if "en" in desc else desc[:50])
                matches = loc.get("matches", [])
                if matches and matches[0].get("bbox"):
                    bbox = matches[0]["bbox"]
                    cx = bbox[0] + bbox[2] // 2 if len(bbox) >= 3 else bbox[0]
                    cy = bbox[1] + bbox[3] // 2 if len(bbox) >= 4 else bbox[1]
                    res = execute_action("click", {"x": cx, "y": cy}, verify_after=True)
                    r["ok"] = res.get("ok", False)
                    r["action"] = "click"
                    if res.get("evidence_after"):
                        evidence.append(res["evidence_after"])
            else:
                r["action"] = "skipped"
                r["ok"] = True
        except Exception as e:
            r["error"] = str(e)
        executed.append(r)
        s["status"] = "done" if r.get("ok") else "failed"
    ok_all = all(x.get("ok", False) for x in executed)
    return {"ok": ok_all, "executed": executed, "evidence": evidence}


def cursor_run(
    goal: str,
    mode: str = "plan_only",
    depth: int = 1,
    context: Optional[Dict[str, Any]] = None,
    prefer_free: bool = True,
    allow_paid: bool = False,
    profile: str = "owner",
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
        n_steps = min(7, max(3, 3 + depth))
        plan_prompt = f"Break down into {n_steps} concrete steps to achieve: {goal}. Output only a short numbered list, one step per line."
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
        auto_result: Dict[str, Any] = {}
        if mode == "plan_only":
            next_actions = ["Use POST /cursor/step/execute to run a step", "Or set mode=controlled/auto for execution"]
        elif mode == "auto":
            auto_result = _cursor_auto_execute(steps, goal)
            evidence = auto_result.get("evidence", [])
            summary = f"Plan: {len(steps)} steps. Auto executed: {sum(1 for x in auto_result.get('executed', []) if x.get('ok'))} ok."
            next_actions = [f"Auto mode: {len(evidence)} evidence saved. Report: ok={auto_result.get('ok')}"]
            if not auto_result.get("ok"):
                error = "auto_execute partial failure"
        else:
            next_actions = ["Execution not yet implemented for controlled; use plan_only or mode=auto"]

        ms = int((time.perf_counter() - t0) * 1000)
        data = {
            "goal": goal,
            "mode": mode,
            "depth": depth,
            "profile": profile,
            "steps": steps,
            "summary": summary,
            "evidence": evidence,
            "next_actions": next_actions,
            "model_routing": model_routing,
            "approval_id": approval_id,
            "ms": ms,
        }
        if mode == "auto":
            data["auto_result"] = auto_result
        set_last_run(data)
        return {"ok": error is None, "data": data, "ms": ms, "error": error}
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        set_last_run({"goal": goal, "mode": mode, "error": str(e), "ms": ms})
        return {"ok": False, "data": None, "ms": ms, "error": str(e)}
