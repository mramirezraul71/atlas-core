"""Task planner (LLM-backed). Uses FAST route and 15s timeout by default."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

# Defaults for plan: fast model, small output, low temp, 15s timeout
PLAN_ROUTE_FAST = "FAST"
PLAN_MAX_TOKENS = 256
PLAN_TEMPERATURE = 0.2
PLAN_TIMEOUT_SEC = 15


class TaskPlanner:
    """Plan tasks using LLM. Requires BrainOrchestrator with run_llm."""

    def __init__(self, brain: Any = None) -> None:
        self._brain = brain

    def set_brain(self, brain: Any) -> None:
        self._brain = brain

    def plan(self, goal: str, context: Optional[Dict[str, Any]] = None, fast: bool = True) -> Dict[str, Any]:
        """Return {ok, steps: [...], error}. Uses LLM with FAST route and 15s timeout by default."""
        if not self._brain or not hasattr(self._brain, "run_llm"):
            return {"ok": False, "steps": [], "error": "Brain/LLM not available"}
        route = PLAN_ROUTE_FAST if fast else "REASON"
        prompt = f"Desglosa en pasos concretos y numerados (solo lista, sin explicación): {goal}"
        try:
            r = self._brain.run_llm(
                prompt,
                route=route,
                temperature=PLAN_TEMPERATURE,
                max_tokens=PLAN_MAX_TOKENS,
                timeout_override=PLAN_TIMEOUT_SEC,
            )
        except Exception as e:
            err = str(e).lower()
            if "timeout" in err or "timed out" in err:
                return {"ok": False, "steps": [], "error": "planner_timeout"}
            return {"ok": False, "steps": [], "error": str(e)}
        if not r.get("ok"):
            err = (r.get("error") or "").lower()
            if "timeout" in err or "timed out" in err:
                return {"ok": False, "steps": [], "error": "planner_timeout"}
            return {"ok": False, "steps": [], "error": r.get("error", "LLM failed")}
        # Parse simple numbered lines from output
        output = (r.get("output") or "").strip()
        steps = []
        for line in output.splitlines():
            line = line.strip()
            if line and (line[0].isdigit() or line.startswith("-")):
                steps.append(line)
        return {"ok": True, "steps": steps, "error": None, "raw_output": output}
