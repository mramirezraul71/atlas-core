"""Task planner (LLM-backed)."""
from __future__ import annotations

from typing import Any, Dict, List, Optional


class TaskPlanner:
    """Plan tasks using LLM. Requires BrainModule with run_llm."""

    def __init__(self, brain: Any = None) -> None:
        self._brain = brain

    def set_brain(self, brain: Any) -> None:
        self._brain = brain

    def plan(self, goal: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Return {ok, steps: [...], error}. Uses LLM to break goal into steps."""
        if not self._brain or not hasattr(self._brain, "run_llm"):
            return {"ok": False, "steps": [], "error": "Brain/LLM not available"}
        prompt = f"Desglosa en pasos concretos y numerados (solo lista, sin explicación): {goal}"
        r = self._brain.run_llm(prompt, route="REASON")
        if not r.get("ok"):
            return {"ok": False, "steps": [], "error": r.get("error", "LLM failed")}
        # Parse simple numbered lines from output
        output = (r.get("output") or "").strip()
        steps = []
        for line in output.splitlines():
            line = line.strip()
            if line and (line[0].isdigit() or line.startswith("-")):
                steps.append(line)
        return {"ok": True, "steps": steps, "error": None, "raw_output": output}
