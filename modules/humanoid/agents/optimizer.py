"""Optimizer: analyzes and suggests improvements. Does not apply; requires approval."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


from .llm import run_llm as _run_llm


class OptimizerAgent(BaseAgent):
    name = "optimizer"
    timeout_sec = 25

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        steps = context.get("steps") or []
        goal = context.get("goal") or ""
        if not steps and not goal:
            return {"ok": True, "output": {"improvements": []}, "error": None, "ms": 0}
        prompt = f"Objetivo: {goal}\nPasos: {str(steps[:10])}\n\nSugiere 1-3 mejoras breves (mantenibilidad, claridad). Solo lista corta."
        r = _run_llm(prompt, "FAST", 300, self.timeout_sec)
        if not r.get("ok"):
            return {"ok": True, "output": {"improvements": []}, "error": None, "ms": r.get("ms", 0)}
        text = (r.get("output") or "").strip()
        improvements = [s.strip() for s in text.splitlines() if s.strip()][:5]
        return {"ok": True, "output": {"improvements": improvements}, "error": None, "ms": r.get("ms", 0)}


agent = OptimizerAgent()
