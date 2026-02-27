"""Strategist: converts goal into high-level strategy."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


from .llm import run_llm as _run_llm


class StrategistAgent(BaseAgent):
    name = "strategist"
    timeout_sec = 25

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        goal = (context.get("goal") or "").strip()
        if not goal:
            return {"ok": False, "output": {}, "error": "empty goal", "ms": 0}
        prompt = f"Objetivo: {goal}\n\nDefine una estrategia general en 3-5 pasos de alto nivel. Responde solo con la lista numerada, sin explicación extra."
        r = _run_llm(prompt, "FAST", 512, self.timeout_sec)
        if not r.get("ok"):
            return {"ok": False, "output": {"strategy_steps": []}, "error": r.get("error", "LLM failed"), "ms": r.get("ms", 0)}
        text = (r.get("output") or "").strip()
        steps = [s.strip() for s in text.splitlines() if s.strip() and (s.strip()[0].isdigit() or s.startswith("-"))]
        return {"ok": True, "output": {"strategy_steps": steps[:10], "raw": text[:500]}, "error": None, "ms": r.get("ms", 0)}


agent = StrategistAgent()
