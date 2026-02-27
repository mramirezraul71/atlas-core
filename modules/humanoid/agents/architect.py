"""Architect: designs technical structure from strategy."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


from .llm import run_llm as _run_llm


class ArchitectAgent(BaseAgent):
    name = "architect"
    timeout_sec = 25

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        goal = context.get("goal") or ""
        strategy_steps = context.get("strategy_steps") or []
        if not goal and not strategy_steps:
            return {"ok": False, "output": {}, "error": "no goal or strategy", "ms": 0}
        prompt = f"Objetivo: {goal}\nEstrategia: {strategy_steps[:5]}\n\nDiseña la estructura técnica (módulos/componentes) en lista breve. Solo lista."
        r = _run_llm(prompt, "FAST", 512, self.timeout_sec)
        if not r.get("ok"):
            return {"ok": False, "output": {"components": []}, "error": r.get("error"), "ms": r.get("ms", 0)}
        text = (r.get("output") or "").strip()
        components = [s.strip() for s in text.splitlines() if s.strip()][:15]
        return {"ok": True, "output": {"components": components, "raw": text[:500]}, "error": None, "ms": r.get("ms", 0)}


agent = ArchitectAgent()
