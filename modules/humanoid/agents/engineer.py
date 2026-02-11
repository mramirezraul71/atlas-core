"""Engineer: produces concrete steps/code from architecture. No execution."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


from .llm import run_llm as _run_llm


class EngineerAgent(BaseAgent):
    name = "engineer"
    timeout_sec = 30

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        goal = context.get("goal") or ""
        components = context.get("components") or []
        prompt = f"Objetivo: {goal}\nComponentes: {components[:10]}\n\nLista pasos de implementación concretos y ejecutables (comandos o tareas). Solo lista numerada."
        r = _run_llm(prompt, "FAST", 600, self.timeout_sec)
        if not r.get("ok"):
            return {"ok": False, "output": {"steps": []}, "error": r.get("error"), "ms": r.get("ms", 0)}
        text = (r.get("output") or "").strip()
        steps = []
        for line in text.splitlines():
            line = line.strip()
            if line and (line[0].isdigit() or line.startswith("-")):
                steps.append(line)
        return {"ok": True, "output": {"steps": steps[:20]}, "error": None, "ms": r.get("ms", 0)}


agent = EngineerAgent()
