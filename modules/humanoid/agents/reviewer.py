"""Reviewer: validates output quality. Mandatory before any write/execute."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


from .llm import run_llm as _run_llm


class ReviewerAgent(BaseAgent):
    name = "reviewer"
    timeout_sec = 25

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        steps = context.get("steps") or []
        goal = context.get("goal") or ""
        if not steps:
            return {"ok": True, "output": {"approved": True, "issues": [], "reason": "no steps to review"}, "error": None, "ms": 0}
        prompt = f"Objetivo: {goal}\nPasos propuestos:\n" + "\n".join(steps[:15]) + "\n\nEvalúa: ¿hay pasos destructivos (rm -rf, format, drop)? Responde SOLO: APPROVED o REJECTED y una frase con el motivo."
        r = _run_llm(prompt, "FAST", 256, self.timeout_sec)
        if not r.get("ok"):
            return {"ok": False, "output": {"approved": False, "issues": ["reviewer LLM failed"]}, "error": r.get("error"), "ms": r.get("ms", 0)}
        text = (r.get("output") or "").upper()
        approved = "REJECTED" not in text and "DESTRUCTIV" not in text
        return {"ok": True, "output": {"approved": approved, "issues": [] if approved else [text[:200]], "reason": (r.get("output") or "")[:300]}, "error": None, "ms": r.get("ms", 0)}


agent = ReviewerAgent()
