"""Researcher: uses Web + Vision to gather information. No execution; returns suggestions."""
from __future__ import annotations

from typing import Any, Dict

from .base import BaseAgent


class ResearcherAgent(BaseAgent):
    name = "researcher"
    timeout_sec = 30

    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        query = (context.get("query") or context.get("goal") or "").strip()
        if not query:
            return {"ok": False, "output": {"sources": [], "summary": ""}, "error": "no query", "ms": 0}
        # Use memory recall as primary (no direct web/vision execution from agent)
        try:
            from modules.humanoid.memory_engine import recall_by_query
            results = recall_by_query(query, limit=10)
            summary = f"Encontrados {len(results)} items en memoria relacionados."
        except Exception:
            results = []
            summary = "Memoria no disponible."
        return {"ok": True, "output": {"sources": [r.get("content", "")[:200] for r in results[:5]], "summary": summary}, "error": None, "ms": 0}


agent = ResearcherAgent()
