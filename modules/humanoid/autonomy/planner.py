"""Task planner (LLM-backed). Uses FAST route and 15s timeout by default."""
from __future__ import annotations

import re
from typing import Any, Dict, List, Optional

PLAN_ROUTE_FAST = "FAST"
PLAN_MAX_TOKENS = 512
PLAN_TEMPERATURE = 0.2
PLAN_TIMEOUT_SEC = 15

_REJECT_PATTERNS = (
    "herramienta de recopilacion", "configurar base de datos para",
    "desarrollar algoritmo", "implementar plataforma", "establecer acuerdos",
    "crear herramienta", "sistema de notificaciones", "informe diario",
    "proceso de seguimiento", "revision periodica", "analisis de patrones",
    "plataforma de gestion",
)


class TaskPlanner:
    """Plan tasks using LLM. Requires BrainOrchestrator with run_llm."""

    def __init__(self, brain: Any = None) -> None:
        self._brain = brain

    def set_brain(self, brain: Any) -> None:
        self._brain = brain

    def plan(self, goal: str, context: Optional[Dict[str, Any]] = None, fast: bool = True) -> Dict[str, Any]:
        if not self._brain or not hasattr(self._brain, "run_llm"):
            return {"ok": False, "steps": [], "error": "Brain/LLM not available"}
        route = PLAN_ROUTE_FAST if fast else "REASON"
        prompt = (
            f"Objetivo: {goal}\n\n"
            f"Dame 3-5 pasos EJECUTABLES para resolver esto.\n"
            f"Cada paso = 1 accion tecnica: comando shell, GET/POST endpoint, editar archivo con ruta.\n"
            f"Endpoints ATLAS: /health, /audit/tail, /api/autonomy/status, /watchdog/status.\n"
            f"PROHIBIDO: 'crear herramienta', 'implementar plataforma', 'establecer proceso', mas de 5 pasos.\n"
            f"Solo lista numerada (1. 2. 3.), sin explicaciones."
        )
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

        output = (r.get("output") or "").strip()
        steps = self._parse_steps(output)

        if self._is_generic(steps):
            steps = steps[:5]
            for i, s in enumerate(steps):
                low = s.lower()
                if any(kw in low for kw in _REJECT_PATTERNS):
                    steps[i] = f"[REVISAR] {s}"

        return {"ok": True, "steps": steps[:5], "error": None, "raw_output": output}

    @staticmethod
    def _parse_steps(output: str) -> List[str]:
        steps = []
        for line in output.splitlines():
            line = line.strip()
            m = re.match(r"^\d+[.)\-]\s*(.+)", line)
            if m:
                steps.append(m.group(1).strip())
            elif line.startswith("- ") and line[2:].strip():
                steps.append(line[2:].strip())
        return steps

    @staticmethod
    def _is_generic(steps: List[str]) -> bool:
        if len(steps) > 7:
            return True
        hits = sum(1 for s in steps if any(kw in s.lower() for kw in _REJECT_PATTERNS))
        return hits >= 2
