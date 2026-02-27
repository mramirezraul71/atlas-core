"""Brain orchestrator: LLM + validators."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.kernel import BaseModule, HealthCheckMixin
from modules.humanoid.brain.coherence import CoherenceValidator
from modules.humanoid.brain.logic import LogicValidator


class BrainOrchestrator(BaseModule, HealthCheckMixin):
    """Wraps LLMService and coherence/logic validators."""

    name = "brain"

    def __init__(self) -> None:
        self._llm = None
        self._LLMRequest = None
        self.coherence = CoherenceValidator()
        self.logic = LogicValidator()

    def init(self) -> None:
        try:
            from modules.llm.schemas import LLMRequest
            from modules.llm.service import LLMService
            self._LLMRequest = LLMRequest
            self._llm = LLMService()
        except Exception:
            self._llm = None
            self._LLMRequest = None

    def run_llm(self, prompt: str, route: str | None = None, **kwargs: Any) -> Dict[str, Any]:
        if self._llm is None or self._LLMRequest is None:
            return {"ok": False, "output": "", "error": "LLMService not available", "route": route}
        req = self._LLMRequest(prompt=prompt, route=route, **kwargs)
        resp = self._llm.run(req)
        return resp.model_dump()

    def health_check(self) -> Dict[str, Any]:
        if self._llm is None:
            return {"ok": False, "message": "LLM not loaded", "details": {}}
        return {"ok": True, "message": "ok", "details": {"validators": ["coherence", "logic"]}}

    def info(self) -> Dict[str, Any]:
        return {"module": self.name, "llm_ready": self._llm is not None}
