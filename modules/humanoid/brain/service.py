"""Brain: LLM integration + validators."""
from __future__ import annotations

from typing import Any, Dict

from modules.humanoid.base import BaseModule, HealthCheckMixin
from modules.humanoid.brain.validators import ConsistencyValidator, LogicalValidator


class BrainModule(BaseModule, HealthCheckMixin):
    """Wraps LLMService and validators."""

    name = "brain"

    def __init__(self) -> None:
        self._llm = None
        self._LLMRequest = None
        self.consistency = ConsistencyValidator()
        self.logical = LogicalValidator()

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
        """Run LLM request. Returns dict with ok, output, route, etc. If LLM not available, returns error."""
        if self._llm is None or self._LLMRequest is None:
            return {"ok": False, "output": "", "error": "LLMService not available", "route": route}
        req = self._LLMRequest(prompt=prompt, route=route, **kwargs)
        resp = self._llm.run(req)
        return resp.model_dump()

    def health_check(self) -> Dict[str, Any]:
        if self._llm is None:
            return {"ok": False, "message": "LLM not loaded", "details": {}}
        return {"ok": True, "message": "ok", "details": {"validators": ["consistency", "logical"]}}

    def info(self) -> Dict[str, Any]:
        return {"module": self.name, "llm_ready": self._llm is not None}
