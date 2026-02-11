"""Base agent: timeout, audit, no direct execution. All agents return structured output only.

Safety: No agent executes shell/update/memory write directly; only Executive authorizes execution
via orchestrator. Reviewer is mandatory before write/execute at depth>=4. Optimizer suggests only."""
from __future__ import annotations

import logging
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

DEFAULT_AGENT_TIMEOUT_SEC = 30


def _audit(agent: str, action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("agents", "system", agent, action, ok, ms, error, payload, None)
    except Exception:
        pass


class BaseAgent(ABC):
    """Agent base: process(context) -> result. No direct execution; Executive authorizes execution."""

    name: str = "base"
    timeout_sec: int = DEFAULT_AGENT_TIMEOUT_SEC

    @abstractmethod
    def process(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """Return {ok, output, error, ms}. Output is structured (no side effects)."""
        pass

    def run(self, context: Dict[str, Any]) -> Dict[str, Any]:
        """Run with timeout and audit."""
        t0 = time.perf_counter()
        try:
            out = self.process(context)
            ms = int((time.perf_counter() - t0) * 1000)
            out["ms"] = out.get("ms", ms)
            out_keys = list((out.get("output") or out).keys()) if isinstance(out.get("output"), dict) else []
            _audit(self.name, "process", out.get("ok", False), {"output_keys": out_keys}, out.get("error"), ms)
            return out
        except Exception as e:
            ms = int((time.perf_counter() - t0) * 1000)
            _audit(self.name, "process", False, None, str(e), ms)
            return {"ok": False, "output": None, "error": str(e), "ms": ms}
