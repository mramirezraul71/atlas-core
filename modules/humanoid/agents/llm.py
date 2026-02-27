"""Shared LLM call for agents. Timeout + FAST fallback. Metrics recorded."""
from __future__ import annotations

from typing import Any, Dict


def run_llm(prompt: str, route: str = "FAST", max_tokens: int = 512, timeout_sec: int = 20) -> Dict[str, Any]:
    """Call brain.run_llm; on failure fallback to FAST. Record latency when ok."""
    try:
        from modules.humanoid import get_humanoid_kernel
        from modules.humanoid.metrics import get_metrics_store
        k = get_humanoid_kernel()
        brain = k.registry.get("brain")
        if brain and hasattr(brain, "run_llm"):
            r = brain.run_llm(prompt, route=route, max_tokens=max_tokens, timeout_override=timeout_sec)
            if r.get("ok"):
                get_metrics_store().record_latency(f"agent_llm_{route}", r.get("ms", 0))
            elif route != "FAST":
                r = brain.run_llm(prompt, route="FAST", max_tokens=max_tokens, timeout_override=timeout_sec)
            return r
    except Exception:
        pass
    return {"ok": False, "output": "", "error": "LLM unavailable"}
