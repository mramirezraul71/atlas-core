"""Fallback to AI_FAST_MODEL if LLM failing."""
from __future__ import annotations

from .base import heal_result


def run(**kwargs) -> dict:
    # Router already uses fallback chain; this is a no-op unless we add env override
    return heal_result(True, "fallback_models", "router handles fallback", {})
