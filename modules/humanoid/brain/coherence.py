"""Coherence validator: output vs schema / context."""
from __future__ import annotations

from typing import Any, Dict, Optional


class CoherenceValidator:
    """Validates output consistency (e.g. LLM response vs expected structure)."""

    def validate(self, output: str, context: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """Return {ok, message, details}. Stub: always ok."""
        return {"ok": True, "message": "consistent", "details": {"length": len(output)}}
