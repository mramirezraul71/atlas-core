"""Base heal."""
from __future__ import annotations

from typing import Any, Dict


def heal_result(ok: bool, heal_id: str, message: str, details: Dict[str, Any] = None, error: str = None) -> Dict[str, Any]:
    return {"ok": ok, "heal_id": heal_id, "message": message, "details": details or {}, "error": error}
