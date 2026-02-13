"""Base check: timeout + standardized result."""
from __future__ import annotations

import time
from typing import Any, Dict


def check_result(ok: bool, check_id: str, message: str, details: Dict[str, Any] = None, severity: str = "low") -> Dict[str, Any]:
    return {
        "ok": ok,
        "check_id": check_id,
        "message": message,
        "details": details or {},
        "severity": severity,
    }


def fingerprint(check_id: str, key: str, value: Any) -> str:
    return f"{check_id}:{key}:{value}"
