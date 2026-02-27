"""Plugin registry: checks + heals."""
from __future__ import annotations

from typing import Any, Callable, Dict, List

_checks: Dict[str, Callable[[], Any]] = {}
_heals: Dict[str, Callable[..., Any]] = {}


def register_check(check_id: str, fn: Callable[[], Any]) -> None:
    _checks[check_id] = fn


def register_heal(heal_id: str, fn: Callable[..., Any]) -> None:
    _heals[heal_id] = fn


def get_checks() -> Dict[str, Callable[[], Any]]:
    return dict(_checks)


def get_heals() -> Dict[str, Callable[..., Any]]:
    return dict(_heals)


def run_check(check_id: str) -> Any:
    fn = _checks.get(check_id)
    if not fn:
        return {"ok": False, "check_id": check_id, "message": "unknown check", "details": {}}
    try:
        return fn()
    except Exception as e:
        return {"ok": False, "check_id": check_id, "message": str(e), "details": {"error": str(e)}}


def run_heal(heal_id: str, **params: Any) -> Any:
    fn = _heals.get(heal_id)
    if not fn:
        return {"ok": False, "heal_id": heal_id, "message": "unknown heal", "error": "unknown heal"}
    try:
        return fn(**params)
    except Exception as e:
        return {"ok": False, "heal_id": heal_id, "message": str(e), "error": str(e)}
