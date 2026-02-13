"""Fallback chain: try another local model; if all fail propose paid upgrade (ApprovalItem)."""
from __future__ import annotations

from typing import Any, Callable, List, Optional

from .models import ModelSpec, RouteDecision
from .registry import get_model_specs, resolve_model_for_route

# Optional: create approval item (high, owner session). Caller can inject.
_create_approval_for_paid: Optional[Callable[[str, str, float], Any]] = None


def set_create_approval_for_paid(fn: Callable[[str, str, float], Any]) -> None:
    global _create_approval_for_paid
    _create_approval_for_paid = fn


def fallback_chain(
    route: str,
    ollama_available: bool,
    prefer_free: bool = True,
) -> List[ModelSpec]:
    """Ordered list of model specs to try for this route (free first, then paid if allowed)."""
    specs = [s for s in get_model_specs(ollama_available) if s.route == route]
    free = [s for s in specs if s.is_free]
    paid = [s for s in specs if not s.is_free]
    if prefer_free:
        return free + paid
    return paid + free


def propose_paid_upgrade(reason: str, estimated_cost_usd: float, route: str) -> Any:
    """Create ApprovalItem high + owner session required; return id or None."""
    if _create_approval_for_paid:
        return _create_approval_for_paid(reason, route, estimated_cost_usd)
    return None
