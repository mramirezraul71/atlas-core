"""Guarded routing for paper/simulated-live/live pipelines."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from atlas_code_quant.operations.runtime_mode import RuntimeMode


Route = Literal[
    "paper_execution",
    "paper_shadow_supervised_live",
    "paper_shadow_guarded_live",
    "simulated_full_live",
    "live_execution",
    "blocked",
]


@dataclass(frozen=True)
class GuardedOrderRoute:
    route: Route
    blocked: bool
    reason: str


def resolve_guarded_order_route(
    *,
    runtime_mode: RuntimeMode,
    effective_live_enabled: bool,
) -> GuardedOrderRoute:
    if runtime_mode == RuntimeMode.PAPER_BASELINE:
        return GuardedOrderRoute(route="paper_execution", blocked=False, reason="paper_baseline_route")
    if runtime_mode == RuntimeMode.PAPER_AGGRESSIVE:
        return GuardedOrderRoute(route="paper_execution", blocked=False, reason="paper_aggressive_route")
    if runtime_mode == RuntimeMode.SUPERVISED_LIVE:
        return GuardedOrderRoute(
            route="live_execution" if effective_live_enabled else "paper_shadow_supervised_live",
            blocked=False,
            reason="supervised_live_guarded_by_switch",
        )
    if runtime_mode == RuntimeMode.GUARDED_LIVE:
        return GuardedOrderRoute(
            route="live_execution" if effective_live_enabled else "paper_shadow_guarded_live",
            blocked=False,
            reason="guarded_live_guarded_by_switch",
        )
    if runtime_mode == RuntimeMode.FULL_LIVE:
        return GuardedOrderRoute(
            route="live_execution" if effective_live_enabled else "simulated_full_live",
            blocked=False,
            reason="full_live_guarded_by_switch",
        )
    return GuardedOrderRoute(route="blocked", blocked=True, reason="unknown_runtime_mode")


def guarded_route_to_payload(route: GuardedOrderRoute) -> dict[str, Any]:
    return {
        "route": route.route,
        "blocked": route.blocked,
        "reason": route.reason,
    }
