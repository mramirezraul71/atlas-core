from __future__ import annotations

from atlas_code_quant.operations.live_order_routing import resolve_guarded_order_route
from atlas_code_quant.operations.runtime_mode import RuntimeMode


def test_guarded_order_routing_degrades_when_live_disabled() -> None:
    route = resolve_guarded_order_route(
        runtime_mode=RuntimeMode.SUPERVISED_LIVE,
        effective_live_enabled=False,
    )
    assert route.blocked is False
    assert route.route == "paper_shadow_supervised_live"


def test_guarded_order_routing_live_enabled() -> None:
    route = resolve_guarded_order_route(
        runtime_mode=RuntimeMode.GUARDED_LIVE,
        effective_live_enabled=True,
    )
    assert route.route == "live_execution"
