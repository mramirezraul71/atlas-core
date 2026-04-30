"""Live switch state resolver with hard lock until readiness + human unlock."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from atlas_code_quant.operations.runtime_mode import RuntimeMode


@dataclass(frozen=True)
class LiveSwitchState:
    live_capable: bool
    live_ready: bool
    live_unlock_requested: bool
    live_unlock_approved: bool
    effective_live_enabled: bool
    blocking_reasons: list[str]


def resolve_live_switch_state(
    *,
    runtime_mode: RuntimeMode,
    live_capable: bool,
    live_ready: bool,
    live_unlock_requested: bool,
    live_unlock_approved: bool,
    guardrails_healthy: bool,
    kill_switch_active: bool,
    require_human_unlock: bool = True,
    full_live_globally_locked: bool = True,
) -> LiveSwitchState:
    blocking: list[str] = []
    requested_live_mode = runtime_mode in {
        RuntimeMode.SUPERVISED_LIVE,
        RuntimeMode.GUARDED_LIVE,
        RuntimeMode.FULL_LIVE,
    }
    if not requested_live_mode:
        blocking.append("runtime_mode_not_live")
    if not live_capable:
        blocking.append("live_capability_disabled")
    if not live_ready:
        blocking.append("readiness_not_met")
    if not guardrails_healthy:
        blocking.append("guardrails_unhealthy")
    if kill_switch_active:
        blocking.append("kill_switch_active")
    if require_human_unlock and not live_unlock_requested:
        blocking.append("human_unlock_not_requested")
    if require_human_unlock and not live_unlock_approved:
        blocking.append("human_unlock_not_approved")
    if runtime_mode == RuntimeMode.FULL_LIVE and full_live_globally_locked:
        blocking.append("full_live_globally_locked")

    enabled = not blocking
    return LiveSwitchState(
        live_capable=bool(live_capable),
        live_ready=bool(live_ready),
        live_unlock_requested=bool(live_unlock_requested),
        live_unlock_approved=bool(live_unlock_approved),
        effective_live_enabled=enabled,
        blocking_reasons=blocking,
    )


def live_switch_state_to_payload(state: LiveSwitchState) -> dict[str, Any]:
    return {
        "live_capable": state.live_capable,
        "live_ready": state.live_ready,
        "live_unlock_requested": state.live_unlock_requested,
        "live_unlock_approved": state.live_unlock_approved,
        "effective_live_enabled": state.effective_live_enabled,
        "blocking_reasons": list(state.blocking_reasons),
    }
