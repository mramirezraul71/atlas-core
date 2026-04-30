from __future__ import annotations

from atlas_code_quant.operations.live_switch import resolve_live_switch_state
from atlas_code_quant.operations.runtime_mode import RuntimeMode


def test_full_live_locked_without_unlock() -> None:
    state = resolve_live_switch_state(
        runtime_mode=RuntimeMode.FULL_LIVE,
        live_capable=True,
        live_ready=True,
        live_unlock_requested=False,
        live_unlock_approved=False,
        guardrails_healthy=True,
        kill_switch_active=False,
        require_human_unlock=True,
        full_live_globally_locked=True,
    )
    assert state.effective_live_enabled is False
    assert "human_unlock_not_requested" in state.blocking_reasons
    assert "full_live_globally_locked" in state.blocking_reasons


def test_live_switch_blocks_with_kill_switch() -> None:
    state = resolve_live_switch_state(
        runtime_mode=RuntimeMode.GUARDED_LIVE,
        live_capable=True,
        live_ready=True,
        live_unlock_requested=True,
        live_unlock_approved=True,
        guardrails_healthy=True,
        kill_switch_active=True,
    )
    assert state.effective_live_enabled is False
    assert "kill_switch_active" in state.blocking_reasons
