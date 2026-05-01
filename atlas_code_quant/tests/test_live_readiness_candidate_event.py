from __future__ import annotations

from atlas_code_quant.learning.learning_orchestrator import build_live_readiness_events
from atlas_code_quant.operations.live_switch import resolve_live_switch_state
from atlas_code_quant.operations.runtime_mode import RuntimeMode


def test_ready_candidate_event_emitted_but_live_stays_locked_without_human_unlock() -> None:
    promotion = {
        "current_stage": "paper_aggressive",
        "target_stage": "supervised_live_candidate",
        "recommendation": "supervised_live_candidate",
        "ready_for_supervised_live": True,
        "ready_for_guarded_live": False,
        "ready_for_full_live": False,
        "scorecard": {"readiness_score": 0.9},
        "blocking_reasons": [],
        "warnings": [],
    }
    events = build_live_readiness_events(
        promotion=promotion,
        readiness_insights={"runtime_stability_score": 0.9},
        guardrails={"pause_recommended": False},
    )
    topics = [ev["topic"] for ev in events]
    assert "live.readiness.ready_candidate" in topics

    switch = resolve_live_switch_state(
        runtime_mode=RuntimeMode.SUPERVISED_LIVE,
        live_capable=True,
        live_ready=True,
        live_unlock_requested=False,
        live_unlock_approved=False,
        guardrails_healthy=True,
        kill_switch_active=False,
        require_human_unlock=True,
    )
    assert switch.effective_live_enabled is False
