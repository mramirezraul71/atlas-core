"""Tests F8 — FSM 14 estados + transiciones permitidas."""
from __future__ import annotations

import pytest

from atlas_code_quant.autonomy.orchestrator import (
    QuantAutonomyOrchestrator,
)
from atlas_code_quant.autonomy.states import (
    ALLOWED_TRANSITIONS,
    QuantAutonomyState,
    is_allowed,
)


def test_states_count_is_14() -> None:
    # 14 estados FSM
    assert len(list(QuantAutonomyState)) == 14


def test_required_states_present() -> None:
    expected = {
        "BOOTING", "DEGRADED", "SCANNING", "OPPORTUNITY_DETECTED",
        "STRATEGY_BUILDING", "BACKTESTING", "PAPER_READY", "PAPER_EXECUTING",
        "LIVE_ARMED", "LIVE_EXECUTING", "MONITORING", "EXITING",
        "KILL_SWITCH", "ERROR_RECOVERY",
    }
    actual = {s.value for s in QuantAutonomyState}
    assert expected == actual


def test_transition_table_covers_all_states() -> None:
    for s in QuantAutonomyState:
        assert s in ALLOWED_TRANSITIONS


def test_kill_switch_is_terminal_except_boot_reset() -> None:
    # Solo se permite KILL_SWITCH -> BOOTING
    allowed = ALLOWED_TRANSITIONS[QuantAutonomyState.KILL_SWITCH]
    assert allowed == {QuantAutonomyState.BOOTING}


def test_orchestrator_default_state_is_booting() -> None:
    o = QuantAutonomyOrchestrator()
    assert o.state == QuantAutonomyState.BOOTING


def test_valid_transition_logged() -> None:
    o = QuantAutonomyOrchestrator()
    evt = o.transition(QuantAutonomyState.SCANNING, reason="boot_complete")
    assert o.state == QuantAutonomyState.SCANNING
    assert evt.src == QuantAutonomyState.BOOTING
    assert evt.dst == QuantAutonomyState.SCANNING
    assert len(o.history()) == 1


def test_invalid_transition_rejected() -> None:
    o = QuantAutonomyOrchestrator()
    with pytest.raises(ValueError, match="transition_not_allowed"):
        # BOOTING -> LIVE_EXECUTING no permitido
        o.transition(QuantAutonomyState.LIVE_EXECUTING, reason="bad")


def test_kill_switch_trip_works_from_any_state() -> None:
    o = QuantAutonomyOrchestrator()
    o.transition(QuantAutonomyState.SCANNING, "boot")
    o.transition(QuantAutonomyState.OPPORTUNITY_DETECTED, "found")
    evt = o.trip_kill_switch(reason="emergency")
    assert o.state == QuantAutonomyState.KILL_SWITCH
    assert evt.dst == QuantAutonomyState.KILL_SWITCH


def test_kill_switch_reset_requires_explicit_flag() -> None:
    o = QuantAutonomyOrchestrator()
    o.trip_kill_switch("emergency")
    with pytest.raises(ValueError):
        o.transition(QuantAutonomyState.BOOTING, "reset")
    o.allow_kill_switch_reset = True
    evt = o.transition(QuantAutonomyState.BOOTING, "reset_authorized")
    assert o.state == QuantAutonomyState.BOOTING
    assert evt.reason == "reset_authorized"


def test_typical_paper_flow() -> None:
    o = QuantAutonomyOrchestrator()
    sequence = [
        QuantAutonomyState.SCANNING,
        QuantAutonomyState.OPPORTUNITY_DETECTED,
        QuantAutonomyState.STRATEGY_BUILDING,
        QuantAutonomyState.BACKTESTING,
        QuantAutonomyState.PAPER_READY,
        QuantAutonomyState.PAPER_EXECUTING,
        QuantAutonomyState.MONITORING,
        QuantAutonomyState.EXITING,
        QuantAutonomyState.SCANNING,
    ]
    for dst in sequence:
        o.transition(dst, "flow")
    assert o.state == QuantAutonomyState.SCANNING
    assert len(o.history()) == len(sequence)


def test_is_allowed_helper_consistent() -> None:
    assert is_allowed(QuantAutonomyState.BOOTING, QuantAutonomyState.SCANNING) is True
    assert is_allowed(QuantAutonomyState.SCANNING, QuantAutonomyState.LIVE_EXECUTING) is False
