from __future__ import annotations

from atlas_code_quant.operations.runtime_mode import (
    RuntimeMode,
    resolve_runtime_mode,
    validate_runtime_mode_transition,
)


def test_runtime_mode_resolver_applies_symbol_override() -> None:
    res = resolve_runtime_mode(
        requested_mode="paper_aggressive",
        policy_variant="aggressive_v1",
        symbol="SPY",
        account_scope="live",
        deployment_scope="prod",
        symbol_mode_overrides={"SPY": "supervised_live"},
    )
    assert res.effective_mode == RuntimeMode.SUPERVISED_LIVE
    assert "symbol_override_applied" in res.reasons


def test_runtime_mode_transition_validation() -> None:
    assert validate_runtime_mode_transition(RuntimeMode.PAPER_AGGRESSIVE, RuntimeMode.SUPERVISED_LIVE)
    assert not validate_runtime_mode_transition(RuntimeMode.PAPER_BASELINE, RuntimeMode.FULL_LIVE)
