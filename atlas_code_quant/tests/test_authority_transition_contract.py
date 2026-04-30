from __future__ import annotations

from atlas_code_quant.operations.authority_transition_contract import (
    build_authority_transition_assessment,
)


def test_authority_transition_contract_shape() -> None:
    out = build_authority_transition_assessment(
        initiator="system",
        authority_level="paper_aggressive",
        recommendation="supervised_live_candidate",
        transition_reason="daily_promotion_assessment",
        rollback_ready=True,
        emergency_stop_ready=True,
        required_human_ack=True,
        additional_checks=["operator_ack_required"],
    )
    assert out["initiator"] == "system"
    assert out["authority_level"] == "paper_aggressive"
    assert out["recommended_transition"] == "supervised_live_candidate"
    assert out["required_human_ack"] is True
    assert out["rollback_ready"] is True
    assert out["emergency_stop_ready"] is True
    assert isinstance(out["additional_checks"], list)
