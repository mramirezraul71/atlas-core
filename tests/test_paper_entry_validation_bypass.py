from __future__ import annotations


def test_paper_bypass_moves_entry_validation_reasons_to_warning() -> None:
    from atlas_code_quant.operations.operation_center import OperationCenter

    config = {"paper_bypass_entry_validation_guard": True}
    entry_validation = {
        "blocked": True,
        "status": "blocked",
        "reasons": [
            "Entry validation blocked submit because spread is 19.30% (> 0.50%).",
            "Entry validation blocked submit because adverse drift is 1.20% (> 0.50%).",
        ],
    }
    reasons = list(entry_validation["reasons"])
    warnings: list[str] = []

    OperationCenter._apply_paper_entry_validation_bypass(
        scope="paper",
        action="submit",
        config=config,
        entry_validation=entry_validation,
        reasons=reasons,
        warnings=warnings,
    )

    assert reasons == []
    assert any("paper bypass" in w for w in warnings)
    assert entry_validation["blocked"] is False
    assert entry_validation["status"] == "warning"

