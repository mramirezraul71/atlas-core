from __future__ import annotations


def test_paper_bypass_moves_exit_now_reason_to_warning() -> None:
    from atlas_code_quant.operations.operation_center import OperationCenter

    config = {"paper_bypass_exit_now_guard": True}
    portfolio_risk_guard = {
        "blocked": True,
        "status": "blocked",
        "reasons": [
            "Portfolio risk guard blocked submit: hay 1 posiciones con criterio de exit_now antes de sumar exposicion."
        ],
        "warnings": [],
    }
    reasons = list(portfolio_risk_guard["reasons"])
    warnings: list[str] = []

    OperationCenter._apply_paper_risk_guard_bypass(
        scope="paper",
        action="submit",
        config=config,
        portfolio_risk_guard=portfolio_risk_guard,
        reasons=reasons,
        warnings=warnings,
    )

    assert reasons == []
    assert any("paper bypass" in w for w in warnings)
    assert portfolio_risk_guard["blocked"] is False
    assert portfolio_risk_guard["status"] == "warning"

