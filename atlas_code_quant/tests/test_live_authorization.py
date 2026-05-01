from __future__ import annotations

from atlas_code_quant.operations.live_authorization import (
    approve_live_unlock,
    build_live_unlock_request,
)


def test_live_authorization_request_and_approval() -> None:
    req = build_live_unlock_request(
        requested_by="operator_a",
        scope_type="symbol",
        scope_value="SPY",
        reason="pilot rollout",
    )
    assert req["status"] == "requested"
    approved = approve_live_unlock(req, approved_by="operator_b", approval_reason="all checks green")
    assert approved["status"] == "approved"
    assert approved["approved_by"] == "operator_b"
