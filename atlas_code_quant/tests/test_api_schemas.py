from __future__ import annotations

from atlas_code_quant.api.schemas import OperationCycleRequest


def test_operation_cycle_request_accepts_order_seed_alias() -> None:
    payload = OperationCycleRequest.model_validate(
        {
            "order_seed": {
                "symbol": "AAOI",
                "side": "buy",
                "size": 7,
                "account_scope": "paper",
                "strategy_type": "equity_long",
                "asset_class": "equity",
                "position_effect": "open",
                "order_type": "market",
                "duration": "day",
            },
            "action": "preview",
            "capture_context": False,
        }
    )

    assert payload.order.symbol == "AAOI"
    assert payload.order.strategy_type == "equity_long"
    assert payload.action == "preview"
    assert payload.capture_context is False
