"""Tests F6 — PaperBroker (in-memory)."""
from __future__ import annotations

from atlas_code_quant.execution.paper_broker import PaperBroker
from atlas_code_quant.execution.tradier_adapter import OrderRequest


def test_submit_buy_creates_long_position() -> None:
    pb = PaperBroker(cash_usd=10_000.0)
    ticket = pb.submit(
        OrderRequest(symbol="AAPL", side="buy_to_open", quantity=1, order_type="limit", limit_price=2.50)
    )
    assert ticket.status == "submitted"
    assert ticket.broker_order_id.startswith("PB-")
    pos = pb.positions()
    assert len(pos) == 1
    assert pos[0].symbol == "AAPL"
    assert pos[0].quantity == 1
    assert pos[0].avg_price == 2.50


def test_flatten_clears_positions() -> None:
    pb = PaperBroker()
    pb.submit(OrderRequest(symbol="MSFT", side="buy_to_open", quantity=2, order_type="limit", limit_price=1.0))
    res = pb.flatten()
    assert res == {"ok": True, "flattened": 1}
    assert pb.positions() == []


def test_orders_log_grows() -> None:
    pb = PaperBroker()
    pb.submit(OrderRequest(symbol="X", side="buy_to_open", quantity=1, limit_price=1.0))
    pb.submit(OrderRequest(symbol="Y", side="buy_to_open", quantity=1, limit_price=1.0))
    assert len(pb.orders()) == 2
