"""Paper Broker — F6 (esqueleto en memoria).

Modelo simple usado por la FSM en estado ``PAPER_EXECUTING``.
- Rellena 100% al precio indicado o al ``mark`` simulado pasado en submit().
- Mantiene un libro de posiciones y de órdenes en memoria.
- Métodos ``cancel`` y ``flatten`` simétricos al adapter Tradier.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from atlas_code_quant.execution.tradier_adapter import OrderRequest, OrderTicket


@dataclass(slots=True)
class PaperPosition:
    symbol: str
    quantity: int = 0
    avg_price: float = 0.0


@dataclass(slots=True)
class PaperBroker:
    cash_usd: float = 100_000.0
    _positions: dict[str, PaperPosition] = field(default_factory=dict)
    _orders: list[OrderTicket] = field(default_factory=list)

    def submit(self, order: OrderRequest, mark_price: float | None = None) -> OrderTicket:
        price = mark_price if mark_price is not None else (order.limit_price or 0.0)
        ticket = OrderTicket(
            request=order,
            status="submitted",
            broker_order_id=f"PB-{uuid.uuid4().hex[:8]}",
            rationale=f"paper_fill@{price:.2f}",
            submitted_at=time.time(),
        )
        self._apply_fill(order, price)
        self._orders.append(ticket)
        return ticket

    def _apply_fill(self, order: OrderRequest, price: float) -> None:
        pos = self._positions.setdefault(order.symbol, PaperPosition(symbol=order.symbol))
        delta = order.quantity if order.side.startswith("buy") else -order.quantity
        new_qty = pos.quantity + delta
        # actualización avg_price (solo aperturas largas como aproximación)
        if delta > 0 and pos.quantity >= 0:
            total_cost = pos.avg_price * pos.quantity + price * delta
            pos.avg_price = total_cost / new_qty if new_qty else 0.0
        pos.quantity = new_qty
        self.cash_usd -= delta * price * 100  # asume contratos opciones (multiplicador 100)

    def positions(self) -> list[PaperPosition]:
        return [p for p in self._positions.values() if p.quantity != 0]

    def orders(self) -> list[OrderTicket]:
        return list(self._orders)

    def flatten(self) -> dict[str, Any]:
        n = sum(1 for p in self._positions.values() if p.quantity != 0)
        for p in self._positions.values():
            p.quantity = 0
        return {"ok": True, "flattened": n}
