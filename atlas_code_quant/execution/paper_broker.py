"""Paper Broker — F6/F9.

Modelo simple usado por la FSM en estado ``PAPER_EXECUTING``.
- Rellena 100% al precio indicado o al ``mark`` simulado pasado en submit().
- Mantiene un libro de posiciones y de órdenes en memoria.
- Métodos ``cancel`` y ``flatten`` simétricos al adapter Tradier.
- F9: ``open_position(plan, entry_price)`` y ``close_position(id, exit_price,
  reason)`` con cálculo de ``realized_pnl`` para una posición de spread.
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from atlas_code_quant.execution.tradier_adapter import OrderRequest, OrderTicket
from atlas_code_quant.strategies.contracts import StrategyPlan


@dataclass(slots=True)
class PaperPosition:
    symbol: str
    quantity: int = 0
    avg_price: float = 0.0


@dataclass(slots=True)
class PaperOpenPosition:
    """Posición F9 ligada a un ``StrategyPlan``.

    A diferencia de :class:`PaperPosition`, este modelo conserva la
    referencia al plan que la abrió y precios de TP/SL/time-stop, lo que
    permite al :class:`PositionMonitor` decidir cierre automático.
    """

    position_id: str
    symbol: str
    strategy: str
    qty: int
    entry_price: float
    take_profit_price: float
    stop_loss_price: float
    time_stop_at: float
    opened_at: float
    trace_id: str = ""
    direction: str = "neutral"
    plan_max_loss_usd: float = 0.0
    plan_notional_usd: float = 0.0
    status: str = "open"  # open | closed
    exit_price: float | None = None
    exit_reason: str | None = None
    realized_pnl_usd: float = 0.0
    closed_at: float | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "position_id": self.position_id,
            "symbol": self.symbol,
            "strategy": self.strategy,
            "qty": self.qty,
            "entry_price": self.entry_price,
            "take_profit_price": self.take_profit_price,
            "stop_loss_price": self.stop_loss_price,
            "time_stop_at": self.time_stop_at,
            "opened_at": self.opened_at,
            "trace_id": self.trace_id,
            "direction": self.direction,
            "plan_max_loss_usd": self.plan_max_loss_usd,
            "plan_notional_usd": self.plan_notional_usd,
            "status": self.status,
            "exit_price": self.exit_price,
            "exit_reason": self.exit_reason,
            "realized_pnl_usd": self.realized_pnl_usd,
            "closed_at": self.closed_at,
        }


@dataclass(slots=True)
class PaperBroker:
    cash_usd: float = 100_000.0
    _positions: dict[str, PaperPosition] = field(default_factory=dict)
    _orders: list[OrderTicket] = field(default_factory=list)
    # F9: posiciones abiertas por plan
    _open_positions: dict[str, PaperOpenPosition] = field(default_factory=dict)
    _closed_positions: list[PaperOpenPosition] = field(default_factory=list)

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

    # ── F9: open/close de posiciones ligadas a un StrategyPlan ─────────────

    def open_position(
        self,
        plan: StrategyPlan,
        *,
        entry_price: float,
        take_profit_pct: float = 0.50,
        stop_loss_pct: float = 1.00,
        time_stop_seconds: int | None = None,
        trace_id: str = "",
    ) -> PaperOpenPosition:
        """Abre una posición paper a partir de un ``StrategyPlan``.

        Idempotencia: si ya existe una posición con el mismo ``trace_id``
        en estado ``open``, devuelve la misma posición.
        """
        if not plan.is_actionable():
            raise ValueError(
                f"plan_not_actionable status={plan.status} legs={len(plan.legs)}"
            )
        tid = trace_id or plan.trace_id
        if tid:
            for pos in self._open_positions.values():
                if pos.trace_id == tid and pos.status == "open":
                    return pos  # idempotencia por trace_id

        pos_id = f"POS-{uuid.uuid4().hex[:10]}"
        # En spreads de débito, take_profit_pct es % sobre prima pagada.
        tp = round(entry_price * (1.0 + take_profit_pct), 4)
        sl = round(entry_price * max(0.0, 1.0 - stop_loss_pct), 4)
        ts_seconds = int(time_stop_seconds if time_stop_seconds is not None
                         else max(60, plan.horizon_min * 60))
        now = time.time()
        pos = PaperOpenPosition(
            position_id=pos_id,
            symbol=plan.symbol,
            strategy=plan.strategy,
            qty=max(1, sum(l.qty for l in plan.legs)),
            entry_price=entry_price,
            take_profit_price=tp,
            stop_loss_price=sl,
            time_stop_at=now + ts_seconds,
            opened_at=now,
            trace_id=tid,
            direction=plan.direction,
            plan_max_loss_usd=plan.max_loss_estimate_usd,
            plan_notional_usd=plan.notional_estimate_usd,
        )
        self._open_positions[pos_id] = pos
        return pos

    def close_position(
        self,
        position_id: str,
        *,
        exit_price: float,
        reason: str,
    ) -> PaperOpenPosition:
        """Cierra una posición paper y calcula realized PnL.

        PnL aproximado para spread de débito: ``(exit - entry) * qty * 100``.
        Para spread de crédito (entry_price < 0) la fórmula es simétrica.
        """
        pos = self._open_positions.get(position_id)
        if pos is None:
            # ¿está ya cerrada? → idempotente, devolvemos la cerrada existente
            for closed in self._closed_positions:
                if closed.position_id == position_id:
                    return closed
            raise KeyError(f"unknown_position id={position_id}")
        if pos.status != "open":
            return pos  # idempotente
        pnl = round((exit_price - pos.entry_price) * pos.qty * 100.0, 2)
        pos.exit_price = exit_price
        pos.exit_reason = reason
        pos.realized_pnl_usd = pnl
        pos.closed_at = time.time()
        pos.status = "closed"
        # liberar margen aproximado (cash neutro, ya no tracking spread real)
        self.cash_usd += pnl
        # mover a cerradas
        del self._open_positions[position_id]
        self._closed_positions.append(pos)
        return pos

    def get_open_position(self, position_id: str) -> PaperOpenPosition | None:
        return self._open_positions.get(position_id)

    def open_positions(self) -> list[PaperOpenPosition]:
        return list(self._open_positions.values())

    def closed_positions(self) -> list[PaperOpenPosition]:
        return list(self._closed_positions)
