"""
Esqueleto de órdenes live estilo Tradier (tipos, builder y sink sin red).

Este archivo **no** abre sockets; las llamadas HTTP al sandbox están en
``tradier_executor.TradierOrderExecutor``. Flujo:

1. ``TradierOrderBuilder`` convierte una ``Position`` del simulador en ``LiveOrder`` + ``LiveOrderLeg``.
2. ``TradierOrderExecutor`` (``broker/tradier_executor.py``) traduce a formulario Tradier y
   puede llamar al **sandbox** (``dry_run=False``, ``preview=True`` recomendado al principio).
3. ``TradierLiveExecutionSink`` sigue disponible para **cero red** (solo registra).

Convención broker (apertura multi-leg):

- Pata **LONG** en la posición → **compra** para abrir, **venta** para cerrar.
- Pata **SHORT** en la posición → **venta** para abrir, **compra** para cerrar.

Precio límite por pata (solo informativo / dry-run):

- ``price_mode='mid'``: ``contract.mid``.
- ``price_mode='bid_ask'``: compra cotiza **ask**, venta cotiza **bid``.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from typing import Literal

from atlas_options_brain.models.option_contract import OptionContract, OptionRight, OptionType
from atlas_options_brain.models.leg import Leg
from atlas_options_brain.simulator.paper import Position

OrderSide = Literal["buy", "sell"]
OrderType = Literal["market", "limit"]
Tif = Literal["day", "gtc"]
LiveOrderStatus = Literal["pending", "simulated", "sent", "rejected", "cancelled"]


@dataclass(frozen=True)
class LiveOrderLeg:
    position_id: str
    leg_index: int
    symbol: str
    contract_symbol: str
    side: OrderSide
    quantity: int
    order_type: OrderType
    limit_price: float | None
    time_in_force: Tif
    strategy_type: str
    tag: str | None = None


@dataclass
class LiveOrder:
    order_id: str
    position_id: str
    symbol: str
    legs: list[LiveOrderLeg]
    status: LiveOrderStatus
    created_at: datetime
    last_update: datetime
    error: str | None = None


def _ensure_contract_symbol(contract: OptionContract) -> str:
    if contract.contract_symbol:
        return contract.contract_symbol
    exp = contract.expiration
    cp = "C" if contract.option_type == OptionType.CALL else "P"
    strike_key = int(round(float(contract.strike) * 1000))
    return f"{contract.symbol}{exp.strftime('%y%m%d')}{cp}{strike_key:08d}"


def _reference_price(contract: OptionContract, side: OrderSide, price_mode: Literal["mid", "bid_ask"]) -> float:
    if price_mode == "mid":
        return float(contract.mid)
    if side == "buy":
        return float(contract.ask)
    return float(contract.bid)


def _broker_side_for_leg(leg: Leg, *, opening: bool) -> OrderSide:
    """LONG: buy al abrir / sell al cerrar. SHORT: sell al abrir / buy al cerrar."""
    is_long = leg.right == OptionRight.LONG
    if opening:
        return "buy" if is_long else "sell"
    return "sell" if is_long else "buy"


def _underlying_symbol(position: Position) -> str:
    if not position.entry_legs:
        return ""
    return position.entry_legs[0].contract.symbol


def _new_order_id() -> str:
    return f"live-{uuid.uuid4().hex}"


def _now() -> datetime:
    return datetime.now(timezone.utc)


class TradierOrderBuilder:
    """
    Construye órdenes multi-leg a partir de ``Position.entry_legs``.

    Por defecto usa órdenes **limit** con precio de referencia según ``price_mode``.
    """

    def __init__(self, *, price_mode: Literal["mid", "bid_ask"] = "mid") -> None:
        self._price_mode = price_mode

    def build_open_order(self, position: Position, strategy_type: str | None = None) -> LiveOrder:
        return self._build(position, strategy_type=strategy_type or "", opening=True, leg_tag="open")

    def build_close_order(self, position: Position, strategy_type: str | None = None) -> LiveOrder:
        return self._build(position, strategy_type=strategy_type or "", opening=False, leg_tag="close")

    def _build(
        self,
        position: Position,
        *,
        strategy_type: str,
        opening: bool,
        leg_tag: str,
    ) -> LiveOrder:
        if not position.position_id:
            raise ValueError("Position.position_id requerido para órdenes live")
        legs_out: list[LiveOrderLeg] = []
        for i, leg in enumerate(position.entry_legs):
            side = _broker_side_for_leg(leg, opening=opening)
            c = leg.contract
            sym = c.symbol
            cs = _ensure_contract_symbol(c)
            limit_px = round(_reference_price(c, side, self._price_mode), 4)
            legs_out.append(
                LiveOrderLeg(
                    position_id=position.position_id,
                    leg_index=i,
                    symbol=sym,
                    contract_symbol=cs,
                    side=side,
                    quantity=int(leg.qty),
                    order_type="limit",
                    limit_price=limit_px,
                    time_in_force="day",
                    strategy_type=strategy_type,
                    tag=leg_tag,
                )
            )
        now = _now()
        u_sym = _underlying_symbol(position)
        return LiveOrder(
            order_id=_new_order_id(),
            position_id=position.position_id,
            symbol=u_sym,
            legs=legs_out,
            status="pending",
            created_at=now,
            last_update=now,
            error=None,
        )


@dataclass
class TradierLiveExecutionSink:
    """
    Destino de órdenes **sin red**: registra y devuelve copia con ``status='simulated'``.
    """

    _recorded: list[LiveOrder] = field(default_factory=list)

    def submit(self, order: LiveOrder) -> LiveOrder:
        now = _now()
        sim = replace(order, status="simulated", last_update=now, error=None)
        self._recorded.append(sim)
        return sim

    @property
    def recorded(self) -> tuple[LiveOrder, ...]:
        return tuple(self._recorded)

    def clear(self) -> None:
        self._recorded.clear()
