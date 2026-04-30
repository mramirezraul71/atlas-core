"""
Borde limpio entre Atlas (planner) y atlas_options_brain: estrategias + paper.

No incluye riesgo, sizing ni ejecución live; solo construcción DSL + PaperSimulator.
"""
from __future__ import annotations

from datetime import date
from typing import Callable, Literal, Optional

from ..dsl.strategy import (
    BearCallSpread,
    BearPutSpread,
    BullCallSpread,
    BullPutSpread,
    CoveredCall,
    IronCondor,
    OptionsStrategy,
)
from ..models.option_contract import OptionsChain
from ..providers.base import OptionsDataProvider
from ..simulator.paper import PaperSimulator, Position, PositionSnapshot

StrategyType = Literal[
    "iron_condor",
    "bull_put",
    "bear_call",
    "bull_call",
    "bear_put",
    "covered_call",
]

ExpirationSelector = Callable[[str, OptionsDataProvider, date], date]


def default_expiration_selector(
    symbol: str,
    provider: OptionsDataProvider,
    as_of: date,
    *,
    min_dte: int = 21,
    soft_max_dte: int = 45,
) -> date:
    """
    Primera expiración con DTE en [min_dte, soft_max_dte] si existe;
    si no, la de menor DTE entre las que cumplen DTE >= min_dte.
    """
    exps = sorted(provider.get_expirations(symbol))
    candidates: list[tuple[int, date]] = []
    for e in exps:
        dte = (e - as_of).days
        if dte >= min_dte:
            candidates.append((dte, e))
    if not candidates:
        raise ValueError(
            f"No hay expiración con DTE >= {min_dte} para {symbol!r} (as_of={as_of})."
        )
    in_window = [(d, e) for d, e in candidates if d <= soft_max_dte]
    if in_window:
        return min(in_window, key=lambda x: x[0])[1]
    return min(candidates, key=lambda x: x[0])[1]


def _build_from_chain(
    strategy_type: StrategyType,
    chain: OptionsChain,
    params: dict,
) -> OptionsStrategy:
    qty = int(params.get("qty", 1))
    if strategy_type == "iron_condor":
        return IronCondor.from_chain(
            chain,
            wing_delta=float(params.get("wing_delta", 0.16)),
            wing_width=float(params.get("wing_width", 5.0)),
            qty=qty,
        )
    if strategy_type == "bull_put":
        return BullPutSpread.from_chain(
            chain,
            short_delta=float(params.get("short_delta", 0.25)),
            width=float(params.get("width", 10.0)),
            qty=qty,
        )
    if strategy_type == "bear_call":
        return BearCallSpread.from_chain(
            chain,
            short_delta=float(params.get("short_delta", 0.25)),
            width=float(params.get("width", 10.0)),
            qty=qty,
        )
    if strategy_type == "bull_call":
        return BullCallSpread.from_chain(
            chain,
            long_delta=float(params.get("long_delta", 0.50)),
            width=float(params.get("width", 10.0)),
            qty=qty,
        )
    if strategy_type == "bear_put":
        return BearPutSpread.from_chain(
            chain,
            long_delta=float(params.get("long_delta", 0.50)),
            width=float(params.get("width", 10.0)),
            qty=qty,
        )
    if strategy_type == "covered_call":
        return CoveredCall.from_chain(
            chain,
            short_delta=float(params.get("short_delta", 0.25)),
            stock_basis=float(params.get("stock_basis", 0.0)),
            qty=qty,
        )
    raise ValueError(f"strategy_type desconocido: {strategy_type!r}")


class AtlasOptionsClient:
    """
    Fachada de alto nivel para Atlas: cadena → estrategia → posición paper → marks.

    Mantiene un ``PaperSimulator`` interno y metadatos ligeros por ``position_id``.
    """

    def __init__(
        self,
        provider: OptionsDataProvider,
        *,
        as_of: Optional[date] = None,
    ) -> None:
        self._provider = provider
        self._as_of = as_of
        self._sim = PaperSimulator()
        self._position_meta: dict[str, dict[str, str]] = {}

    @property
    def provider(self) -> OptionsDataProvider:
        return self._provider

    @property
    def simulator(self) -> PaperSimulator:
        """Expuesto para tests avanzados; Atlas debería preferir métodos de alto nivel."""
        return self._sim

    def _calendar_today(self) -> date:
        return self._as_of if self._as_of is not None else date.today()

    def build_strategy_from_chain(
        self,
        symbol: str,
        strategy_type: StrategyType,
        *,
        provider: Optional[OptionsDataProvider] = None,
        expiration_selector: Optional[ExpirationSelector] = None,
        params: Optional[dict] = None,
    ) -> OptionsStrategy:
        """
        Obtiene expiración (política por defecto o ``expiration_selector``),
        descarga cadena vía proveedor y construye la estrategia DSL.
        """
        prov = provider or self._provider
        as_of = self._calendar_today()
        selector = expiration_selector or default_expiration_selector
        expiration = selector(symbol, prov, as_of)
        chain = prov.get_chain(symbol, expiration)
        p = dict(params or {})
        strat = _build_from_chain(strategy_type, chain, p)
        if not strat.legs:
            raise ValueError("Estrategia sin legs.")
        return strat

    def open_paper_position(
        self,
        strategy: OptionsStrategy,
        *,
        atlas_strategy_type: Optional[StrategyType] = None,
    ) -> Position:
        """Abre posición paper en el simulador interno y registra tipo/símbolo para vistas Atlas."""
        pos = self._sim.open_position(strategy)
        sym = strategy.legs[0].contract.symbol if strategy.legs else ""
        st_key = atlas_strategy_type if atlas_strategy_type is not None else strategy.name
        self._position_meta[pos.position_id] = {
            "strategy_type": st_key,
            "symbol": sym,
        }
        return pos

    def get_position_meta(self, position_id: str) -> dict[str, str]:
        """Metadatos guardados al abrir (p. ej. nombre de estrategia y símbolo)."""
        return dict(self._position_meta.get(position_id, {}))

    def list_open_positions(self) -> list[Position]:
        return self._sim.list_open_positions()

    def list_closed_positions(self) -> list[Position]:
        return self._sim.list_closed_positions()

    def get_position(self, position_id: str) -> Optional[Position]:
        return self._sim.get_position(position_id)

    def mark_all_positions(self) -> list[PositionSnapshot]:
        """
        Para cada posición abierta: cadena fresca del proveedor (mismo símbolo/vencimiento
        que la primera pata) y ``mark_with_chain(..., record=True)``.

        El orden de la lista coincide con ``list_open_positions()``.
        """
        snapshots: list[PositionSnapshot] = []
        for pos in self._sim.list_open_positions():
            if not pos.entry_legs:
                continue
            ref = pos.entry_legs[0].contract
            chain = self._provider.get_chain(ref.symbol, ref.expiration)
            snapshots.append(pos.mark_with_chain(chain, record=True))
        return snapshots

    def atlas_view_row(
        self,
        position: Position,
        snapshot: Optional[PositionSnapshot] = None,
    ) -> str:
        """Una línea compacta tipo planner (requiere snapshot reciente o None → último del historial)."""
        meta = self._position_meta.get(position.position_id, {})
        sym = meta.get("symbol", "")
        st = meta.get("strategy_type", "")
        snap = snapshot
        if snap is None and position.snapshots:
            snap = position.snapshots[-1]
        upnl = snap.unrealized_pnl if snap is not None else None
        upnl_s = f"{upnl}" if upnl is not None else "n/a"
        return (
            f"id={position.position_id} symbol={sym} strategy={st} "
            f"net_premium={position.entry_net_premium} uPnL={upnl_s}"
        )
