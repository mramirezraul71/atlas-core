"""
Paper simulator mínimo: posición multi-leg, MTM por mids, cierre manual.

Convención de flujo de caja (alineada con Leg.premium al abrir):
- SHORT al abrir: crédito +mid×multiplier×qty
- LONG al abrir: débito -mid×multiplier×qty

Cierre al mid actual (liquidación instantánea):
- Cerrar LONG: vender → +mid×mult×qty
- Cerrar SHORT: recomprar → -mid×mult×qty

PnL no realizado / realizado: entry_net + liquidation_net
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from datetime import datetime, timezone
from enum import Enum
from typing import TYPE_CHECKING, List, Literal, Optional, Sequence, Tuple, overload

from ..models.leg import Leg
from ..models.option_contract import OptionContract, OptionRight, OptionsChain

if TYPE_CHECKING:
    from ..dsl.strategy import OptionsStrategy


class MatchKind(str, Enum):
    """Origen del contrato usado al marcar desde una cadena."""

    EXACT = "exact"
    NEAR = "near"
    FROZEN = "frozen"


@dataclass(frozen=True)
class LegMatchInfo:
    """Diagnóstico de resolución chain → contrato para una pata."""

    leg_index: int
    original_contract_symbol: str
    used_contract_symbol: str
    match_kind: MatchKind


def _strike_eq(a: float, b: float) -> bool:
    return math.isclose(a, b, rel_tol=0.0, abs_tol=1e-6)


def _contract_quality_key(c: OptionContract) -> tuple:
    """Mayor tupla = mejor candidato (cotización útil, liquidez, desempate estable)."""
    full_quote = 1 if (c.bid > 0 and c.ask > 0) else 0
    any_quote = 1 if (c.bid > 0 or c.ask > 0) else 0
    return (full_quote, any_quote, c.volume, c.open_interest, c.strike, c.contract_symbol)


def _pick_best_contract(candidates: Sequence[OptionContract]) -> OptionContract:
    return max(candidates, key=_contract_quality_key)


def _pool_for_leg(leg: Leg, chain_contracts: Sequence[OptionContract]) -> List[OptionContract]:
    ref = leg.contract
    return [
        c
        for c in chain_contracts
        if c.symbol == ref.symbol
        and c.expiration == ref.expiration
        and c.option_type == ref.option_type
    ]


def _match_leg_to_chain_contract_with_kind(
    leg: Leg,
    chain_contracts: Sequence[OptionContract],
    strike_tolerance: float,
) -> tuple[OptionContract, MatchKind]:
    """
    Resuelve un contrato de cadena para una pata: exacto → strike cercano → fallback al congelado.
    """
    pool = _pool_for_leg(leg, chain_contracts)
    if not pool:
        return replace(leg.contract), MatchKind.FROZEN

    target = leg.contract.strike
    tol = max(0.0, float(strike_tolerance))

    exact = [c for c in pool if _strike_eq(c.strike, target)]
    if exact:
        return _pick_best_contract(exact), MatchKind.EXACT

    within: List[OptionContract] = [
        c for c in pool if abs(c.strike - target) <= tol + 1e-9
    ]
    if not within:
        return replace(leg.contract), MatchKind.FROZEN

    best_dist = min(abs(c.strike - target) for c in within)
    closest = [c for c in within if abs(c.strike - target) <= best_dist + 1e-9]
    return _pick_best_contract(closest), MatchKind.NEAR


@overload
def rebuild_contracts_from_chain(
    position: "Position",
    chain: OptionsChain,
    *,
    strike_tolerance: float = 1.0,
    with_diagnostics: Literal[False] = False,
) -> list[OptionContract]: ...


@overload
def rebuild_contracts_from_chain(
    position: "Position",
    chain: OptionsChain,
    *,
    strike_tolerance: float = 1.0,
    with_diagnostics: Literal[True],
) -> Tuple[list[OptionContract], list[LegMatchInfo]]: ...


def rebuild_contracts_from_chain(
    position: "Position",
    chain: OptionsChain,
    *,
    strike_tolerance: float = 1.0,
    with_diagnostics: bool = False,
) -> list[OptionContract] | Tuple[list[OptionContract], list[LegMatchInfo]]:
    """
    Devuelve una lista de ``OptionContract`` alineada con ``position.entry_legs``,
    tomando cotizaciones de ``chain`` cuando hay match; si no, conserva el contrato
    congelado de la posición (copia vía ``replace``).

    Si ``with_diagnostics=True``, devuelve ``(contratos, lista LegMatchInfo)``.
    """
    out: list[OptionContract] = []
    diag: list[LegMatchInfo] = []
    for i, leg in enumerate(position.entry_legs):
        c, kind = _match_leg_to_chain_contract_with_kind(
            leg, chain.contracts, strike_tolerance=strike_tolerance
        )
        out.append(c)
        if with_diagnostics:
            diag.append(
                LegMatchInfo(
                    leg_index=i,
                    original_contract_symbol=leg.contract.contract_symbol,
                    used_contract_symbol=c.contract_symbol,
                    match_kind=kind,
                )
            )
    if with_diagnostics:
        return out, diag
    return out


def _clone_leg(leg: Leg) -> Leg:
    """Copia superficial del contrato para congelar estado al abrir."""
    return Leg(replace(leg.contract), leg.right, leg.qty)


def _liquidation_cashflow(leg: Leg, contract_now: OptionContract) -> float:
    """Efectivo neto al cerrar el leg al mid de contract_now (sin slippage)."""
    mult = contract_now.multiplier
    m = contract_now.mid
    if leg.right == OptionRight.LONG:
        return round(m * leg.qty * mult, 2)
    return round(-m * leg.qty * mult, 2)


@dataclass
class PositionSnapshot:
    """Instantánea de MTM en un instante."""

    timestamp: datetime
    unrealized_pnl: float
    entry_net_premium: float
    leg_mids: tuple[float, ...] = ()
    match_info: Optional[list[LegMatchInfo]] = None


@dataclass
class Position:
    """Posición paper derivada de una estrategia DSL ya construida."""

    entry_legs: tuple[Leg, ...]
    opened_at: datetime
    entry_net_premium: float
    closed_at: Optional[datetime] = None
    exit_liquidation_net: Optional[float] = None
    realized_pnl: Optional[float] = None
    snapshots: list[PositionSnapshot] = field(default_factory=list)
    position_id: str = ""

    @property
    def is_open(self) -> bool:
        return self.closed_at is None

    @property
    def last_snapshot(self) -> Optional[PositionSnapshot]:
        """Última entrada del historial en memoria, o ``None`` si está vacío."""
        if not self.snapshots:
            return None
        return self.snapshots[-1]

    @property
    def snapshot_count(self) -> int:
        return len(self.snapshots)

    def record_snapshot(self, snapshot: PositionSnapshot) -> None:
        """
        Añade un snapshot al historial. Solo permitido mientras ``is_open``;
        una vez cerrada la posición, el PnL realizado es la referencia y no se
        mezcla MTM en el mismo buffer (política conservadora).
        """
        if not self.is_open:
            raise RuntimeError(
                "No se pueden añadir snapshots al historial de una posición cerrada; "
                "el historial solo registra MTM mientras la posición está abierta."
            )
        self.snapshots.append(snapshot)

    def unrealized_pnl(self, current_contracts: Sequence[OptionContract]) -> float:
        if len(current_contracts) != len(self.entry_legs):
            raise ValueError(
                f"Se esperaban {len(self.entry_legs)} contratos, hay {len(current_contracts)}."
            )
        liq = sum(
            _liquidation_cashflow(leg, c)
            for leg, c in zip(self.entry_legs, current_contracts, strict=True)
        )
        return round(self.entry_net_premium + liq, 2)

    def mark_with_chain(
        self,
        chain: OptionsChain,
        *,
        at: Optional[datetime] = None,
        strike_tolerance: float = 1.0,
        record: bool = False,
    ) -> PositionSnapshot:
        """MTM usando cotizaciones de ``chain``; patas sin match usan el contrato congelado."""
        contracts, infos = rebuild_contracts_from_chain(
            self,
            chain,
            strike_tolerance=strike_tolerance,
            with_diagnostics=True,
        )
        u = self.unrealized_pnl(contracts)
        mids = tuple(c.mid for c in contracts)
        snap = PositionSnapshot(
            timestamp=at or datetime.now(timezone.utc),
            unrealized_pnl=u,
            entry_net_premium=self.entry_net_premium,
            leg_mids=mids,
            match_info=list(infos),
        )
        if record:
            self.record_snapshot(snap)
        return snap

    def close(
        self,
        current_contracts: Sequence[OptionContract],
        closed_at: Optional[datetime] = None,
    ) -> float:
        if not self.is_open:
            raise RuntimeError("La posición ya está cerrada.")
        when = closed_at or datetime.now(timezone.utc)
        liq = sum(
            _liquidation_cashflow(leg, c)
            for leg, c in zip(self.entry_legs, current_contracts, strict=True)
        )
        pnl = round(self.entry_net_premium + liq, 2)
        self.closed_at = when
        self.exit_liquidation_net = round(liq, 2)
        self.realized_pnl = pnl
        return pnl


class PaperSimulator:
    """Fachada mínima: abrir desde estrategia, marcar, cerrar, registro multi-posición."""

    def __init__(self) -> None:
        self._id_seq: int = 1
        self._open: dict[str, Position] = {}
        self._closed: dict[str, Position] = {}

    def _next_position_id(self) -> str:
        pid = f"pos-{self._id_seq:04d}"
        self._id_seq += 1
        return pid

    @property
    def open_positions(self) -> dict[str, Position]:
        """Copia superficial del mapa id → posición abierta."""
        return dict(self._open)

    @property
    def closed_positions(self) -> dict[str, Position]:
        """Posiciones cerradas en orden de cierre (insertion order)."""
        return dict(self._closed)

    def get_position(self, position_id: str) -> Optional[Position]:
        """Abierta o cerrada; ``None`` si el id no existe en este simulador."""
        return self._open.get(position_id) or self._closed.get(position_id)

    def list_open_positions(self) -> list[Position]:
        return sorted(self._open.values(), key=lambda p: p.position_id)

    def list_closed_positions(self) -> list[Position]:
        return list(self._closed.values())

    def open_position(
        self,
        strategy: "OptionsStrategy",
        opened_at: Optional[datetime] = None,
    ) -> Position:
        if not strategy.legs:
            raise ValueError("La estrategia no tiene legs.")
        when = opened_at or datetime.now(timezone.utc)
        frozen = tuple(_clone_leg(leg) for leg in strategy.legs)
        entry = round(sum(leg.premium for leg in frozen), 2)
        pid = self._next_position_id()
        pos = Position(
            entry_legs=frozen,
            opened_at=when,
            entry_net_premium=entry,
            position_id=pid,
        )
        self._open[pid] = pos
        return pos

    def snapshot(
        self,
        position: Position,
        current_contracts: Sequence[OptionContract],
        at: Optional[datetime] = None,
        record: bool = False,
    ) -> PositionSnapshot:
        u = position.unrealized_pnl(current_contracts)
        mids = tuple(c.mid for c in current_contracts)
        snap = PositionSnapshot(
            timestamp=at or datetime.now(timezone.utc),
            unrealized_pnl=u,
            entry_net_premium=position.entry_net_premium,
            leg_mids=mids,
        )
        if record:
            position.record_snapshot(snap)
        return snap

    def close_position(
        self,
        position: Position,
        current_contracts: Sequence[OptionContract],
        closed_at: Optional[datetime] = None,
    ) -> float:
        pid = position.position_id
        if not pid or pid not in self._open:
            raise RuntimeError(
                "La posición no está abierta en este simulador (p. ej. no fue "
                "abierta con open_position o pertenece a otro PaperSimulator)."
            )
        pnl = position.close(current_contracts, closed_at=closed_at)
        del self._open[pid]
        self._closed[pid] = position
        return pnl
