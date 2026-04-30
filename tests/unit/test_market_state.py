"""Tests de contrato de :mod:`atlas_push.state`.

Fijan la forma y garantías de ``MarketState`` y sus value types tal y
como quedaron acordadas en ``docs/atlas_push/ARCHITECTURE.md`` §3.1.

No ejercitan lógica de negocio (en D el motor es pass-through); solo
congelan el contrato para que pasos posteriores no lo muevan por
accidente.
"""

from __future__ import annotations

from dataclasses import FrozenInstanceError, fields, is_dataclass
from datetime import datetime

import pytest

from atlas_push.state import (
    AccountSnapshot,
    MarketState,
    Position,
    Quote,
    RiskContext,
)


# ---------------------------------------------------------------------------
# Fixtures locales
# ---------------------------------------------------------------------------


def _account() -> AccountSnapshot:
    return AccountSnapshot(
        equity=1000.0,
        cash=500.0,
        buying_power=1500.0,
        realized_pnl_today=0.0,
    )


def _state() -> MarketState:
    return MarketState(
        as_of=datetime(2026, 4, 21, 13, 0, 0),
        account=_account(),
    )


# ---------------------------------------------------------------------------
# Dataclasses e inmutabilidad
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cls",
    [AccountSnapshot, Position, Quote, RiskContext, MarketState],
)
def test_state_types_are_frozen_dataclasses(cls: type) -> None:
    """Todos los value types de ``state`` son dataclasses ``frozen=True``."""
    assert is_dataclass(cls), f"{cls.__name__} debe ser dataclass"
    # Una dataclass frozen rechaza setattr en instancias.
    # Construimos el mínimo viable y comprobamos que falla.
    if cls is AccountSnapshot:
        inst = _account()
    elif cls is Position:
        inst = Position(
            symbol="AAPL",
            qty=1.0,
            avg_price=100.0,
            market_value=100.0,
            unrealized_pnl=0.0,
        )
    elif cls is Quote:
        inst = Quote(
            symbol="AAPL",
            bid=99.0,
            ask=101.0,
            last=100.0,
            as_of=datetime(2026, 4, 21),
        )
    elif cls is RiskContext:
        inst = RiskContext()
    else:  # MarketState
        inst = _state()

    with pytest.raises(FrozenInstanceError):
        # mypy: este setattr es precisamente lo que queremos rechazar
        setattr(inst, "__atlas_push_test_mutation__", 1)


def test_market_state_minimal_construction() -> None:
    """``MarketState`` se construye con solo ``as_of`` y ``account``."""
    st = _state()
    assert st.positions == ()
    assert st.quotes == {}
    assert st.indicators == {}
    assert st.risk_context == RiskContext()
    assert st.meta == {}


# ---------------------------------------------------------------------------
# Igualdad estructural
# ---------------------------------------------------------------------------


def test_market_state_structural_equality() -> None:
    """Dos ``MarketState`` con el mismo contenido son iguales."""
    assert _state() == _state()


def test_risk_context_structural_equality() -> None:
    """``RiskContext`` compara por valor, no por identidad."""
    assert RiskContext() == RiskContext()
    assert RiskContext(halted=True, reasons=("halt",)) == RiskContext(
        halted=True, reasons=("halt",)
    )
    assert RiskContext(halted=True) != RiskContext(halted=False)


# ---------------------------------------------------------------------------
# Colecciones: tuple (no list) donde el contrato lo exige
# ---------------------------------------------------------------------------


def test_positions_field_is_tuple_by_default() -> None:
    """``MarketState.positions`` es una tupla, no una lista."""
    st = _state()
    assert isinstance(st.positions, tuple)
    assert not isinstance(st.positions, list)


def test_risk_context_reasons_field_is_tuple() -> None:
    """``RiskContext.reasons`` es una tupla, no una lista."""
    ctx = RiskContext()
    assert isinstance(ctx.reasons, tuple)


# ---------------------------------------------------------------------------
# Defaults independientes entre instancias
# ---------------------------------------------------------------------------


def test_mutable_defaults_are_independent_between_instances() -> None:
    """Los defaults ``dict``/``RiskContext`` no se comparten entre instancias."""
    a = _state()
    b = _state()
    # dicts distintos en memoria, aunque iguales por valor
    assert a.quotes is not b.quotes
    assert a.indicators is not b.indicators
    assert a.meta is not b.meta
    # RiskContext default factory produce instancias separadas pero iguales
    assert a.risk_context == b.risk_context


# ---------------------------------------------------------------------------
# Forma del contrato: campos declarados (§3.1)
# ---------------------------------------------------------------------------


def test_account_snapshot_declared_fields() -> None:
    """``AccountSnapshot`` fija equity/cash/buying_power/realized_pnl_today."""
    names = {f.name for f in fields(AccountSnapshot)}
    assert names == {"equity", "cash", "buying_power", "realized_pnl_today"}


def test_position_declared_fields() -> None:
    """``Position`` fija los campos acordados en ARCHITECTURE §3.1."""
    names = {f.name for f in fields(Position)}
    assert names == {
        "symbol",
        "qty",
        "avg_price",
        "market_value",
        "unrealized_pnl",
    }


def test_market_state_declared_fields() -> None:
    """``MarketState`` expone exactamente los 7 campos del contrato."""
    names = {f.name for f in fields(MarketState)}
    assert names == {
        "as_of",
        "account",
        "positions",
        "quotes",
        "indicators",
        "risk_context",
        "meta",
    }
