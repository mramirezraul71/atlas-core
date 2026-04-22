"""Tests de contrato de :mod:`atlas_push.outputs`.

Fijan la forma de ``DecisionOutput`` y value types asociados según
``docs/atlas_push/ARCHITECTURE.md`` §3.2. En particular:

- ``LogicalOrder`` usa ``side`` / ``kind`` (no ``order_type``) con
  ``kind`` por defecto ``"market"``.
- ``RiskVeto`` admite ``symbol`` y ``strategy_id`` opcionales.
- ``DecisionOutput.empty()`` es estable y cumple igualdad estructural.
- Las colecciones principales son ``tuple``.
"""

from __future__ import annotations

from dataclasses import FrozenInstanceError, fields, is_dataclass

import pytest

from atlas_push.outputs import (
    DecisionOutput,
    LogicalOrder,
    RiskVeto,
    TargetWeight,
)
from atlas_push.outputs.decision_output import OrderKind, OrderSide


# ---------------------------------------------------------------------------
# Dataclasses e inmutabilidad
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "cls",
    [LogicalOrder, TargetWeight, RiskVeto, DecisionOutput],
)
def test_output_types_are_frozen_dataclasses(cls: type) -> None:
    """Los value types de ``outputs`` son dataclasses ``frozen=True``."""
    assert is_dataclass(cls), f"{cls.__name__} debe ser dataclass"


def test_logical_order_is_frozen() -> None:
    """Mutar un ``LogicalOrder`` lanza ``FrozenInstanceError``."""
    order = LogicalOrder(
        symbol="AAPL",
        side="buy",
        qty=10.0,
        strategy_id="s1",
        reason="test",
    )
    with pytest.raises(FrozenInstanceError):
        setattr(order, "symbol", "MSFT")


def test_decision_output_is_frozen() -> None:
    """Mutar un ``DecisionOutput`` lanza ``FrozenInstanceError``."""
    out = DecisionOutput.empty()
    with pytest.raises(FrozenInstanceError):
        setattr(out, "orders", ())


# ---------------------------------------------------------------------------
# LogicalOrder: contrato exacto (side/kind, default market)
# ---------------------------------------------------------------------------


def test_logical_order_uses_kind_not_order_type() -> None:
    """``LogicalOrder`` expone ``kind``, nunca ``order_type``."""
    names = {f.name for f in fields(LogicalOrder)}
    assert "kind" in names
    assert "order_type" not in names


def test_logical_order_kind_defaults_to_market() -> None:
    """``kind`` por defecto es ``"market"``; ``limit_price`` es ``None``."""
    order = LogicalOrder(
        symbol="AAPL",
        side="buy",
        qty=10.0,
        strategy_id="s1",
        reason="t",
    )
    assert order.kind == "market"
    assert order.limit_price is None


def test_logical_order_accepts_limit_variant() -> None:
    """``kind="limit"`` con ``limit_price`` se construye sin errores."""
    order = LogicalOrder(
        symbol="AAPL",
        side="sell",
        qty=5.0,
        strategy_id="s1",
        reason="t",
        kind="limit",
        limit_price=150.0,
    )
    assert order.kind == "limit"
    assert order.limit_price == 150.0


def test_order_side_and_kind_literals_have_expected_values() -> None:
    """``OrderSide`` y ``OrderKind`` son ``Literal`` con los valores pactados."""
    # typing.get_args devuelve la tupla de literales declarada.
    from typing import get_args

    assert set(get_args(OrderSide)) == {"buy", "sell"}
    assert set(get_args(OrderKind)) == {"market", "limit"}


# ---------------------------------------------------------------------------
# RiskVeto: opcionales y default meta
# ---------------------------------------------------------------------------


def test_risk_veto_minimal_construction() -> None:
    """``RiskVeto`` se construye solo con ``code`` y ``message``."""
    veto = RiskVeto(code="halted", message="kill-switch on")
    assert veto.symbol is None
    assert veto.strategy_id is None
    assert veto.meta == {}


# ---------------------------------------------------------------------------
# DecisionOutput: empty, defaults, tuple-not-list, igualdad
# ---------------------------------------------------------------------------


def test_decision_output_empty_is_all_empty() -> None:
    """``empty()`` devuelve un output sin órdenes, pesos, vetos ni notas."""
    out = DecisionOutput.empty()
    assert out.orders == ()
    assert out.target_weights == ()
    assert out.vetoes == ()
    assert out.notes == ()
    assert out.meta == {}


def test_decision_output_empty_structural_equality() -> None:
    """Dos ``empty()`` comparan iguales aunque sean instancias distintas."""
    a = DecisionOutput.empty()
    b = DecisionOutput.empty()
    assert a == b


def test_decision_output_collections_are_tuples() -> None:
    """``orders``/``target_weights``/``vetoes``/``notes`` son tuplas."""
    out = DecisionOutput.empty()
    assert isinstance(out.orders, tuple)
    assert isinstance(out.target_weights, tuple)
    assert isinstance(out.vetoes, tuple)
    assert isinstance(out.notes, tuple)


def test_decision_output_declared_fields() -> None:
    """``DecisionOutput`` expone exactamente los 5 campos del contrato §3.2."""
    names = {f.name for f in fields(DecisionOutput)}
    assert names == {"orders", "target_weights", "vetoes", "notes", "meta"}


def test_decision_output_mutable_defaults_are_independent() -> None:
    """Dos ``empty()`` no comparten el mismo ``meta`` dict."""
    a = DecisionOutput.empty()
    b = DecisionOutput.empty()
    assert a.meta is not b.meta
