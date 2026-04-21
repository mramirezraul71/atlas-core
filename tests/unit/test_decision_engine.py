"""Tests de contrato de :mod:`atlas_push.engine`.

Fijan las garantías del scaffold del Paso D:

- Constructibilidad con y sin estrategias / risk.
- Firma ``decide(MarketState) -> DecisionOutput`` y pass-through.
- Pureza: mismo input → mismo output estructural.
- Composabilidad sin ejecución de estrategias (no-op en D).
- Regla de oro a nivel de fuente: ni ``engine/`` ni ``state/`` ni
  ``outputs/`` importan símbolos del brain core mayor de ATLAS.
- Invariante de hermanos: ``atlas_push.engine`` y ``atlas_push.intents``
  no se importan mutuamente.
"""

from __future__ import annotations

import pathlib
import re
from dataclasses import FrozenInstanceError, is_dataclass
from datetime import datetime

import pytest

from atlas_push.engine import (
    DecisionEngine,
    RiskManager,
    Strategy,
    StrategyProposal,
)
from atlas_push.outputs import DecisionOutput
from atlas_push.state import AccountSnapshot, MarketState


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _state(hour: int = 13) -> MarketState:
    return MarketState(
        as_of=datetime(2026, 4, 21, hour, 0, 0),
        account=AccountSnapshot(
            equity=1000.0,
            cash=500.0,
            buying_power=1500.0,
            realized_pnl_today=0.0,
        ),
    )


class _NoopStrategy:
    """Estrategia dummy que cumple estructuralmente el Protocol."""

    def propose(self, state: MarketState) -> StrategyProposal:  # noqa: ARG002
        return StrategyProposal()


class _NoopRisk:
    """Risk manager dummy que cumple estructuralmente el Protocol."""

    def apply(
        self, state: MarketState, draft: DecisionOutput
    ) -> DecisionOutput:  # noqa: ARG002
        return draft


# ---------------------------------------------------------------------------
# Constructibilidad y forma
# ---------------------------------------------------------------------------


def test_decision_engine_is_frozen_dataclass() -> None:
    """``DecisionEngine`` es dataclass ``frozen=True``."""
    assert is_dataclass(DecisionEngine)
    eng = DecisionEngine()
    with pytest.raises(FrozenInstanceError):
        setattr(eng, "risk", None)


def test_decision_engine_default_construction() -> None:
    """``DecisionEngine()`` sin args: strategies tupla vacía, risk None."""
    eng = DecisionEngine()
    assert eng.strategies == ()
    assert isinstance(eng.strategies, tuple)
    assert eng.risk is None


def test_decision_engine_accepts_strategies_and_risk() -> None:
    """Se puede construir con estrategias y risk; ambos quedan accesibles."""
    s = _NoopStrategy()
    r = _NoopRisk()
    eng = DecisionEngine(strategies=(s,), risk=r)
    assert eng.strategies == (s,)
    assert eng.risk is r


def test_strategy_proposal_is_empty_frozen_dataclass() -> None:
    """``StrategyProposal`` es dataclass frozen vacía (placeholder)."""
    assert is_dataclass(StrategyProposal)
    sp = StrategyProposal()
    with pytest.raises(FrozenInstanceError):
        setattr(sp, "foo", 1)


# ---------------------------------------------------------------------------
# Pass-through, pureza, composabilidad
# ---------------------------------------------------------------------------


def test_decide_returns_empty_decision_output_for_any_state() -> None:
    """``decide(state)`` devuelve ``DecisionOutput.empty()`` siempre."""
    eng = DecisionEngine()
    out = eng.decide(_state())
    assert out == DecisionOutput.empty()


def test_decide_is_pure_same_state_same_output() -> None:
    """Mismo estado → misma salida estructural entre llamadas."""
    eng = DecisionEngine()
    st = _state()
    assert eng.decide(st) == eng.decide(st)


def test_decide_ignores_different_states_in_d_passthrough() -> None:
    """En D, estados distintos producen el mismo output (no-op)."""
    eng = DecisionEngine()
    assert eng.decide(_state(hour=13)) == eng.decide(_state(hour=14))


def test_decide_does_not_execute_configured_strategies_in_d() -> None:
    """En D el scaffold no invoca las estrategias configuradas."""
    calls: list[MarketState] = []

    class RecordingStrategy:
        def propose(self, state: MarketState) -> StrategyProposal:
            calls.append(state)
            return StrategyProposal()

    eng = DecisionEngine(strategies=(RecordingStrategy(),))
    eng.decide(_state())
    assert calls == []  # pass-through: la estrategia no se ejecuta


def test_protocols_are_runtime_checkable() -> None:
    """``Strategy`` y ``RiskManager`` aceptan ``isinstance`` structural."""
    assert isinstance(_NoopStrategy(), Strategy)
    assert isinstance(_NoopRisk(), RiskManager)


# ---------------------------------------------------------------------------
# Regla de oro (source-level): sin símbolos del brain core mayor
# ---------------------------------------------------------------------------


_FORBIDDEN_IMPORTS_RE = re.compile(
    r"^\s*(?:from|import)\s+"
    r"(?:"
    r"brain_core|mission_manager|safety_kernel|state_bus|"
    r"arbitration|policy_store|modules\.command_router"
    r")\b",
    re.MULTILINE,
)


def _package_sources(pkg_relpath: str) -> list[pathlib.Path]:
    """Devuelve los ``.py`` de primer/segundo nivel de un subpaquete.

    Busca desde la raíz del repo (tres niveles por encima de este
    archivo): ``<repo>/atlas_push/<pkg_relpath>/**/*.py``.
    """
    repo_root = pathlib.Path(__file__).resolve().parents[2]
    pkg_root = repo_root / "atlas_push" / pkg_relpath
    return sorted(pkg_root.rglob("*.py"))


@pytest.mark.parametrize("pkg", ["engine", "state", "outputs"])
def test_source_does_not_import_major_brain_core_symbols(pkg: str) -> None:
    """Ningún ``.py`` de engine/state/outputs importa símbolos prohibidos."""
    offenders: list[str] = []
    for path in _package_sources(pkg):
        text = path.read_text(encoding="utf-8")
        if _FORBIDDEN_IMPORTS_RE.search(text):
            offenders.append(str(path))
    assert offenders == [], (
        "Fuentes con imports prohibidos del brain core mayor: "
        f"{offenders}"
    )


# ---------------------------------------------------------------------------
# Invariante de hermanos: engine <-> intents desacoplados
# ---------------------------------------------------------------------------


_ENGINE_IN_INTENTS_RE = re.compile(
    r"^\s*(?:from|import)\s+atlas_push\.engine\b",
    re.MULTILINE,
)

_INTENTS_IN_ENGINE_RE = re.compile(
    r"^\s*(?:from|import)\s+atlas_push\.intents\b",
    re.MULTILINE,
)


def test_engine_does_not_import_intents() -> None:
    """``atlas_push.engine`` no importa ``atlas_push.intents``."""
    offenders: list[str] = []
    for path in _package_sources("engine"):
        if _INTENTS_IN_ENGINE_RE.search(path.read_text(encoding="utf-8")):
            offenders.append(str(path))
    assert offenders == [], (
        "engine/ no debe importar atlas_push.intents: " f"{offenders}"
    )


def test_intents_does_not_import_engine() -> None:
    """``atlas_push.intents`` no importa ``atlas_push.engine``."""
    offenders: list[str] = []
    for path in _package_sources("intents"):
        if _ENGINE_IN_INTENTS_RE.search(path.read_text(encoding="utf-8")):
            offenders.append(str(path))
    assert offenders == [], (
        "intents/ no debe importar atlas_push.engine: " f"{offenders}"
    )
