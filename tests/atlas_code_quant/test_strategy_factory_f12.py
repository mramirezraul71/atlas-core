"""Tests F12 — Strategy Factory dispatcher sobre RadarOpportunityInternal.

Cubre:

* Reglas duras de gating: reject, score bajo, no optionable, asset_class
  fuera de {equity, etf, index}.
* Direccional vs neutral: combinaciones de builders coherentes.
* Filtro por ``classification == watchlist`` → solo defined-risk.
* Garantía "nunca lanza" sobre 100+ oportunidades sintéticas.
* AST guard: factory NO importa execution / autonomy / risk / vision /
  atlas_adapter / tradier / broker_router / live_*.
"""

from __future__ import annotations

import ast
import itertools
from pathlib import Path

import pytest

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.strategies.factory.dispatch import (
    DEFAULT_MIN_SCORE,
    DEFINED_RISK_TYPES,
    VALID_ASSET_CLASSES,
    build_strategies_for_opportunities,
    build_strategies_for_opportunity,
)
from atlas_code_quant.strategies.options.intent import (
    VALID_STRATEGY_TYPES,
    StrategyIntent,
)


def _make_opp(
    *,
    symbol: str = "SPY",
    asset_class: str = "etf",
    direction: str = "long",
    horizon_min: int | None = 240,
    classification: str = "high_conviction",
    score: float = 80.0,
    optionable: bool = True,
) -> RadarOpportunityInternal:
    return RadarOpportunityInternal.from_payload(
        {
            "symbol": symbol,
            "asset_class": asset_class,
            "sector": None,
            "optionable": optionable,
            "score": score,
            "classification": classification,
            "direction": direction,
            "horizon_min": horizon_min,
            "snapshot": {},
            "degradations_active": [],
            "source": "quant",
            "trace_id": "trace-f12",
            "timestamp": "2026-04-27T13:00:00+00:00",
        }
    )


# ---------------------------------------------------------------------------
# Sección 1 — Gates
# ---------------------------------------------------------------------------


class TestGates:
    def test_none_input_returns_empty(self):
        assert build_strategies_for_opportunity(None) == []

    def test_non_opp_input_returns_empty(self):
        assert build_strategies_for_opportunity({"symbol": "X"}) == []  # type: ignore[arg-type]

    def test_empty_symbol(self):
        assert build_strategies_for_opportunity(_make_opp(symbol="")) == []

    def test_reject_classification(self):
        opp = _make_opp(classification="reject", score=99.0)
        assert build_strategies_for_opportunity(opp) == []

    def test_low_score_blocked(self):
        opp = _make_opp(score=50.0)
        assert build_strategies_for_opportunity(opp) == []

    def test_min_score_threshold_inclusive(self):
        opp = _make_opp(score=DEFAULT_MIN_SCORE)
        intents = build_strategies_for_opportunity(opp)
        assert intents  # exactamente en el umbral debe pasar.

    def test_not_optionable_blocked(self):
        opp = _make_opp(optionable=False)
        assert build_strategies_for_opportunity(opp) == []

    @pytest.mark.parametrize("ac", ["forex", "crypto", "future", "unknown"])
    def test_invalid_asset_class_blocked(self, ac):
        opp = _make_opp(asset_class=ac)
        assert build_strategies_for_opportunity(opp) == []

    def test_unknown_direction_blocked(self):
        # ``RadarOpportunityInternal`` normaliza directions desconocidas a
        # "neutral", por lo que para forzar este caso construimos
        # directamente. Aquí validamos que normalización a "neutral" SÍ
        # funciona — y que cualquier direction válida produce intents
        # cuando todo lo demás es válido.
        opp = _make_opp(direction="not-a-direction", classification="high_conviction")
        # normalizado a "neutral" → debería producir intents neutrales.
        intents = build_strategies_for_opportunity(opp)
        assert intents
        for it in intents:
            assert it.strategy_type in {
                "iron_condor",
                "iron_butterfly",
                "straddle",
                "strangle",
            }


# ---------------------------------------------------------------------------
# Sección 2 — Direccional
# ---------------------------------------------------------------------------


class TestDirectional:
    def test_long_etf_short_horizon_produces_vertical_and_long_call(self):
        opp = _make_opp(direction="long", horizon_min=120)  # SHORT bucket
        intents = build_strategies_for_opportunity(opp)
        types = sorted({i.strategy_type for i in intents})
        assert types == ["long_call", "vertical_spread"]
        # todas las legs DTE_SHORT
        for it in intents:
            for leg in it.legs:
                assert leg.expiry_rel_dte == 7

    def test_short_equity_medium_horizon(self):
        opp = _make_opp(
            asset_class="equity", direction="short", horizon_min=60 * 24
        )
        intents = build_strategies_for_opportunity(opp)
        types = sorted({i.strategy_type for i in intents})
        assert types == ["long_put", "vertical_spread"]
        # vertical es bear put debit
        vertical = [i for i in intents if i.strategy_type == "vertical_spread"][0]
        rights = {leg.right for leg in vertical.legs}
        assert rights == {"put"}


# ---------------------------------------------------------------------------
# Sección 3 — Neutral (index/horizon largo es escenario clásico de condor)
# ---------------------------------------------------------------------------


class TestNeutral:
    def test_neutral_index_long_horizon_emits_condor_and_strangle(self):
        opp = _make_opp(
            asset_class="index",
            direction="neutral",
            horizon_min=60 * 24 * 10,  # > 5d → DTE_LONG
        )
        intents = build_strategies_for_opportunity(opp)
        types = sorted({i.strategy_type for i in intents})
        # iron_butterfly bloqueado en DTE_LONG
        assert "iron_butterfly" not in types
        assert "iron_condor" in types
        assert "straddle" in types
        assert "strangle" in types

    def test_neutral_short_horizon_butterfly_yes_strangle_no(self):
        opp = _make_opp(
            asset_class="index", direction="neutral", horizon_min=60
        )
        intents = build_strategies_for_opportunity(opp)
        types = sorted({i.strategy_type for i in intents})
        assert "iron_butterfly" in types
        assert "strangle" not in types  # bloqueado en DTE_SHORT
        assert "iron_condor" in types
        assert "straddle" in types


# ---------------------------------------------------------------------------
# Sección 4 — Filtro por classification
# ---------------------------------------------------------------------------


class TestClassificationFilter:
    def test_watchlist_keeps_only_defined_risk(self):
        opp = _make_opp(
            asset_class="index",
            direction="neutral",
            horizon_min=60,
            classification="watchlist",
            score=72.0,
        )
        intents = build_strategies_for_opportunity(opp)
        assert intents
        for it in intents:
            assert it.strategy_type in DEFINED_RISK_TYPES

    def test_watchlist_directional_only_defined_risk(self):
        opp = _make_opp(
            direction="long", classification="watchlist", score=72.0
        )
        intents = build_strategies_for_opportunity(opp)
        # long_single NO es defined-risk en F12 → debe filtrarse
        types = {i.strategy_type for i in intents}
        assert "long_call" not in types
        assert "vertical_spread" in types


# ---------------------------------------------------------------------------
# Sección 5 — Robustez "nunca lanza" sobre 100+ oportunidades
# ---------------------------------------------------------------------------


class TestRobustness:
    def test_synthetic_grid_never_raises(self):
        asset_classes = ["equity", "etf", "index", "forex"]
        directions = ["long", "short", "neutral"]
        horizons = [None, -1, 0, 60, 60 * 24, 60 * 24 * 10]
        classifications = ["high_conviction", "watchlist", "reject"]
        scores = [0.0, 50.0, 80.0]
        optionables = [True, False]
        combos = list(
            itertools.product(
                asset_classes,
                directions,
                horizons,
                classifications,
                scores,
                optionables,
            )
        )
        assert len(combos) >= 100

        for ac, d, h, c, s, o in combos:
            opp = _make_opp(
                asset_class=ac,
                direction=d,
                horizon_min=h,
                classification=c,
                score=s,
                optionable=o,
            )
            intents = build_strategies_for_opportunity(opp)
            assert isinstance(intents, list)
            for it in intents:
                assert isinstance(it, StrategyIntent)
                assert it.strategy_type in VALID_STRATEGY_TYPES
                assert it.num_legs > 0
                # opp.symbol propagado
                assert it.opportunity.symbol == opp.symbol

    def test_batch_helper(self):
        opps = [
            _make_opp(symbol="SPY", direction="long"),
            _make_opp(symbol="QQQ", direction="neutral", asset_class="index"),
            _make_opp(symbol="AAPL", direction="short", asset_class="equity"),
            _make_opp(symbol="X", classification="reject", score=99.0),
        ]
        intents = build_strategies_for_opportunities(opps)
        symbols = {i.opportunity.symbol for i in intents}
        assert "SPY" in symbols and "QQQ" in symbols and "AAPL" in symbols
        # X (reject) no aparece
        assert "X" not in symbols

    def test_batch_helper_none(self):
        assert build_strategies_for_opportunities(None) == []  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Sección 6 — AST guard de aislamiento
# ---------------------------------------------------------------------------


_F12_MODULE = Path("atlas_code_quant/strategies/factory/dispatch.py")

_PROHIBITED_IMPORTS = (
    "atlas_code_quant.execution",
    "atlas_code_quant.operations",
    "atlas_code_quant.autonomy",
    "atlas_code_quant.risk",
    "atlas_code_quant.vision",
    "atlas_adapter",
    "tradier",
    "broker_router",
    "live_loop",
    "live_activation",
)


def test_factory_has_no_prohibited_imports():
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / _F12_MODULE).read_text("utf-8")
    tree = ast.parse(src)
    bad: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                for prohibited in _PROHIBITED_IMPORTS:
                    if alias.name == prohibited or alias.name.startswith(prohibited + "."):
                        bad.append(alias.name)
        elif isinstance(node, ast.ImportFrom):
            mod = node.module or ""
            for prohibited in _PROHIBITED_IMPORTS:
                if mod == prohibited or mod.startswith(prohibited + "."):
                    bad.append(mod)
    assert not bad, f"factory importa módulos prohibidos: {bad}"


def test_factory_constants_match_intent_module():
    """Los tipos defined-risk y asset_classes deben ser un subset de
    los conocidos en el módulo de intent."""
    assert DEFINED_RISK_TYPES.issubset(VALID_STRATEGY_TYPES)
    assert VALID_ASSET_CLASSES == frozenset({"equity", "etf", "index"})
