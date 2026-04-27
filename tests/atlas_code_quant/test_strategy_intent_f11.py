"""Tests F11 — StrategyIntent + builders sobre RadarOpportunityInternal.

Cubre:

* Modelos canónicos (``StrategyIntent``, ``StrategyLeg``, ``RiskLimits``,
  ``OpportunityRef``).
* Builders por estrategia: vertical, condor, butterfly, straddle,
  strangle, long_single.
* Defensas: input ``None``, símbolo vacío, horizon raro,
  classification reject, asset_class extraño → nunca lanzan.
* AST guard: ningún módulo F11 importa execution / autonomy / risk /
  vision / atlas_adapter / Tradier / locks.
* Promesa "nunca lanza" sobre 100+ oportunidades sintéticas.
"""

from __future__ import annotations

import ast
import itertools
import logging
from pathlib import Path
from typing import Any

import pytest

from atlas_code_quant.intake.opportunity import RadarOpportunityInternal
from atlas_code_quant.strategies.options import builder as builder_mod
from atlas_code_quant.strategies.options import intent as intent_mod
from atlas_code_quant.strategies.options.builder import (
    DTE_LONG,
    DTE_MEDIUM,
    DTE_SHORT,
    build_iron_butterfly_intents_from_opportunity,
    build_iron_condor_intents_from_opportunity,
    build_long_single_intents_from_opportunity,
    build_straddle_intents_from_opportunity,
    build_strangle_intents_from_opportunity,
    build_vertical_spread_intents_from_opportunity,
    choose_dte_for_horizon,
)
from atlas_code_quant.strategies.options.intent import (
    OpportunityRef,
    RiskLimits,
    StrategyIntent,
    StrategyLeg,
    VALID_STRATEGY_TYPES,
    build_opportunity_ref,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_opp(
    *,
    symbol: str = "SPY",
    asset_class: str = "etf",
    direction: str = "long",
    horizon_min: int | None = 240,
    classification: str = "high_conviction",
    score: float = 80.0,
    sector: str | None = "broad_market",
    optionable: bool = True,
    source: str = "quant",
    trace_id: str = "trace-test",
) -> RadarOpportunityInternal:
    """Construye una oportunidad sintética válida para builders."""
    payload = {
        "symbol": symbol,
        "asset_class": asset_class,
        "sector": sector,
        "optionable": optionable,
        "score": score,
        "classification": classification,
        "direction": direction,
        "horizon_min": horizon_min,
        "snapshot": {},
        "degradations_active": [],
        "source": source,
        "trace_id": trace_id,
        "timestamp": "2026-04-27T13:00:00+00:00",
    }
    return RadarOpportunityInternal.from_payload(payload)


# ---------------------------------------------------------------------------
# Sección 1 — Modelos
# ---------------------------------------------------------------------------


class TestModels:
    def test_valid_strategy_types_complete(self):
        # Si añadimos un nuevo tipo en intent.py debe ir también aquí.
        assert "vertical_spread" in VALID_STRATEGY_TYPES
        assert "iron_condor" in VALID_STRATEGY_TYPES
        assert "iron_butterfly" in VALID_STRATEGY_TYPES
        assert "straddle" in VALID_STRATEGY_TYPES
        assert "strangle" in VALID_STRATEGY_TYPES
        assert "long_call" in VALID_STRATEGY_TYPES
        assert "long_put" in VALID_STRATEGY_TYPES

    def test_strategy_leg_to_dict_roundtrip(self):
        leg = StrategyLeg(
            side="buy", right="call", strike_offset_steps=1, expiry_rel_dte=21
        )
        d = leg.to_dict()
        assert d["side"] == "buy"
        assert d["right"] == "call"
        assert d["strike_offset_steps"] == 1
        assert d["expiry_rel_dte"] == 21
        assert d["quantity"] == 1

    def test_risk_limits_defaults(self):
        rl = RiskLimits()
        assert rl.max_loss_usd is None
        assert rl.target_r_multiple is None
        assert rl.sizing_hint == "single_contract"

    def test_opportunity_ref_from_internal(self):
        opp = _make_opp(symbol="qqq", trace_id="abc")
        ref = build_opportunity_ref(opp)
        assert ref.symbol == "QQQ"
        assert ref.asset_class == "etf"
        assert ref.trace_id == "abc"

    def test_opportunity_ref_with_garbage_input(self):
        # Robustez: input que no es RadarOpportunityInternal → ref vacío.
        ref = build_opportunity_ref("not-an-opp")  # type: ignore[arg-type]
        assert isinstance(ref, OpportunityRef)
        assert ref.symbol == ""
        assert ref.classification == "reject"

    def test_strategy_intent_with_metadata(self):
        opp = _make_opp()
        ref = build_opportunity_ref(opp)
        leg = StrategyLeg(
            side="buy", right="call", strike_offset_steps=0, expiry_rel_dte=21
        )
        intent = StrategyIntent(
            opportunity=ref, strategy_type="long_call", legs=(leg,)
        )
        intent2 = intent.with_metadata(extra=1)
        assert intent2.metadata["extra"] == 1
        assert "extra" not in intent.metadata
        assert intent2.num_legs == 1
        assert intent2.is_multi_leg is False

    def test_strategy_intent_to_dict_shape(self):
        opp = _make_opp()
        intents = build_vertical_spread_intents_from_opportunity(opp)
        assert intents
        d = intents[0].to_dict()
        for k in ("opportunity", "strategy_type", "legs", "risk_limits", "metadata"):
            assert k in d
        assert isinstance(d["legs"], list)
        assert d["strategy_type"] in VALID_STRATEGY_TYPES


# ---------------------------------------------------------------------------
# Sección 2 — choose_dte_for_horizon
# ---------------------------------------------------------------------------


class TestDTEMapping:
    @pytest.mark.parametrize(
        "horizon, expected",
        [
            (None, DTE_MEDIUM),
            (-100, DTE_MEDIUM),
            (0, DTE_MEDIUM),
            (60, DTE_SHORT),
            (60 * 4, DTE_SHORT),
            (60 * 4 + 1, DTE_MEDIUM),
            (60 * 24 * 5, DTE_MEDIUM),
            (60 * 24 * 5 + 1, DTE_LONG),
            (60 * 24 * 30, DTE_LONG),
        ],
    )
    def test_dte_mapping(self, horizon, expected):
        assert choose_dte_for_horizon(horizon) == expected

    def test_dte_with_garbage(self):
        assert choose_dte_for_horizon("not-a-number") == DTE_MEDIUM  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Sección 3 — Builders direccionales (vertical, long_single)
# ---------------------------------------------------------------------------


class TestVerticalSpread:
    def test_long_direction_builds_bull_call_debit(self):
        opp = _make_opp(direction="long", horizon_min=240)
        intents = build_vertical_spread_intents_from_opportunity(opp)
        assert len(intents) == 1
        intent = intents[0]
        assert intent.strategy_type == "vertical_spread"
        # 2 patas, ambas calls, en mismo DTE.
        assert intent.num_legs == 2
        rights = {leg.right for leg in intent.legs}
        assert rights == {"call"}
        sides = sorted(leg.side for leg in intent.legs)
        assert sides == ["buy", "sell"]
        # offsets coherentes: long ATM, short OTM positivo.
        offsets = sorted(leg.strike_offset_steps for leg in intent.legs)
        assert offsets == [0, 2]
        # mismo DTE en ambas patas
        dtes = {leg.expiry_rel_dte for leg in intent.legs}
        assert len(dtes) == 1

    def test_short_direction_builds_bear_put_debit(self):
        opp = _make_opp(direction="short", horizon_min=10)
        intents = build_vertical_spread_intents_from_opportunity(opp)
        assert len(intents) == 1
        intent = intents[0]
        rights = {leg.right for leg in intent.legs}
        assert rights == {"put"}
        # short DTE bucket
        for leg in intent.legs:
            assert leg.expiry_rel_dte == DTE_SHORT

    def test_neutral_direction_no_vertical(self):
        opp = _make_opp(direction="neutral")
        assert build_vertical_spread_intents_from_opportunity(opp) == []

    def test_garbage_input(self):
        assert build_vertical_spread_intents_from_opportunity(None) == []
        assert build_vertical_spread_intents_from_opportunity("oops") == []  # type: ignore[arg-type]


class TestLongSingle:
    def test_long_direction_long_call(self):
        opp = _make_opp(direction="long")
        intents = build_long_single_intents_from_opportunity(opp)
        assert len(intents) == 1
        assert intents[0].strategy_type == "long_call"
        assert intents[0].num_legs == 1
        assert intents[0].legs[0].right == "call"
        assert intents[0].legs[0].side == "buy"

    def test_short_direction_long_put(self):
        opp = _make_opp(direction="short")
        intents = build_long_single_intents_from_opportunity(opp)
        assert len(intents) == 1
        assert intents[0].strategy_type == "long_put"
        assert intents[0].legs[0].right == "put"

    def test_neutral_no_long_single(self):
        opp = _make_opp(direction="neutral")
        assert build_long_single_intents_from_opportunity(opp) == []


# ---------------------------------------------------------------------------
# Sección 4 — Builders neutrales (condor, butterfly, straddle, strangle)
# ---------------------------------------------------------------------------


class TestNeutralBuilders:
    def test_iron_condor_neutral_only(self):
        opp_long = _make_opp(direction="long")
        opp_neutral = _make_opp(direction="neutral", horizon_min=60 * 24 * 10)
        assert build_iron_condor_intents_from_opportunity(opp_long) == []
        intents = build_iron_condor_intents_from_opportunity(opp_neutral)
        assert len(intents) == 1
        intent = intents[0]
        assert intent.strategy_type == "iron_condor"
        assert intent.num_legs == 4
        # ofrece dos puts y dos calls
        rights = sorted(leg.right for leg in intent.legs)
        assert rights == ["call", "call", "put", "put"]

    def test_iron_butterfly_blocks_long_horizon(self):
        opp_short = _make_opp(direction="neutral", horizon_min=60 * 24)
        intents = build_iron_butterfly_intents_from_opportunity(opp_short)
        assert len(intents) == 1
        # horizon largo → no se propone butterfly
        opp_long = _make_opp(direction="neutral", horizon_min=60 * 24 * 10)
        assert build_iron_butterfly_intents_from_opportunity(opp_long) == []

    def test_iron_butterfly_directional_skipped(self):
        opp = _make_opp(direction="long")
        assert build_iron_butterfly_intents_from_opportunity(opp) == []

    def test_straddle_neutral_only(self):
        assert build_straddle_intents_from_opportunity(_make_opp(direction="long")) == []
        intents = build_straddle_intents_from_opportunity(
            _make_opp(direction="neutral", horizon_min=240)
        )
        assert len(intents) == 1
        intent = intents[0]
        assert intent.num_legs == 2
        rights = sorted(leg.right for leg in intent.legs)
        assert rights == ["call", "put"]
        # ambas legs ATM (offset=0)
        for leg in intent.legs:
            assert leg.strike_offset_steps == 0

    def test_strangle_blocks_short_dte(self):
        # SHORT bucket (intraday) → strangle desactivado
        intents = build_strangle_intents_from_opportunity(
            _make_opp(direction="neutral", horizon_min=60)
        )
        assert intents == []
        # Medio DTE → propone strangle
        intents_ok = build_strangle_intents_from_opportunity(
            _make_opp(direction="neutral", horizon_min=60 * 24)
        )
        assert len(intents_ok) == 1
        intent = intents_ok[0]
        assert intent.strategy_type == "strangle"
        offsets = sorted(leg.strike_offset_steps for leg in intent.legs)
        assert offsets == [-2, 2]


# ---------------------------------------------------------------------------
# Sección 5 — Robustez "nunca lanza"
# ---------------------------------------------------------------------------


class TestNeverRaises:
    BUILDERS = [
        build_vertical_spread_intents_from_opportunity,
        build_iron_condor_intents_from_opportunity,
        build_iron_butterfly_intents_from_opportunity,
        build_straddle_intents_from_opportunity,
        build_strangle_intents_from_opportunity,
        build_long_single_intents_from_opportunity,
    ]

    @pytest.mark.parametrize("builder", BUILDERS)
    def test_none_input(self, builder):
        assert builder(None) == []

    @pytest.mark.parametrize("builder", BUILDERS)
    def test_non_opp_input(self, builder):
        assert builder({"symbol": "SPY"}) == []  # type: ignore[arg-type]
        assert builder("oops") == []  # type: ignore[arg-type]
        assert builder(123) == []  # type: ignore[arg-type]

    @pytest.mark.parametrize("builder", BUILDERS)
    def test_empty_symbol(self, builder):
        opp = _make_opp(symbol="")
        assert builder(opp) == []

    def test_batch_synthetic_100_plus_opportunities(self):
        # Genera >100 oportunidades sintéticas combinando ejes:
        # asset_class × direction × horizon × classification × score
        asset_classes = ["equity", "etf", "index", "unknown"]
        directions = ["long", "short", "neutral"]
        horizons = [None, -1, 0, 30, 60 * 4, 60 * 24, 60 * 24 * 10]
        classifications = ["high_conviction", "watchlist", "reject"]
        scores = [0.0, 50.0, 80.0]
        combos = list(
            itertools.product(asset_classes, directions, horizons, classifications, scores)
        )
        assert len(combos) >= 100

        for ac, d, h, c, s in combos:
            opp = _make_opp(
                asset_class=ac,
                direction=d,
                horizon_min=h,
                classification=c,
                score=s,
            )
            for builder in self.BUILDERS:
                intents = builder(opp)
                assert isinstance(intents, list)
                for intent in intents:
                    assert isinstance(intent, StrategyIntent)
                    assert intent.strategy_type in VALID_STRATEGY_TYPES
                    assert intent.num_legs > 0
                    # opp ref no debe perder symbol
                    assert intent.opportunity.symbol == opp.symbol

    def test_builder_swallow_internal_exception(self, monkeypatch, caplog):
        """Un fallo interno se traduce a [] + warning, sin propagarse."""

        def boom(*a, **kw):
            raise RuntimeError("synthetic failure")

        # Forzamos fallo dentro del builder vertical reemplazando el helper
        # interno.
        monkeypatch.setattr(builder_mod, "_vertical_spread_legs", boom)

        opp = _make_opp(direction="long")
        with caplog.at_level(logging.WARNING, logger="atlas.code_quant.strategies.options.builder"):
            out = build_vertical_spread_intents_from_opportunity(opp)
        assert out == []
        assert any(
            "fallo construyendo intents" in rec.getMessage()
            for rec in caplog.records
        )


# ---------------------------------------------------------------------------
# Sección 6 — AST guard de aislamiento
# ---------------------------------------------------------------------------


_F11_MODULES = [
    Path("atlas_code_quant/strategies/options/intent.py"),
    Path("atlas_code_quant/strategies/options/builder.py"),
]


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


@pytest.mark.parametrize("rel", [str(p) for p in _F11_MODULES])
def test_f11_modules_have_no_prohibited_imports(rel):
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / rel).read_text(encoding="utf-8")
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
    assert not bad, f"{rel} importa módulos prohibidos: {bad}"


def test_f11_intent_does_not_import_radar_client():
    """Intent NO debe importar el cliente Radar (sólo el modelo)."""
    repo_root = Path(__file__).resolve().parents[2]
    src = (repo_root / "atlas_code_quant/strategies/options/intent.py").read_text("utf-8")
    assert "radar_client" not in src
    assert "RadarClient" not in src


def test_f11_modules_do_not_touch_locks():
    """AST: ningún identificador `paper_only` / `full_live_globally_locked`
    se usa como Name (lectura de variable) en el código F11. La mención
    en docstrings es intencional (es la regla que estamos cumpliendo)."""
    repo_root = Path(__file__).resolve().parents[2]
    forbidden = {"paper_only", "full_live_globally_locked"}
    for rel in _F11_MODULES:
        src = (repo_root / rel).read_text("utf-8")
        tree = ast.parse(src)
        bad: list[str] = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Name) and node.id in forbidden:
                bad.append(node.id)
            if isinstance(node, ast.Attribute) and node.attr in forbidden:
                bad.append(node.attr)
        assert not bad, f"{rel} hace referencia a locks prohibidos: {bad}"
