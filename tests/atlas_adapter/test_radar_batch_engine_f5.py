"""F5 — Tests del RadarBatchEngine (multi-símbolo).

Inyecta un ``symbol_builder`` falso que NO depende de Quant ni del runtime
HTTP. Verifica:
    * Mapeo summary → RadarOpportunity (score 0..100, classification, direction).
    * Tolerancia a fallo parcial (un símbolo lanza excepción → batch sigue).
    * Filtros: ``min_score``, ``asset_class``, ``sector``, ``optionable``.
    * Orden: score desc, símbolo asc.
    * ``limit`` se aplica DESPUÉS del scoring/filtrado.
    * ``trace_id`` por batch y por oportunidad.
"""
from __future__ import annotations

from typing import Any

import pytest

from atlas_adapter.services.radar_batch_engine import (
    HIGH_CONVICTION_MIN_SCORE,
    WATCHLIST_MIN_SCORE,
    RadarBatchEngine,
    compute_opportunity_score,
    opportunity_from_summary,
)
from atlas_adapter.services.universe_provider import UniverseEntry, UniverseProvider


def _summary(
    *,
    fast: float = 0.5,
    structural: float = 0.5,
    bias: str = "neutral",
    classification: str = "operable_with_degradation",
    timestamp: str = "2026-01-01T00:00:00+00:00",
    degradations: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    return {
        "symbol": "X",
        "last_update": timestamp,
        "transport": {"sse": True, "stub": False, "quant": True},
        "radar": {
            "signal": {
                "timestamp": timestamp,
                "bias": bias,
                "meta": {
                    "snapshot_classification": classification,
                    "fast_pressure_score": fast,
                    "structural_confidence_score": structural,
                    "fast_structural_alignment": 0.0,
                    "fast_structural_divergence_score": 0.0,
                    "horizon_conflict": False,
                    "cross_horizon_alignment": "-",
                },
            }
        },
        "decision_gate": {"recent": [], "latest": None},
        "camera_context": {},
        "provider_health_summary": {"providers_checked": 1, "degraded_count": 0},
        "degradations_active": list(degradations or []),
    }


def _make_universe() -> UniverseProvider:
    return UniverseProvider(
        (
            UniverseEntry("AAA", "ETF A", "etf", "broad_market", True, True),
            UniverseEntry("BBB", "Equity B", "equity", "technology", True, True),
            UniverseEntry("CCC", "Index C", "index", "broad_market", False, True),
            UniverseEntry("DDD", "Equity D", "equity", "technology", True, True),
        )
    )


@pytest.mark.asyncio
async def test_score_and_classification_thresholds() -> None:
    # fast=0.9, struct=0.9 → composite=0.9 → score=90 → high_conviction.
    summary = _summary(fast=0.9, structural=0.9, bias="long")
    score = compute_opportunity_score(summary)
    assert score >= HIGH_CONVICTION_MIN_SCORE

    # fast=0.5, struct=0.5 → 50 → watchlist.
    summary = _summary(fast=0.5, structural=0.5, bias="short")
    score = compute_opportunity_score(summary)
    assert WATCHLIST_MIN_SCORE <= score < HIGH_CONVICTION_MIN_SCORE


@pytest.mark.asyncio
async def test_opportunity_from_summary_direction_and_source() -> None:
    entry = UniverseEntry("AAA", "n", "etf", "x", True, True)
    summary = _summary(fast=0.8, structural=0.6, bias="long")
    opp = opportunity_from_summary(
        entry=entry, summary=summary, live=True, batch_trace_id="trace1"
    )
    assert opp["direction"] == "long"
    assert opp["source"] == "quant"
    assert opp["trace_id"].startswith("trace1:")
    assert opp["snapshot"]["bias"] == "long"


@pytest.mark.asyncio
async def test_compute_opportunities_orders_by_score_desc_then_symbol_asc() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        # Forzamos scores conocidos por símbolo.
        scores = {"AAA": 0.2, "BBB": 0.8, "CCC": 0.8, "DDD": 0.5}
        return _summary(fast=scores[symbol], structural=scores[symbol], bias="long"), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(limit=10, min_score=0.0)
    syms = [it["symbol"] for it in res.items]
    # BBB y CCC empatados a 80; orden tie-break por símbolo asc → BBB antes que CCC.
    assert syms == ["BBB", "CCC", "DDD", "AAA"]
    assert res.evaluated == 4
    assert res.succeeded == 4
    assert res.failed == 0
    assert res.any_live is True
    assert res.source == "quant"


@pytest.mark.asyncio
async def test_partial_failure_does_not_abort_batch() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        if symbol == "BBB":
            raise RuntimeError("simulated_quant_failure")
        return _summary(fast=0.7, structural=0.7, bias="long"), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(limit=10, min_score=0.0)
    assert res.evaluated == 4
    assert res.succeeded == 3
    assert res.failed == 1
    assert "BBB" not in {it["symbol"] for it in res.items}
    codes = [d["code"] for d in res.global_degradations]
    assert "OPPORTUNITY_BUILD_FAILED" in codes


@pytest.mark.asyncio
async def test_min_score_filter_excludes_low_scores() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        scores = {"AAA": 0.1, "BBB": 0.9, "CCC": 0.5, "DDD": 0.3}
        return _summary(fast=scores[symbol], structural=scores[symbol]), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(limit=10, min_score=50.0)
    syms = [it["symbol"] for it in res.items]
    assert syms == ["BBB", "CCC"]


@pytest.mark.asyncio
async def test_filters_asset_class_and_optionable() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        return _summary(fast=0.6, structural=0.6), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(asset_class="equity", limit=10)
    syms = sorted(it["symbol"] for it in res.items)
    assert syms == ["BBB", "DDD"]

    res2 = await engine.compute_opportunities(optionable=True, limit=10)
    syms2 = sorted(it["symbol"] for it in res2.items)
    assert "CCC" not in syms2  # index no optionable filtrado


@pytest.mark.asyncio
async def test_limit_applies_after_scoring() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        scores = {"AAA": 0.1, "BBB": 0.9, "CCC": 0.8, "DDD": 0.7}
        return _summary(fast=scores[symbol], structural=scores[symbol]), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(limit=2, min_score=0.0)
    assert [it["symbol"] for it in res.items] == ["BBB", "CCC"]
    assert res.evaluated == 4  # universo completo evaluado
    assert res.returned == 2


@pytest.mark.asyncio
async def test_compute_opportunity_for_returns_none_for_unknown_symbol() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        return _summary(), True, None

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunity_for("ZZZ_NOT_REAL")
    assert res is None


@pytest.mark.asyncio
async def test_all_stub_marks_global_degradation() -> None:
    universe = _make_universe()

    async def builder(symbol: str) -> tuple[dict[str, Any], bool, dict[str, Any] | None]:
        return _summary(), False, None  # live=False ⇒ stub

    engine = RadarBatchEngine(universe=universe, symbol_builder=builder)
    res = await engine.compute_opportunities(limit=10, min_score=0.0)
    assert res.any_live is False
    assert res.source == "stub"
    codes = [d["code"] for d in res.global_degradations]
    assert "QUANT_UNREACHABLE_BATCH" in codes
    for it in res.items:
        assert it["source"] == "stub"
