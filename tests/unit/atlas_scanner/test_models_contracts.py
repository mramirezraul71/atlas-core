from __future__ import annotations

import atlas_scanner

from atlas_scanner.models import (
    CandidateOpportunity,
    ScanSnapshot,
    ScannerRunMetrics,
    ScannerRunResult,
    ScoreBreakdown,
    ScoreComponent,
    SymbolSnapshot,
)


def test_package_import_is_clean() -> None:
    assert hasattr(atlas_scanner, "SCORING_CONFIG")


def test_contracts_are_frozen_dataclasses() -> None:
    contracts = (
        SymbolSnapshot,
        ScanSnapshot,
        ScoreComponent,
        ScoreBreakdown,
        CandidateOpportunity,
        ScannerRunMetrics,
        ScannerRunResult,
    )
    for contract in contracts:
        assert getattr(contract, "__dataclass_params__").frozen is True


def test_contracts_minimal_instantiation_and_defaults() -> None:
    symbol = SymbolSnapshot(
        symbol="SPY",
        asset_type="etf",
        base_currency="USD",
        ref_price=520.25,
        volatility_lookback=0.18,
        liquidity_score=0.95,
    )
    symbol.meta["source"] = "unit-test"
    assert symbol.meta["source"] == "unit-test"

    snapshot = ScanSnapshot(
        snapshot_id="snap-001",
        created_at="2026-04-22T12:00:00Z",
        universe_name="SPY_NQ_INTRADAY",
        symbols=(symbol,),
        config_version="scanner-s0-v1",
    )
    assert snapshot.meta == {}

    component = ScoreComponent(
        name="momentum_stub",
        contribution=0.12,
        weight=0.20,
        raw=0.60,
    )
    breakdown = ScoreBreakdown(raw_score=0.62, normalized_score=0.62)
    assert breakdown.components == ()

    detailed_breakdown = ScoreBreakdown(
        raw_score=0.62,
        normalized_score=0.62,
        components=(component,),
    )
    assert len(detailed_breakdown.components) == 1

    candidate = CandidateOpportunity(
        candidate_id="cand-001",
        snapshot_id=snapshot.snapshot_id,
        symbol="SPY",
        direction="long",
        thesis="mean_reversion_intraday",
        score=0.62,
        score_breakdown=detailed_breakdown,
        time_horizon_minutes=60,
        max_risk_pct=0.02,
        expected_rr=2.0,
    )
    assert candidate.tags == ()
    assert candidate.meta == {}

    metrics = ScannerRunMetrics(
        total_symbols=1,
        evaluated_symbols=1,
        generated_candidates=1,
        avg_score=0.62,
        p95_score=0.62,
        error_count=0,
    )
    assert metrics.warnings == ()

    result = ScannerRunResult(
        snapshot_id=snapshot.snapshot_id,
        started_at="2026-04-22T12:00:01Z",
        finished_at="2026-04-22T12:00:02Z",
        candidates=(candidate,),
        metrics=metrics,
    )
    assert result.meta == {}
    assert result.candidates[0].candidate_id == "cand-001"

