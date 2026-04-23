from __future__ import annotations

from datetime import date, datetime, timezone

import pytest

from atlas_scanner.backtest.engine import BacktestRequest, BacktestResult, DatedScanResult
from atlas_scanner.backtest.evaluation import (
    EvaluationRequest,
    evaluate_historical_backtest,
)
from atlas_scanner.config_loader import build_default_scan_config_offline
from atlas_scanner.models import SymbolSnapshot
from atlas_scanner.runner.offline import OfflineScanResult
from atlas_scanner.scoring.offline import ComponentExplanation, ScoredSymbol


def _scored_symbol(
    symbol: str,
    score: float,
    *,
    vol_score: float,
    vol_status: str,
    gamma_score: float | None,
    gamma_status: str,
) -> ScoredSymbol:
    snapshot = SymbolSnapshot(
        symbol=symbol,
        asset_type="equity",
        base_currency="USD",
        ref_price=100.0,
        volatility_lookback=0.2,
        liquidity_score=0.8,
        meta={},
    )
    component_scores = {"vol": vol_score}
    if gamma_score is not None:
        component_scores["gamma"] = gamma_score
    component_explanations = {
        "vol": ComponentExplanation("vol", vol_score, vol_status, ("vol reason",)),
        "gamma": ComponentExplanation("gamma", gamma_score, gamma_status, ("gamma reason",)),
    }
    return ScoredSymbol(
        symbol_snapshot=snapshot,
        score=score,
        component_scores=component_scores,
        component_explanations=component_explanations,
        top_reasons=("reason",),
        explanation="x",
        strengths=(),
        penalties=(),
    )


def _offline_scan(
    as_of: date,
    ranked_symbols: tuple[ScoredSymbol, ...],
    *,
    provider_status: tuple[str, str],
) -> OfflineScanResult:
    config = build_default_scan_config_offline()
    provider_meta = {
        "vol_macro": {"status": provider_status[0]},
        "gamma_oi": {"status": provider_status[1]},
    }
    return OfflineScanResult(
        config=config,
        reference_datetime=datetime(as_of.year, as_of.month, as_of.day, tzinfo=timezone.utc),
        selected_symbols=tuple(item.symbol_snapshot for item in ranked_symbols),
        ranked_symbols=ranked_symbols,
        universe_name=config.universe_name,
        data_source_path=("mem",),
        meta={"providers": provider_meta},
        candidate_opportunities=(),
    )


def _sample_backtest_result() -> BacktestResult:
    day1 = _offline_scan(
        date(2026, 1, 2),
        ranked_symbols=(
            _scored_symbol("AAA", 0.90, vol_score=0.90, vol_status="positive", gamma_score=None, gamma_status="unavailable"),
            _scored_symbol("BBB", 0.50, vol_score=0.50, vol_status="neutral", gamma_score=0.20, gamma_status="negative"),
        ),
        provider_status=("ok", "ok"),
    )
    day2 = _offline_scan(
        date(2026, 1, 3),
        ranked_symbols=(
            _scored_symbol("CCC", 0.80, vol_score=0.80, vol_status="positive", gamma_score=0.90, gamma_status="positive"),
            _scored_symbol("AAA", 0.70, vol_score=0.30, vol_status="negative", gamma_score=None, gamma_status="unavailable"),
        ),
        provider_status=("empty", "error"),
    )
    request = BacktestRequest(start_date=date(2026, 1, 2), end_date=date(2026, 1, 3))
    return BacktestResult(
        request=request,
        results=(
            DatedScanResult(as_of=date(2026, 1, 2), scan=day1),
            DatedScanResult(as_of=date(2026, 1, 3), scan=day2),
        ),
        meta={},
    )


def test_evaluate_historical_backtest_computes_global_and_symbol_metrics() -> None:
    result = evaluate_historical_backtest(
        EvaluationRequest(
            backtest_result=_sample_backtest_result(),
            top_k=1,
            score_threshold=0.75,
        )
    )

    assert result.meta["num_days"] == 2
    assert result.meta["num_symbols_seen"] == 3
    assert result.meta["num_ranked_total"] == 4
    assert result.meta["avg_ranked_per_day"] == pytest.approx(2.0)
    assert result.meta["avg_candidates_per_day"] == pytest.approx(0.0)
    assert result.meta["provider_status_counts"]["vol_macro"]["ok"] == 1
    assert result.meta["provider_status_counts"]["vol_macro"]["empty"] == 1
    assert result.meta["provider_status_counts"]["gamma_oi"]["error"] == 1

    by_symbol = {item.symbol: item for item in result.symbol_summaries}
    assert by_symbol["AAA"].num_observations == 2
    assert by_symbol["AAA"].avg_score == pytest.approx(0.80)
    assert by_symbol["AAA"].top_k_frequency == pytest.approx(0.5)
    assert by_symbol["AAA"].threshold_pass_frequency == pytest.approx(0.5)
    assert by_symbol["BBB"].top_k_frequency == pytest.approx(0.0)
    assert by_symbol["CCC"].top_k_frequency == pytest.approx(1.0)


def test_evaluate_historical_backtest_component_unavailable_and_coverage() -> None:
    result = evaluate_historical_backtest(
        EvaluationRequest(
            backtest_result=_sample_backtest_result(),
            top_k=2,
            score_threshold=None,
        )
    )

    by_component = {item.component: item for item in result.component_summaries}
    gamma_summary = by_component["gamma"]
    assert gamma_summary.num_observations == 4
    assert gamma_summary.avg_score == pytest.approx(0.55)
    assert gamma_summary.positive_ratio == pytest.approx(0.25)
    assert gamma_summary.negative_ratio == pytest.approx(0.25)
    assert gamma_summary.unavailable_ratio == pytest.approx(0.5)
    assert gamma_summary.coverage_ratio == pytest.approx(0.5)

    aaa = next(item for item in result.symbol_summaries if item.symbol == "AAA")
    assert aaa.threshold_pass_frequency is None


def test_evaluate_historical_backtest_handles_empty_results() -> None:
    empty = BacktestResult(
        request=BacktestRequest(start_date=date(2026, 1, 1), end_date=date(2026, 1, 1)),
        results=(),
        meta={},
    )
    result = evaluate_historical_backtest(EvaluationRequest(backtest_result=empty))

    assert result.meta["num_days"] == 0
    assert result.meta["num_symbols_seen"] == 0
    assert result.meta["num_ranked_total"] == 0
    assert result.meta["avg_ranked_per_day"] is None
    assert result.meta["avg_candidates_per_day"] is None
    assert result.symbol_summaries == ()
    assert result.component_summaries == ()

