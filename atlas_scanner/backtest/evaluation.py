from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from statistics import fmean
from typing import Any

from atlas_scanner.backtest.engine import BacktestResult


@dataclass(frozen=True)
class EvaluationRequest:
    backtest_result: BacktestResult
    top_k: int = 10
    score_threshold: float | None = None


@dataclass(frozen=True)
class SymbolEvaluationSummary:
    symbol: str
    num_observations: int
    avg_score: float | None
    max_score: float | None
    min_score: float | None
    top_k_frequency: float | None
    threshold_pass_frequency: float | None


@dataclass(frozen=True)
class ComponentEvaluationSummary:
    component: str
    num_observations: int
    avg_score: float | None
    positive_ratio: float | None
    neutral_ratio: float | None
    negative_ratio: float | None
    unavailable_ratio: float | None
    coverage_ratio: float | None


@dataclass(frozen=True)
class HistoricalEvaluationResult:
    request: EvaluationRequest
    meta: dict[str, Any]
    symbol_summaries: tuple[SymbolEvaluationSummary, ...]
    component_summaries: tuple[ComponentEvaluationSummary, ...]


def _derive_provider_status_counts(backtest_result: BacktestResult) -> dict[str, dict[str, int]]:
    counts: dict[str, dict[str, int]] = {
        "vol_macro": {"ok": 0, "empty": 0, "error": 0, "unknown": 0},
        "gamma_oi": {"ok": 0, "empty": 0, "error": 0, "unknown": 0},
    }
    for dated in backtest_result.results:
        providers_meta = dated.scan.meta.get("providers")
        if not isinstance(providers_meta, dict):
            counts["vol_macro"]["unknown"] += 1
            counts["gamma_oi"]["unknown"] += 1
            continue
        for provider in ("vol_macro", "gamma_oi"):
            provider_meta = providers_meta.get(provider)
            status = provider_meta.get("status") if isinstance(provider_meta, dict) else "unknown"
            normalized = str(status).lower()
            if normalized not in counts[provider]:
                normalized = "unknown"
            counts[provider][normalized] += 1
    return counts


def _evaluate_symbols(
    *,
    request: EvaluationRequest,
) -> tuple[SymbolEvaluationSummary, ...]:
    score_by_symbol: dict[str, list[float]] = defaultdict(list)
    top_k_hits: dict[str, int] = defaultdict(int)
    threshold_hits: dict[str, int] = defaultdict(int)

    for dated in request.backtest_result.results:
        for rank_idx, ranked in enumerate(dated.scan.ranked_symbols):
            symbol = ranked.symbol_snapshot.symbol
            score = float(ranked.score)
            score_by_symbol[symbol].append(score)

            if request.top_k > 0 and rank_idx < request.top_k:
                top_k_hits[symbol] += 1
            if request.score_threshold is not None and score >= request.score_threshold:
                threshold_hits[symbol] += 1

    summaries: list[SymbolEvaluationSummary] = []
    for symbol in sorted(score_by_symbol):
        scores = score_by_symbol[symbol]
        observations = len(scores)
        threshold_frequency = None
        if request.score_threshold is not None:
            threshold_frequency = threshold_hits[symbol] / observations
        summaries.append(
            SymbolEvaluationSummary(
                symbol=symbol,
                num_observations=observations,
                avg_score=fmean(scores),
                max_score=max(scores),
                min_score=min(scores),
                top_k_frequency=top_k_hits[symbol] / observations if request.top_k > 0 else 0.0,
                threshold_pass_frequency=threshold_frequency,
            )
        )
    return tuple(summaries)


def _evaluate_components(
    *,
    backtest_result: BacktestResult,
) -> tuple[ComponentEvaluationSummary, ...]:
    score_by_component: dict[str, list[float]] = defaultdict(list)
    status_counts: dict[str, dict[str, int]] = defaultdict(
        lambda: {"positive": 0, "neutral": 0, "negative": 0, "unavailable": 0}
    )
    observations: dict[str, int] = defaultdict(int)

    for dated in backtest_result.results:
        for ranked in dated.scan.ranked_symbols:
            explanation_components = set(ranked.component_explanations.keys())
            score_components = set(ranked.component_scores.keys())
            for component in explanation_components | score_components:
                observations[component] += 1
                explanation = ranked.component_explanations.get(component)
                status = explanation.status if explanation is not None else "unavailable"
                if status not in status_counts[component]:
                    status = "unavailable"
                status_counts[component][status] += 1

                score = ranked.component_scores.get(component)
                if score is not None and status != "unavailable":
                    score_by_component[component].append(float(score))

    summaries: list[ComponentEvaluationSummary] = []
    for component in sorted(observations):
        total = observations[component]
        positive = status_counts[component]["positive"] / total
        neutral = status_counts[component]["neutral"] / total
        negative = status_counts[component]["negative"] / total
        unavailable = status_counts[component]["unavailable"] / total
        scores = score_by_component.get(component, [])
        summaries.append(
            ComponentEvaluationSummary(
                component=component,
                num_observations=total,
                avg_score=fmean(scores) if scores else None,
                positive_ratio=positive,
                neutral_ratio=neutral,
                negative_ratio=negative,
                unavailable_ratio=unavailable,
                coverage_ratio=1.0 - unavailable,
            )
        )
    return tuple(summaries)


def evaluate_historical_backtest(request: EvaluationRequest) -> HistoricalEvaluationResult:
    results = request.backtest_result.results
    num_days = len(results)
    num_ranked_total = sum(len(day.scan.ranked_symbols) for day in results)
    num_candidates_total = sum(len(day.scan.candidate_opportunities) for day in results)
    symbols_seen = {
        ranked.symbol_snapshot.symbol
        for day in results
        for ranked in day.scan.ranked_symbols
    }

    provider_status_counts = request.backtest_result.meta.get("provider_status_counts")
    if not isinstance(provider_status_counts, dict):
        provider_status_counts = _derive_provider_status_counts(request.backtest_result)

    meta: dict[str, Any] = {
        "num_days": num_days,
        "num_symbols_seen": len(symbols_seen),
        "num_ranked_total": num_ranked_total,
        "avg_ranked_per_day": (num_ranked_total / num_days) if num_days > 0 else None,
        "avg_candidates_per_day": (num_candidates_total / num_days) if num_days > 0 else None,
        "provider_status_counts": provider_status_counts,
        "top_k": request.top_k,
        "score_threshold": request.score_threshold,
    }

    return HistoricalEvaluationResult(
        request=request,
        meta=meta,
        symbol_summaries=_evaluate_symbols(request=request),
        component_summaries=_evaluate_components(backtest_result=request.backtest_result),
    )

