from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date, timedelta
from statistics import fmean
from typing import Any

from atlas_scanner.config_loader import ScanConfig, build_default_scan_config_offline
from atlas_scanner.ports.gamma_oi_provider import GammaOIProvider
from atlas_scanner.ports.vol_macro_provider import VolMacroProvider
from atlas_scanner.runner.offline import OfflineScanResult, run_offline_scan


@dataclass(frozen=True)
class BacktestRequest:
    start_date: date
    end_date: date
    universe_symbols: tuple[str, ...] = ()
    config: ScanConfig | None = None
    vol_macro_provider: VolMacroProvider | None = None
    gamma_oi_provider: GammaOIProvider | None = None
    score_threshold: float = 0.60
    scan_filters: dict[str, float | int] = field(default_factory=dict)


@dataclass(frozen=True)
class DatedScanResult:
    as_of: date
    scan: OfflineScanResult


@dataclass(frozen=True)
class BacktestResult:
    request: BacktestRequest
    results: tuple[DatedScanResult, ...]
    meta: dict[str, Any]


def _iter_dates(start_date: date, end_date: date) -> tuple[date, ...]:
    if end_date < start_date:
        return ()
    day_count = (end_date - start_date).days + 1
    return tuple(start_date + timedelta(days=offset) for offset in range(day_count))


def _status_counts(results: tuple[DatedScanResult, ...]) -> dict[str, dict[str, int]]:
    counts = {
        "vol_macro": {"ok": 0, "empty": 0, "error": 0, "unknown": 0},
        "gamma_oi": {"ok": 0, "empty": 0, "error": 0, "unknown": 0},
    }
    for dated in results:
        providers_meta = dated.scan.meta.get("providers")
        if not isinstance(providers_meta, dict):
            counts["vol_macro"]["unknown"] += 1
            counts["gamma_oi"]["unknown"] += 1
            continue

        for provider_key in ("vol_macro", "gamma_oi"):
            provider_meta = providers_meta.get(provider_key)
            status = provider_meta.get("status") if isinstance(provider_meta, dict) else "unknown"
            normalized = str(status).lower()
            if normalized not in counts[provider_key]:
                normalized = "unknown"
            counts[provider_key][normalized] += 1
    return counts


def _symbol_score_summary(
    results: tuple[DatedScanResult, ...],
    *,
    score_threshold: float,
) -> dict[str, dict[str, float | int]]:
    by_symbol: dict[str, list[float]] = {}
    for dated in results:
        for ranked in dated.scan.ranked_symbols:
            by_symbol.setdefault(ranked.symbol_snapshot.symbol, []).append(float(ranked.score))

    summary: dict[str, dict[str, float | int]] = {}
    for symbol, scores in by_symbol.items():
        opportunities = sum(1 for score in scores if score >= score_threshold)
        summary[symbol] = {
            "num_observations": len(scores),
            "avg_score": fmean(scores),
            "min_score": min(scores),
            "max_score": max(scores),
            "opportunity_count": opportunities,
            "opportunity_ratio": opportunities / len(scores),
        }
    return summary


def _component_score_summary(results: tuple[DatedScanResult, ...]) -> dict[str, dict[str, float | int]]:
    by_component: dict[str, list[float]] = {}
    for dated in results:
        for ranked in dated.scan.ranked_symbols:
            for component, value in ranked.component_scores.items():
                by_component.setdefault(component, []).append(float(value))

    summary: dict[str, dict[str, float | int]] = {}
    for component, values in by_component.items():
        summary[component] = {
            "num_observations": len(values),
            "avg_score": fmean(values),
            "min_score": min(values),
            "max_score": max(values),
        }
    return summary


def _build_backtest_meta(
    *,
    mode: str,
    request: BacktestRequest,
    dates_evaluated: tuple[date, ...],
    results: tuple[DatedScanResult, ...],
    window_days: int | None = None,
) -> dict[str, Any]:
    symbols_total = sum(len(item.scan.selected_symbols) for item in results)
    ranked_total = sum(len(item.scan.ranked_symbols) for item in results)
    candidates_total = sum(len(item.scan.candidate_opportunities) for item in results)
    provider_status_counts = _status_counts(results)

    meta: dict[str, Any] = {
        "mode": mode,
        "num_days": len(dates_evaluated),
        "evaluated_dates": tuple(day.isoformat() for day in dates_evaluated),
        "num_symbols_total": symbols_total,
        "num_ranked_total": ranked_total,
        "num_candidates_total": candidates_total,
        "provider_status_counts": provider_status_counts,
        "score_threshold": request.score_threshold,
        "symbol_score_summary": _symbol_score_summary(
            results,
            score_threshold=request.score_threshold,
        ),
        "component_score_summary": _component_score_summary(results),
    }
    if window_days is not None:
        meta["window_days"] = window_days
    return meta


def run_backtest(request: BacktestRequest) -> BacktestResult:
    dates_evaluated = _iter_dates(request.start_date, request.end_date)
    effective_config = request.config or build_default_scan_config_offline()

    results = tuple(
        DatedScanResult(
            as_of=as_of,
            scan=run_offline_scan(
                config=effective_config,
                vol_macro_provider=request.vol_macro_provider,
                gamma_oi_provider=request.gamma_oi_provider,
                as_of_date=as_of,
                symbol_allowlist=request.universe_symbols or None,
                **request.scan_filters,
            ),
        )
        for as_of in dates_evaluated
    )
    return BacktestResult(
        request=request,
        results=results,
        meta=_build_backtest_meta(
            mode="backtest",
            request=request,
            dates_evaluated=dates_evaluated,
            results=results,
        ),
    )


def run_walk_forward(request: BacktestRequest, window_days: int) -> BacktestResult:
    if window_days <= 0:
        raise ValueError("window_days must be > 0")

    first_evaluation = request.start_date + timedelta(days=window_days - 1)
    dates_evaluated = _iter_dates(first_evaluation, request.end_date)
    effective_config = request.config or build_default_scan_config_offline()

    results = tuple(
        DatedScanResult(
            as_of=as_of,
            scan=run_offline_scan(
                config=effective_config,
                vol_macro_provider=request.vol_macro_provider,
                gamma_oi_provider=request.gamma_oi_provider,
                as_of_date=as_of,
                symbol_allowlist=request.universe_symbols or None,
                **request.scan_filters,
            ),
        )
        for as_of in dates_evaluated
    )
    return BacktestResult(
        request=request,
        results=results,
        meta=_build_backtest_meta(
            mode="walk_forward",
            request=request,
            dates_evaluated=dates_evaluated,
            results=results,
            window_days=window_days,
        ),
    )

