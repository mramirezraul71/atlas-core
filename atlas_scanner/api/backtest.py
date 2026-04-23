from __future__ import annotations

from dataclasses import dataclass
from datetime import date
from typing import Mapping, Sequence

from atlas_scanner.backtest.engine import BacktestRequest, BacktestResult, run_backtest, run_walk_forward
from atlas_scanner.config_loader import ScanConfig
from atlas_scanner.ports.gamma_oi_provider import GammaOIProvider
from atlas_scanner.ports.vol_macro_provider import VolMacroProvider


@dataclass
class BacktestScanner:
    """
    High-level backtest facade for the offline scanner engine.
    """

    def scan_backtest(
        self,
        *,
        start_date: date,
        end_date: date,
        universe_symbols: Sequence[str] | None = None,
        config: ScanConfig | None = None,
        vol_macro_provider: VolMacroProvider | None = None,
        gamma_oi_provider: GammaOIProvider | None = None,
        score_threshold: float = 0.60,
        scan_filters: Mapping[str, float | int] | None = None,
    ) -> BacktestResult:
        request = BacktestRequest(
            start_date=start_date,
            end_date=end_date,
            universe_symbols=tuple(universe_symbols or ()),
            config=config,
            vol_macro_provider=vol_macro_provider,
            gamma_oi_provider=gamma_oi_provider,
            score_threshold=score_threshold,
            scan_filters=dict(scan_filters or {}),
        )
        return run_backtest(request)

    def scan_walk_forward(
        self,
        *,
        start_date: date,
        end_date: date,
        window_days: int,
        universe_symbols: Sequence[str] | None = None,
        config: ScanConfig | None = None,
        vol_macro_provider: VolMacroProvider | None = None,
        gamma_oi_provider: GammaOIProvider | None = None,
        score_threshold: float = 0.60,
        scan_filters: Mapping[str, float | int] | None = None,
    ) -> BacktestResult:
        request = BacktestRequest(
            start_date=start_date,
            end_date=end_date,
            universe_symbols=tuple(universe_symbols or ()),
            config=config,
            vol_macro_provider=vol_macro_provider,
            gamma_oi_provider=gamma_oi_provider,
            score_threshold=score_threshold,
            scan_filters=dict(scan_filters or {}),
        )
        return run_walk_forward(request, window_days=window_days)


_default_backtest_scanner = BacktestScanner()


def scan_backtest_offline(
    *,
    start_date: date,
    end_date: date,
    universe_symbols: Sequence[str] | None = None,
    config: ScanConfig | None = None,
    vol_macro_provider: VolMacroProvider | None = None,
    gamma_oi_provider: GammaOIProvider | None = None,
    score_threshold: float = 0.60,
    scan_filters: Mapping[str, float | int] | None = None,
) -> BacktestResult:
    return _default_backtest_scanner.scan_backtest(
        start_date=start_date,
        end_date=end_date,
        universe_symbols=universe_symbols,
        config=config,
        vol_macro_provider=vol_macro_provider,
        gamma_oi_provider=gamma_oi_provider,
        score_threshold=score_threshold,
        scan_filters=scan_filters,
    )


def scan_walk_forward_offline(
    *,
    start_date: date,
    end_date: date,
    window_days: int,
    universe_symbols: Sequence[str] | None = None,
    config: ScanConfig | None = None,
    vol_macro_provider: VolMacroProvider | None = None,
    gamma_oi_provider: GammaOIProvider | None = None,
    score_threshold: float = 0.60,
    scan_filters: Mapping[str, float | int] | None = None,
) -> BacktestResult:
    return _default_backtest_scanner.scan_walk_forward(
        start_date=start_date,
        end_date=end_date,
        window_days=window_days,
        universe_symbols=universe_symbols,
        config=config,
        vol_macro_provider=vol_macro_provider,
        gamma_oi_provider=gamma_oi_provider,
        score_threshold=score_threshold,
        scan_filters=scan_filters,
    )

