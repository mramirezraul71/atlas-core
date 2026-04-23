from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date

from atlas_scanner.backtest.engine import BacktestRequest, run_backtest, run_walk_forward
from atlas_scanner.ports.gamma_oi_provider import GammaData, GammaOIProvider, OIFlowData
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider


@dataclass
class _TrackingVolMacroProvider(VolMacroProvider):
    macro_calls: list[date] = field(default_factory=list)
    vol_calls: list[tuple[str, date]] = field(default_factory=list)

    def get_vol_data(self, symbol: str, as_of: date) -> VolData:
        self.vol_calls.append((symbol, as_of))
        base = 20.0 + float(as_of.day % 10)
        return VolData(
            iv_history=(base - 2.0, base - 1.0, base, base + 1.0),
            iv_current=base,
            rv_annualized={"20d": base - 3.0},
        )

    def get_macro_data(self, as_of: date) -> MacroData:
        self.macro_calls.append(as_of)
        return MacroData(
            vix=18.0 + float(as_of.day % 4),
            macro_regime="risk_on",
            seasonal_factor=1.2,
        )


@dataclass
class _TrackingGammaOIProvider(GammaOIProvider):
    gamma_calls: list[tuple[str, date]] = field(default_factory=list)
    flow_calls: list[tuple[str, date]] = field(default_factory=list)

    def get_gamma_data(self, symbol: str, as_of: date) -> GammaData:
        self.gamma_calls.append((symbol, as_of))
        return GammaData(net_gex=-100_000.0)

    def get_oi_flow_data(self, symbol: str, as_of: date) -> OIFlowData:
        self.flow_calls.append((symbol, as_of))
        return OIFlowData(
            oi_change_1d_pct=5.0,
            call_put_volume_ratio=1.4,
            volume_imbalance=0.2,
            call_volume=1200.0,
            put_volume=900.0,
        )


def test_run_backtest_collects_one_result_per_day_with_metadata() -> None:
    vol_provider = _TrackingVolMacroProvider()
    gamma_provider = _TrackingGammaOIProvider()
    request = BacktestRequest(
        start_date=date(2026, 1, 2),
        end_date=date(2026, 1, 4),
        universe_symbols=("SPY", "QQQ"),
        vol_macro_provider=vol_provider,
        gamma_oi_provider=gamma_provider,
    )

    result = run_backtest(request)

    assert len(result.results) == 3
    assert tuple(item.as_of for item in result.results) == (
        date(2026, 1, 2),
        date(2026, 1, 3),
        date(2026, 1, 4),
    )
    assert all(item.scan.meta["as_of_date"] == item.as_of.isoformat() for item in result.results)
    assert result.meta["mode"] == "backtest"
    assert result.meta["num_days"] == 3
    assert result.meta["num_symbols_total"] >= 6
    assert "provider_status_counts" in result.meta
    assert result.meta["provider_status_counts"]["vol_macro"]["ok"] == 3
    assert result.meta["provider_status_counts"]["gamma_oi"]["ok"] == 3
    assert vol_provider.macro_calls == [date(2026, 1, 2), date(2026, 1, 3), date(2026, 1, 4)]


def test_run_walk_forward_evaluates_expected_dates() -> None:
    vol_provider = _TrackingVolMacroProvider()
    gamma_provider = _TrackingGammaOIProvider()
    request = BacktestRequest(
        start_date=date(2026, 1, 1),
        end_date=date(2026, 1, 5),
        universe_symbols=("SPY",),
        vol_macro_provider=vol_provider,
        gamma_oi_provider=gamma_provider,
    )

    result = run_walk_forward(request, window_days=3)

    assert tuple(item.as_of for item in result.results) == (
        date(2026, 1, 3),
        date(2026, 1, 4),
        date(2026, 1, 5),
    )
    assert result.meta["mode"] == "walk_forward"
    assert result.meta["window_days"] == 3
    assert result.meta["num_days"] == 3
    assert vol_provider.macro_calls == [date(2026, 1, 3), date(2026, 1, 4), date(2026, 1, 5)]


def test_run_walk_forward_rejects_invalid_window() -> None:
    request = BacktestRequest(start_date=date(2026, 1, 1), end_date=date(2026, 1, 2))
    try:
        run_walk_forward(request, window_days=0)
    except ValueError as exc:
        assert "window_days must be > 0" in str(exc)
    else:
        raise AssertionError("expected ValueError for invalid window_days")

