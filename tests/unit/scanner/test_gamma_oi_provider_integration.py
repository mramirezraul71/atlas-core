from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.fixtures.offline import OFFLINE_REFERENCE_DATETIME
from atlas_scanner.models import ScanSnapshot, SymbolSnapshot
from atlas_scanner.ports.gamma_oi_provider import (
    GammaData,
    GammaOIProvider,
    OIFlowData,
    StrikeGammaData,
)
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider
from atlas_scanner.runner.offline import run_offline_scan


@dataclass
class _StaticVolMacroProvider(VolMacroProvider):
    def get_vol_data(self, symbol: str, as_of) -> VolData:
        _ = (symbol, as_of)
        return VolData()

    def get_macro_data(self, as_of) -> MacroData:
        _ = as_of
        return MacroData()


@dataclass
class _DummyGammaOIProvider(GammaOIProvider):
    def get_gamma_data(self, symbol: str, as_of) -> GammaData:
        _ = (symbol, as_of)
        return GammaData()

    def get_oi_flow_data(self, symbol: str, as_of) -> OIFlowData:
        _ = (symbol, as_of)
        return OIFlowData()


@dataclass
class _StaticGammaOIProvider(GammaOIProvider):
    def get_gamma_data(self, symbol: str, as_of) -> GammaData:
        _ = (symbol, as_of)
        return GammaData(
            strikes=(
                StrikeGammaData(strike=490.0, call_gamma=120.0, put_gamma=-40.0),
                StrikeGammaData(strike=510.0, call_gamma=40.0, put_gamma=-120.0),
            ),
            net_gex=200000.0,
        )

    def get_oi_flow_data(self, symbol: str, as_of) -> OIFlowData:
        _ = (symbol, as_of)
        return OIFlowData(
            oi_change_1d_pct=8.0,
            call_put_volume_ratio=1.8,
            volume_imbalance=0.5,
            call_volume=120000.0,
            put_volume=60000.0,
        )


def test_dummy_gamma_oi_provider_returns_valid_empty_payloads() -> None:
    provider = _DummyGammaOIProvider()
    gamma = provider.get_gamma_data("SPY", OFFLINE_REFERENCE_DATETIME.date())
    flow = provider.get_oi_flow_data("SPY", OFFLINE_REFERENCE_DATETIME.date())
    assert gamma.strikes == ()
    assert gamma.net_gex is None
    assert flow.oi_change_1d_pct is None
    assert flow.call_put_volume_ratio is None


def test_runner_consumes_gamma_oi_provider_and_updates_meta_and_scores(monkeypatch) -> None:
    snapshot_symbol = SymbolSnapshot(
        symbol="SPY",
        asset_type="etf",
        base_currency="USD",
        ref_price=500.0,
        volatility_lookback=0.20,
        liquidity_score=0.3,
        meta={"event_risk": 0.2, "bid_ask_spread": 0.2},
    )
    mocked_snapshot = ScanSnapshot(
        snapshot_id="offline-default-1",
        created_at=OFFLINE_REFERENCE_DATETIME.isoformat(),
        universe_name="default",
        symbols=(snapshot_symbol,),
        config_version=SCORING_CONFIG.config_version,
        meta={},
    )

    def _fake_build_snapshot(*args, **kwargs):
        _ = (args, kwargs)
        return mocked_snapshot

    monkeypatch.setattr("atlas_scanner.runner.offline.build_offline_snapshot", _fake_build_snapshot)
    result = run_offline_scan(
        vol_macro_provider=_StaticVolMacroProvider(),
        gamma_oi_provider=_StaticGammaOIProvider(),
    )
    assert len(result.selected_symbols) == 1
    selected = result.selected_symbols[0]
    assert "gamma_strikes" in selected.meta
    assert "net_gex" in selected.meta
    assert "oi_change_1d_pct" in selected.meta
    assert "call_put_volume_ratio" in selected.meta

    scored = result.ranked_symbols[0]
    assert scored.component_scores["gamma"] > 0.5
    assert scored.component_scores["oi_flow"] > 0.5

