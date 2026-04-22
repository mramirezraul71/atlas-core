from __future__ import annotations

from dataclasses import dataclass

from atlas_scanner.config import SCORING_CONFIG
from atlas_scanner.data.openbb_vol_macro import OpenBBVolMacroProvider
from atlas_scanner.fixtures.offline import OFFLINE_REFERENCE_DATETIME
from atlas_scanner.models import ScanSnapshot, SymbolSnapshot
from atlas_scanner.ports.vol_macro_provider import MacroData, VolData, VolMacroProvider
from atlas_scanner.runner.offline import run_offline_scan


@dataclass
class _FakeResponse:
    results: list[dict[str, float]]


class _FakeOptionsClient:
    def chains(self, symbol: str, *, start_date: str, end_date: str) -> _FakeResponse:
        _ = (symbol, start_date, end_date)
        return _FakeResponse(
            results=[
                {"implied_volatility": 0.20},
                {"implied_volatility": 0.24},
                {"implied_volatility": 0.28},
            ]
        )


class _FakePriceClient:
    def historical(self, symbol: str, *, start_date: str, end_date: str) -> _FakeResponse:
        _ = (start_date, end_date)
        if symbol == "^VIX":
            return _FakeResponse(results=[{"close": 18.0}])
        closes = [{"close": 100.0 + idx * 0.5} for idx in range(130)]
        return _FakeResponse(results=closes)


class _FakeObbClient:
    def __init__(self) -> None:
        class _DerivativesNamespace:
            options = _FakeOptionsClient()

        class _EquityNamespace:
            class _PriceNamespace:
                historical = _FakePriceClient().historical

            price = _PriceNamespace()

        self.derivatives = _DerivativesNamespace()
        self.equity = _EquityNamespace()


def test_openbb_provider_get_vol_data_maps_iv_and_rv() -> None:
    provider = OpenBBVolMacroProvider(obb_client=_FakeObbClient())
    vol_data = provider.get_vol_data(symbol="SPY", as_of=OFFLINE_REFERENCE_DATETIME.date())
    assert tuple(vol_data.iv_history) == (0.20, 0.24, 0.28)
    assert vol_data.iv_current == 0.28
    assert "5d" in vol_data.rv_annualized
    assert "20d" in vol_data.rv_annualized


def test_openbb_provider_get_macro_data_maps_vix_and_regime() -> None:
    provider = OpenBBVolMacroProvider(obb_client=_FakeObbClient())
    macro_data = provider.get_macro_data(as_of=OFFLINE_REFERENCE_DATETIME.date())
    assert macro_data.vix == 18.0
    assert macro_data.macro_regime == "favorable"
    assert macro_data.seasonal_factor is not None


@dataclass
class _StaticVolMacroProvider(VolMacroProvider):
    def get_vol_data(self, symbol: str, as_of) -> VolData:
        _ = (symbol, as_of)
        return VolData(
            iv_history=(0.10, 0.20, 0.30, 0.40),
            iv_current=0.40,
            rv_annualized={"20d": 0.25, "10d": 0.22},
        )

    def get_macro_data(self, as_of) -> MacroData:
        _ = as_of
        return MacroData(vix=17.0, macro_regime="favorable", seasonal_factor=1.3)


def test_runner_uses_vol_macro_provider_data_in_meta_and_scoring(monkeypatch) -> None:
    snapshot_symbol = SymbolSnapshot(
        symbol="SPY",
        asset_type="etf",
        base_currency="USD",
        ref_price=500.0,
        volatility_lookback=0.15,
        liquidity_score=0.2,
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
    result = run_offline_scan(vol_macro_provider=_StaticVolMacroProvider())
    assert len(result.selected_symbols) == 1
    selected = result.selected_symbols[0]
    assert "iv_history" in selected.meta
    assert "rv_annualized" in selected.meta
    assert selected.meta.get("macro_regime") == "favorable"
    assert result.ranked_symbols[0].component_scores["vol"] > 0.0
    assert result.ranked_symbols[0].component_scores["macro"] > 0.0

