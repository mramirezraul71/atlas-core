from __future__ import annotations

from dataclasses import dataclass

import pytest

from atlas_scanner.data.openbb_gamma_oi import OpenBBGammaOIProvider
from atlas_scanner.fixtures.offline import OFFLINE_REFERENCE_DATETIME


@dataclass
class _FakeResponse:
    results: list[dict[str, float | str]]


class _FakeOptionsClient:
    def chains(self, symbol: str, *, start_date: str, end_date: str) -> _FakeResponse:
        _ = (symbol, start_date, end_date)
        return _FakeResponse(
            results=[
                {
                    "strike": 100.0,
                    "option_type": "call",
                    "gamma": 0.20,
                    "volume": 1000.0,
                    "open_interest": 5000.0,
                    "oi_change_1d_pct": 5.0,
                    "net_gex": 100.0,
                },
                {
                    "strike": 100.0,
                    "option_type": "put",
                    "gamma": -0.15,
                    "volume": 800.0,
                    "open_interest": 4500.0,
                    "oi_change_1d_pct": -1.0,
                    "net_gex": -50.0,
                },
                {
                    "strike": 105.0,
                    "call_gamma": 0.30,
                    "put_gamma": -0.10,
                },
            ]
        )


class _FailingOptionsClient:
    def chains(self, symbol: str, *, start_date: str, end_date: str) -> _FakeResponse:
        _ = (symbol, start_date, end_date)
        raise RuntimeError("chains failed")


class _FakeObbClient:
    def __init__(self) -> None:
        class _DerivativesNamespace:
            options = _FakeOptionsClient()

        self.derivatives = _DerivativesNamespace()


class _FailingObbClient:
    def __init__(self) -> None:
        class _DerivativesNamespace:
            options = _FailingOptionsClient()

        self.derivatives = _DerivativesNamespace()


def test_openbb_gamma_oi_provider_maps_gamma_and_flow_fields() -> None:
    provider = OpenBBGammaOIProvider(obb_client=_FakeObbClient())
    as_of = OFFLINE_REFERENCE_DATETIME.date()

    gamma = provider.get_gamma_data("SPY", as_of)
    flow = provider.get_oi_flow_data("SPY", as_of)

    assert len(gamma.strikes) == 2
    strike_100 = next(item for item in gamma.strikes if item.strike == 100.0)
    strike_105 = next(item for item in gamma.strikes if item.strike == 105.0)
    assert strike_100.call_gamma == pytest.approx(0.20)
    assert strike_100.put_gamma == pytest.approx(-0.15)
    assert strike_105.call_gamma == pytest.approx(0.30)
    assert strike_105.put_gamma == pytest.approx(-0.10)
    assert gamma.net_gex == pytest.approx(50.0)

    assert flow.call_volume == pytest.approx(1000.0)
    assert flow.put_volume == pytest.approx(800.0)
    assert flow.call_put_volume_ratio == pytest.approx(1.25)
    assert flow.volume_imbalance == pytest.approx((1000.0 - 800.0) / 1800.0)
    assert flow.oi_change_1d_pct == pytest.approx(2.0)
    assert flow.meta["call_open_interest"] == pytest.approx(5000.0)
    assert flow.meta["put_open_interest"] == pytest.approx(4500.0)


def test_openbb_gamma_oi_provider_returns_empty_when_client_unavailable() -> None:
    provider = OpenBBGammaOIProvider(obb_client=None)
    as_of = OFFLINE_REFERENCE_DATETIME.date()

    gamma = provider.get_gamma_data("SPY", as_of)
    flow = provider.get_oi_flow_data("SPY", as_of)

    assert gamma.strikes == ()
    assert gamma.net_gex is None
    assert flow.oi_change_1d_pct is None
    assert flow.call_put_volume_ratio is None
    assert flow.volume_imbalance is None
    assert flow.call_volume is None
    assert flow.put_volume is None
    assert flow.meta == {}
    diagnostics = provider.get_diagnostics()
    assert diagnostics["gamma_data"] == "no_backend"
    assert diagnostics["oi_flow_data"] == "no_backend"


def test_openbb_gamma_oi_provider_handles_vendor_errors_fail_soft() -> None:
    provider = OpenBBGammaOIProvider(obb_client=_FailingObbClient())
    as_of = OFFLINE_REFERENCE_DATETIME.date()

    gamma = provider.get_gamma_data("SPY", as_of)
    flow = provider.get_oi_flow_data("SPY", as_of)

    assert gamma.strikes == ()
    assert gamma.net_gex is None
    assert flow.oi_change_1d_pct is None
    assert flow.call_put_volume_ratio is None
    assert flow.meta == {}
    diagnostics = provider.get_diagnostics()
    assert diagnostics["gamma_data"] == "error"
    assert diagnostics["oi_flow_data"] == "error"


def test_openbb_gamma_oi_provider_stays_as_pure_translator_contract() -> None:
    provider = OpenBBGammaOIProvider(obb_client=_FakeObbClient())
    gamma = provider.get_gamma_data("SPY", OFFLINE_REFERENCE_DATETIME.date())
    flow = provider.get_oi_flow_data("SPY", OFFLINE_REFERENCE_DATETIME.date())

    assert not hasattr(gamma, "score")
    assert not hasattr(flow, "score")
    assert not hasattr(gamma, "component_scores")
    assert not hasattr(flow, "component_scores")

