"""
Tests de MarketContextBuilder y AtlasOptionsAutoPlanner.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_market_context import (  # noqa: E402
    AtlasOptionsAutoPlanner,
    MarketContextBuilder,
)
from atlas.core.options_planner import MarketContext  # noqa: E402


class FakeSignalProvider:
    def __init__(
        self,
        *,
        spot: float = 100.0,
        trend: str = "bull",
        iv_rank: float | None = 50.0,
        regime: str | None = None,
    ) -> None:
        self._spot = spot
        self._trend = trend
        self._iv = iv_rank
        self._regime = regime

    def get_spot(self, symbol: str) -> float:
        return self._spot

    def get_trend(self, symbol: str) -> str:
        return self._trend

    def get_iv_rank(self, symbol: str) -> float | None:
        return self._iv

    def get_regime(self, symbol: str) -> str | None:
        return self._regime


class StubPlannerService:
    def __init__(self) -> None:
        self.calls: list[MarketContext] = []

    def plan_and_open(self, context: MarketContext) -> str:
        self.calls.append(context)
        return "paper-pos-stub-1"


class TestMarketContextBuilder(unittest.TestCase):
    def test_build_for_symbol_basic(self) -> None:
        prov = FakeSignalProvider(
            spot=450.25,
            trend="bear",
            iv_rank=40.0,
            regime="stress",
        )
        b = MarketContextBuilder(prov)
        ctx = b.build_for_symbol("SPY")
        self.assertEqual(ctx.symbol, "SPY")
        self.assertEqual(ctx.spot, 450.25)
        self.assertEqual(ctx.trend, "bear")
        self.assertEqual(ctx.iv_rank, 40.0)
        self.assertEqual(ctx.regime, "stress")

    def test_trend_uptrend_maps_bull(self) -> None:
        prov = FakeSignalProvider(trend="uptrend")
        ctx = MarketContextBuilder(prov).build_for_symbol("QQQ")
        self.assertEqual(ctx.trend, "bull")

    def test_trend_unknown_uses_fallback(self) -> None:
        prov = FakeSignalProvider(trend="quantum")
        ctx = MarketContextBuilder(prov).build_for_symbol("IWM")
        self.assertEqual(ctx.trend, "sideways")

    def test_custom_trend_fallback(self) -> None:
        prov = FakeSignalProvider(trend="nope")
        b = MarketContextBuilder(
            prov,
            default_symbol_config={"trend_fallback": "bear"},
        )
        self.assertEqual(b.build_for_symbol("X").trend, "bear")

    def test_iv_rank_clamp(self) -> None:
        hi = MarketContextBuilder(FakeSignalProvider(iv_rank=150)).build_for_symbol("A")
        self.assertEqual(hi.iv_rank, 100.0)
        lo = MarketContextBuilder(FakeSignalProvider(iv_rank=-10)).build_for_symbol("A")
        self.assertEqual(lo.iv_rank, 0.0)

    def test_spot_non_positive_raises(self) -> None:
        with self.assertRaises(ValueError) as ar:
            MarketContextBuilder(FakeSignalProvider(spot=0)).build_for_symbol("SPY")
        self.assertIn("spot", str(ar.exception).lower())

    def test_regime_map(self) -> None:
        prov = FakeSignalProvider(regime="LONG_EQUITY")
        b = MarketContextBuilder(
            prov,
            default_symbol_config={
                "regime_map": {"long_equity": "has_stock"},
            },
        )
        self.assertEqual(b.build_for_symbol("DIA").regime, "has_stock")

    def test_build_many(self) -> None:
        class MultiProv:
            def get_spot(self, symbol: str) -> float:
                return 100.0

            def get_trend(self, symbol: str) -> str:
                return "bull" if symbol == "A" else "bear"

            def get_iv_rank(self, symbol: str) -> float | None:
                return 50.0

            def get_regime(self, symbol: str) -> str | None:
                return None

        b = MarketContextBuilder(MultiProv())
        ctxs = b.build_many(["A", "B"])
        self.assertEqual(len(ctxs), 2)
        self.assertEqual(ctxs[0].symbol, "A")
        self.assertEqual(ctxs[0].trend, "bull")
        self.assertEqual(ctxs[1].trend, "bear")


class TestAtlasOptionsAutoPlanner(unittest.TestCase):
    def test_auto_plan_and_open_delegates(self) -> None:
        prov = FakeSignalProvider(spot=200.0, trend="sideways", iv_rank=None)
        builder = MarketContextBuilder(prov)
        stub = StubPlannerService()
        auto = AtlasOptionsAutoPlanner(builder, stub)
        pid = auto.auto_plan_and_open(" SPY  ")
        self.assertEqual(pid, "paper-pos-stub-1")
        self.assertEqual(len(stub.calls), 1)
        c0 = stub.calls[0]
        self.assertIsInstance(c0, MarketContext)
        self.assertEqual(c0.symbol, "SPY")
        self.assertEqual(c0.spot, 200.0)
        self.assertEqual(c0.trend, "sideways")


if __name__ == "__main__":
    unittest.main()
