"""
Tests del planner de opciones (MarketContext → estrategia) y orquestación con el servicio.
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

from atlas.core.options_client import AtlasOptionsRiskError  # noqa: E402
from atlas.core.options_planner import (  # noqa: E402
    AtlasOptionsPlannerService,
    MarketContext,
    OptionsStrategyPlanner,
)


class TestOptionsStrategyPlanner(unittest.TestCase):
    def test_sideways_high_iv_iron_condor(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("SPY", 520.0, "sideways", iv_rank=65.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "iron_condor")
        self.assertIn("wing_delta", params)
        self.assertIn("wing_width", params)

    def test_bull_high_iv_bull_put(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("SPY", 520.0, "bull", iv_rank=70.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "bull_put")
        self.assertAlmostEqual(params["short_delta"], 0.25)
        self.assertEqual(params["width"], 10.0)

    def test_bull_low_iv_bull_call(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("SPY", 520.0, "bull", iv_rank=25.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "bull_call")
        self.assertAlmostEqual(params["long_delta"], 0.50)

    def test_bear_high_iv_bear_call(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("QQQ", 400.0, "bear", iv_rank=60.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "bear_call")

    def test_bear_low_iv_bear_put(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("QQQ", 400.0, "bear", iv_rank=28.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "bear_put")

    def test_config_custom_thresholds_and_width(self) -> None:
        p = OptionsStrategyPlanner(
            {
                "iv_high_threshold": 55.0,
                "iv_low_threshold": 25.0,
                "default_width_credit": 15.0,
                "default_width_debit": 8.0,
                "default_qty": 2,
                "iron_condor_wing_delta": 0.16,
            }
        )
        ctx = MarketContext("SPY", 520.0, "bull", iv_rank=56.0)
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "bull_put")
        self.assertEqual(params["width"], 15.0)
        self.assertEqual(params["qty"], 2)

        ctx2 = MarketContext("SPY", 520.0, "bull", iv_rank=24.0)
        st2, params2 = p.choose_strategy(ctx2)
        self.assertEqual(st2, "bull_call")
        self.assertEqual(params2["width"], 8.0)

        ctx3 = MarketContext("SPY", 520.0, "sideways", iv_rank=50.0)
        st3, params3 = p.choose_strategy(ctx3)
        self.assertEqual(st3, "iron_condor")
        self.assertAlmostEqual(params3["wing_delta"], 0.16)

    def test_covered_call_when_has_stock_sideways(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext(
            "SPY", 510.0, "sideways", iv_rank=50.0, regime="has_stock"
        )
        st, params = p.choose_strategy(ctx)
        self.assertEqual(st, "covered_call")
        self.assertEqual(params["stock_basis"], 510.0)

    def test_slightly_bull_like_bull(self) -> None:
        p = OptionsStrategyPlanner()
        ctx = MarketContext("SPY", 520.0, "slightly_bull", iv_rank=65.0)
        st, _ = p.choose_strategy(ctx)
        self.assertEqual(st, "bull_put")


class StubOptionsService:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, dict]] = []
        self.to_raise: BaseException | None = None
        self.return_id = "pos-stub-1"

    def build_and_open(
        self,
        symbol: str,
        strategy_type: str,
        params: dict | None = None,
    ) -> str:
        if self.to_raise is not None:
            raise self.to_raise
        self.calls.append((symbol, strategy_type, dict(params or {})))
        return self.return_id


class TestAtlasOptionsPlannerService(unittest.TestCase):
    def test_plan_and_open_delegates_to_build_and_open(self) -> None:
        stub = StubOptionsService()
        orch = AtlasOptionsPlannerService(stub)
        ctx = MarketContext("SPY", 520.0, "bear", iv_rank=20.0)
        pid = orch.plan_and_open(ctx)
        self.assertEqual(pid, "pos-stub-1")
        self.assertEqual(len(stub.calls), 1)
        sym, st, pr = stub.calls[0]
        self.assertEqual(sym, "SPY")
        self.assertEqual(st, "bear_put")
        self.assertIn("long_delta", pr)

    def test_plan_and_open_propagates_atlas_options_risk_error(self) -> None:
        stub = StubOptionsService()
        err = AtlasOptionsRiskError(
            "límite",
            code="max_open_positions",
            detail={"current_open": 99},
        )
        stub.to_raise = err
        orch = AtlasOptionsPlannerService(stub)
        ctx = MarketContext("SPY", 520.0, "bull", iv_rank=70.0)
        with self.assertRaises(AtlasOptionsRiskError) as cm:
            orch.plan_and_open(ctx)
        self.assertIs(cm.exception, err)
        self.assertEqual(cm.exception.code, "max_open_positions")


if __name__ == "__main__":
    unittest.main()
