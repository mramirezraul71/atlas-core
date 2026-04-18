"""
Integración AtlasOptionsService ↔ atlas_options_brain (sin claves API).

Ejecutar desde la raíz ATLAS_PUSH con PYTHONPATH que incluya atlas_options_brain_fase1, p. ej.:

  set PYTHONPATH=c:\\ATLAS_PUSH;c:\\ATLAS_PUSH\\atlas_options_brain_fase1
  python -m unittest atlas.tests.test_options_client_integration -v
"""
from __future__ import annotations

import sys
import unittest
from datetime import date
from pathlib import Path

# Monorepo: paquete atlas_options_brain vive junto al core Atlas
_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_client import (  # noqa: E402
    AtlasOptionsRiskError,
    AtlasOptionsService,
)
from atlas_options_brain.models.option_contract import OptionsChain  # noqa: E402
from atlas_options_brain.providers.base import OptionsDataProvider  # noqa: E402
from atlas_options_brain.tests.test_dsl_mock import make_chain  # noqa: E402
from dataclasses import replace  # noqa: E402


class FakeProvider(OptionsDataProvider):
    def __init__(self, expirations: list[date], *, spot: float = 520.0) -> None:
        self._exps = sorted(expirations)
        self._spot = spot
        self._bump = 0.0

    def bump(self, delta: float) -> None:
        self._bump += delta

    def get_expirations(self, symbol: str) -> list[date]:
        return list(self._exps)

    def get_quote(self, symbol: str) -> float:
        return self._spot

    def get_chain(
        self,
        symbol: str,
        expiration: date,
        option_type: str | None = None,
    ) -> OptionsChain:
        base = make_chain(symbol=symbol, spot=self._spot, exp=expiration)
        if self._bump == 0.0:
            return base
        contracts = [
            replace(
                c,
                bid=round(c.bid + self._bump, 2),
                ask=round(c.ask + self._bump, 2),
                last_price=round(c.last_price + self._bump, 4),
            )
            for c in base.contracts
        ]
        return OptionsChain(
            symbol=base.symbol,
            expiration=base.expiration,
            spot_price=base.spot_price,
            contracts=contracts,
        )


class TestAtlasOptionsServiceIntegration(unittest.TestCase):
    def setUp(self) -> None:
        self.as_of = date(2025, 5, 1)
        self.exp = date(2025, 6, 20)
        self.fake = FakeProvider([self.exp], spot=520.0)
        self.svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=self.fake,
            as_of=self.as_of,
        )

    def test_build_and_open_returns_position_id(self) -> None:
        pid = self.svc.build_and_open(
            "SPY",
            "bull_put",
            params={"short_delta": 0.25, "width": 10},
        )
        self.assertTrue(pid.startswith("pos-"))
        self.assertEqual(self.svc.client.get_position(pid).position_id, pid)

    def test_mark_all_rows_have_required_keys(self) -> None:
        self.svc.build_and_open("SPY", "iron_condor", params={"wing_delta": 0.2, "wing_width": 10})
        rows = self.svc.mark_all()
        self.assertEqual(len(rows), 1)
        row = rows[0]
        for key in ("id", "symbol", "strategy_type", "net_premium", "uPnL"):
            self.assertIn(key, row)
        self.assertEqual(row["symbol"], "SPY")
        self.assertEqual(row["strategy_type"], "iron_condor")

    def test_get_position_includes_last_snapshot_after_mark(self) -> None:
        pid = self.svc.build_and_open("SPY", "bull_put", params={"short_delta": 0.25, "width": 10})
        self.svc.mark_all()
        d = self.svc.get_position(pid)
        self.assertEqual(d["position_id"], pid)
        self.assertTrue(d["is_open"])
        self.assertGreaterEqual(d["snapshot_count"], 1)
        self.assertIsNotNone(d["last_snapshot"])
        assert d["last_snapshot"] is not None
        self.assertIn("unrealized_pnl", d["last_snapshot"])
        self.assertIn("timestamp", d["last_snapshot"])

    def test_invalid_strategy_type(self) -> None:
        with self.assertRaises(ValueError):
            self.svc.build_and_open("SPY", "iron_butterfly", params={})  # type: ignore[arg-type]

    def test_invalid_param_key(self) -> None:
        with self.assertRaises(ValueError):
            self.svc.build_and_open(
                "SPY",
                "bull_put",
                params={"short_delta": 0.25, "width": 10, "bogus": 1},
            )


class TestAtlasOptionsServiceRiskLimits(unittest.TestCase):
    def setUp(self) -> None:
        self.as_of = date(2025, 5, 1)
        self.exp = date(2025, 6, 20)
        self.fake = FakeProvider([self.exp], spot=520.0)

    def test_risk_limits_max_open_positions(self) -> None:
        svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=self.fake,
            as_of=self.as_of,
            risk_limits={"max_open_positions": 1},
        )
        svc.build_and_open(
            "SPY",
            "bull_put",
            params={"short_delta": 0.25, "width": 10},
        )
        with self.assertRaises(AtlasOptionsRiskError) as ctx:
            svc.build_and_open(
                "SPY",
                "bear_call",
                params={"short_delta": 0.25, "width": 10},
            )
        self.assertEqual(ctx.exception.code, "max_open_positions")

    def test_risk_limits_max_notional_per_symbol(self) -> None:
        svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=self.fake,
            as_of=self.as_of,
            risk_limits={"max_notional_per_symbol": 500.0},
        )
        with self.assertRaises(AtlasOptionsRiskError) as ctx:
            svc.build_and_open(
                "SPY",
                "bull_put",
                params={"short_delta": 0.25, "width": 10},
            )
        self.assertEqual(ctx.exception.code, "max_notional_per_symbol")
        self.assertGreater(ctx.exception.detail.get("proposed", 0), 500.0)


class TestAtlasOptionsServiceLiveStubs(unittest.TestCase):
    def setUp(self) -> None:
        self.as_of = date(2025, 5, 1)
        self.exp = date(2025, 6, 20)
        self.fake = FakeProvider([self.exp], spot=520.0)

    def test_place_live_order_rejects_when_live_mode_off(self) -> None:
        svc = AtlasOptionsService(
            "yfinance", {}, provider=self.fake, as_of=self.as_of, live_mode="off"
        )
        with self.assertRaises(RuntimeError) as ctx:
            svc.place_live_order("pos-0001")
        self.assertIn("live_mode", str(ctx.exception).lower())

    def test_sync_live_positions_rejects_when_not_live_mode(self) -> None:
        svc = AtlasOptionsService(
            "yfinance", {}, provider=self.fake, as_of=self.as_of, live_mode="off"
        )
        with self.assertRaises(RuntimeError) as ctx:
            svc.sync_live_positions()
        self.assertIn("live_mode", str(ctx.exception).lower())

    def test_place_live_order_not_implemented_when_live_armed(self) -> None:
        svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=self.fake,
            as_of=self.as_of,
            live_mode="live",
            live_enabled=True,
        )
        with self.assertRaises(NotImplementedError):
            svc.place_live_order("pos-0001")

    def test_sync_live_not_implemented_when_live_armed(self) -> None:
        svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=self.fake,
            as_of=self.as_of,
            live_mode="live",
            live_enabled=True,
        )
        with self.assertRaises(NotImplementedError):
            svc.sync_live_positions()


if __name__ == "__main__":
    unittest.main()
