"""Tests del adapter Atlas ↔ Options Brain (proveedor fake, sin red)."""
from __future__ import annotations

import unittest
from dataclasses import replace
from datetime import date

from atlas_options_brain.dsl.strategy import IronCondor
from atlas_options_brain.integration.atlas_adapter import AtlasOptionsClient, default_expiration_selector
from atlas_options_brain.models.option_contract import OptionsChain
from atlas_options_brain.providers.base import OptionsDataProvider
from atlas_options_brain.tests.test_dsl_mock import make_chain


class FakeProvider(OptionsDataProvider):
    """Cadena sintética; ``bump_quotes`` desplaza bid/ask para simular mercado."""

    def __init__(
        self,
        expirations: list[date],
        *,
        spot: float = 520.0,
    ) -> None:
        self._exps = sorted(expirations)
        self._spot = spot
        self._bump = 0.0

    def bump_quotes(self, delta: float) -> None:
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


class TestAtlasAdapter(unittest.TestCase):
    def test_build_strategy_iron_condor_valid(self):
        as_of = date(2025, 5, 1)
        exp = date(2025, 6, 20)
        prov = FakeProvider([exp])
        client = AtlasOptionsClient(prov, as_of=as_of)
        strat = client.build_strategy_from_chain(
            "SPY",
            "iron_condor",
            params={"wing_delta": 0.20, "wing_width": 10.0},
        )
        self.assertIsInstance(strat, IronCondor)
        self.assertEqual(len(strat.legs), 4)
        self.assertGreater(strat.net_premium, 0)

    def test_open_paper_position_registers_in_simulator(self):
        as_of = date(2025, 5, 1)
        exp = date(2025, 6, 20)
        prov = FakeProvider([exp])
        client = AtlasOptionsClient(prov, as_of=as_of)
        strat = client.build_strategy_from_chain("SPY", "bull_put", params={})
        pos = client.open_paper_position(strat, atlas_strategy_type="bull_put")
        self.assertTrue(pos.is_open)
        self.assertEqual(len(client.list_open_positions()), 1)
        self.assertEqual(client.get_position_meta(pos.position_id)["strategy_type"], "bull_put")

    def test_mark_all_positions_updates_snapshots(self):
        as_of = date(2025, 5, 1)
        exp = date(2025, 6, 20)
        prov = FakeProvider([exp])
        client = AtlasOptionsClient(prov, as_of=as_of)
        strat = client.build_strategy_from_chain("SPY", "bull_put", params={})
        client.open_paper_position(strat, atlas_strategy_type="bull_put")
        self.assertEqual(client.list_open_positions()[0].snapshot_count, 0)
        snaps = client.mark_all_positions()
        self.assertEqual(len(snaps), 1)
        self.assertIsNotNone(snaps[0].match_info)
        self.assertEqual(client.list_open_positions()[0].snapshot_count, 1)
        prov.bump_quotes(0.05)
        client.mark_all_positions()
        pos = client.list_open_positions()[0]
        self.assertEqual(pos.snapshot_count, 2)
        self.assertIsNotNone(pos.last_snapshot)

    def test_default_expiration_selector_prefers_window(self):
        as_of = date(2025, 5, 1)
        prov = FakeProvider([date(2025, 5, 5), date(2025, 6, 15), date(2025, 12, 1)])
        e = default_expiration_selector("SPY", prov, as_of, min_dte=21, soft_max_dte=45)
        self.assertEqual(e, date(2025, 6, 15))


if __name__ == "__main__":
    unittest.main()
