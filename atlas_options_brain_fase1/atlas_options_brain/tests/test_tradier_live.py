"""Tests Tradier live order skeleton (sin red)."""
from __future__ import annotations

import os
import sys
import unittest
from datetime import date, datetime, timezone

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from atlas_options_brain.broker.tradier_live import (
    TradierLiveExecutionSink,
    TradierOrderBuilder,
    _ensure_contract_symbol,
)
from atlas_options_brain.dsl.strategy import BullPutSpread
from atlas_options_brain.models.option_contract import (
    OptionContract,
    OptionType,
    Greeks,
)
from atlas_options_brain.simulator.paper import Position
from atlas_options_brain.tests.test_dsl_mock import make_chain


class TestTradierOrderBuilder(unittest.TestCase):
    def setUp(self) -> None:
        self._chain = make_chain(symbol="SPY", spot=520.0, exp=date(2025, 6, 20))
        self._bps = BullPutSpread.from_chain(self._chain, short_delta=0.25, width=10)
        self._position = Position(
            entry_legs=tuple(self._bps.legs),
            opened_at=datetime(2025, 5, 1, 12, 0, tzinfo=timezone.utc),
            entry_net_premium=self._bps.net_premium,
            position_id="paper-pos-99",
        )

    def test_build_open_maps_long_buy_short_sell(self) -> None:
        b = TradierOrderBuilder(price_mode="mid")
        lo = b.build_open_order(self._position, strategy_type="bull_put")
        self.assertEqual(lo.position_id, "paper-pos-99")
        self.assertEqual(lo.symbol, "SPY")
        self.assertEqual(lo.status, "pending")
        self.assertEqual(len(lo.legs), 2)
        self.assertEqual(lo.legs[0].tag, "open")
        self.assertEqual(lo.legs[0].side, "buy")
        self.assertEqual(lo.legs[1].side, "sell")
        self.assertEqual(lo.legs[0].strategy_type, "bull_put")
        self.assertEqual(lo.legs[0].order_type, "limit")
        self.assertIsNotNone(lo.legs[0].limit_price)

    def test_build_close_inverts_sides(self) -> None:
        b = TradierOrderBuilder()
        lo = b.build_close_order(self._position, strategy_type="bull_put")
        self.assertEqual(lo.legs[0].tag, "close")
        self.assertEqual(lo.legs[0].side, "sell")
        self.assertEqual(lo.legs[1].side, "buy")

    def test_bid_ask_mode_uses_ask_for_buy(self) -> None:
        b = TradierOrderBuilder(price_mode="bid_ask")
        lo = b.build_open_order(self._position)
        leg0 = lo.legs[0]
        c0 = self._position.entry_legs[0].contract
        if leg0.side == "buy":
            self.assertEqual(leg0.limit_price, round(float(c0.ask), 4))

    def test_missing_position_id_raises(self) -> None:
        bad = Position(
            entry_legs=self._position.entry_legs,
            opened_at=self._position.opened_at,
            entry_net_premium=self._position.entry_net_premium,
            position_id="",
        )
        with self.assertRaises(ValueError):
            TradierOrderBuilder().build_open_order(bad)

    def test_synthetic_contract_symbol_when_empty(self) -> None:
        c = OptionContract(
            symbol="SPY",
            option_type=OptionType.CALL,
            strike=520.0,
            expiration=date(2025, 6, 20),
            last_price=5.0,
            bid=4.9,
            ask=5.1,
            volume=1,
            open_interest=1,
            greeks=Greeks(),
            contract_symbol="",
        )
        sym = _ensure_contract_symbol(c)
        self.assertTrue(sym.startswith("SPY"))
        self.assertIn("C", sym)
        self.assertIn("00520000", sym)


class TestTradierLiveExecutionSink(unittest.TestCase):
    def test_submit_records_simulated(self) -> None:
        chain = make_chain()
        bps = BullPutSpread.from_chain(chain, short_delta=0.25, width=10)
        pos = Position(
            entry_legs=tuple(bps.legs),
            opened_at=datetime.now(timezone.utc),
            entry_net_premium=bps.net_premium,
            position_id="p1",
        )
        order = TradierOrderBuilder().build_open_order(pos)
        sink = TradierLiveExecutionSink()
        out = sink.submit(order)
        self.assertEqual(out.status, "simulated")
        self.assertEqual(len(sink.recorded), 1)
        self.assertEqual(sink.recorded[0].order_id, order.order_id)


if __name__ == "__main__":
    unittest.main()
