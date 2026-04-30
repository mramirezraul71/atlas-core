"""Tests TradierOrderExecutor (dry_run, formulario, HTTP mockeado)."""
from __future__ import annotations

import json
import os
import sys
import unittest
from dataclasses import replace
from datetime import datetime, timezone
from unittest.mock import MagicMock, patch

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from atlas_options_brain.broker.tradier_executor import (  # noqa: E402
    TradierOrderExecutor,
    _build_form_body,
    _tradier_side,
)
from atlas_options_brain.broker.tradier_live import LiveOrder, LiveOrderLeg  # noqa: E402


def _sample_order() -> LiveOrder:
    now = datetime(2026, 4, 17, 12, 0, 0, tzinfo=timezone.utc)
    legs = [
        LiveOrderLeg(
            position_id="p1",
            leg_index=0,
            symbol="SPY",
            contract_symbol="SPY250620P00500000",
            side="buy",
            quantity=1,
            order_type="limit",
            limit_price=1.1,
            time_in_force="day",
            strategy_type="bull_put",
            tag="open",
        ),
        LiveOrderLeg(
            position_id="p1",
            leg_index=1,
            symbol="SPY",
            contract_symbol="SPY250620P00510000",
            side="sell",
            quantity=1,
            order_type="limit",
            limit_price=2.2,
            time_in_force="day",
            strategy_type="bull_put",
            tag="open",
        ),
    ]
    return LiveOrder(
        order_id="live-test",
        position_id="p1",
        symbol="SPY",
        legs=legs,
        status="pending",
        created_at=now,
        last_update=now,
        error=None,
    )


class TestTradierSideMapping(unittest.TestCase):
    def test_open_long_buy(self) -> None:
        leg = _sample_order().legs[0]
        self.assertEqual(_tradier_side(leg), "buy_to_open")

    def test_open_short_sell(self) -> None:
        leg = _sample_order().legs[1]
        self.assertEqual(_tradier_side(leg), "sell_to_open")

    def test_close_long_sell(self) -> None:
        lo = _sample_order()
        l0 = replace(lo.legs[0], side="sell", tag="close")
        self.assertEqual(_tradier_side(l0), "sell_to_close")

    def test_close_short_buy(self) -> None:
        lo = _sample_order()
        l1 = replace(lo.legs[1], side="buy", tag="close")
        self.assertEqual(_tradier_side(l1), "buy_to_close")


class TestTradierOrderExecutor(unittest.TestCase):
    def test_production_blocked_without_flag(self) -> None:
        with self.assertRaises(ValueError):
            TradierOrderExecutor(
                account_id="1",
                token="x",
                sandbox=False,
                allow_production=False,
            )

    def test_production_allowed_with_flag(self) -> None:
        ex = TradierOrderExecutor(
            account_id="1",
            token="x",
            sandbox=False,
            allow_production=True,
        )
        self.assertIn("api.tradier.com", ex.base_url)

    def test_dry_run_no_http(self) -> None:
        ex = TradierOrderExecutor(
            account_id="ACC",
            token="TOK",
            sandbox=True,
            dry_run=True,
        )
        out = ex.place_multileg_order(_sample_order(), preview=True, multileg_type="market")
        self.assertTrue(out["dry_run"])
        self.assertEqual(out["form"]["class"], "multileg")
        self.assertEqual(out["form"]["symbol"], "SPY")
        self.assertEqual(out["form"]["preview"], "true")
        self.assertEqual(out["form"]["side[0]"], "buy_to_open")
        self.assertEqual(out["form"]["side[1]"], "sell_to_open")

    @patch("atlas_options_brain.broker.tradier_executor.urllib.request.urlopen")
    def test_sandbox_post_parses_json(self, mock_urlopen: MagicMock) -> None:
        cm = MagicMock()
        cm.__enter__.return_value.read.return_value = json.dumps(
            {"order": {"id": "sandbox-1", "status": "ok"}}
        ).encode()
        cm.__enter__.return_value.status = 200
        mock_urlopen.return_value = cm

        ex = TradierOrderExecutor(
            account_id="ACC",
            token="TOK",
            sandbox=True,
            dry_run=False,
        )
        r = ex.place_multileg_order(_sample_order(), preview=True, multileg_type="market")
        self.assertTrue(r["ok"])
        self.assertEqual(r["http_status"], 200)
        self.assertTrue(r["sandbox"])
        mock_urlopen.assert_called_once()
        call_req = mock_urlopen.call_args[0][0]
        self.assertEqual(call_req.get_full_url(), f"{ex.base_url}/accounts/ACC/orders")

    def test_limit_adds_price(self) -> None:
        order = _sample_order()
        pairs = _build_form_body(
            order, preview=False, multileg_type="limit", duration="day"
        )
        d = dict(pairs)
        self.assertEqual(d["type"], "limit")
        self.assertIn("price", d)


if __name__ == "__main__":
    unittest.main()
