"""
Tests OptionsLiveExecutor y validaciones sandbox-only.
"""
from __future__ import annotations

import sys
import unittest
from datetime import date, datetime, timezone
from pathlib import Path
from unittest.mock import MagicMock

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_live import AtlasOptionsLiveService, OptionsLiveExecutor  # noqa: E402
from atlas_options_brain.broker.tradier_executor import TradierOrderExecutor  # noqa: E402
from atlas_options_brain.broker.tradier_live import TradierOrderBuilder  # noqa: E402
from atlas_options_brain.models.option_contract import (  # noqa: E402
    OptionContract,
    OptionRight,
    OptionType,
    Greeks,
)
from atlas_options_brain.models.leg import Leg  # noqa: E402
from atlas_options_brain.simulator.paper import Position  # noqa: E402


def _sample_position(pid: str = "p-open") -> Position:
    exp = date(2025, 6, 20)
    c1 = OptionContract(
        symbol="SPY",
        option_type=OptionType.PUT,
        strike=500.0,
        expiration=exp,
        last_price=1.0,
        bid=0.95,
        ask=1.05,
        volume=1,
        open_interest=1,
        greeks=Greeks(delta=0.3),
        contract_symbol="SPY250620P00500000",
    )
    c2 = OptionContract(
        symbol="SPY",
        option_type=OptionType.PUT,
        strike=510.0,
        expiration=exp,
        last_price=1.5,
        bid=1.45,
        ask=1.55,
        volume=1,
        open_interest=1,
        greeks=Greeks(delta=0.25),
        contract_symbol="SPY250620P00510000",
    )
    legs = (Leg(c1, OptionRight.LONG, 1), Leg(c2, OptionRight.SHORT, 1))
    return Position(
        entry_legs=legs,
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=50.0,
        position_id=pid,
    )


class TestOptionsLiveExecutor(unittest.TestCase):
    def test_off_blocks_send(self) -> None:
        ex = OptionsLiveExecutor(mode="off")
        order = TradierOrderBuilder().build_open_order(_sample_position())
        with self.assertRaises(RuntimeError):
            ex.send_order(order)

    def test_dry_run_simulated(self) -> None:
        ex = OptionsLiveExecutor(mode="dry_run")
        order = TradierOrderBuilder().build_open_order(_sample_position())
        out = ex.send_order(order, preview=True)
        self.assertEqual(out.status, "simulated")
        self.assertIsNotNone(ex.last_send_result)
        self.assertTrue(ex.last_send_result.get("simulated"))

    def test_sandbox_requires_executor(self) -> None:
        ex = OptionsLiveExecutor(mode="sandbox")
        order = TradierOrderBuilder().build_open_order(_sample_position())
        with self.assertRaises(RuntimeError):
            ex.send_order(order)

    def test_rejects_non_sandbox_executor(self) -> None:
        with self.assertRaises(ValueError):
            OptionsLiveExecutor(
                mode="sandbox",
                tradier_executor=TradierOrderExecutor(
                    account_id="a",
                    token="t",
                    sandbox=False,
                    allow_production=True,
                ),
            )

    def test_rejects_allow_production_even_if_sandbox_flag(self) -> None:
        te = TradierOrderExecutor(
            account_id="a",
            token="t",
            sandbox=True,
            allow_production=True,
        )
        with self.assertRaises(ValueError):
            OptionsLiveExecutor(mode="sandbox", tradier_executor=te)

    def test_sandbox_preview_calls_place(self) -> None:
        te = TradierOrderExecutor(
            account_id="a",
            token="t",
            sandbox=True,
            dry_run=True,
        )
        ex = OptionsLiveExecutor(mode="sandbox", tradier_executor=te)
        order = TradierOrderBuilder().build_open_order(_sample_position())
        out = ex.send_order(order, preview=True)
        self.assertEqual(out.status, "pending")
        self.assertTrue(ex.last_send_result.get("dry_run"))

    def test_sandbox_submit_sent_on_ok(self) -> None:
        te = MagicMock(spec=TradierOrderExecutor)
        te.sandbox = True
        te.allow_production = False
        te.place_multileg_order.return_value = {
            "ok": True,
            "http_status": 200,
            "preview": False,
            "sandbox": True,
            "body": {"order": {"id": "1"}},
        }
        ex = OptionsLiveExecutor(mode="sandbox", tradier_executor=te)
        order = TradierOrderBuilder().build_open_order(_sample_position())
        out = ex.send_order(order, preview=False)
        self.assertEqual(out.status, "sent")
        te.place_multileg_order.assert_called_once()
        call_kw = te.place_multileg_order.call_args
        self.assertEqual(call_kw.kwargs.get("preview"), False)

    def test_sandbox_submit_rejected_on_fail(self) -> None:
        te = MagicMock(spec=TradierOrderExecutor)
        te.sandbox = True
        te.allow_production = False
        te.place_multileg_order.return_value = {
            "ok": False,
            "http_status": 400,
            "error": "HTTPError",
            "body": "bad",
        }
        ex = OptionsLiveExecutor(mode="sandbox", tradier_executor=te)
        order = TradierOrderBuilder().build_open_order(_sample_position())
        out = ex.send_order(order, preview=False)
        self.assertEqual(out.status, "rejected")
        self.assertIsNotNone(out.error)


class StubOptionsForLive:
    def __init__(self, pos: Position) -> None:
        self._pos = pos

    def list_open_positions(self) -> list[Position]:
        if self._pos.is_open:
            return [self._pos]
        return []

    def list_closed_positions(self) -> list[Position]:
        if not self._pos.is_open:
            return [self._pos]
        return []

    @property
    def client(self):
        class C:
            @staticmethod
            def get_position_meta(pid: str) -> dict:
                return {"strategy_type": "bull_put", "symbol": "SPY"}

        return C()


class TestAtlasOptionsLiveService(unittest.TestCase):
    def test_send_open_resolves_position(self) -> None:
        pos = _sample_position("x1")
        te = TradierOrderExecutor(account_id="a", token="t", sandbox=True, dry_run=True)
        live_ex = OptionsLiveExecutor(mode="sandbox", tradier_executor=te)
        svc = AtlasOptionsLiveService(StubOptionsForLive(pos), live_ex)  # type: ignore[arg-type]
        r = svc.send_open_sandbox("x1", preview=True)
        self.assertEqual(r["status"], "ok")
        self.assertEqual(r["action"], "open")
        self.assertIn("tradier", r)

    def test_not_found(self) -> None:
        te = TradierOrderExecutor(account_id="a", token="t", sandbox=True, dry_run=True)
        live_ex = OptionsLiveExecutor(mode="sandbox", tradier_executor=te)
        pos = _sample_position("x1")
        svc = AtlasOptionsLiveService(StubOptionsForLive(pos), live_ex)  # type: ignore[arg-type]
        r = svc.send_open_sandbox("missing", preview=True)
        self.assertEqual(r["status"], "error")
        self.assertEqual(r["error"], "position_not_found")


if __name__ == "__main__":
    unittest.main()
