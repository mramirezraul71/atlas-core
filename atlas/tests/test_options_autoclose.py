"""
Tests OptionsAutoCloseEngine / OptionsAutoCloser.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_autoclose import (  # noqa: E402
    AutoCloseConfig,
    AutoCloseRule,
    OptionsAutoCloseEngine,
    OptionsAutoCloser,
)


class StubOptionsAutoClose:
    def __init__(self) -> None:
        self.open_ids = ["bp1", "ic1", "ok1"]
        self._positions: dict[str, dict] = {}
        self.mark_calls = 0

    def mark_all(self) -> list:
        self.mark_calls += 1
        return []

    def list_open_positions(self) -> list[SimpleNamespace]:
        return [SimpleNamespace(position_id=pid) for pid in self.open_ids]

    def get_position(self, position_id: str) -> dict:
        return dict(self._positions[position_id])


class StubLiveService:
    def __init__(self) -> None:
        self.closes: list[tuple[str, bool]] = []

    def send_close_sandbox(self, position_id: str, *, preview: bool = True) -> dict:
        self.closes.append((position_id, preview))
        return {
            "status": "ok",
            "position_id": position_id,
            "order_status": "pending",
            "preview": preview,
        }


class TestOptionsAutoCloseEngine(unittest.TestCase):
    def setUp(self) -> None:
        self.stub = StubOptionsAutoClose()
        self.stub._positions = {
            "bp1": {
                "position_id": "bp1",
                "strategy_type": "bull_put",
                "entry_net_premium": 100.0,
                "is_open": True,
                "last_snapshot": {"unrealized_pnl": 70.0},
            },
            "ic1": {
                "position_id": "ic1",
                "strategy_type": "iron_condor",
                "entry_net_premium": 80.0,
                "is_open": True,
                "last_snapshot": {"unrealized_pnl": -170.0},
            },
            "ok1": {
                "position_id": "ok1",
                "strategy_type": "iron_condor",
                "entry_net_premium": 50.0,
                "is_open": True,
                "last_snapshot": {"unrealized_pnl": -20.0},
            },
        }
        self.config = AutoCloseConfig(
            rules=[
                AutoCloseRule(
                    strategy_type="bull_put",
                    take_profit_pct=0.6,
                    stop_loss_pct=-1.5,
                ),
                AutoCloseRule(
                    strategy_type="iron_condor",
                    take_profit_pct=0.9,
                    stop_loss_pct=-2.0,
                ),
            ],
            default_rule=AutoCloseRule(
                strategy_type="*",
                stop_loss_pct=-3.0,
            ),
        )
        self.engine = OptionsAutoCloseEngine(self.stub, self.config)  # type: ignore[arg-type]

    def test_take_profit_bull_put(self) -> None:
        ev = self.engine.evaluate_position(self.stub._positions["bp1"])
        self.assertTrue(ev["should_close"])
        self.assertIn("take_profit", ev["reasons"])
        self.assertEqual(ev["pnl"], 70.0)

    def test_stop_loss_iron_condor(self) -> None:
        ev = self.engine.evaluate_position(self.stub._positions["ic1"])
        self.assertTrue(ev["should_close"])
        self.assertIn("stop_loss", ev["reasons"])

    def test_no_close_moderate(self) -> None:
        ev = self.engine.evaluate_position(self.stub._positions["ok1"])
        self.assertFalse(ev["should_close"])
        self.assertEqual(ev["reasons"], [])

    def test_scan_open_positions(self) -> None:
        rows = self.engine.scan_open_positions()
        self.assertEqual(self.stub.mark_calls, 1)
        self.assertEqual(len(rows), 3)
        ids = {r["position_id"] for r in rows if r["should_close"]}
        self.assertEqual(ids, {"bp1", "ic1"})

    def test_min_max_pnl_abs(self) -> None:
        cfg = AutoCloseConfig(
            rules=[
                AutoCloseRule(
                    strategy_type="bull_call",
                    min_pnl_abs=25.0,
                    max_pnl_abs=40.0,
                ),
            ],
        )
        eng = OptionsAutoCloseEngine(self.stub, cfg)  # type: ignore[arg-type]
        hit_tp = eng.evaluate_position(
            {
                "position_id": "x",
                "strategy_type": "bull_call",
                "entry_net_premium": -200.0,
                "last_snapshot": {"unrealized_pnl": 30.0},
            }
        )
        self.assertTrue(hit_tp["should_close"])
        self.assertIn("min_pnl_abs", hit_tp["reasons"])
        hit_sl = eng.evaluate_position(
            {
                "position_id": "y",
                "strategy_type": "bull_call",
                "entry_net_premium": -200.0,
                "last_snapshot": {"unrealized_pnl": -50.0},
            }
        )
        self.assertTrue(hit_sl["should_close"])
        self.assertIn("max_pnl_abs", hit_sl["reasons"])

    def test_no_rule(self) -> None:
        eng = OptionsAutoCloseEngine(
            self.stub,
            AutoCloseConfig(rules=[], default_rule=None),
        )  # type: ignore[arg-type]
        ev = eng.evaluate_position(self.stub._positions["bp1"])
        self.assertFalse(ev["should_close"])
        self.assertIsNone(ev["rule_applied"])


class TestOptionsAutoCloser(unittest.TestCase):
    def test_close_candidates_calls_live(self) -> None:
        stub = StubOptionsAutoClose()
        stub._positions = {
            "bp1": {
                "position_id": "bp1",
                "strategy_type": "bull_put",
                "entry_net_premium": 100.0,
                "last_snapshot": {"unrealized_pnl": 80.0},
            },
        }
        stub.open_ids = ["bp1"]
        cfg = AutoCloseConfig(
            rules=[
                AutoCloseRule(strategy_type="bull_put", take_profit_pct=0.5),
            ],
        )
        engine = OptionsAutoCloseEngine(stub, cfg)  # type: ignore[arg-type]
        live = StubLiveService()
        closer = OptionsAutoCloser(stub, engine, live)  # type: ignore[arg-type]
        out = closer.close_candidates(preview=True)
        self.assertEqual(len(out), 1)
        self.assertEqual(live.closes, [("bp1", True)])
        self.assertEqual(out[0]["auto_close_reasons"], ["take_profit"])
        self.assertEqual(out[0]["status"], "ok")


if __name__ == "__main__":
    unittest.main()
