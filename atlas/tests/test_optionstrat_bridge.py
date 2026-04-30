"""
Tests de OptionStratBridge (live JSON + histórico deduplicado).
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.optionstrat_bridge import OptionStratBridge  # noqa: E402
from atlas_options_brain.simulator.paper import Position  # noqa: E402


def _pos(pid: str, premium: float = -10.0) -> Position:
    return Position(
        entry_legs=tuple(),
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=premium,
        closed_at=None,
        realized_pnl=None,
        position_id=pid,
    )


def _closed_pos(pid: str, premium: float = -10.0) -> Position:
    return Position(
        entry_legs=tuple(),
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=premium,
        closed_at=datetime(2025, 5, 2, tzinfo=timezone.utc),
        realized_pnl=25.5,
        position_id=pid,
    )


class StubOptionsService:
    def __init__(self) -> None:
        self.open_list: list[Position] = []
        self.closed_list: list[Position] = []
        self._positions: dict[str, dict] = {}

    def mark_all(self) -> list:
        return []

    def list_open_positions(self) -> list[Position]:
        return list(self.open_list)

    def list_closed_positions(self) -> list[Position]:
        return list(self.closed_list)

    def get_position(self, position_id: str) -> dict:
        return dict(self._positions[position_id])

    @property
    def client(self) -> SimpleNamespace:
        return SimpleNamespace(provider=SimpleNamespace(get_quote=lambda s: 100.0))


class TestOptionStratBridge(unittest.TestCase):
    def test_sync_writes_live_payload_shape(self) -> None:
        stub = StubOptionsService()
        stub.open_list = [_pos("o1")]
        stub._positions["o1"] = {
            "position_id": "o1",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": True,
            "entry_net_premium": 50.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 1,
            "last_snapshot": {
                "timestamp": "2025-05-01T12:00:00+00:00",
                "unrealized_pnl": 1.25,
                "entry_net_premium": 50.0,
                "leg_mids": [0.4, 0.9],
                "match_info": [{"leg_index": 0, "match": "exact"}],
            },
        }
        with tempfile.TemporaryDirectory() as td:
            live_p = Path(td) / "live.json"
            hist_p = Path(td) / "hist.json"
            br = OptionStratBridge(
                stub,  # type: ignore[arg-type]
                live_output_path=str(live_p),
                history_output_path=str(hist_p),
            )
            summary = br.sync_from_options()
            self.assertEqual(summary["open_count"], 1)
            self.assertEqual(summary["closed_count"], 0)
            with open(live_p, encoding="utf-8") as f:
                doc = json.load(f)
            self.assertIn("updated_at", doc)
            self.assertEqual(doc["open_count"], 1)
            self.assertEqual(len(doc["positions"]), 1)
            self.assertIn("metrics", doc)
            self.assertIn("daily_pnl", doc["metrics"])
            self.assertIn("strategy_distribution", doc["metrics"])
            self.assertIn("symbol_exposure", doc["metrics"])
            self.assertIn("pnl_by_day_hour", doc["metrics"])
            row = doc["positions"][0]
            self.assertEqual(row["id"], "o1")
            self.assertEqual(row["position_id"], "o1")
            self.assertEqual(row["symbol"], "SPY")
            self.assertEqual(row["strategy_type"], "bull_put")
            self.assertEqual(row["entry_net_premium"], 50.0)
            self.assertEqual(row["unrealized_pnl"], 1.25)
            self.assertIn("match_info", row)

    def test_history_appends_once_per_position_id(self) -> None:
        stub = StubOptionsService()
        stub.closed_list = [_closed_pos("c1")]
        stub._positions["c1"] = {
            "position_id": "c1",
            "symbol": "QQQ",
            "strategy_type": "iron_condor",
            "is_open": False,
            "entry_net_premium": 80.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": "2025-05-02T00:00:00+00:00",
            "realized_pnl": 12.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        with tempfile.TemporaryDirectory() as td:
            live_p = Path(td) / "live.json"
            hist_p = Path(td) / "hist.json"
            br = OptionStratBridge(
                stub,  # type: ignore[arg-type]
                live_output_path=str(live_p),
                history_output_path=str(hist_p),
            )
            br.sync_from_options()
            br.sync_from_options()
            with open(hist_p, encoding="utf-8") as f:
                hist = json.load(f)
            self.assertEqual(len(hist), 1)
            self.assertEqual(hist[0]["position_id"], "c1")
            self.assertEqual(hist[0]["realized_pnl"], 12.0)

    def test_history_no_change_when_no_new_closed(self) -> None:
        stub = StubOptionsService()
        stub.closed_list = [_closed_pos("c1")]
        stub._positions["c1"] = {
            "position_id": "c1",
            "symbol": "QQQ",
            "strategy_type": "bull_put",
            "is_open": False,
            "entry_net_premium": 1.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": "2025-05-02T00:00:00+00:00",
            "realized_pnl": 2.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        with tempfile.TemporaryDirectory() as td:
            live_p = Path(td) / "live.json"
            hist_p = Path(td) / "hist.json"
            br = OptionStratBridge(
                stub,  # type: ignore[arg-type]
                live_output_path=str(live_p),
                history_output_path=str(hist_p),
            )
            br.sync_from_options()
            first = hist_p.read_text(encoding="utf-8")
            br.sync_from_options()
            second = hist_p.read_text(encoding="utf-8")
            self.assertEqual(first, second)

    def test_summary_counts(self) -> None:
        stub = StubOptionsService()
        stub.open_list = [_pos("a"), _pos("b")]
        stub.closed_list = [_closed_pos("c")]
        for pid in ("a", "b"):
            stub._positions[pid] = {
                "position_id": pid,
                "symbol": "SPY",
                "strategy_type": "bull_put",
                "is_open": True,
                "entry_net_premium": 1.0,
                "opened_at": "2025-05-01T00:00:00+00:00",
                "closed_at": None,
                "realized_pnl": None,
                "snapshot_count": 0,
                "last_snapshot": None,
            }
        stub._positions["c"] = {
            "position_id": "c",
            "symbol": "SPY",
            "strategy_type": "bear_call",
            "is_open": False,
            "entry_net_premium": 2.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": "2025-05-03T00:00:00+00:00",
            "realized_pnl": 3.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        with tempfile.TemporaryDirectory() as td:
            br = OptionStratBridge(
                stub,  # type: ignore[arg-type]
                live_output_path=str(Path(td) / "l.json"),
                history_output_path=str(Path(td) / "h.json"),
            )
            s = br.sync_from_options()
            self.assertEqual(s["open_count"], 2)
            self.assertEqual(s["closed_count"], 1)

    def test_live_empty_when_all_closed(self) -> None:
        stub = StubOptionsService()
        stub.closed_list = [_closed_pos("x")]
        stub._positions["x"] = {
            "position_id": "x",
            "symbol": "IWM",
            "strategy_type": "bull_put",
            "is_open": False,
            "entry_net_premium": 1.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": "2025-05-02T00:00:00+00:00",
            "realized_pnl": 0.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        with tempfile.TemporaryDirectory() as td:
            live_p = Path(td) / "live.json"
            br = OptionStratBridge(
                stub,  # type: ignore[arg-type]
                live_output_path=str(live_p),
                history_output_path=str(Path(td) / "h.json"),
            )
            br.sync_from_options()
            with open(live_p, encoding="utf-8") as f:
                doc = json.load(f)
            self.assertEqual(doc["open_count"], 0)
            self.assertEqual(doc["positions"], [])


if __name__ == "__main__":
    unittest.main()
