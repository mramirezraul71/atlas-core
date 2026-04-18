"""
Tests de OptionsMetrics (daily_pnl, distribuciones, heatmap horario UTC).
"""
from __future__ import annotations

import sys
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

from atlas.core.options_metrics import OptionsMetrics  # noqa: E402
from atlas_options_brain.simulator.paper import Position  # noqa: E402


def _open_pos(pid: str) -> Position:
    return Position(
        entry_legs=tuple(),
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=-10.0,
        closed_at=None,
        realized_pnl=None,
        position_id=pid,
    )


def _closed_pos(pid: str, *, closed_at: datetime, realized: float) -> Position:
    return Position(
        entry_legs=tuple(),
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=-10.0,
        closed_at=closed_at,
        realized_pnl=realized,
        position_id=pid,
    )


class StubSvc:
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


class TestOptionsMetrics(unittest.TestCase):
    def test_daily_pnl_sums_realized_by_close_date(self) -> None:
        stub = StubSvc()
        stub.closed_list = [
            _closed_pos(
                "c1",
                closed_at=datetime(2026, 4, 10, 10, 0, tzinfo=timezone.utc),
                realized=10.0,
            ),
            _closed_pos(
                "c2",
                closed_at=datetime(2026, 4, 10, 18, 0, tzinfo=timezone.utc),
                realized=5.0,
            ),
        ]
        stub._positions["c1"] = {
            "position_id": "c1",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": False,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-09T00:00:00+00:00",
            "closed_at": "2026-04-10T10:00:00+00:00",
            "realized_pnl": 10.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        stub._positions["c2"] = {
            "position_id": "c2",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": False,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-09T00:00:00+00:00",
            "closed_at": "2026-04-10T18:00:00+00:00",
            "realized_pnl": 5.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        m = OptionsMetrics(stub)  # type: ignore[arg-type]
        agg = m.compute_aggregates()
        by_date = {r["date"]: r for r in agg["daily_pnl"]}
        self.assertEqual(by_date["2026-04-10"]["realized_pnl"], 15.0)
        self.assertEqual(by_date["2026-04-10"]["unrealized_pnl"], 0.0)

    def test_daily_pnl_includes_today_unrealized(self) -> None:
        today = datetime.now(timezone.utc).date().isoformat()
        stub = StubSvc()
        stub.open_list = [_open_pos("o1")]
        stub._positions["o1"] = {
            "position_id": "o1",
            "symbol": "QQQ",
            "strategy_type": "iron_condor",
            "is_open": True,
            "entry_net_premium": 2.0,
            "opened_at": "2026-04-17T09:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 1,
            "last_snapshot": {
                "timestamp": "2026-04-17T11:00:00+00:00",
                "unrealized_pnl": 7.5,
                "entry_net_premium": 2.0,
                "leg_mids": [],
            },
        }
        agg = OptionsMetrics(stub).compute_aggregates()  # type: ignore[arg-type]
        by_date = {r["date"]: r for r in agg["daily_pnl"]}
        self.assertEqual(by_date[today]["unrealized_pnl"], 7.5)

    def test_strategy_distribution_weights(self) -> None:
        stub = StubSvc()
        stub.open_list = [_open_pos("a"), _open_pos("b")]
        stub._positions["a"] = {
            "position_id": "a",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": True,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-17T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        stub._positions["b"] = {
            "position_id": "b",
            "symbol": "SPY",
            "strategy_type": "iron_condor",
            "is_open": True,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-17T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        dist = OptionsMetrics(stub).compute_aggregates()["strategy_distribution"]  # type: ignore[arg-type]
        self.assertEqual(len(dist), 2)
        by_st = {d["strategy_type"]: d for d in dist}
        self.assertEqual(by_st["bull_put"]["open_count"], 1)
        self.assertEqual(by_st["iron_condor"]["open_count"], 1)
        # Sin patas, notional 0 → pesos 0 (total 0)
        self.assertEqual(sum(d["weight_notional"] for d in dist), 0.0)

    def test_symbol_exposure_weights(self) -> None:
        stub = StubSvc()
        stub.open_list = [_open_pos("x"), _open_pos("y")]
        stub._positions["x"] = {
            "position_id": "x",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": True,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-17T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        stub._positions["y"] = {
            "position_id": "y",
            "symbol": "QQQ",
            "strategy_type": "bull_put",
            "is_open": True,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-17T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        exp = OptionsMetrics(stub).compute_aggregates()["symbol_exposure"]  # type: ignore[arg-type]
        self.assertEqual(len(exp), 2)
        self.assertEqual(sum(e["weight"] for e in exp), 0.0)

    def test_pnl_by_day_hour(self) -> None:
        stub = StubSvc()
        stub.closed_list = [
            _closed_pos(
                "c1",
                closed_at=datetime(2026, 4, 11, 14, 30, tzinfo=timezone.utc),
                realized=100.0,
            ),
        ]
        stub._positions["c1"] = {
            "position_id": "c1",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "is_open": False,
            "entry_net_premium": 1.0,
            "opened_at": "2026-04-10T00:00:00+00:00",
            "closed_at": "2026-04-11T14:30:00+00:00",
            "realized_pnl": 100.0,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        heat = OptionsMetrics(stub).compute_aggregates()["pnl_by_day_hour"]  # type: ignore[arg-type]
        self.assertEqual(len(heat), 1)
        self.assertEqual(heat[0]["date"], "2026-04-11")
        self.assertEqual(heat[0]["hour"], 14)
        self.assertEqual(heat[0]["realized_pnl"], 100.0)
        self.assertEqual(heat[0]["unrealized_pnl"], 0.0)


if __name__ == "__main__":
    unittest.main()
