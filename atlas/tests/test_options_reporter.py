"""
Tests de OptionsReporter y volcado JSON.
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from datetime import date, datetime, timezone
from pathlib import Path
from types import SimpleNamespace

_REPO_ROOT = Path(__file__).resolve().parents[2]
_BRAIN_ROOT = _REPO_ROOT / "atlas_options_brain_fase1"
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))
if str(_BRAIN_ROOT) not in sys.path:
    sys.path.insert(0, str(_BRAIN_ROOT))

from atlas.core.options_client import AtlasOptionsService  # noqa: E402
from atlas.core.options_reporter import (  # noqa: E402
    OptionsReporter,
    options_state_to_json_file,
)
from atlas_options_brain.models.leg import Leg  # noqa: E402
from atlas_options_brain.models.option_contract import (  # noqa: E402
    OptionContract,
    OptionRight,
    OptionType,
    Greeks,
)
from atlas_options_brain.simulator.paper import Position  # noqa: E402
from atlas_options_brain.tests.test_atlas_adapter import FakeProvider  # noqa: E402


def _make_stub_position(
    pid: str,
    sym: str,
    *,
    put_strikes: tuple[float, float] | None = None,
    net_premium: float = -100.0,
    is_open: bool = True,
) -> Position:
    exp = date(2025, 6, 20)
    if put_strikes:
        lo, hi = put_strikes
        c_lo = OptionContract(
            symbol=sym,
            option_type=OptionType.PUT,
            strike=lo,
            expiration=exp,
            last_price=1.0,
            bid=0.9,
            ask=1.1,
            volume=1,
            open_interest=1,
            greeks=Greeks(delta=0.4),
        )
        c_hi = OptionContract(
            symbol=sym,
            option_type=OptionType.PUT,
            strike=hi,
            expiration=exp,
            last_price=1.5,
            bid=1.4,
            ask=1.6,
            volume=1,
            open_interest=1,
            greeks=Greeks(delta=0.35),
        )
        legs = (
            Leg(c_lo, OptionRight.LONG, 1),
            Leg(c_hi, OptionRight.SHORT, 1),
        )
    else:
        legs = tuple()
    return Position(
        entry_legs=legs,
        opened_at=datetime(2025, 5, 1, tzinfo=timezone.utc),
        entry_net_premium=net_premium,
        closed_at=None if is_open else datetime(2025, 5, 2, tzinfo=timezone.utc),
        realized_pnl=50.0 if not is_open else None,
        position_id=pid,
    )


class StubOptionsForReporter:
    """Sustituto mínimo de AtlasOptionsService para tests deterministas."""

    def __init__(self) -> None:
        self.p_open: list[Position] = []
        self.p_closed: list[Position] = []
        self._details: dict[str, dict] = {}

    def mark_all(self) -> list[dict]:
        return []

    def list_open_positions(self) -> list[Position]:
        return list(self.p_open)

    def list_closed_positions(self) -> list[Position]:
        return list(self.p_closed)

    def get_position(self, position_id: str) -> dict:
        return dict(self._details[position_id])

    @property
    def client(self) -> SimpleNamespace:
        return SimpleNamespace(provider=SimpleNamespace(get_quote=lambda s: 520.0))


class TestOptionsReporterStub(unittest.TestCase):
    def test_snapshot_structure_and_aggregates(self) -> None:
        stub = StubOptionsForReporter()
        p1 = _make_stub_position("pos-1", "SPY", put_strikes=(460.0, 470.0))
        p2 = _make_stub_position("pos-2", "SPY", put_strikes=(460.0, 470.0))
        stub.p_open = [p1, p2]
        stub._details["pos-1"] = {
            "position_id": "pos-1",
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
                "unrealized_pnl": 10.0,
                "entry_net_premium": 50.0,
                "leg_mids": [0.45, 0.95],
            },
        }
        stub._details["pos-2"] = {
            "position_id": "pos-2",
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
                "unrealized_pnl": -5.0,
                "entry_net_premium": 50.0,
                "leg_mids": [0.45, 0.95],
            },
        }
        rep = OptionsReporter(stub)  # type: ignore[arg-type]
        snap = rep.snapshot_state()
        self.assertIn("timestamp", snap)
        self.assertIn("portfolio", snap)
        self.assertIn("by_symbol", snap)
        self.assertIn("by_strategy_type", snap)
        self.assertIn("positions", snap)
        self.assertEqual(snap["portfolio"]["total_open_positions"], 2)
        self.assertEqual(snap["portfolio"]["total_net_premium"], 100.0)
        self.assertEqual(snap["portfolio"]["total_unrealized_pnl"], 5.0)
        # notional: (470-460)*100*1 = 1000 cada una
        self.assertEqual(snap["portfolio"]["total_notional_open"], 2000.0)
        self.assertEqual(len(snap["by_symbol"]), 1)
        self.assertEqual(snap["by_symbol"][0]["open_positions"], 2)
        self.assertEqual(snap["by_symbol"][0]["notional_open"], 2000.0)
        self.assertEqual(len(snap["by_strategy_type"]), 1)
        self.assertEqual(snap["by_strategy_type"][0]["count_open"], 2)
        self.assertEqual(len(snap["positions"]), 2)

    def test_options_state_to_json_file(self) -> None:
        stub = StubOptionsForReporter()
        p1 = _make_stub_position("pos-1", "QQQ", put_strikes=(400.0, 410.0))
        stub.p_open = [p1]
        stub._details["pos-1"] = {
            "position_id": "pos-1",
            "symbol": "QQQ",
            "strategy_type": "bull_put",
            "is_open": True,
            "entry_net_premium": 40.0,
            "opened_at": "2025-05-01T00:00:00+00:00",
            "closed_at": None,
            "realized_pnl": None,
            "snapshot_count": 0,
            "last_snapshot": None,
        }
        rep = OptionsReporter(stub)  # type: ignore[arg-type]
        with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".json") as tmp:
            path = tmp.name
        try:
            options_state_to_json_file(path, rep)
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
            self.assertEqual(data["portfolio"]["total_open_positions"], 1)
            self.assertIn("by_symbol", data)
        finally:
            Path(path).unlink(missing_ok=True)


class TestOptionsReporterIntegration(unittest.TestCase):
    def test_real_service_snapshot(self) -> None:
        as_of = date(2025, 5, 1)
        exp = date(2025, 6, 20)
        prov = FakeProvider([exp], spot=520.0)
        svc = AtlasOptionsService(
            "yfinance",
            {},
            provider=prov,
            as_of=as_of,
        )
        svc.build_and_open(
            "SPY",
            "bull_put",
            params={"short_delta": 0.25, "width": 10},
        )
        rep = OptionsReporter(svc)
        snap = rep.snapshot_state()
        self.assertGreaterEqual(snap["portfolio"]["total_open_positions"], 1)
        self.assertTrue(any(p["symbol"] == "SPY" for p in snap["positions"]))


if __name__ == "__main__":
    unittest.main()
