"""
Tests de OptionsIntentRouter (intents JSON → planner/servicio).
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

from atlas.core.options_client import AtlasOptionsRiskError  # noqa: E402
from atlas.core.options_planner import MarketContext  # noqa: E402
from atlas.core.options_router import OptionsIntentRouter  # noqa: E402


class StubPlannerService:
    def __init__(self) -> None:
        self.contexts: list[MarketContext] = []
        self.return_id = "pos-from-planner"

    def plan_and_open(self, context: MarketContext) -> str:
        self.contexts.append(context)
        return self.return_id


class StubOptionsService:
    def __init__(self) -> None:
        self.build_calls: list[tuple[str, str, dict | None]] = []
        self.meta = {"strategy_type": "bull_put", "symbol": "SPY"}
        self.mark_rows = [{"id": "pos-1", "symbol": "SPY", "uPnL": 0.0}]
        self.positions: dict[str, dict] = {
            "pos-1": {"position_id": "pos-1", "is_open": True},
        }
        self.risk_on_build = False

    @property
    def client(self) -> SimpleNamespace:
        parent = self

        class _C:
            @staticmethod
            def get_position_meta(pid: str) -> dict:
                return dict(parent.meta)

        return SimpleNamespace(get_position_meta=_C.get_position_meta)

    def build_and_open(
        self,
        symbol: str,
        strategy_type: str,
        params: dict | None = None,
    ) -> str:
        if self.risk_on_build:
            raise AtlasOptionsRiskError(
                "límite",
                code="max_open_positions",
                detail={"limit": 1},
            )
        self.build_calls.append((symbol, strategy_type, params))
        return "pos-direct"

    def mark_all(self) -> list[dict]:
        return list(self.mark_rows)

    def get_position(self, position_id: str) -> dict:
        if position_id not in self.positions:
            raise KeyError(position_id)
        return self.positions[position_id]


class StubLiveService:
    """Sustituto mínimo de AtlasOptionsLiveService para intents sandbox."""

    def __init__(self) -> None:
        self.open_calls: list[tuple[str, bool]] = []
        self.close_calls: list[tuple[str, bool]] = []

    def send_open_sandbox(self, position_id: str, *, preview: bool = True) -> dict:
        self.open_calls.append((position_id, preview))
        return {
            "status": "ok",
            "position_id": position_id,
            "action": "open",
            "preview": preview,
            "order_id": "ord-1",
            "order_status": "pending",
        }

    def send_close_sandbox(self, position_id: str, *, preview: bool = True) -> dict:
        self.close_calls.append((position_id, preview))
        return {
            "status": "ok",
            "position_id": position_id,
            "action": "close",
            "preview": preview,
            "order_id": "ord-2",
            "order_status": "pending",
        }


class TestOptionsIntentRouter(unittest.TestCase):
    def setUp(self) -> None:
        self.stub_planner = StubPlannerService()
        self.stub_options = StubOptionsService()
        self.router = OptionsIntentRouter(
            self.stub_planner,  # type: ignore[arg-type]
            self.stub_options,  # type: ignore[arg-type]
        )

    def test_plan_and_open_calls_planner_with_context(self) -> None:
        payload = {
            "intent": "options_plan_and_open",
            "symbol": "SPY",
            "spot": 520.0,
            "trend": "bull",
            "iv_rank": 65.0,
            "regime": None,
        }
        out = self.router.handle_intent(payload)
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["position_id"], "pos-from-planner")
        self.assertEqual(len(self.stub_planner.contexts), 1)
        ctx = self.stub_planner.contexts[0]
        self.assertEqual(ctx.symbol, "SPY")
        self.assertEqual(ctx.spot, 520.0)
        self.assertEqual(ctx.trend, "bull")
        self.assertEqual(ctx.iv_rank, 65.0)
        self.assertIsNone(ctx.regime)

    def test_build_and_open_delegates(self) -> None:
        payload = {
            "intent": "options_build_and_open",
            "symbol": "SPY",
            "strategy_type": "bull_put",
            "params": {"short_delta": 0.25, "width": 10},
        }
        out = self.router.handle_intent(payload)
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["position_id"], "pos-direct")
        self.assertEqual(len(self.stub_options.build_calls), 1)
        sym, st, pr = self.stub_options.build_calls[0]
        self.assertEqual(sym, "SPY")
        self.assertEqual(st, "bull_put")
        self.assertEqual(pr, {"short_delta": 0.25, "width": 10})

    def test_mark_all_returns_positions(self) -> None:
        out = self.router.handle_intent({"intent": "options_mark_all"})
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["positions"], self.stub_options.mark_rows)

    def test_get_position_wrapped(self) -> None:
        out = self.router.handle_intent(
            {"intent": "options_get_position", "position_id": "pos-1"}
        )
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["position"]["position_id"], "pos-1")

    def test_get_position_not_found(self) -> None:
        out = self.router.handle_intent(
            {"intent": "options_get_position", "position_id": "missing"}
        )
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["error"], "position_not_found")

    def test_risk_error_returns_payload(self) -> None:
        self.stub_options.risk_on_build = True
        out = self.router.handle_intent(
            {
                "intent": "options_build_and_open",
                "symbol": "SPY",
                "strategy_type": "bull_put",
                "params": {},
            }
        )
        self.assertEqual(out["status"], "risk_error")
        self.assertEqual(out["code"], "max_open_positions")
        self.assertEqual(out["detail"]["limit"], 1)

    def test_unknown_intent(self) -> None:
        out = self.router.handle_intent({"intent": "options_foo"})
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["error"], "unknown_intent")

    def test_missing_field_plan(self) -> None:
        out = self.router.handle_intent(
            {"intent": "options_plan_and_open", "symbol": "SPY", "spot": 1.0}
        )
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["error"], "missing_field")
        self.assertEqual(out["field"], "trend")

    def test_send_sandbox_not_configured(self) -> None:
        out = self.router.handle_intent(
            {
                "intent": "options_send_sandbox_order",
                "position_id": "p1",
                "action": "open",
            }
        )
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["error"], "live_service_not_configured")

    def test_send_sandbox_open_delegates(self) -> None:
        stub_live = StubLiveService()
        router = OptionsIntentRouter(
            self.stub_planner,  # type: ignore[arg-type]
            self.stub_options,  # type: ignore[arg-type]
            live_service=stub_live,  # type: ignore[arg-type]
        )
        out = router.handle_intent(
            {
                "intent": "options_send_sandbox_order",
                "position_id": "pos-99",
                "action": "open",
                "preview": True,
            }
        )
        self.assertEqual(out["status"], "ok")
        self.assertEqual(out["action"], "open")
        self.assertEqual(stub_live.open_calls, [("pos-99", True)])

    def test_send_sandbox_invalid_action(self) -> None:
        stub_live = StubLiveService()
        router = OptionsIntentRouter(
            self.stub_planner,  # type: ignore[arg-type]
            self.stub_options,  # type: ignore[arg-type]
            live_service=stub_live,  # type: ignore[arg-type]
        )
        out = router.handle_intent(
            {
                "intent": "options_send_sandbox_order",
                "position_id": "p1",
                "action": "flip",
            }
        )
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["error"], "invalid_action")


if __name__ == "__main__":
    unittest.main()
