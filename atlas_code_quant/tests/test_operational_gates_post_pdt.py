from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from operations.operation_center import OperationCenter


class _Tracker:
    def build_summary(self, **_: object) -> dict:
        return {
            "account_session": {"scope": "paper", "account_id": "paper-test"},
            "balances": {"total_equity": 100000.0, "cash": 50000.0, "option_buying_power": 0.0},
            "alerts": [],
            "totals": {"positions": 0},
            "strategies": [],
            "pdt_status": {},
            "refresh_interval_sec": 5,
        }


class _Journal:
    def __init__(self, items: list[dict] | None = None) -> None:
        self.items = list(items or [])

    def snapshot(self, limit: int = 400) -> dict:
        return {"count": len(self.items[:limit]), "items": list(self.items[:limit])}

    def attribution_integrity_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {"enabled": True, "account_type": account_type, "summary": {}, "alerts": [], "flagged_entries": [], "limit": limit}

    def position_management_snapshot(self, account_type: str | None = None, limit: int = 12) -> dict:
        return {"enabled": True, "account_type": account_type, "summary": {}, "alerts": [], "watchlist": [], "limit": limit}

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {"enabled": True, "account_type": account_type, "summary": {}, "alerts": [], "recommendations": [], "limit": limit}

    def post_trade_learning_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {"enabled": True, "account_type": account_type, "summary": {}, "root_cause_breakdown": [], "strategy_learning": [], "policy_candidates": [], "limit": limit}

    def options_governance_adoption_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {"enabled": True, "account_type": account_type, "summary": {}, "alerts": [], "strategy_mix": [], "limit": limit}


class _Vision:
    def status(self) -> dict:
        return {
            "provider": "desktop_capture",
            "provider_ready": True,
            "operator_present": True,
            "screen_integrity_ok": True,
        }


class _Executor:
    def status(self) -> dict:
        return {"mode": "paper_api", "kill_switch_active": False}


class _Brain:
    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}


class _Learning:
    def status(self, account_scope: str | None = None) -> dict:
        return {"account_scope": account_scope, "policy_ready": True}


def _center(tmp_path: Path, items: list[dict] | None = None) -> OperationCenter:
    return OperationCenter(
        tracker=_Tracker(),  # type: ignore[arg-type]
        journal=_Journal(items=items),  # type: ignore[arg-type]
        vision=_Vision(),  # type: ignore[arg-type]
        executor=_Executor(),  # type: ignore[arg-type]
        brain=_Brain(),  # type: ignore[arg-type]
        learning=_Learning(),  # type: ignore[arg-type]
        state_path=tmp_path / "operation_center_state.json",
    )


@pytest.mark.unit
class TestOperationalGatesPostPdt:
    def test_pdt_limit_deprecated_always_ok(self, tmp_path: Path) -> None:
        center = _center(tmp_path)
        result, reason = center.check_pdt_limit("ACC-1")
        assert result is True
        assert reason == "pdt_limit_removed"

    def test_capital_minimum_removed(self, tmp_path: Path) -> None:
        center = _center(tmp_path)
        for capital in [1000.0, 5000.0, 10000.0]:
            ok, reason = center.validate_account_capital({"equity": capital})
            assert ok is True
            assert reason == "capital_sufficient"

    def test_daily_trade_limit_50_live(self, tmp_path: Path) -> None:
        today = f"{datetime.utcnow().date().isoformat()}T14:30:00"
        items_ok = [{"entry_time": today, "realized_pnl": 1.0} for _ in range(50)]
        center_ok = _center(tmp_path, items=items_ok)
        ok, reason, meta = center_ok.check_daily_trade_limit(account_scope="live")
        assert ok is True
        assert reason == "daily_trade_limit_ok"
        assert meta["trade_count_today"] == 50

        items_block = [{"entry_time": today, "realized_pnl": 1.0} for _ in range(51)]
        center_block = _center(tmp_path, items=items_block)
        blocked, blocked_reason, _ = center_block.check_daily_trade_limit(account_scope="live")
        assert blocked is False
        assert "daily_trade_limit_reached_50" in blocked_reason

    def test_daily_trade_limit_100_paper(self, tmp_path: Path) -> None:
        today = f"{datetime.utcnow().date().isoformat()}T14:30:00"
        items_ok = [{"entry_time": today, "realized_pnl": 1.0} for _ in range(100)]
        center_ok = _center(tmp_path, items=items_ok)
        ok, reason, meta = center_ok.check_daily_trade_limit(account_scope="paper")
        assert ok is True
        assert reason == "daily_trade_limit_ok"
        assert meta["trade_count_today"] == 100

        items_block = [{"entry_time": today, "realized_pnl": 1.0} for _ in range(101)]
        center_block = _center(tmp_path, items=items_block)
        blocked, blocked_reason, _ = center_block.check_daily_trade_limit(account_scope="paper")
        assert blocked is False
        assert "daily_trade_limit_reached_100" in blocked_reason

    def test_intraday_exposure_limit(self, tmp_path: Path) -> None:
        center = _center(tmp_path)
        ok, reason, _ = center.check_intraday_exposure(
            equity=10000.0,
            current_exposure=4000.0,
            proposed_exposure=1000.0,
        )
        assert ok is True
        assert reason == "intraday_exposure_ok"

        blocked, blocked_reason, _ = center.check_intraday_exposure(
            equity=10000.0,
            current_exposure=4000.0,
            proposed_exposure=2000.0,
        )
        assert blocked is False
        assert "intraday_exposure_limit_exceeded" in blocked_reason

    def test_intraday_drawdown_limit(self, tmp_path: Path) -> None:
        center = _center(tmp_path)
        ok, reason, _ = center.check_intraday_drawdown(opening_equity=10000.0, current_equity=9200.0)
        assert ok is True
        assert reason == "intraday_drawdown_ok"

        blocked, blocked_reason, _ = center.check_intraday_drawdown(opening_equity=10000.0, current_equity=8900.0)
        assert blocked is False
        assert "intraday_drawdown_exceeded" in blocked_reason

    def test_consecutive_loss_cooldown(self, tmp_path: Path) -> None:
        today = f"{datetime.utcnow().date().isoformat()}T14:30:00"
        ok_items = [
            {"entry_time": today, "realized_pnl": -50.0},
            {"entry_time": today, "realized_pnl": -75.0},
            {"entry_time": today, "realized_pnl": 30.0},
        ]
        center_ok = _center(tmp_path, items=ok_items)
        ok, reason, _ = center_ok.check_consecutive_loss_cooldown()
        assert ok is True
        assert reason == "loss_cooldown_ok"

        blocked_items = [
            {"entry_time": today, "realized_pnl": 10.0},
            {"entry_time": today, "realized_pnl": -25.0},
            {"entry_time": today, "realized_pnl": -35.0},
            {"entry_time": today, "realized_pnl": -55.0},
        ]
        center_block = _center(tmp_path, items=blocked_items)
        blocked, blocked_reason, _ = center_block.check_consecutive_loss_cooldown()
        assert blocked is False
        assert "cooldown" in blocked_reason
