from __future__ import annotations

import sys
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pytest
import pytz

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from api.schemas import OrderRequest
import operations.operation_center as operation_center_module
from operations.operation_center import OperationCenter
from scanner.asset_classifier import AssetClass


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
    def snapshot(self, limit: int = 3) -> dict:
        return {"recent_entries_count": 0, "recent_entries": [], "limit": limit}

    def attribution_integrity_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {
                "open_untracked_count": 0,
                "recent_flagged_count": 0,
                "attributed_open_positions_pct": 100.0,
                "open_untracked_ratio_pct": 0.0,
            },
            "alerts": [],
            "flagged_entries": [],
            "limit": limit,
        }

    def position_management_snapshot(self, account_type: str | None = None, limit: int = 12) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"open_positions": 0, "watchlist_count": 0},
            "alerts": [],
            "watchlist": [],
            "limit": limit,
        }

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"exit_now_count": 0, "de_risk_count": 0, "take_profit_count": 0},
            "alerts": [],
            "recommendations": [],
            "limit": limit,
        }

    def post_trade_learning_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"closed_trades": 0, "policy_candidate_count": 0},
            "root_cause_breakdown": [],
            "strategy_learning": [],
            "policy_candidates": [],
            "limit": limit,
        }

    def options_governance_adoption_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {"option_entries_total": 0, "time_spread_count": 0},
            "alerts": [],
            "strategy_mix": [],
            "limit": limit,
        }


class _Vision:
    def status(self) -> dict:
        return {
            "provider": "desktop_capture",
            "provider_ready": True,
            "operator_present": True,
            "screen_integrity_ok": True,
        }


class _Executor:
    def __init__(self) -> None:
        self.calls = 0

    def status(self) -> dict:
        return {"mode": "paper_api", "kill_switch_active": False}

    def execute(self, **_: object) -> dict:
        self.calls += 1
        return {"decision": "paper_submit_sent"}

    def configure(self, **_: object) -> None:
        return None

    def emergency_stop(self, reason: str = "manual_stop") -> None:
        return None

    def clear_emergency_stop(self) -> None:
        return None


class _Brain:
    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}

    def record_operation_cycle(self, *_: object, **__: object) -> dict:
        return {"ok": True}


class _Learning:
    def status(self, account_scope: str | None = None) -> dict:
        return {"account_scope": account_scope, "policy_ready": True}


@pytest.fixture
def center(tmp_path: Path) -> OperationCenter:
    oc = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
    )
    oc.update_config({"auton_mode": "paper_autonomous", "reset_fail_safe": True})
    return oc


def _freeze_et(monkeypatch, *, year: int, month: int, day: int, hour: int, minute: int) -> None:
    et = pytz.timezone("America/New_York")
    frozen_et = et.localize(datetime(year, month, day, hour, minute, 0))

    class _FrozenDatetime(datetime):
        @classmethod
        def now(cls, tz=None):  # type: ignore[override]
            if tz is None:
                return frozen_et
            return frozen_et.astimezone(tz)

    monkeypatch.setattr(operation_center_module, "datetime", _FrozenDatetime)


@pytest.mark.unit
class TestOperationCenterMarketHours:
    def test_market_hours_equity_during_market_hours(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=14, minute=0)  # Monday
        monkeypatch.setattr(
            operation_center_module,
            "classify_asset",
            MagicMock(return_value=SimpleNamespace(asset_class=AssetClass.EQUITY_ETF)),
        )
        ok, reason = center._check_market_hours("SPY", "paper")
        assert ok is True
        assert reason == "market_hours_ok"

    def test_market_hours_equity_before_open(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=8, minute=0)  # Monday
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        ok, reason = center._check_market_hours("SPY", "paper")
        assert ok is False
        assert "market_not_open_yet" in reason
        assert "min_to_open" in reason

    def test_market_hours_equity_after_close(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=16, minute=30)  # Monday
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        ok, reason = center._check_market_hours("SPY", "paper")
        assert ok is False
        assert reason == "market_closed_for_day"

    def test_market_hours_equity_weekend(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=11, hour=10, minute=0)  # Saturday
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_ETF))
        ok, reason = center._check_market_hours("SPY", "paper")
        assert ok is False
        assert "market_closed_weekend" in reason

    def test_market_hours_crypto_24h(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=11, hour=3, minute=0)  # Saturday
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.CRYPTO))
        ok, reason = center._check_market_hours("BTC", "paper")
        assert ok is True
        assert reason == "crypto_market_24h"

    def test_market_hours_crypto_weekend(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=12, hour=14, minute=0)  # Sunday
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.CRYPTO))
        ok, reason = center._check_market_hours("ETH", "paper")
        assert ok is True
        assert reason == "crypto_market_24h"

    def test_preflight_blocks_equity_outside_hours(self, monkeypatch, center: OperationCenter) -> None:
        with patch.object(center, "_check_market_hours", return_value=(False, "market_closed_for_day")):
            payload = center.evaluate_candidate(
                order=OrderRequest(
                    symbol="SPY",
                    side="buy",
                    size=1,
                    order_type="market",
                    asset_class="equity",
                    account_scope="live",
                    strategy_type="equity_long",
                ),
                action="submit",
                capture_context=False,
            )
        assert payload["allowed"] is False
        assert payload["gates"]["market_hours"]["blocked"] is True
        assert "market_hours" in payload["gates"]
        assert any("Market hours gate blocked submit" in reason for reason in payload["reasons"])

    def test_preflight_allows_equity_during_hours(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=14, minute=0)
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        payload = center.evaluate_candidate(
            order=OrderRequest(
                symbol="SPY",
                side="buy",
                size=1,
                order_type="market",
                asset_class="equity",
                account_scope="paper",
                strategy_type="equity_long",
            ),
            action="preview",
            capture_context=False,
        )
        assert payload["gates"]["market_hours"]["reason"] == "market_hours_ok"
        assert payload["gates"]["market_hours"]["blocked"] is False
        assert payload["gates"]["market_hours"]["status"] == "ok"

    def test_market_hours_readiness_payload(self, monkeypatch, center: OperationCenter) -> None:
        _freeze_et(monkeypatch, year=2026, month=4, day=11, hour=10, minute=0)
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        payload = center.market_hours_readiness(symbol="SPY", account_scope="paper")
        assert payload["status"] in {"OK", "BLOCKED", "DEGRADED"}
        assert isinstance(payload["market_hours_ok"], bool)
        assert isinstance(payload["market_hours_reason"], str)
        assert isinstance(payload["blocked"], bool)

    def test_paper_market_hours_strict_blocks_submit_outside_rth(self, monkeypatch, center: OperationCenter) -> None:
        center.update_config({"paper_market_hours_strict": True, "reset_fail_safe": True})
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=18, minute=0)
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        payload = center.evaluate_candidate(
            order=OrderRequest(
                symbol="SPY",
                side="buy",
                size=1,
                order_type="market",
                asset_class="equity",
                account_scope="paper",
                strategy_type="equity_long",
            ),
            action="submit",
            capture_context=False,
        )
        assert payload["allowed"] is False
        assert payload["gates"]["market_hours"]["blocked"] is True
        assert any("paper_market_hours_strict" in str(r) for r in payload["reasons"])

    def test_paper_close_exempt_from_market_hours_strict(self, monkeypatch, center: OperationCenter) -> None:
        center.update_config({"paper_market_hours_strict": True, "reset_fail_safe": True})
        _freeze_et(monkeypatch, year=2026, month=4, day=13, hour=18, minute=0)
        monkeypatch.setattr(operation_center_module, "classify_asset", lambda symbol: SimpleNamespace(asset_class=AssetClass.EQUITY_STOCK))
        payload = center.evaluate_candidate(
            order=OrderRequest(
                symbol="SPY",
                side="sell",
                size=1,
                order_type="market",
                asset_class="equity",
                account_scope="paper",
                strategy_type="equity_long",
                position_effect="close",
            ),
            action="submit",
            capture_context=False,
        )
        assert payload["gates"]["market_hours"]["reason"] == "close_order_exempt_paper"
