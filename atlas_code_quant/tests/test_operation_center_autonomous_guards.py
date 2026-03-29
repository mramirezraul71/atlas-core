from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from api.schemas import OrderRequest
from operations.operation_center import OperationCenter


class _Tracker:
    def __init__(self, *, strategies: list[dict] | None = None, balances: dict | None = None) -> None:
        self._strategies = strategies or []
        self._balances = balances or {"total_equity": 100000.0, "cash": 50000.0, "option_buying_power": 0.0}

    def build_summary(self, **_: object) -> dict:
        return {
            "account_session": {"scope": "paper", "account_id": "paper-test"},
            "balances": dict(self._balances),
            "alerts": [],
            "totals": {"positions": len(self._strategies)},
            "strategies": list(self._strategies),
            "pdt_status": {},
            "refresh_interval_sec": 5,
        }


class _Journal:
    def snapshot(self, limit: int = 3) -> dict:
        return {"recent_entries_count": 0, "recent_entries": [], "limit": limit}

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


class _Vision:
    def status(self) -> dict:
        return {"provider": "desktop_capture", "provider_ready": True, "operator_present": True, "screen_integrity_ok": True}

    def capture_context_snapshot(self, *, label: str = "operation") -> dict:
        return {"capture_ok": True, "label": label, "source": "test"}


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


class _ExecutorWithResponse(_Executor):
    def __init__(self, response: dict[str, object]) -> None:
        super().__init__()
        self._response = response

    def execute(self, **_: object) -> dict:
        self.calls += 1
        return dict(self._response)


class _Brain:
    def status(self) -> dict:
        return {"last_memory_ok": True, "last_error": ""}

    def record_operation_cycle(self, *_: object, **__: object) -> dict:
        return {"ok": True}


class _Learning:
    def status(self, account_scope: str | None = None) -> dict:
        return {"account_scope": account_scope, "policy_ready": True}


class _ChartExecution:
    def __init__(self, open_ok: bool = True) -> None:
        self.open_ok = open_ok

    def status(self) -> dict:
        return {"auto_open_enabled": True, "browser_available": True}

    def ensure_chart_mission(self, *, chart_plan: dict | None, camera_plan: dict | None, symbol: str) -> dict:
        targets = list((chart_plan or {}).get("targets") or [])
        return {
            "requested": bool(targets),
            "provider": (chart_plan or {}).get("provider") or "tradingview",
            "target_count": len(targets),
            "requested_timeframes": [target.get("timeframe") for target in targets],
            "camera_required": bool((camera_plan or {}).get("required")),
            "open_attempted": True,
            "open_ok": self.open_ok,
            "open_mode": "test",
            "symbol_match": True,
            "timeframe_match": True,
            "warnings": [],
        }


def _quote_provider_factory(payload: dict[str, object]):
    def _provider(**_: object) -> dict[str, object]:
        return dict(payload)

    return _provider


def test_blocks_missing_strategy_type_for_autonomous_equity_orders(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is False
    assert payload["blocked"] is True
    assert "strategy_type is required" in payload["reasons"][0]
    assert executor.calls == 0


def test_blocks_reentry_when_symbol_is_already_open(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(
            strategies=[
                {
                    "underlying": "AAPL",
                    "positions": [{"symbol": "AAPL", "asset_class": "equity"}],
                }
            ]
        ),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
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
    assert any("Open-symbol guard" in reason for reason in payload["reasons"])
    assert executor.calls == 0


def test_blocks_submit_when_reconciliation_is_not_healthy_and_uses_total_equity(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(balances={"total_equity": 123456.0, "cash": 60000.0, "option_buying_power": 0.0}),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        reconciliation_provider=lambda **_: {"reconciliation": {"state": "failed", "max_positions_gap": 2}},
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="MSFT",
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
    assert any("Reconciliation gate blocked submit" in reason for reason in payload["reasons"])
    assert payload["what_if"]["current_equity"] == 123456.0
    assert executor.calls == 0


def test_blocks_submit_when_adverse_entry_drift_is_too_high(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "AAPL", "bid": 101.10, "ask": 101.30, "last": 101.20}),
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            entry_reference_price=100.0,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is False
    assert any("adverse drift" in reason for reason in payload["reasons"])
    assert payload["entry_validation"]["blocked"] is True
    assert executor.calls == 0


def test_blocks_submit_when_equity_spread_is_too_wide(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "MSFT", "bid": 99.0, "ask": 101.0, "last": 100.0}),
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="MSFT",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            entry_reference_price=100.0,
            max_entry_drift_pct=5.0,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is False
    assert any("spread is" in reason for reason in payload["reasons"])
    assert payload["entry_validation"]["metrics"]["spread_pct"] == 2.0
    assert executor.calls == 0


def test_execution_quality_detects_missing_route_payload(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "AAPL", "bid": 100.0, "ask": 100.1, "last": 100.05}),
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            entry_reference_price=100.0,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is True
    assert payload["execution_quality"]["degraded"] is True
    assert any("no route payload" in issue.lower() for issue in payload["execution_quality"]["critical_issues"])


def test_execution_quality_confirms_matching_route_payload(tmp_path: Path) -> None:
    executor = _ExecutorWithResponse(
        {
            "decision": "paper_submit_sent",
            "executor_mode": "paper_api",
            "response": {
                "provider": "tradier",
                "route": "tradier",
                "mode": "submit",
                "preview": False,
                "order_class": "equity",
                "position_effect": "open",
                "request_payload": {
                    "symbol": "MSFT",
                    "side": "buy",
                    "quantity": "2",
                },
                "tradier_response": {
                    "id": 12345,
                    "status": "ok",
                },
            },
        }
    )
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "MSFT", "bid": 100.0, "ask": 100.05, "last": 100.02}),
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="MSFT",
            side="buy",
            size=2,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            entry_reference_price=100.0,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is True
    assert payload["execution_quality"]["degraded"] is False
    assert payload["execution_quality"]["checks"]["symbol_match"] is True
    assert payload["execution_quality"]["checks"]["quantity_match"] is True
    assert payload["execution_quality"]["checks"]["broker_status_ok"] is True


def test_visual_entry_gate_warns_when_camera_validation_is_skipped(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True),
        state_path=tmp_path / "operation_center_state.json",
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            chart_plan={
                "provider": "tradingview",
                "targets": [
                    {"title": "AAPL disparo 1h", "timeframe": "1h", "url": "https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=60"}
                ],
            },
            camera_plan={"required": True, "provider": "direct_nexus"},
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["visual_entry_gate"]["degraded"] is True
    assert any("capture_context=False" in warning for warning in payload["visual_entry_gate"]["warnings"])


def test_visual_entry_gate_reports_chart_execution_when_context_is_enabled(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "AAPL", "bid": 100.0, "ask": 100.05, "last": 100.02}),
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            entry_reference_price=100.0,
            chart_plan={
                "provider": "tradingview",
                "targets": [
                    {"title": "AAPL disparo 1h", "timeframe": "1h", "url": "https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=60"},
                    {"title": "AAPL confirmacion 4h", "timeframe": "4h", "url": "https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=240"},
                ],
            },
            camera_plan={"required": True, "provider": "direct_nexus"},
        ),
        action="submit",
        capture_context=True,
    )

    assert payload["visual_entry_gate"]["chart_execution"]["open_ok"] is True
    assert payload["visual_entry_gate"]["context_capture_requested"] is True
    assert payload["visual_entry_gate"]["context_capture_ok"] is True
