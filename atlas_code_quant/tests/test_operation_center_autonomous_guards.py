from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
QUANT_ROOT = ROOT / "atlas_code_quant"
if str(QUANT_ROOT) not in sys.path:
    sys.path.insert(0, str(QUANT_ROOT))

from api.schemas import OrderRequest
import operations.operation_center as operation_center_module
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
            "summary": {
                "open_positions": 0,
                "watchlist_count": 0,
                "var_status": "ok",
                "var_method": "monte_carlo_portfolio",
                "var_95_usd": 0.0,
            },
            "alerts": [],
            "var_monitor": {
                "enabled": True,
                "method": "monte_carlo_portfolio",
                "status": "ok",
                "var_95_usd": 0.0,
            },
            "watchlist": [],
            "limit": limit,
        }

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        return {
            "enabled": True,
            "account_type": account_type,
            "summary": {
                "exit_now_count": 0,
                "de_risk_count": 0,
                "take_profit_count": 0,
                "var_status": "ok",
                "var_method": "monte_carlo_portfolio",
                "var_95_usd": 0.0,
            },
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
            "summary": {
                "options_entries_count": 0,
                "time_spread_count": 0,
                "vertical_spread_count": 0,
                "single_leg_count": 0,
                "structural_diversity_score_pct": 0.0,
                "operationally_exercised": False,
            },
            "alerts": [],
            "structures": [],
            "limit": limit,
        }


class _Vision:
    def __init__(
        self,
        *,
        provider: str = "desktop_capture",
        provider_ready: bool = True,
        capture_ok: bool = True,
        include_evidence: bool = True,
        ocr_confidence: float | None = None,
        chart_color: str | None = None,
        pattern: str | None = None,
        prices: list[float] | None = None,
    ) -> None:
        self.provider = provider
        self.provider_ready = provider_ready
        self.capture_ok = capture_ok
        self.include_evidence = include_evidence
        self.ocr_confidence = ocr_confidence
        self.chart_color = chart_color
        self.pattern = pattern
        self.prices = list(prices or [])

    def status(self) -> dict:
        return {
            "provider": self.provider,
            "provider_ready": self.provider_ready,
            "operator_present": True,
            "screen_integrity_ok": True,
        }

    def capture_context_snapshot(self, *, label: str = "operation") -> dict:
        payload = {"capture_ok": self.capture_ok, "label": label, "source": "test", "provider": self.provider}
        if self.include_evidence:
            payload["capture_path"] = f"C:/tmp/{label}.png"
            payload["meta_path"] = f"C:/tmp/{label}.json"
        if self.ocr_confidence is not None:
            payload["ocr"] = {
                "confidence": self.ocr_confidence,
                "pattern": self.pattern or "trend_hold",
                "chart_color": self.chart_color or "neutral",
                "prices": list(self.prices),
            }
        return payload


class _RiskGuardJournal(_Journal):
    def __init__(
        self,
        *,
        var_status: str = "ok",
        var_95_usd: float = 0.0,
        exit_now_count: int = 0,
        de_risk_count: int = 0,
    ) -> None:
        self.var_status = var_status
        self.var_95_usd = var_95_usd
        self.exit_now_count = exit_now_count
        self.de_risk_count = de_risk_count

    def position_management_snapshot(self, account_type: str | None = None, limit: int = 12) -> dict:
        payload = super().position_management_snapshot(account_type=account_type, limit=limit)
        payload["summary"].update(
            {
                "open_positions": max(self.exit_now_count + self.de_risk_count, 1),
                "watchlist_count": max(self.exit_now_count + self.de_risk_count, 1),
                "var_status": self.var_status,
                "var_method": "monte_carlo_portfolio",
                "var_95_usd": self.var_95_usd,
            }
        )
        payload["var_monitor"].update(
            {
                "enabled": True,
                "method": "monte_carlo_portfolio",
                "status": self.var_status,
                "var_95_usd": self.var_95_usd,
            }
        )
        return payload

    def exit_governance_snapshot(self, account_type: str | None = None, limit: int = 10) -> dict:
        payload = super().exit_governance_snapshot(account_type=account_type, limit=limit)
        payload["summary"].update(
            {
                "exit_now_count": self.exit_now_count,
                "de_risk_count": self.de_risk_count,
                "take_profit_count": 0,
                "var_status": self.var_status,
                "var_method": "monte_carlo_portfolio",
                "var_95_usd": self.var_95_usd,
            }
        )
        if self.exit_now_count > 0:
            payload["alerts"] = [
                {
                    "level": "warning",
                    "code": "exit_now_candidates_present",
                    "message": "Hay posiciones con criterio de salida inmediata.",
                }
            ]
        return payload


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
    def __init__(
        self,
        open_ok: bool = True,
        *,
        open_mode: str = "test",
        symbol_match: bool = True,
        timeframe_match: bool = True,
        manual_required: bool = False,
        readiness_score_pct: float | None = None,
    ) -> None:
        self.open_ok = open_ok
        self.open_mode = open_mode
        self.symbol_match = symbol_match
        self.timeframe_match = timeframe_match
        self.manual_required = manual_required
        self.readiness_score_pct = 100.0 if readiness_score_pct is None and open_ok else (35.0 if readiness_score_pct is None else readiness_score_pct)

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
            "open_mode": self.open_mode,
            "execution_state": self.open_mode,
            "symbol_match": self.symbol_match,
            "timeframe_match": self.timeframe_match,
            "manual_required": self.manual_required,
            "operator_review_required": self.manual_required or not self.open_ok,
            "readiness_score_pct": self.readiness_score_pct,
            "warnings": [],
        }


def _quote_provider_factory(payload: dict[str, object]):
    def _provider(**_: object) -> dict[str, object]:
        return dict(payload)

    return _provider


def _enable_paper_autonomy(center: OperationCenter, *, mode: str = "paper_autonomous") -> None:
    center.update_config({"auton_mode": mode, "reset_fail_safe": True})


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
    assert any("strategy_type is required" in reason for reason in payload["reasons"])
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


def test_reset_after_journal_rebuild_arms_paper_guard_and_clears_runtime_state(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
    )
    center.update_config(
        {
            "auton_mode": "paper_autonomous",
            "reset_fail_safe": True,
        }
    )

    result = center.reset_after_journal_rebuild(account_scope="paper", guard_hours=1.0)
    state = center.get_config()

    assert result["ok"] is True
    assert state["last_decision"] is None
    assert state["last_candidate"] is None
    assert state["post_journal_rebuild_guard"]["active"] is True
    assert state["post_journal_rebuild_guard"]["scope"] == "paper"


def test_open_symbol_guard_is_bypassed_temporarily_after_journal_rebuild(tmp_path: Path) -> None:
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
        reconciliation_provider=lambda **_: {"reconciliation": {"state": "failed", "max_positions_gap": 3}},
        quote_provider=_quote_provider_factory({"symbol": "AAPL", "bid": 100.0, "ask": 100.2, "last": 100.1}),
    )
    center.update_config({"auton_mode": "paper_autonomous", "reset_fail_safe": True})
    center.reset_after_journal_rebuild(account_scope="paper", guard_hours=1.0)

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
            max_entry_drift_pct=2.0,
            max_entry_spread_pct=0.5,
        ),
        action="preview",
        capture_context=False,
    )

    assert not any("Open-symbol guard blocked" in reason for reason in payload["reasons"])
    assert any("Open-symbol guard bypassed" in warning for warning in payload["warnings"])


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

    # In paper mode, reconciliation is degraded but not blocking (paper_local mismatch is expected)
    assert any("Reconciliation degraded" in w for w in payload.get("warnings", []))
    assert payload["what_if"]["current_equity"] == 123456.0


def test_market_context_gate_warns_in_paper_when_context_is_explicitly_blocked(tmp_path: Path) -> None:
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
    _enable_paper_autonomy(center)

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="market",
            asset_class="equity",
            account_scope="paper",
            strategy_type="equity_long",
            market_context={
                "confidence_pct": 41.0,
                "macro_bias": {"state": "defensive"},
                "regime": {"primary_regime": "risk_extreme"},
                "context_report": {"permission": "block"},
                "decision_gate": {
                    "action": "block",
                    "blocked": True,
                    "degraded": False,
                    "reasons": ["regimen clasificado como riesgo extremo"],
                    "warnings": [],
                },
            },
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is True
    assert payload["blocked"] is False
    assert payload["market_context_gate"]["blocked"] is True
    assert any("Market context blocked submit" in warning for warning in payload["warnings"])
    assert executor.calls == 1


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
    _enable_paper_autonomy(center)

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
    _enable_paper_autonomy(center)

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
    _enable_paper_autonomy(center)

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

    assert payload["allowed"] is True
    assert payload["blocked"] is False
    assert payload["visual_entry_gate"]["degraded"] is True
    assert payload["visual_entry_gate"]["status"] == "manual_review"
    assert payload["visual_entry_gate"]["blocking_ready"] is False
    assert payload["visual_entry_gate"]["operator_review_required"] is True
    assert any("Visual gate would block submit" in warning for warning in payload["visual_entry_gate"]["warnings"])
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
    _enable_paper_autonomy(center)

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
    assert payload["visual_entry_gate"]["status"] == "visual_ready"
    assert payload["visual_entry_gate"]["context_capture_requested"] is True
    assert payload["visual_entry_gate"]["context_capture_ok"] is True
    assert payload["visual_entry_gate"]["readiness_score_pct"] == 100.0
    assert payload["allowed"] is True


def test_visual_entry_gate_requires_manual_review_when_chart_mission_needs_manual_open(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=False, open_mode="manual_required", manual_required=True, readiness_score_pct=35.0),
        state_path=tmp_path / "operation_center_state.json",
    )
    _enable_paper_autonomy(center)

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
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["allowed"] is True
    assert payload["blocked"] is False
    assert payload["visual_entry_gate"]["status"] == "manual_required"
    assert payload["visual_entry_gate"]["manual_required"] is True
    assert payload["visual_entry_gate"]["blocking_ready"] is False
    assert payload["visual_entry_gate"]["blocking_reason"] == "chart mission requires manual opening"
    assert payload["visual_entry_gate"]["readiness_score_pct"] == 35.0
    assert any("Visual gate would block submit" in warning for warning in payload["visual_entry_gate"]["warnings"])


def test_visual_entry_gate_allows_supervised_preview_when_chart_open_is_manual(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=False, open_mode="manual_required", manual_required=True, readiness_score_pct=35.0),
        state_path=tmp_path / "operation_center_state.json",
    )
    _enable_paper_autonomy(center, mode="paper_supervised")

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
        ),
        action="preview",
        capture_context=False,
    )

    assert payload["allowed"] is True
    assert payload["blocked"] is False
    assert payload["visual_entry_gate"]["status"] == "manual_assist_preview"
    assert payload["visual_entry_gate"]["manual_required"] is True
    assert payload["visual_entry_gate"]["blocking_ready"] is True
    assert payload["visual_entry_gate"]["blocking_reason"] == "chart mission requires manual opening"
    assert any("supervised preview is allowed" in warning for warning in payload["visual_entry_gate"]["warnings"])
    assert not any("Visual gate blocked preview" in reason for reason in payload["reasons"])


def test_visual_entry_gate_marks_chart_pending_when_symbol_match_is_not_confirmed(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, open_mode="opened", symbol_match=False, readiness_score_pct=60.0),
        state_path=tmp_path / "operation_center_state.json",
    )
    _enable_paper_autonomy(center)

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
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["visual_entry_gate"]["degraded"] is True
    assert payload["visual_entry_gate"]["status"] == "manual_review"
    assert payload["visual_entry_gate"]["blocking_ready"] is False
    assert any("symbol match" in warning for warning in payload["visual_entry_gate"]["warnings"])
    assert payload["allowed"] is True
    assert any("Visual gate would block submit" in warning for warning in payload["visual_entry_gate"]["warnings"])


def test_visual_entry_gate_blocks_when_camera_capture_fails(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(capture_ok=False, include_evidence=False),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, open_mode="opened", readiness_score_pct=100.0),
        state_path=tmp_path / "operation_center_state.json",
    )
    _enable_paper_autonomy(center)

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
            camera_plan={"required": True, "provider": "direct_nexus", "visual_fit_pct": 88.0},
        ),
        action="submit",
        capture_context=True,
    )

    assert payload["allowed"] is True
    assert payload["blocked"] is False
    assert payload["visual_entry_gate"]["status"] == "camera_unavailable"
    assert payload["visual_entry_gate"]["blocking_reason"] == "visual context capture failed"
    assert payload["visual_entry_gate"]["context_evidence"]["capture_ok"] is False
    assert any("Visual gate would block submit" in warning for warning in payload["visual_entry_gate"]["warnings"])


def test_visual_entry_gate_blocks_when_visual_fit_is_too_low(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(capture_ok=True, include_evidence=True, ocr_confidence=0.92),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, open_mode="opened", readiness_score_pct=100.0),
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
                    {"title": "AAPL disparo 5m", "timeframe": "5m", "url": "https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=5"}
                ],
            },
            camera_plan={"required": True, "provider": "direct_nexus", "visual_fit_pct": 58.0},
        ),
        action="submit",
        capture_context=True,
    )

    assert payload["allowed"] is False
    assert payload["blocked"] is True
    assert payload["visual_entry_gate"]["status"] == "manual_review"
    assert payload["visual_entry_gate"]["blocking_ready"] is False
    assert payload["visual_entry_gate"]["context_evidence"]["ocr_confidence_pct"] == 92.0
    assert any("camera visual fit" in warning for warning in payload["visual_entry_gate"]["warnings"])


def test_visual_entry_gate_blocks_when_ocr_thesis_is_misaligned(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(capture_ok=True, include_evidence=True, ocr_confidence=0.95, chart_color="bearish", pattern="breakdown"),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, open_mode="opened", readiness_score_pct=100.0),
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
            camera_plan={
                "required": True,
                "provider": "direct_nexus",
                "visual_fit_pct": 88.0,
                "expected_visual": {
                    "symbol": "AAPL",
                    "direction": "alcista",
                    "timeframe": "1h",
                    "higher_timeframe": "4h",
                    "expected_chart_bias": "bullish",
                    "expected_patterns": ["breakout", "uptrend", "pullback", "alcista"],
                },
            },
        ),
        action="submit",
        capture_context=True,
    )

    assert payload["allowed"] is False
    assert payload["blocked"] is True
    assert payload["visual_entry_gate"]["status"] == "manual_review"
    assert payload["visual_entry_gate"]["blocking_ready"] is False
    assert payload["visual_entry_gate"]["blocking_reason"] == "visual thesis is misaligned with OCR evidence"
    assert payload["visual_entry_gate"]["context_evidence"]["visual_alignment"]["confirmed"] is False
    assert payload["visual_entry_gate"]["context_evidence"]["visual_alignment"]["alignment_score_pct"] == 0.0


def test_visual_entry_gate_blocks_option_setup_when_ocr_confidence_is_below_profile_threshold(tmp_path: Path) -> None:
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_Journal(),
        vision=_Vision(capture_ok=True, include_evidence=True, ocr_confidence=0.74, chart_color="bullish", pattern="breakout", prices=[100.5, 101.2]),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, open_mode="opened", readiness_score_pct=100.0),
        state_path=tmp_path / "operation_center_state.json",
    )

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="AAPL",
            side="buy",
            size=1,
            order_type="debit",
            asset_class="multileg",
            account_scope="paper",
            strategy_type="bull_call_debit_spread",
            chart_plan={
                "provider": "tradingview",
                "targets": [
                    {"title": "AAPL disparo 1h", "timeframe": "1h", "url": "https://www.tradingview.com/chart/?symbol=NASDAQ%3AAAPL&interval=60"}
                ],
            },
            camera_plan={
                "required": True,
                "provider": "direct_nexus",
                "visual_fit_pct": 84.0,
                "expected_visual": {
                    "symbol": "AAPL",
                    "direction": "alcista",
                    "timeframe": "1h",
                    "higher_timeframe": "4h",
                    "expected_chart_bias": "bullish",
                    "expected_patterns": ["breakout", "uptrend", "pullback", "alcista"],
                },
                "validation_profile": {
                    "validation_mode": "defined_risk_structure_confirmation",
                    "min_ocr_confidence_pct": 84.0,
                    "min_alignment_score_pct": 80.0,
                    "require_pattern_confirmation": True,
                    "require_price_evidence": True,
                },
            },
        ),
        action="submit",
        capture_context=True,
    )

    assert payload["allowed"] is False
    assert payload["blocked"] is True
    assert payload["visual_entry_gate"]["blocking_reason"] == "OCR confidence 74.00% is below the required threshold (84.00%)"
    assert payload["visual_entry_gate"]["context_evidence"]["validation_profile"]["validation_mode"] == "defined_risk_structure_confirmation"


def test_preview_reuses_precomputed_probability_payload_without_recomputing(tmp_path: Path, monkeypatch) -> None:
    center = OperationCenter(
        tracker=_Tracker(balances={"total_equity": 100000.0, "cash": 50000.0, "option_buying_power": 50000.0}),
        journal=_Journal(),
        vision=_Vision(),
        executor=_Executor(),
        brain=_Brain(),
        learning=_Learning(),
        chart_execution=_ChartExecution(open_ok=True, manual_required=False),
        state_path=tmp_path / "operation_center_state.json",
    )

    def _unexpected_probability(**_: object):
        raise AssertionError("get_winning_probability should not be called when preview already has precomputed probability")

    monkeypatch.setattr(operation_center_module, "get_winning_probability", _unexpected_probability)

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="ABT",
            side="sell_to_open",
            size=1,
            order_type="credit",
            asset_class="multileg",
            tradier_class="multileg",
            strategy_type="bear_call_credit_spread",
            account_scope="paper",
            probability_payload={
                "win_rate_pct": 72.5,
                "selected_legs": [
                    {
                        "side": "short",
                        "option_type": "call",
                        "strike": 120.0,
                        "premium_mid": 1.8,
                        "expiration": "2026-05-15",
                        "dte": 45,
                        "symbol": "ABT260515C00120000",
                        "bid": 1.7,
                        "ask": 1.9,
                        "volume": 100,
                        "open_interest": 200,
                        "implied_volatility": 0.22,
                    },
                    {
                        "side": "long",
                        "option_type": "call",
                        "strike": 125.0,
                        "premium_mid": 0.8,
                        "expiration": "2026-05-15",
                        "dte": 45,
                        "symbol": "ABT260515C00125000",
                        "bid": 0.75,
                        "ask": 0.85,
                        "volume": 100,
                        "open_interest": 200,
                        "implied_volatility": 0.21,
                    },
                ],
                "market_snapshot": {"spot": 116.0},
                "net_premium": 1.0,
            },
            chart_plan={"targets": []},
            camera_plan={"required": False},
        ),
        action="preview",
        capture_context=False,
    )

    assert payload["probability_source"] == "precomputed"
    assert payload["probability"]["win_rate_pct"] == 72.5
    assert payload["evaluation_timings"]["probability_sec"] >= 0.0
    assert payload["evaluation_profile"]["dominant_stage"] in payload["evaluation_timings"]
    assert payload["evaluation_profile"]["captured_stage_count"] >= 1
    assert "monitor_summary_sec" in payload["evaluation_timings"]


def test_portfolio_risk_guard_blocks_submit_when_exit_now_candidates_exist(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_RiskGuardJournal(exit_now_count=2, var_status="warning", var_95_usd=430.0),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "AAPL", "bid": 100.0, "ask": 100.1, "last": 100.05}),
    )
    center.update_config({"auton_mode": "paper_autonomous", "reset_fail_safe": True})

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
            max_entry_drift_pct=2.0,
            max_entry_spread_pct=0.5,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["blocked"] is True
    assert payload["portfolio_risk_guard"]["blocked"] is True
    assert payload["gates"]["portfolio_risk_guard"]["blocked"] is True
    assert any("Portfolio risk guard blocked submit" in reason for reason in payload["reasons"])
    assert executor.calls == 0


def test_portfolio_risk_guard_blocks_submit_when_var_is_critical(tmp_path: Path) -> None:
    executor = _Executor()
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_RiskGuardJournal(var_status="critical", var_95_usd=640.0),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "MSFT", "bid": 100.0, "ask": 100.1, "last": 100.05}),
    )
    center.update_config({"auton_mode": "paper_autonomous", "reset_fail_safe": True})

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
            max_entry_drift_pct=2.0,
            max_entry_spread_pct=0.5,
        ),
        action="submit",
        capture_context=False,
    )

    assert payload["blocked"] is True
    assert payload["portfolio_risk_guard"]["status"] == "blocked"
    assert any("VaR Monte Carlo del libro esta en CRITICAL" in reason for reason in payload["reasons"])
    assert executor.calls == 0


def test_portfolio_risk_guard_does_not_block_close_orders(tmp_path: Path) -> None:
    executor = _ExecutorWithResponse({"decision": "paper_submit_sent", "route": {"side": "sell", "quantity": 1}})
    center = OperationCenter(
        tracker=_Tracker(),
        journal=_RiskGuardJournal(exit_now_count=3, var_status="critical", var_95_usd=700.0),
        vision=_Vision(),
        executor=executor,
        brain=_Brain(),
        learning=_Learning(),
        state_path=tmp_path / "operation_center_state.json",
        quote_provider=_quote_provider_factory({"symbol": "NVDA", "bid": 100.0, "ask": 100.1, "last": 100.05}),
    )
    center.update_config({"auton_mode": "paper_autonomous", "reset_fail_safe": True})

    payload = center.evaluate_candidate(
        order=OrderRequest(
            symbol="NVDA",
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

    assert payload["portfolio_risk_guard"]["status"] == "not_applicable"
    assert payload["portfolio_risk_guard"]["blocked"] is False
    assert payload["gates"]["portfolio_risk_guard"]["blocked"] is False
    assert not any("Portfolio risk guard blocked submit" in reason for reason in payload["reasons"])
