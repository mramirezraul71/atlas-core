"""Operational control plane for Atlas Code-Quant."""
from __future__ import annotations

import json
import time
import threading
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Literal

from api.schemas import OrderRequest
from backtesting.winning_probability import SUPPORTED_STRATEGIES, StrategyLeg, _capital_at_risk, _safe_float, get_winning_probability
from config.settings import settings
from execution.tradier_controls import resolve_account_session
from learning.adaptive_policy import AdaptiveLearningService
from learning.trading_implementation_scorecard import build_trading_implementation_scorecard
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.journal_pro import JournalProService
from operations.sensor_vision import SensorVisionService


Level4CreditStrategy = Literal[
    "bear_call_credit_spread",
    "bull_put_credit_spread",
    "call_credit_butterfly",
    "put_credit_butterfly",
    "call_credit_condor",
    "put_credit_condor",
    "iron_condor",
    "iron_butterfly",
]

_LEVEL4_CREDIT_STRATEGIES: set[str] = {
    "bear_call_credit_spread",
    "bull_put_credit_spread",
    "call_credit_butterfly",
    "put_credit_butterfly",
    "call_credit_condor",
    "put_credit_condor",
    "iron_condor",
    "iron_butterfly",
}

_DEFAULT_STATE = {
    "account_scope": "paper",
    "paper_only": True,
    "auton_mode": "paper_supervised",
    "executor_mode": "paper_api",
    "vision_mode": "direct_nexus",
    "require_operator_present": False,
    "operator_present": True,
    "screen_integrity_ok": True,
    "sentiment_score": 0.0,
    "sentiment_source": "manual",
    "min_auton_win_rate_pct": 65.0,
    "max_level4_bpr_pct": 20.0,
    "auto_pause_on_operational_errors": True,
    "operational_error_limit": 3,
    "operational_error_count": 0,
    "operational_error_day": None,
    "fail_safe_active": False,
    "fail_safe_reason": None,
    "last_operational_error": None,
    "notes": "Plano de control orientado a simulada con camara directa robot",
    "last_decision": None,
    "last_candidate": None,
}


def _utc_day() -> str:
    return datetime.utcnow().date().isoformat()


class OperationCenter:
    def __init__(
        self,
        *,
        tracker: StrategyTracker | None = None,
        journal: JournalProService | None = None,
        vision: SensorVisionService | None = None,
        executor: AutonExecutorService | None = None,
        brain: QuantBrainBridge | None = None,
        learning: AdaptiveLearningService | None = None,
        state_path: Path | None = None,
        reconciliation_provider: Callable[..., dict[str, Any]] | None = None,
        quote_provider: Callable[..., dict[str, Any] | None] | None = None,
        scorecard_provider: Callable[[], dict[str, Any]] | None = None,
    ) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "operation_center_state.json")
        self.root_path = Path(__file__).resolve().parents[2]
        self.tracker = tracker or StrategyTracker()
        self.journal = journal or JournalProService()
        self.vision = vision or SensorVisionService()
        self.executor = executor or AutonExecutorService()
        self.brain = brain or QuantBrainBridge()
        self.learning = learning or AdaptiveLearningService()
        self.reconciliation_provider = reconciliation_provider
        self.quote_provider = quote_provider or self._default_quote_provider
        self.scorecard_provider = scorecard_provider or self._build_scorecard_snapshot
        self._monitor_summary_lock = threading.Lock()
        self._monitor_summary_cache: dict[str, dict[str, Any]] = {}
        self._monitor_summary_cache_at: dict[str, float] = {}
        self._monitor_summary_workers: dict[str, threading.Thread] = {}
        self._monitor_summary_errors: dict[str, str] = {}
        self._scorecard_lock = threading.Lock()
        self._scorecard_cache: dict[str, Any] | None = None
        self._scorecard_cache_at = 0.0
        self._ensure_state()

    def _ensure_state(self) -> None:
        if not self.state_path.exists():
            self._save(_DEFAULT_STATE)

    def _load(self) -> dict[str, Any]:
        self._ensure_state()
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
            if isinstance(data, dict):
                merged = deepcopy(_DEFAULT_STATE)
                merged.update(data)
                return self._normalize_runtime_state(merged)
        except Exception:
            pass
        return self._normalize_runtime_state(deepcopy(_DEFAULT_STATE))

    def _normalize_runtime_state(self, state: dict[str, Any]) -> dict[str, Any]:
        today = _utc_day()
        if state.get("operational_error_day") != today:
            state["operational_error_day"] = today
            state["operational_error_count"] = 0
            state["last_operational_error"] = None
            if str(state.get("fail_safe_reason") or "").startswith("error_limit:"):
                state["fail_safe_active"] = False
                state["fail_safe_reason"] = None
        state["operational_error_limit"] = max(1, int(state.get("operational_error_limit") or 3))
        state["operational_error_count"] = max(0, int(state.get("operational_error_count") or 0))
        return state

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        merged = self._normalize_runtime_state(merged)
        self.state_path.write_text(json.dumps(merged, ensure_ascii=True, indent=2), encoding="utf-8")
        return merged

    def _empty_monitor_summary(self, scope: str, *, reason: str | None = None) -> dict[str, Any]:
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "refresh_interval_sec": settings.tradier_probability_refresh_sec,
            "account_session": {"scope": scope},
            "balances": {},
            "alerts": (
                [
                    {
                        "level": "warning",
                        "message": reason,
                        "scope": scope,
                    }
                ]
                if reason
                else []
            ),
            "totals": {},
            "strategies": [],
            "pdt_status": {},
        }

    def _cached_monitor_summary(self, scope: str) -> dict[str, Any] | None:
        with self._monitor_summary_lock:
            cached = self._monitor_summary_cache.get(scope)
            return deepcopy(cached) if cached else None

    def _monitor_summary_alert(self, monitor: dict[str, Any], *, message: str, scope: str) -> dict[str, Any]:
        payload = deepcopy(monitor)
        payload.setdefault("alerts", [])
        payload["alerts"] = list(payload.get("alerts") or [])
        payload["alerts"].append(
            {
                "level": "warning",
                "message": message,
                "scope": scope,
            }
        )
        return payload

    def _start_monitor_summary_refresh(self, scope: str) -> threading.Thread:
        with self._monitor_summary_lock:
            current = self._monitor_summary_workers.get(scope)
            if current and current.is_alive():
                return current

            def _worker() -> None:
                try:
                    monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
                    with self._monitor_summary_lock:
                        self._monitor_summary_cache[scope] = deepcopy(monitor)
                        self._monitor_summary_cache_at[scope] = time.monotonic()
                        self._monitor_summary_errors.pop(scope, None)
                except Exception as exc:
                    with self._monitor_summary_lock:
                        self._monitor_summary_errors[scope] = str(exc)
                finally:
                    with self._monitor_summary_lock:
                        current_thread = self._monitor_summary_workers.get(scope)
                        if current_thread is threading.current_thread():
                            self._monitor_summary_workers.pop(scope, None)

            worker = threading.Thread(target=_worker, name=f"monitor-summary-{scope}", daemon=True)
            self._monitor_summary_workers[scope] = worker
            worker.start()
            return worker

    def _load_monitor_summary(self, scope: str) -> dict[str, Any]:
        ttl_sec = max(2, int(settings.tradier_monitor_cache_ttl_sec or 300))
        # `operation/status` debe degradar rapido; no puede quedar secuestrado por el broker.
        timeout_sec = max(1, min(int(settings.tradier_timeout_sec or 15), 5))
        now = time.monotonic()
        with self._monitor_summary_lock:
            cached = deepcopy(self._monitor_summary_cache.get(scope)) if self._monitor_summary_cache.get(scope) else None
            cached_at = self._monitor_summary_cache_at.get(scope, 0.0)
            worker = self._monitor_summary_workers.get(scope)
            last_error = self._monitor_summary_errors.get(scope)
        if cached and (now - cached_at) <= ttl_sec:
            return cached

        worker = self._start_monitor_summary_refresh(scope)
        if cached:
            if worker.is_alive():
                return self._monitor_summary_alert(
                    cached,
                    message="Refreshing monitor summary in background",
                    scope=scope,
                )
            if last_error:
                return self._monitor_summary_alert(
                    cached,
                    message=f"Using cached monitor summary after tracker error: {last_error}",
                    scope=scope,
                )
            return cached

        worker.join(timeout=timeout_sec)
        with self._monitor_summary_lock:
            monitor = deepcopy(self._monitor_summary_cache.get(scope)) if self._monitor_summary_cache.get(scope) else None
            last_error = self._monitor_summary_errors.get(scope)
            still_running = bool(self._monitor_summary_workers.get(scope))
        if monitor:
            return monitor
        if last_error:
            return self._monitor_summary_alert(
                self._empty_monitor_summary(
                    scope,
                    reason=f"tracker.build_summary error: {last_error}",
                ),
                message=f"Using empty monitor summary after tracker error: {last_error}",
                scope=scope,
            )
        if still_running:
            return self._monitor_summary_alert(
                self._empty_monitor_summary(
                    scope,
                    reason=f"tracker.build_summary timeout after {timeout_sec}s",
                ),
                message=f"Monitor summary still refreshing after timeout ({timeout_sec}s)",
                scope=scope,
            )
        return self._empty_monitor_summary(scope, reason="tracker.build_summary returned no data")

    def _build_scorecard_snapshot(self) -> dict[str, Any]:
        payload = build_trading_implementation_scorecard(
            root=self.root_path,
            protocol_path=self.root_path / "reports/trading_self_audit_protocol.json",
            journal_db_path=self.root_path / "atlas_code_quant/data/journal/trading_journal.sqlite3",
            brain_state_path=self.root_path / "atlas_code_quant/data/operation/quant_brain_bridge_state.json",
            grafana_check_path=self.root_path / "reports/atlas_grafana_provisioning_check_latest.json",
            pytest_result=None,
        )
        metric_summary = {
            name: {
                "value": metric.get("value"),
                "status": metric.get("status"),
            }
            for name, metric in (payload.get("metrics") or {}).items()
        }
        return {
            "available": True,
            "generated_at": payload.get("generated_at"),
            "headline": payload.get("headline") or {},
            "metrics": metric_summary,
            "supporting_indicators": payload.get("supporting_indicators") or {},
            "next_actions": payload.get("next_actions") or [],
            "report_path": str(self.root_path / "reports/atlas_quant_implementation_scorecard_20260328.md"),
            "json_path": str(self.root_path / "reports/atlas_quant_implementation_scorecard.json"),
        }

    def _load_scorecard(self) -> dict[str, Any]:
        ttl_sec = 30.0
        now = time.monotonic()
        with self._scorecard_lock:
            if self._scorecard_cache is not None and (now - self._scorecard_cache_at) <= ttl_sec:
                return deepcopy(self._scorecard_cache)
        try:
            payload = self.scorecard_provider()
        except Exception as exc:
            payload = {
                "available": False,
                "error": str(exc),
                "generated_at": datetime.utcnow().isoformat(),
            }
        with self._scorecard_lock:
            self._scorecard_cache = deepcopy(payload)
            self._scorecard_cache_at = now
        return payload

    def _dispatch_brain_task(self, task_name: str, callback: Any, /, *args: Any, **kwargs: Any) -> None:
        def _runner() -> None:
            try:
                callback(*args, **kwargs)
            except Exception:
                pass

        threading.Thread(target=_runner, name=f"brain-{task_name}", daemon=True).start()

    @staticmethod
    def _is_opening_equity_order(order: OrderRequest) -> bool:
        asset_class = str(order.asset_class or "auto").lower()
        if asset_class != "equity":
            return False
        if str(order.position_effect or "auto").lower() == "close":
            return False
        return str(order.side or "").lower() in {"buy", "sell_short"}

    @staticmethod
    def _extract_open_symbols(monitor: dict[str, Any]) -> set[str]:
        symbols: set[str] = set()
        for strategy in monitor.get("strategies") or []:
            underlying = str(strategy.get("underlying") or "").strip().upper()
            if underlying:
                symbols.add(underlying)
            for position in strategy.get("positions") or []:
                symbol = str(position.get("symbol") or "").strip().upper()
                if symbol:
                    symbols.add(symbol)
        return symbols

    def _load_reconciliation(self, *, account_scope: str, account_id: str | None) -> dict[str, Any] | None:
        if self.reconciliation_provider is None:
            return None
        try:
            payload = self.reconciliation_provider(account_scope=account_scope, account_id=account_id)
        except Exception:
            return None
        if not isinstance(payload, dict):
            return None
        reconciliation = payload.get("reconciliation")
        return reconciliation if isinstance(reconciliation, dict) else None

    def _default_quote_provider(
        self,
        *,
        symbol: str,
        account_scope: str,
        account_id: str | None,
    ) -> dict[str, Any] | None:
        try:
            client, _ = resolve_account_session(
                account_scope=account_scope,  # type: ignore[arg-type]
                account_id=account_id,
            )
            payload = client.quote(symbol)
        except Exception:
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def _entry_price_basis(order: OrderRequest, quote: dict[str, Any]) -> tuple[float | None, float | None, float | None, float | None]:
        bid = _safe_float(quote.get("bid"), 0.0)
        ask = _safe_float(quote.get("ask"), 0.0)
        last = _safe_float(quote.get("last"), _safe_float(quote.get("close"), 0.0))
        mid = ((bid + ask) / 2.0) if bid > 0 and ask > 0 else (ask if ask > 0 else bid if bid > 0 else last if last > 0 else 0.0)
        basis = ask if str(order.side or "").lower() in {"buy", "buy_to_open", "buy_to_cover"} else bid
        if basis <= 0:
            basis = last if last > 0 else mid
        spread_pct = None
        if bid > 0 and ask > 0 and mid > 0:
            spread_pct = ((ask - bid) / mid) * 100.0
        return (
            basis if basis > 0 else None,
            spread_pct,
            mid if mid > 0 else None,
            last if last > 0 else None,
        )

    def _build_entry_validation(
        self,
        *,
        order: OrderRequest,
        account_scope: str,
        action: Literal["evaluate", "preview", "submit"],
        opening_equity_order: bool,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "enabled": bool(settings.entry_validation_enabled),
            "stage": "entry_validation",
            "status": "not_applicable",
            "blocked": False,
            "reasons": [],
            "warnings": [],
            "reference_price": order.entry_reference_price,
            "expected_move_pct": order.entry_expected_move_pct,
            "confidence_reference_pct": order.entry_confidence_reference_pct,
            "max_adverse_drift_pct": order.max_entry_drift_pct or settings.entry_max_adverse_drift_pct,
            "max_spread_pct": order.max_entry_spread_pct or settings.entry_max_equity_spread_pct,
            "quote": {},
            "metrics": {},
        }
        if not payload["enabled"]:
            payload["status"] = "disabled"
            return payload
        if not opening_equity_order:
            payload["status"] = "not_applicable"
            return payload

        reference_price = _safe_float(order.entry_reference_price, 0.0)
        if reference_price <= 0:
            payload["status"] = "missing_reference"
            payload["warnings"].append("entry_reference_price is missing; drift validation skipped.")
            return payload

        quote = self.quote_provider(
            symbol=str(order.symbol or "").upper(),
            account_scope=account_scope,
            account_id=order.account_id,
        )
        if not isinstance(quote, dict) or not quote:
            payload["status"] = "quote_unavailable"
            payload["warnings"].append("Entry validation quote unavailable; spread/drift checks skipped.")
            return payload

        basis_price, spread_pct, mid_price, last_price = self._entry_price_basis(order, quote)
        if basis_price is None or basis_price <= 0:
            payload["status"] = "quote_incomplete"
            payload["warnings"].append("Entry validation quote did not provide a usable basis price.")
            payload["quote"] = quote
            return payload

        side = str(order.side or "").lower()
        if side in {"buy", "buy_to_open", "buy_to_cover"}:
            signed_drift_pct = ((basis_price - reference_price) / reference_price) * 100.0
        else:
            signed_drift_pct = ((reference_price - basis_price) / reference_price) * 100.0
        adverse_drift_pct = max(signed_drift_pct, 0.0)
        favorable_drift_pct = max(-signed_drift_pct, 0.0)
        expected_move_pct = _safe_float(order.entry_expected_move_pct, 0.0)
        drift_vs_expected_move_pct = (
            (adverse_drift_pct / expected_move_pct) * 100.0 if expected_move_pct > 0 else None
        )

        payload["status"] = "validated"
        payload["quote"] = {
            "symbol": quote.get("symbol") or order.symbol,
            "bid": round(_safe_float(quote.get("bid"), 0.0), 4) or None,
            "ask": round(_safe_float(quote.get("ask"), 0.0), 4) or None,
            "last": round(_safe_float(last_price, 0.0), 4) if last_price is not None else None,
            "mid": round(_safe_float(mid_price, 0.0), 4) if mid_price is not None else None,
            "basis_price": round(basis_price, 4),
        }
        payload["metrics"] = {
            "signed_drift_pct": round(signed_drift_pct, 4),
            "adverse_drift_pct": round(adverse_drift_pct, 4),
            "favorable_drift_pct": round(favorable_drift_pct, 4),
            "spread_pct": round(spread_pct, 4) if spread_pct is not None else None,
            "drift_vs_expected_move_pct": round(drift_vs_expected_move_pct, 2) if drift_vs_expected_move_pct is not None else None,
        }

        max_adverse_drift_pct = _safe_float(payload["max_adverse_drift_pct"], settings.entry_max_adverse_drift_pct)
        max_spread_pct = _safe_float(payload["max_spread_pct"], settings.entry_max_equity_spread_pct)
        if adverse_drift_pct > max_adverse_drift_pct and action in {"preview", "submit"}:
            payload["blocked"] = True
            payload["reasons"].append(
                f"Entry validation blocked {action} because adverse drift is {adverse_drift_pct:.2f}% (> {max_adverse_drift_pct:.2f}%)."
            )
        if spread_pct is not None and spread_pct > max_spread_pct and action in {"preview", "submit"}:
            payload["blocked"] = True
            payload["reasons"].append(
                f"Entry validation blocked {action} because spread is {spread_pct:.2f}% (> {max_spread_pct:.2f}%)."
            )
        warn_share = _safe_float(settings.entry_warn_drift_vs_expected_move_pct, 25.0)
        if drift_vs_expected_move_pct is not None and drift_vs_expected_move_pct > warn_share:
            payload["warnings"].append(
                f"Adverse drift already consumes {drift_vs_expected_move_pct:.1f}% of the expected move."
            )
        return payload

    @staticmethod
    def _stringify_order_size(value: float) -> str:
        numeric = float(value)
        if numeric.is_integer():
            return str(int(numeric))
        return f"{numeric:.8f}".rstrip("0").rstrip(".")

    def _build_execution_quality(
        self,
        *,
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
        execution_result: dict[str, Any] | None,
        reconciliation: dict[str, Any] | None,
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "stage": "execution_quality",
            "status": "not_executed" if execution_result is None else "evaluated",
            "degraded": False,
            "critical_issues": [],
            "warnings": [],
            "planned_order": {
                "symbol": order.symbol,
                "side": order.side,
                "size": float(order.size),
                "order_type": order.order_type,
                "preview": bool(order.preview),
                "asset_class": order.asset_class,
                "strategy_type": order.strategy_type,
                "account_scope": order.account_scope,
                "position_effect": order.position_effect,
            },
            "executor": execution_result or {},
            "route": {},
            "checks": {},
            "reconciliation_state": str((reconciliation or {}).get("state") or "").lower() or None,
        }
        if execution_result is None:
            return payload

        response = execution_result.get("response") or {}
        route_payload = response if isinstance(response, dict) else {}
        request_payload = route_payload.get("request_payload") or {}
        tradier_response = route_payload.get("tradier_response") or {}
        expected_preview = action != "submit"
        expected_quantity = self._stringify_order_size(float(order.size))

        order_id = tradier_response.get("id")
        partner_id = tradier_response.get("partner_id")
        broker_status = str(tradier_response.get("status") or "").lower()
        route_mode = str(route_payload.get("mode") or "").lower()
        route_preview = route_payload.get("preview")
        request_symbol = str(request_payload.get("symbol") or "")
        request_side = str(request_payload.get("side") or "")
        request_quantity = str(request_payload.get("quantity") or "")

        route_present = bool(route_payload)
        symbol_match = request_symbol.upper() == str(order.symbol or "").upper() if request_symbol else False
        side_match = request_side.lower() == str(order.side or "").lower() if request_side else False
        quantity_match = request_quantity == expected_quantity if request_quantity else False
        preview_match = bool(route_preview) == expected_preview if route_payload else False
        mode_match = route_mode == action if route_mode else False
        broker_status_ok = broker_status in {"ok", "accepted", "filled", "submitted"} if broker_status else False
        order_id_present = bool(order_id or partner_id)

        payload["route"] = {
            "provider": route_payload.get("provider"),
            "route": route_payload.get("route"),
            "mode": route_payload.get("mode"),
            "preview": route_payload.get("preview"),
            "order_class": route_payload.get("order_class"),
            "position_effect": route_payload.get("position_effect"),
            "request_payload": request_payload,
            "tradier_response": tradier_response,
            "broker_status": broker_status or None,
            "order_id": order_id,
            "partner_id": partner_id,
        }
        payload["checks"] = {
            "route_present": route_present,
            "symbol_match": symbol_match,
            "side_match": side_match,
            "quantity_match": quantity_match,
            "preview_match": preview_match,
            "mode_match": mode_match,
            "order_id_present": order_id_present,
            "broker_status_ok": broker_status_ok,
        }

        if action in {"preview", "submit"} and execution_result.get("decision") in {"paper_preview_sent", "paper_submit_sent"}:
            if not route_present:
                payload["critical_issues"].append("Execution reported sent but no route payload was returned.")
            if route_present and not symbol_match:
                payload["critical_issues"].append("Execution request symbol does not match the planned order.")
            if route_present and not side_match:
                payload["critical_issues"].append("Execution request side does not match the planned order.")
            if route_present and not quantity_match:
                payload["critical_issues"].append("Execution request quantity does not match the planned order.")
            if route_present and not preview_match:
                payload["critical_issues"].append("Execution preview flag does not match the requested action.")
            if route_present and not mode_match:
                payload["critical_issues"].append("Execution route mode does not match the requested action.")
            if route_present and not order_id_present:
                payload["warnings"].append("Execution response does not expose broker order identifiers.")
            if route_present and broker_status and not broker_status_ok:
                payload["critical_issues"].append(f"Execution broker status is '{broker_status}', not an accepted status.")

        if payload["critical_issues"]:
            payload["degraded"] = True
            payload["status"] = "degraded"
        elif payload["warnings"]:
            payload["status"] = "warning"
        return payload

    def _register_operational_error(self, state: dict[str, Any], *, reason: str) -> dict[str, Any]:
        state = self._normalize_runtime_state(state)
        state["operational_error_count"] = int(state.get("operational_error_count") or 0) + 1
        state["last_operational_error"] = {
            "reason": reason,
            "timestamp": datetime.utcnow().isoformat(),
        }
        limit = int(state.get("operational_error_limit") or 3)
        if bool(state.get("auto_pause_on_operational_errors", True)) and state["operational_error_count"] >= limit:
            state["fail_safe_active"] = True
            state["fail_safe_reason"] = f"error_limit:{state['operational_error_count']}/{limit}"
            self.executor.emergency_stop(reason="operation_fail_safe")
        return self._save(state)

    def get_config(self) -> dict[str, Any]:
        state = self._load()
        state["vision"] = self.vision.status()
        state["executor"] = self.executor.status()
        return state

    def update_config(self, payload: dict[str, Any]) -> dict[str, Any]:
        state = self._load()
        for key in (
            "account_scope",
            "paper_only",
            "auton_mode",
            "executor_mode",
            "vision_mode",
            "require_operator_present",
            "sentiment_score",
            "sentiment_source",
            "min_auton_win_rate_pct",
            "max_level4_bpr_pct",
            "auto_pause_on_operational_errors",
            "operational_error_limit",
            "notes",
        ):
            if key in payload and payload[key] is not None:
                state[key] = payload[key]
        if "operator_present" in payload or "screen_integrity_ok" in payload or "notes" in payload or "vision_mode" in payload:
            self.vision.update(
                provider=payload.get("vision_mode"),
                operator_present=payload.get("operator_present"),
                screen_integrity_ok=payload.get("screen_integrity_ok"),
                notes=payload.get("notes"),
            )
        self.executor.configure(
            mode=payload.get("executor_mode"),
            kill_switch_active=payload.get("kill_switch_active"),
            notes=payload.get("notes"),
        )
        return self._save(state)

    def status(self) -> dict[str, Any]:
        config = self._load()
        scope = str(config.get("account_scope") or "paper")
        monitor = self._load_monitor_summary(scope)
        scorecard = self._load_scorecard()
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        journal_snapshot = self.journal.snapshot(limit=3)
        position_management = self.journal.position_management_snapshot(account_type=scope, limit=10)
        exit_governance = self.journal.exit_governance_snapshot(account_type=scope, limit=10)
        post_trade_learning = self.journal.post_trade_learning_snapshot(account_type=scope, limit=10)
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "config": {
                **config,
                "executor_mode": executor_status.get("mode"),
                "vision_mode": vision_status.get("provider"),
                "kill_switch_active": executor_status.get("kill_switch_active", False),
            },
            "pdt_status": monitor.get("pdt_status") or {},
            "win_rate_positions": [
                {
                    "strategy_id": item.get("strategy_id"),
                    "underlying": item.get("underlying"),
                    "strategy_type": item.get("strategy_type"),
                    "win_rate_pct": item.get("win_rate_pct"),
                    "open_pnl": item.get("open_pnl"),
                    "bpr": item.get("bpr"),
                    "alert": item.get("alert"),
                }
                for item in (monitor.get("strategies") or [])[:8]
            ],
            "ai_sentiment": {
                "score": _safe_float(config.get("sentiment_score"), 0.0),
                "source": config.get("sentiment_source") or "manual",
                "note": "Puntaje manual para pruebas simuladas hasta integrar el sentimiento de Grok.",
            },
            "vision": vision_status,
            "executor": executor_status,
            "brain": self.brain.status(),
            "learning": self.learning.status(account_scope=scope),
            "journal": {
                "recent_entries_count": journal_snapshot.get("recent_entries_count", 0),
                "recent_entries": journal_snapshot.get("recent_entries", []),
            },
            "position_management": position_management,
            "exit_governance": exit_governance,
            "post_trade_learning": post_trade_learning,
            "failsafe": {
                "active": bool(config.get("fail_safe_active")),
                "reason": config.get("fail_safe_reason"),
                "operational_error_count": int(config.get("operational_error_count") or 0),
                "operational_error_limit": int(config.get("operational_error_limit") or 3),
                "auto_pause_on_operational_errors": bool(config.get("auto_pause_on_operational_errors", True)),
                "last_operational_error": config.get("last_operational_error"),
                "day": config.get("operational_error_day"),
            },
            "monitor_summary": {
                "account_session": monitor.get("account_session"),
                "balances": monitor.get("balances"),
                "alerts": monitor.get("alerts") or [],
                "totals": monitor.get("totals") or {},
            },
            "scorecard": scorecard,
            "auton_mode_active": str(config.get("auton_mode") or "off") != "off" and not executor_status.get("kill_switch_active", False),
            "last_decision": config.get("last_decision"),
            "last_candidate": config.get("last_candidate"),
        }

    def emergency_stop(self, *, reason: str = "manual_stop") -> dict[str, Any]:
        self.executor.emergency_stop(reason=reason)
        state = self._load()
        state["last_decision"] = {
            "decision": "emergency_stop",
            "reason": reason,
            "timestamp": datetime.utcnow().isoformat(),
        }
        self._save(state)
        self._dispatch_brain_task(
            "emergency-stop",
            self.brain.emit,
            kind="emergency_stop",
            level="warning",
            message=f"[QUANT][EMERGENCY] Parada total activada: {reason}",
            tags=["quant", "operation", "emergency_stop"],
            data={"reason": reason},
            description="Parada total activada desde Quant",
            context=json.dumps({"reason": reason}, ensure_ascii=False),
            outcome="stopped",
            memorize=False,
        )
        return self.status()

    def clear_emergency_stop(self) -> dict[str, Any]:
        self.executor.clear_emergency_stop()
        self._dispatch_brain_task(
            "emergency-reset",
            self.brain.emit,
            kind="emergency_reset",
            level="info",
            message="[QUANT][EMERGENCY] Parada total desactivada",
            tags=["quant", "operation", "emergency_reset"],
            data={},
            description="Parada total desactivada desde Quant",
            context="{}",
            outcome="resumed",
            memorize=False,
        )
        return self.status()

    def evaluate_candidate(
        self,
        *,
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"] = "evaluate",
        capture_context: bool = True,
    ) -> dict[str, Any]:
        config = self._load()
        order_copy = order.model_copy(deep=True)
        if not order_copy.account_scope:
            order_copy.account_scope = str(config.get("account_scope") or "paper")  # type: ignore[assignment]
        scope = str(order_copy.account_scope or "paper")
        opening_equity_order = self._is_opening_equity_order(order_copy)
        monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
        balances = monitor.get("balances") or {}
        pdt_status = monitor.get("pdt_status") or {}
        reconciliation = self._load_reconciliation(account_scope=scope, account_id=order_copy.account_id)
        open_symbols = self._extract_open_symbols(monitor)
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        reasons: list[str] = []
        warnings: list[str] = []
        probability_payload: dict[str, Any] | None = None
        strategy_type = str(order_copy.strategy_type or order_copy.probability_gate.strategy_type) if order_copy.probability_gate and order_copy.strategy_type is None else (str(order_copy.strategy_type) if order_copy.strategy_type else None)

        if bool(config.get("paper_only", True)) and scope != "paper":
            reasons.append("El plano de control esta bloqueado en modo solo simulada.")
        if executor_status.get("kill_switch_active"):
            reasons.append("La parada de emergencia esta activa.")
        if bool(config.get("fail_safe_active")) and action in {"preview", "submit"}:
            reasons.append(f"Failsafe operativo activo: {config.get('fail_safe_reason') or 'error_limit'}.")
        if bool(config.get("require_operator_present")) and not vision_status.get("operator_present"):
            reasons.append("La politica actual exige presencia del operador.")
        if str(config.get("vision_mode") or "manual") != "off" and not vision_status.get("screen_integrity_ok"):
            reasons.append("La verificacion de pantallas no esta en estado OK.")
        if action == "submit" and str(config.get("auton_mode") or "off") == "off":
            reasons.append("El modo autonomo esta apagado.")
        if pdt_status.get("blocked_opening"):
            reasons.append(str(pdt_status.get("reason") or "PDT blocked opening trades."))
        if opening_equity_order and not strategy_type:
            reasons.append("strategy_type is required for autonomous equity orders.")
        symbol_upper = str(order_copy.symbol or "").strip().upper()
        if opening_equity_order and symbol_upper and symbol_upper in open_symbols:
            reasons.append(f"Open-symbol guard blocked re-entry for {symbol_upper}.")
        reconciliation_state = str((reconciliation or {}).get("state") or "").lower()
        if opening_equity_order and action == "submit" and reconciliation_state and reconciliation_state != "healthy":
            reasons.append(f"Reconciliation gate blocked submit because state is '{reconciliation_state}'.")

        entry_validation = self._build_entry_validation(
            order=order_copy,
            account_scope=scope,
            action=action,
            opening_equity_order=opening_equity_order,
        )
        reasons.extend(list(entry_validation.get("reasons") or []))
        warnings.extend(list(entry_validation.get("warnings") or []))

        risk_dollars = None
        bpr_pct = None
        if strategy_type and strategy_type in SUPPORTED_STRATEGIES:
            probability_result = get_winning_probability(
                symbol=order_copy.symbol,
                strategy_type=strategy_type,  # type: ignore[arg-type]
                account_scope=scope,  # type: ignore[arg-type]
            )
            probability_payload = probability_result.to_dict()
            legs = [
                StrategyLeg(
                    side=str(leg.get("side") or "long"),  # type: ignore[arg-type]
                    option_type=str(leg.get("option_type") or "call"),  # type: ignore[arg-type]
                    strike=float(leg.get("strike") or 0.0),
                    premium_mid=float(leg.get("premium_mid") or 0.0),
                    expiration=leg.get("expiration"),
                    dte=leg.get("dte"),
                    symbol=str(leg.get("symbol") or ""),
                    bid=float(leg.get("bid") or 0.0),
                    ask=float(leg.get("ask") or 0.0),
                    volume=float(leg.get("volume") or 0.0),
                    open_interest=float(leg.get("open_interest") or 0.0),
                    implied_volatility=_safe_float(leg.get("implied_volatility"), float("nan")),
                )
                for leg in (probability_payload.get("selected_legs") or [])
            ]
            spot = _safe_float((probability_payload.get("market_snapshot") or {}).get("spot"), 0.0)
            risk_per_unit = _capital_at_risk(strategy_type, legs, spot) * 100.0
            risk_dollars = round(risk_per_unit * max(float(order_copy.size), 1.0), 4)
            buying_power = max(
                _safe_float(balances.get("option_buying_power"), 0.0),
                _safe_float(balances.get("cash"), 0.0),
            )
            bpr_pct = round((risk_dollars / buying_power) * 100.0, 2) if buying_power > 0 else None
            min_win_rate = _safe_float(config.get("min_auton_win_rate_pct"), 65.0)
            if _safe_float(probability_payload.get("win_rate_pct"), 0.0) < min_win_rate:
                reasons.append(f"La probabilidad de exito esta por debajo del filtro ({probability_payload.get('win_rate_pct'):.2f}% < {min_win_rate:.2f}%).")
            if strategy_type in _LEVEL4_CREDIT_STRATEGIES and bpr_pct is not None:
                max_bpr_pct = _safe_float(config.get("max_level4_bpr_pct"), 20.0)
                if bpr_pct > max_bpr_pct:
                    reasons.append(f"Level 4 credit structure uses {bpr_pct:.2f}% of buying power (> {max_bpr_pct:.2f}%).")
        elif not strategy_type:
            warnings.append("strategy_type is missing; autonomous gate is operating without Monte Carlo validation.")
        elif not opening_equity_order:
            warnings.append(f"strategy_type '{strategy_type}' has no configured probability model.")

        context_snapshot = self.vision.capture_context_snapshot(label=f"{scope}_{action}") if capture_context else None
        if capture_context and isinstance(context_snapshot, dict) and context_snapshot.get("capture_ok") is False:
            warnings.append("La captura visual no fue confirmada correctamente.")
        allowed = not reasons
        execution_result: dict[str, Any] | None = None
        operational_failure_reason: str | None = None
        if allowed and action in {"preview", "submit"}:
            if scope != "paper" and bool(config.get("paper_only", True)):
                allowed = False
                reasons.append("La ejecucion real sigue deshabilitada mientras solo simulada este activa.")
            else:
                order_copy.preview = action != "submit"
                try:
                    execution_result = self.executor.execute(order=order_copy, action=action, mode=str(config.get("executor_mode") or "disabled"))
                    if execution_result.get("decision") == "blocked":
                        allowed = False
                        block_reason = str(execution_result.get("reason") or "El ejecutor bloqueo la operacion.")
                        reasons.append(block_reason)
                        operational_failure_reason = block_reason
                except Exception as exc:
                    allowed = False
                    operational_failure_reason = f"executor_exception:{exc}"
                    reasons.append(f"Fallo operativo del ejecutor: {exc}")
                    execution_result = {"decision": "error", "error": str(exc)}

        execution_quality = self._build_execution_quality(
            order=order_copy,
            action=action,
            execution_result=execution_result,
            reconciliation=reconciliation,
        )
        warnings.extend(list(execution_quality.get("warnings") or []))

        projected_position_count = len(monitor.get("strategies") or []) + (1 if action in {"preview", "submit"} and allowed else 0)
        option_bp = _safe_float(balances.get("option_buying_power"), 0.0)
        cash = _safe_float(balances.get("cash"), 0.0)
        equity = _safe_float(balances.get("total_equity"), _safe_float(balances.get("equity"), 0.0))
        bp_base = max(option_bp, cash, 0.0)
        projected_bp = max(bp_base - _safe_float(risk_dollars, 0.0), 0.0)
        projected_risk_usage_pct = round((_safe_float(risk_dollars, 0.0) / bp_base) * 100.0, 2) if bp_base > 0 else None
        approval_lane = "ligero"
        if projected_risk_usage_pct is not None:
            if projected_risk_usage_pct >= 20:
                approval_lane = "alto"
            elif projected_risk_usage_pct >= 10:
                approval_lane = "moderado"

        if operational_failure_reason:
            config = self._register_operational_error(config, reason=operational_failure_reason)
        else:
            config = self._save(config)

        decision = (
            "blocked"
            if not allowed
            else ("submit_ready" if action == "submit" else "preview_ready" if action == "preview" else "eligible")
        )
        payload = {
            "generated_at": datetime.utcnow().isoformat(),
            "decision": decision,
            "allowed": allowed,
            "blocked": not allowed,
            "action": action,
            "reasons": reasons,
            "warnings": warnings,
            "gates": {
                "scope": scope,
                "pdt_status": pdt_status,
                "win_rate_pct": probability_payload.get("win_rate_pct") if probability_payload else None,
                "min_win_rate_pct": config.get("min_auton_win_rate_pct"),
                "risk_dollars": risk_dollars,
                "level4_bpr_pct": bpr_pct,
                "max_level4_bpr_pct": config.get("max_level4_bpr_pct"),
                "reconciliation": reconciliation,
                "entry_validation": {
                    "status": entry_validation.get("status"),
                    "blocked": bool(entry_validation.get("blocked")),
                },
                "execution_quality": {
                    "status": execution_quality.get("status"),
                    "degraded": bool(execution_quality.get("degraded")),
                },
            },
            "what_if": {
                "current_equity": round(equity, 2),
                "current_cash": round(cash, 2),
                "current_buying_power": round(bp_base, 2),
                "projected_buying_power": round(projected_bp, 2),
                "projected_risk_usage_pct": projected_risk_usage_pct,
                "projected_position_count": projected_position_count,
                "approval_lane": approval_lane,
                "capital_buffer_after_trade": round(projected_bp, 2),
            },
            "execution": execution_result,
            "probability": probability_payload,
            "vision_snapshot": context_snapshot,
            "entry_validation": entry_validation,
            "execution_quality": execution_quality,
            "monitor_summary": {
                "account_session": monitor.get("account_session"),
                "balances": balances,
                "totals": monitor.get("totals") or {},
                "reconciliation": reconciliation,
            },
            "sentiment": {
                "score": _safe_float(config.get("sentiment_score"), 0.0),
                "source": config.get("sentiment_source") or "manual",
            },
        }
        config["last_candidate"] = {
            "symbol": order_copy.symbol,
            "strategy_type": strategy_type,
            "scope": scope,
            "action": action,
        }
        config["last_decision"] = {
            "decision": decision,
            "allowed": allowed,
            "timestamp": payload["generated_at"],
            "reason": reasons[0] if reasons else "ok",
        }
        self._save(config)
        payload["operation_status"] = self.status()
        self._dispatch_brain_task(
            "operation-cycle",
            self.brain.record_operation_cycle,
            payload,
            order={
                "symbol": order_copy.symbol,
                "strategy_type": strategy_type,
                "account_scope": scope,
                "action": action,
                "size": float(order_copy.size),
            },
        )
        return payload
