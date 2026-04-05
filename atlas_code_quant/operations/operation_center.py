"""Operational control plane for Atlas Code-Quant."""
from __future__ import annotations

import json
import logging
import time
import threading
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Literal

from api.schemas import OrderRequest, TradierOrderLeg
from backtesting.winning_probability import SUPPORTED_STRATEGIES, StrategyLeg, _capital_at_risk, _safe_float, get_winning_probability
from config.settings import settings
from execution.tradier_controls import resolve_account_session
from execution.tradier_execution import build_tradier_order_payload
from learning.adaptive_policy import AdaptiveLearningService
from learning.trading_implementation_scorecard import (
    build_trading_implementation_scorecard,
    write_trading_implementation_scorecard_json,
    write_trading_implementation_scorecard_report,
)
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.chart_execution import ChartExecutionService
from operations.journal_pro import JournalProService
from operations.sensor_vision import SensorVisionService

logger = logging.getLogger("quant.operation_center")

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
    "auton_mode": "off",
    "executor_mode": "disabled",
    "vision_mode": "direct_nexus",
    "require_operator_present": False,
    "operator_present": True,
    "screen_integrity_ok": True,
    "sentiment_score": 0.0,
    "sentiment_source": "manual",
    "min_auton_win_rate_pct": 35.0,
    "max_level4_bpr_pct": 20.0,
    "auto_pause_on_operational_errors": True,
    "operational_error_limit": 10,
    "operational_error_count": 0,
    "operational_error_day": None,
    "fail_safe_active": True,
    "fail_safe_reason": "safe_default_bootstrap",
    "last_operational_error": None,
    "notes": "Default safe bootstrap: autonomy disabled until operator enables it.",
    "last_decision": None,
    "last_candidate": None,
    "attribution_integrity_alert_active": False,
    "attribution_integrity_last_count": 0,
    "attribution_integrity_last_event_at": None,
    "visual_gate_stats": {
        "evaluated_count": 0,
        "applies_count": 0,
        "blocked_count": 0,
        "passed_count": 0,
        "manual_review_count": 0,
        "last_status": "not_requested",
        "last_blocked": False,
        "last_manual_required": False,
        "last_readiness_score_pct": 100.0,
        "last_alignment_score_pct": None,
        "last_updated_at": None,
        "last_symbol": None,
        "last_action": None,
        "last_blocking_reason": None,
    },
    "last_options_strategy_governance": {},
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
        chart_execution: ChartExecutionService | None = None,
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
        self.chart_execution = chart_execution or ChartExecutionService()
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
        self._scorecard_worker: threading.Thread | None = None
        self._scorecard_error: str | None = None
        self._status_cache_lock = threading.Lock()
        self._status_cache_payload: dict[str, Any] | None = None
        self._status_cache_at = 0.0
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
        visual_gate = state.get("visual_gate_stats")
        if not isinstance(visual_gate, dict):
            visual_gate = {}
        normalized_visual_gate = deepcopy(_DEFAULT_STATE["visual_gate_stats"])
        normalized_visual_gate.update(visual_gate)
        for key in ("evaluated_count", "applies_count", "blocked_count", "passed_count", "manual_review_count"):
            normalized_visual_gate[key] = max(0, int(normalized_visual_gate.get(key) or 0))
        normalized_visual_gate["last_blocked"] = bool(normalized_visual_gate.get("last_blocked"))
        normalized_visual_gate["last_manual_required"] = bool(normalized_visual_gate.get("last_manual_required"))
        normalized_visual_gate["last_readiness_score_pct"] = round(
            _safe_float(normalized_visual_gate.get("last_readiness_score_pct"), 100.0),
            2,
        )
        alignment = normalized_visual_gate.get("last_alignment_score_pct")
        normalized_visual_gate["last_alignment_score_pct"] = (
            round(_safe_float(alignment, 0.0), 2) if alignment is not None else None
        )
        state["visual_gate_stats"] = normalized_visual_gate
        return state

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        merged = self._normalize_runtime_state(merged)
        self.state_path.write_text(json.dumps(merged, ensure_ascii=True, indent=2), encoding="utf-8")
        self._invalidate_status_cache()
        return merged

    def _invalidate_status_cache(self) -> None:
        with self._status_cache_lock:
            self._status_cache_payload = None
            self._status_cache_at = 0.0

    def _record_visual_gate_metrics(
        self,
        *,
        state: dict[str, Any],
        gate: dict[str, Any],
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
    ) -> dict[str, Any]:
        stats = deepcopy(state.get("visual_gate_stats") or _DEFAULT_STATE["visual_gate_stats"])
        stats["evaluated_count"] = int(stats.get("evaluated_count") or 0) + 1
        applies = bool(gate.get("applies"))
        if applies:
            stats["applies_count"] = int(stats.get("applies_count") or 0) + 1
        blocked = bool(gate.get("applies")) and (
            not bool(gate.get("blocking_ready"))
            or any(str(reason).lower().startswith("visual gate blocked") for reason in (gate.get("reasons") or []))
        )
        if blocked:
            stats["blocked_count"] = int(stats.get("blocked_count") or 0) + 1
        elif applies:
            stats["passed_count"] = int(stats.get("passed_count") or 0) + 1
        if bool(gate.get("manual_required")) or bool(gate.get("operator_review_required")):
            stats["manual_review_count"] = int(stats.get("manual_review_count") or 0) + 1

        alignment = ((gate.get("context_evidence") or {}).get("visual_alignment") or {}).get("alignment_score_pct")
        stats["last_status"] = str(gate.get("status") or "not_requested")
        stats["last_blocked"] = blocked
        stats["last_manual_required"] = bool(gate.get("manual_required")) or bool(gate.get("operator_review_required"))
        stats["last_readiness_score_pct"] = round(_safe_float(gate.get("readiness_score_pct"), 100.0), 2)
        stats["last_alignment_score_pct"] = round(_safe_float(alignment, 0.0), 2) if alignment is not None else None
        stats["last_updated_at"] = datetime.utcnow().isoformat()
        stats["last_symbol"] = str(order.symbol or "")
        stats["last_action"] = action
        stats["last_blocking_reason"] = gate.get("blocking_reason")
        state["visual_gate_stats"] = stats
        return state

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

    def _load_monitor_summary(self, scope: str, *, fast: bool = False) -> dict[str, Any]:
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
        if fast:
            if cached:
                return self._monitor_summary_alert(
                    cached,
                    message="Refreshing monitor summary in background",
                    scope=scope,
                )
            reason = f"tracker.build_summary refreshing in background for scope {scope}"
            return self._monitor_summary_alert(
                self._empty_monitor_summary(scope, reason=reason),
                message=f"Monitor summary refreshing in background for {scope}",
                scope=scope,
            )
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
        json_path = write_trading_implementation_scorecard_json(
            payload,
            self.root_path / "reports/atlas_quant_implementation_scorecard.json",
        )
        report_path = write_trading_implementation_scorecard_report(
            payload,
            self.root_path / "reports/atlas_quant_implementation_scorecard_latest.md",
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
            "report_path": str(report_path),
            "json_path": str(json_path),
        }

    def _empty_scorecard_snapshot(self, *, status: str, error: str | None = None) -> dict[str, Any]:
        payload = {
            "available": False,
            "status": status,
            "generated_at": datetime.utcnow().isoformat(),
            "headline": {},
            "metrics": {},
            "supporting_indicators": {},
            "next_actions": [],
        }
        if error:
            payload["error"] = error
        return payload

    def _start_scorecard_refresh(self) -> threading.Thread:
        with self._scorecard_lock:
            current = self._scorecard_worker
            if current and current.is_alive():
                return current

            def _worker() -> None:
                try:
                    payload = self.scorecard_provider()
                except Exception as exc:
                    payload = self._empty_scorecard_snapshot(status="error", error=str(exc))
                    with self._scorecard_lock:
                        self._scorecard_error = str(exc)
                else:
                    with self._scorecard_lock:
                        self._scorecard_error = None
                finally:
                    with self._scorecard_lock:
                        self._scorecard_cache = deepcopy(payload)
                        self._scorecard_cache_at = time.monotonic()
                        if self._scorecard_worker is threading.current_thread():
                            self._scorecard_worker = None

            worker = threading.Thread(target=_worker, name="scorecard-refresh", daemon=True)
            self._scorecard_worker = worker
            worker.start()
            return worker

    def _load_scorecard(self, *, fast: bool = False) -> dict[str, Any]:
        ttl_sec = 30.0
        now = time.monotonic()
        with self._scorecard_lock:
            if self._scorecard_cache is not None and (now - self._scorecard_cache_at) <= ttl_sec:
                return deepcopy(self._scorecard_cache)
            cached = deepcopy(self._scorecard_cache) if self._scorecard_cache is not None else None
            worker = self._scorecard_worker
            last_error = self._scorecard_error
        worker = self._start_scorecard_refresh()
        if fast:
            if cached is not None:
                cached["status"] = "refreshing" if worker.is_alive() else (cached.get("status") or "cached")
                if last_error:
                    cached["error"] = last_error
                return cached
            return self._empty_scorecard_snapshot(status="refreshing", error=last_error)
        worker.join(timeout=2.0)
        with self._scorecard_lock:
            if self._scorecard_cache is not None:
                payload = deepcopy(self._scorecard_cache)
                if self._scorecard_error and "error" not in payload:
                    payload["error"] = self._scorecard_error
                return payload
            error = self._scorecard_error
        return self._empty_scorecard_snapshot(status="timeout", error=error)

    def _dispatch_brain_task(self, task_name: str, callback: Any, /, *args: Any, **kwargs: Any) -> None:
        def _runner() -> None:
            try:
                callback(*args, **kwargs)
            except Exception:
                pass

        threading.Thread(target=_runner, name=f"brain-{task_name}", daemon=True).start()

    def _maybe_emit_attribution_integrity_event(
        self,
        *,
        state: dict[str, Any],
        snapshot: dict[str, Any],
        scope: str,
    ) -> dict[str, Any]:
        summary = snapshot.get("summary") or {}
        flagged_count = int(summary.get("recent_flagged_count") or 0)
        alert_active = bool(state.get("attribution_integrity_alert_active"))
        previous_count = int(state.get("attribution_integrity_last_count") or 0)
        should_emit = flagged_count > 0 and not alert_active and previous_count <= 0

        state["attribution_integrity_alert_active"] = flagged_count > 0
        state["attribution_integrity_last_count"] = flagged_count
        if not should_emit:
            return state

        state["attribution_integrity_last_event_at"] = datetime.utcnow().isoformat()
        brain_emit = getattr(self.brain, "emit", None)
        if callable(brain_emit):
            self._dispatch_brain_task(
                "attribution-integrity",
                brain_emit,
                kind="attribution_integrity_alert",
                level="warning",
                message=(
                    f"[QUANT][ATTRIBUTION] {flagged_count} entradas recientes con atribucion invalida "
                    f"detectadas en scope {scope}"
                ),
                tags=["quant", "journal", "attribution_integrity", scope],
                data={
                    "scope": scope,
                    "summary": summary,
                    "alerts": snapshot.get("alerts") or [],
                    "flagged_entries": snapshot.get("flagged_entries") or [],
                },
                description="Quant detecto entradas recientes sin strategy_type valido en el journal operativo",
                context=json.dumps(
                    {
                        "scope": scope,
                        "summary": summary,
                    },
                    ensure_ascii=False,
                ),
                outcome="attribution_integrity_alerted",
                memorize=True,
            )
        return state

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

    def _build_market_context_gate(
        self,
        *,
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
    ) -> dict[str, Any]:
        context = dict(order.market_context or {})
        gate = dict(context.get("decision_gate") or {})
        payload: dict[str, Any] = {
            "enabled": bool(context),
            "stage": "market_context",
            "status": "missing" if not context else str(gate.get("action") or "allow"),
            "blocked": False,
            "degraded": False,
            "reasons": [],
            "warnings": [],
            "confidence_pct": _safe_float(context.get("confidence_pct"), 0.0) if context else None,
            "macro_bias": ((context.get("macro_bias") or {}).get("state") if context else None),
            "regime": ((context.get("regime") or {}).get("primary_regime") if context else None),
            "permission": ((context.get("context_report") or {}).get("permission") if context else None),
            "source_trace": dict(context.get("source_trace") or {}) if context else {},
        }
        if not context:
            payload["warnings"].append("market_context is missing; context-first gate cannot validate the setup.")
            return payload

        payload["warnings"].extend(list(gate.get("warnings") or []))
        gate_reasons = list(gate.get("reasons") or [])
        if bool(gate.get("blocked")) and action in {"preview", "submit"}:
            payload["blocked"] = True
            payload["status"] = "blocked"
            payload["reasons"].extend(f"Market context blocked {action}: {reason}" for reason in gate_reasons)
        elif bool(gate.get("degraded")):
            payload["degraded"] = True
            payload["status"] = "degraded"
            payload["reasons"].extend(f"Market context degraded setup: {reason}" for reason in gate_reasons)
        return payload

    @staticmethod
    def _stringify_order_size(value: float) -> str:
        numeric = float(value)
        if numeric.is_integer():
            return str(int(numeric))
        return f"{numeric:.8f}".rstrip("0").rstrip(".")

    @staticmethod
    def _estimate_equity_opening_notional(
        order: OrderRequest,
        *,
        entry_validation: dict[str, Any],
    ) -> float | None:
        if not OperationCenter._is_opening_equity_order(order):
            return None
        quote = entry_validation.get("quote") or {}
        reference_candidates = (
            quote.get("basis_price"),
            quote.get("mid"),
            quote.get("last"),
            quote.get("ask"),
            order.entry_reference_price,
        )
        reference_price = 0.0
        for candidate in reference_candidates:
            price = _safe_float(candidate, 0.0)
            if price > 0:
                reference_price = price
                break
        if reference_price <= 0:
            return None
        quantity = abs(_safe_float(order.size, 0.0))
        if quantity <= 0:
            return None
        return round(reference_price * quantity, 2)

    @staticmethod
    def _uses_option_buying_power(order: OrderRequest, strategy_type: str | None) -> bool:
        asset_class = str(order.asset_class or "auto").strip().lower()
        if asset_class in {"option", "multileg", "combo"}:
            return True
        if bool(order.option_symbol) or bool(order.legs):
            return True
        normalized_strategy = str(strategy_type or "").strip().lower()
        return bool(normalized_strategy and normalized_strategy not in {"equity_long", "equity_short"})

    @staticmethod
    def _strategy_leg_side_map(position_effect: str) -> dict[str, str]:
        normalized = str(position_effect or "open").strip().lower()
        if normalized == "close":
            return {"long": "sell_to_close", "short": "buy_to_close"}
        return {"long": "buy_to_open", "short": "sell_to_open"}

    def _build_execution_preflight(
        self,
        *,
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
    ) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "stage": "execution_preflight",
            "status": "not_requested",
            "blocked": False,
            "critical_issues": [],
            "warnings": [],
            "route": {},
        }
        if action not in {"preview", "submit"}:
            return payload
        try:
            order_class, position_effect, request_payload = build_tradier_order_payload(order)
            payload["route"] = {
                "order_class": order_class,
                "position_effect": position_effect,
                "request_payload": request_payload,
            }
            requested_effect = str(order.position_effect or "").strip().lower()
            request_side = str(request_payload.get("side") or "").strip().lower()
            request_leg_sides = [
                str(value or "").strip().lower()
                for key, value in request_payload.items()
                if str(key).startswith("side[")
            ]
            if requested_effect == "close" and position_effect != "close":
                payload["critical_issues"].append(
                    "Execution preflight resolved an opening route for a close order."
                )
            if requested_effect == "close":
                if request_side in {"buy_to_open", "sell_to_open"}:
                    payload["critical_issues"].append(
                        "Execution preflight produced an opening single-leg side for a close order."
                    )
                if any(side in {"buy_to_open", "sell_to_open"} for side in request_leg_sides):
                    payload["critical_issues"].append(
                        "Execution preflight produced opening multileg sides for a close order."
                    )
        except Exception as exc:
            payload["critical_issues"].append(f"Execution preflight failed: {exc}")

        if payload["critical_issues"]:
            payload["blocked"] = True
            payload["status"] = "blocked"
        else:
            payload["status"] = "ok"
        return payload

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
        if payload.get("reset_fail_safe"):
            state["fail_safe_active"] = False
            state["fail_safe_reason"] = None
            state["operational_error_count"] = 0
            state["last_operational_error"] = None
            self.executor.clear_emergency_stop()
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

    def _selector_session_payload(self) -> dict[str, Any]:
        return {
            "mode": str(settings.selector_session_mode or "balanced").lower(),
            "require_optionable_candidate": bool(settings.selector_options_require_available),
            "options_enabled": bool(settings.options_enabled),
        }

    def _lightweight_operation_status(
        self,
        *,
        scope: str,
        config: dict[str, Any],
        monitor: dict[str, Any],
        vision_status: dict[str, Any],
        executor_status: dict[str, Any],
    ) -> dict[str, Any]:
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "scope": scope,
            "config": {
                "account_scope": config.get("account_scope"),
                "paper_only": bool(config.get("paper_only", True)),
                "auton_mode": config.get("auton_mode"),
                "executor_mode": executor_status.get("mode"),
                "vision_mode": vision_status.get("provider"),
                "kill_switch_active": bool(executor_status.get("kill_switch_active", False)),
            },
            "vision": vision_status,
            "executor": executor_status,
            "brain": self.brain.status(),
            "monitor_summary": {
                "account_session": monitor.get("account_session"),
                "balances": monitor.get("balances") or {},
                "totals": monitor.get("totals") or {},
            },
        }

    def status_lite(self) -> dict[str, Any]:
        config = self._load()
        scope = str(config.get("account_scope") or "paper")
        monitor = self._load_monitor_summary(scope, fast=True)
        try:
            vision_status = self.vision.status(fast=True)
        except TypeError:
            vision_status = self.vision.status()
        executor_status = self.executor.status()
        scorecard = self._load_scorecard(fast=True)
        payload = self._lightweight_operation_status(
            scope=scope,
            config=config,
            monitor=monitor,
            vision_status=vision_status,
            executor_status=executor_status,
        )
        payload["selector_session"] = self._selector_session_payload()
        payload["failsafe"] = {
            "active": bool(config.get("fail_safe_active")),
            "reason": config.get("fail_safe_reason"),
            "operational_error_count": int(config.get("operational_error_count") or 0),
            "operational_error_limit": int(config.get("operational_error_limit") or 3),
        }
        payload["scorecard"] = {
            "available": bool(scorecard.get("available")),
            "status": scorecard.get("status"),
            "error": scorecard.get("error"),
            "generated_at": scorecard.get("generated_at"),
            "headline": deepcopy(scorecard.get("headline") or {}),
        }
        payload["chart_execution"] = self.chart_execution.status()
        try:
            payload["learning"] = self.learning.status(account_scope=scope, fast=True)
        except TypeError:
            payload["learning"] = self.learning.status(account_scope=scope)
        payload["auton_mode_active"] = (
            str(config.get("auton_mode") or "off") != "off"
            and not executor_status.get("kill_switch_active", False)
        )
        return payload

    def _safe_journal_payload(
        self,
        *,
        label: str,
        scope: str,
        limit: int,
        loader: Callable[[], dict[str, Any]],
    ) -> dict[str, Any]:
        try:
            return loader()
        except Exception as exc:
            logger.warning("journal snapshot fallback for %s: %s", label, exc)
            return {
                "generated_at": datetime.utcnow().isoformat(),
                "account_type": scope,
                "enabled": False,
                "error": str(exc),
                "summary": {},
                "alerts": [
                    {
                        "level": "warning",
                        "code": f"{label}_fallback",
                        "message": f"Journal snapshot '{label}' fell back because the journal database was busy or unavailable.",
                    }
                ],
                "limit": limit,
            }

    def status(self) -> dict[str, Any]:
        ttl_sec = 2.0
        now = time.monotonic()
        with self._status_cache_lock:
            if self._status_cache_payload is not None and (now - self._status_cache_at) <= ttl_sec:
                return deepcopy(self._status_cache_payload)
        config = self._load()
        scope = str(config.get("account_scope") or "paper")
        monitor = self._load_monitor_summary(scope)
        scorecard = self._load_scorecard()
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        journal_snapshot = self._safe_journal_payload(
            label="snapshot",
            scope=scope,
            limit=3,
            loader=lambda: self.journal.snapshot(limit=3),
        )
        attribution_integrity = self._safe_journal_payload(
            label="attribution_integrity",
            scope=scope,
            limit=10,
            loader=lambda: self.journal.attribution_integrity_snapshot(account_type=scope, limit=10),
        )
        config = self._maybe_emit_attribution_integrity_event(
            state=config,
            snapshot=attribution_integrity,
            scope=scope,
        )
        config = self._save(config)
        position_management = self._safe_journal_payload(
            label="position_management",
            scope=scope,
            limit=10,
            loader=lambda: self.journal.position_management_snapshot(account_type=scope, limit=10),
        )
        exit_governance = self._safe_journal_payload(
            label="exit_governance",
            scope=scope,
            limit=10,
            loader=lambda: self.journal.exit_governance_snapshot(account_type=scope, limit=10),
        )
        post_trade_learning = self._safe_journal_payload(
            label="post_trade_learning",
            scope=scope,
            limit=10,
            loader=lambda: self.journal.post_trade_learning_snapshot(account_type=scope, limit=10),
        )
        visual_benchmark = self._load_visual_benchmark_snapshot(scorecard)
        visual_gate_metrics = deepcopy(config.get("visual_gate_stats") or {})
        selector_session = self._selector_session_payload()
        options_strategy_governance = deepcopy(config.get("last_options_strategy_governance") or {})
        options_governance_adoption = self._safe_journal_payload(
            label="options_governance_adoption",
            scope=scope,
            limit=10,
            loader=lambda: self.journal.options_governance_adoption_snapshot(account_type=scope, limit=10),
        )
        payload = {
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
            "chart_execution": self.chart_execution.status(),
            "learning": self.learning.status(account_scope=scope),
            "journal": {
                "recent_entries_count": journal_snapshot.get("recent_entries_count", 0),
                "recent_entries": journal_snapshot.get("recent_entries", []),
            },
            "attribution_integrity": attribution_integrity,
            "position_management": position_management,
            "exit_governance": exit_governance,
            "post_trade_learning": post_trade_learning,
            "visual_benchmark": visual_benchmark,
            "visual_gate_metrics": visual_gate_metrics,
            "selector_session": selector_session,
            "options_strategy_governance": options_strategy_governance,
            "options_governance_adoption": options_governance_adoption,
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
        with self._status_cache_lock:
            self._status_cache_payload = deepcopy(payload)
            self._status_cache_at = time.monotonic()
        return payload

    def _load_visual_benchmark_snapshot(self, scorecard: dict[str, Any]) -> dict[str, Any]:
        protocol_path = self.root_path / "reports/trading_self_audit_protocol.json"
        protocol: dict[str, Any] = {}
        try:
            if protocol_path.exists():
                payload = json.loads(protocol_path.read_text(encoding="utf-8"))
                if isinstance(payload, dict):
                    protocol = payload
        except Exception:
            protocol = {}
        focus = protocol.get("visual_entry_benchmark_focus") or {}
        indicators = scorecard.get("supporting_indicators") or {}
        metric = (scorecard.get("metrics") or {}).get("visual_benchmark_feedback_score") or {}
        return {
            "enabled": bool(focus),
            "current_focus": focus.get("current_focus"),
            "status": metric.get("status"),
            "score": metric.get("value"),
            "visual_source_count": indicators.get("visual_benchmark_source_count", 0),
            "translation_pct": indicators.get("visual_benchmark_translation_pct", 0.0),
            "human_best_practice": list(focus.get("human_best_practice") or []),
            "automation_translation": list(focus.get("automation_translation") or []),
            "recommended_metrics": list(focus.get("recommended_metrics") or []),
            "web_feedback_loop": list(focus.get("web_feedback_loop") or []),
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
        started_at = time.perf_counter()
        evaluation_timings: dict[str, float] = {}
        config = self._load()
        order_copy = order.model_copy(deep=True)
        if not order_copy.account_scope:
            order_copy.account_scope = str(config.get("account_scope") or "paper")  # type: ignore[assignment]
        scope = str(order_copy.account_scope or "paper")
        opening_equity_order = self._is_opening_equity_order(order_copy)
        monitor_started = time.perf_counter()
        monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
        evaluation_timings["monitor_summary_sec"] = round(time.perf_counter() - monitor_started, 4)
        balances = monitor.get("balances") or {}
        option_bp = _safe_float(balances.get("option_buying_power"), 0.0)
        cash = _safe_float(balances.get("cash"), 0.0)
        pdt_status = monitor.get("pdt_status") or {}
        reconciliation = self._load_reconciliation(account_scope=scope, account_id=order_copy.account_id)
        open_symbols = self._extract_open_symbols(monitor)
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        reasons: list[str] = []
        warnings: list[str] = []
        probability_payload: dict[str, Any] | None = None
        probability_source = "none"
        strategy_type = str(order_copy.strategy_type or order_copy.probability_gate.strategy_type) if order_copy.probability_gate and order_copy.strategy_type is None else (str(order_copy.strategy_type) if order_copy.strategy_type else None)

        is_close_order = str(order_copy.position_effect or "").lower() == "close"
        if bool(config.get("paper_only", True)) and scope != "paper":
            reasons.append("El plano de control esta bloqueado en modo solo simulada.")
        if not is_close_order and executor_status.get("kill_switch_active"):
            reasons.append("La parada de emergencia esta activa.")
        if not is_close_order and bool(config.get("fail_safe_active")) and action in {"preview", "submit"}:
            reasons.append(f"Failsafe operativo activo: {config.get('fail_safe_reason') or 'error_limit'}.")
        if not is_close_order and bool(config.get("require_operator_present")) and not vision_status.get("operator_present"):
            reasons.append("La politica actual exige presencia del operador.")
        if str(config.get("vision_mode") or "manual") != "off" and not vision_status.get("screen_integrity_ok"):
            if scope != "paper":
                reasons.append("La verificacion de pantallas no esta en estado OK.")
            else:
                warnings.append("Vision: screen_integrity_ok es false (no bloquea en paper).")
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
            # In paper mode, the paper_local simulator always reports 0 positions,
            # causing a permanent reconciliation mismatch. Only block if we're NOT in paper mode.
            if scope != "paper":
                reasons.append(f"Reconciliation gate blocked submit because state is '{reconciliation_state}'.")
            else:
                warnings.append(f"Reconciliation degraded (state='{reconciliation_state}') — bypassed in paper mode.")

        entry_validation = self._build_entry_validation(
            order=order_copy,
            account_scope=scope,
            action=action,
            opening_equity_order=opening_equity_order,
        )
        reasons.extend(list(entry_validation.get("reasons") or []))
        warnings.extend(list(entry_validation.get("warnings") or []))
        estimated_open_notional = self._estimate_equity_opening_notional(
            order_copy,
            entry_validation=entry_validation,
        )
        if (
            opening_equity_order
            and not is_close_order
            and action in {"preview", "submit"}
            and estimated_open_notional is not None
        ):
            available_buying_power = max(option_bp, cash, 0.0)
            if available_buying_power <= 0:
                reasons.append("Current buying power es 0. La apertura de equity queda bloqueada.")
            elif estimated_open_notional > available_buying_power:
                reasons.append(
                    "Buying power insuficiente para la apertura de equity: "
                    f"requiere {estimated_open_notional:.2f} y solo hay {available_buying_power:.2f}."
                )

        market_context_gate = self._build_market_context_gate(
            order=order_copy,
            action=action,
        )
        mcg_reasons = list(market_context_gate.get("reasons") or [])
        if is_close_order:
            warnings.extend([f"[close bypass] {r}" for r in mcg_reasons])
        elif scope == "paper" and mcg_reasons:
            warnings.extend([f"[paper bypass] {r}" for r in mcg_reasons])
        else:
            reasons.extend(mcg_reasons)
        warnings.extend(list(market_context_gate.get("warnings") or []))

        risk_dollars = None
        bpr_pct = None
        legs: list = []
        uses_option_bp = self._uses_option_buying_power(order_copy, strategy_type)
        if strategy_type and strategy_type in SUPPORTED_STRATEGIES:
            probability_started = time.perf_counter()
            precomputed_probability = dict(order_copy.probability_payload or {})
            if action != "submit" and precomputed_probability:
                probability_payload = precomputed_probability
                probability_source = "precomputed"
            else:
                probability_result = get_winning_probability(
                    symbol=order_copy.symbol,
                    strategy_type=strategy_type,  # type: ignore[arg-type]
                    account_scope=scope,  # type: ignore[arg-type]
                )
                probability_payload = probability_result.to_dict()
                probability_source = "live"
            evaluation_timings["probability_sec"] = round(time.perf_counter() - probability_started, 4)
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
            buying_power = max(option_bp, 0.0) if uses_option_bp else max(cash, option_bp, 0.0)
            bpr_pct = round((risk_dollars / buying_power) * 100.0, 2) if buying_power > 0 else None
            min_win_rate = _safe_float(config.get("min_auton_win_rate_pct"), 35.0)
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

        visual_entry_gate = self._build_visual_entry_gate(
            order=order_copy,
            action=action,
            capture_context=capture_context,
            vision_status=vision_status,
        )
        reasons.extend(list(visual_entry_gate.get("reasons") or []))
        warnings.extend(list(visual_entry_gate.get("warnings") or []))

        context_capture_started = time.perf_counter()
        context_snapshot = self.vision.capture_context_snapshot(label=f"{scope}_{action}") if capture_context else None
        if capture_context:
            evaluation_timings["context_capture_sec"] = round(time.perf_counter() - context_capture_started, 4)
        if capture_context and isinstance(context_snapshot, dict) and context_snapshot.get("capture_ok") is False:
            warnings.append("La captura visual no fue confirmada correctamente.")
            visual_entry_gate.setdefault("warnings", []).append("visual context snapshot failed.")
        visual_entry_gate["context_capture_requested"] = bool(capture_context)
        visual_entry_gate["context_capture_ok"] = bool(isinstance(context_snapshot, dict) and context_snapshot.get("capture_ok") is not False) if capture_context else False
        visual_entry_gate = self._finalize_visual_entry_gate(
            gate=visual_entry_gate,
            order=order_copy,
            action=action,
            auton_mode=str(config.get("auton_mode") or "off"),
            context_snapshot=context_snapshot if isinstance(context_snapshot, dict) else None,
            scope=scope,
        )
        reasons.extend(list(visual_entry_gate.get("reasons") or []))
        warnings.extend(list(visual_entry_gate.get("warnings") or []))
        if reasons:
            logger.info("[evaluate_candidate] BLOCKED %s — reasons: %s", order_copy.symbol, reasons)
        allowed = not reasons
        execution_result: dict[str, Any] | None = None
        operational_failure_reason: str | None = None
        execution_preflight: dict[str, Any] = {
            "stage": "execution_preflight",
            "status": "not_requested",
            "blocked": False,
            "critical_issues": [],
            "warnings": [],
            "route": {},
        }
        if allowed and action in {"preview", "submit"}:
            if scope != "paper" and bool(config.get("paper_only", True)):
                allowed = False
                reasons.append("La ejecucion real sigue deshabilitada mientras solo simulada este activa.")
            else:
                order_copy.preview = action != "submit"
                if not order_copy.legs and strategy_type and strategy_type not in {"equity_long", "equity_short"}:
                    if legs and len(legs) >= 2:
                        _side_map = self._strategy_leg_side_map(order_copy.position_effect or "open")
                        order_copy.legs = [
                            TradierOrderLeg(
                                option_symbol=leg.symbol,
                                side=_side_map.get(leg.side, "buy_to_open"),
                                quantity=max(float(order_copy.size), 1.0),
                            )
                            for leg in legs
                        ]
                        if not order_copy.tradier_class:
                            order_copy.tradier_class = "multileg"
                        _net_prem = _safe_float((probability_payload or {}).get("net_premium"), 0.0)
                        if strategy_type in _LEVEL4_CREDIT_STRATEGIES:
                            order_copy.order_type = "credit"
                            order_copy.price = round(abs(_net_prem), 2) if _net_prem != 0.0 else 0.01
                        elif _net_prem > 0.0:
                            order_copy.order_type = "debit"
                            order_copy.price = round(_net_prem, 2)
                        logger.info("[evaluate_candidate] Populated %d legs for %s (%s) order_type=%s price=%s", len(order_copy.legs), order_copy.symbol, strategy_type, order_copy.order_type, order_copy.price)
                    else:
                        allowed = False
                        reasons.append(f"Estrategia {strategy_type} requiere legs pero el probability genero {len(legs)} (minimo 2).")
                        logger.warning("[evaluate_candidate] Insufficient legs (%d) for %s (%s) — blocking execution", len(legs), order_copy.symbol, strategy_type)
                if allowed and not is_close_order and uses_option_bp and action in {"preview", "submit"}:
                    if option_bp <= 0.0:
                        allowed = False
                        reasons.append("Option buying power es 0. La apertura de opciones queda bloqueada.")
                    elif risk_dollars is not None and risk_dollars > option_bp:
                        allowed = False
                        reasons.append(
                            f"Risk budget insuficiente: requiere {risk_dollars:.2f} y option buying power disponible es {option_bp:.2f}."
                        )
                if allowed:
                    execution_preflight = self._build_execution_preflight(order=order_copy, action=action)
                    warnings.extend(list(execution_preflight.get("warnings") or []))
                    if execution_preflight.get("blocked"):
                        allowed = False
                        reasons.extend(
                            [f"Execution preflight blocked: {issue}" for issue in (execution_preflight.get("critical_issues") or [])]
                        )
                if not allowed:
                    logger.info("[evaluate_candidate] Skipping executor — order already blocked for %s", order_copy.symbol)
                else:
                    try:
                        execution_started = time.perf_counter()
                        execution_result = self.executor.execute(order=order_copy, action=action, mode=str(config.get("executor_mode") or "disabled"))
                        evaluation_timings["executor_sec"] = round(time.perf_counter() - execution_started, 4)
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
        equity = _safe_float(balances.get("total_equity"), _safe_float(balances.get("equity"), 0.0))
        bp_base = max(option_bp, 0.0) if (uses_option_bp and not is_close_order) else max(option_bp, cash, 0.0)
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
                "capital_guard": {
                    "status": "evaluated" if estimated_open_notional is not None else "not_applicable",
                    "estimated_open_notional": estimated_open_notional,
                    "available_buying_power": round(max(option_bp, cash, 0.0), 2),
                    "blocked": any("buying power" in str(reason).lower() for reason in reasons),
                },
                "market_context": {
                    "status": market_context_gate.get("status"),
                    "blocked": bool(market_context_gate.get("blocked")),
                    "degraded": bool(market_context_gate.get("degraded")),
                },
                "visual_entry_gate": {
                    "status": visual_entry_gate.get("status"),
                    "degraded": bool(visual_entry_gate.get("degraded")),
                },
                "execution_preflight": {
                    "status": execution_preflight.get("status"),
                    "blocked": bool(execution_preflight.get("blocked")),
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
            "probability_source": probability_source,
            "vision_snapshot": context_snapshot,
            "entry_validation": entry_validation,
            "market_context": order_copy.market_context or {},
            "market_context_gate": market_context_gate,
            "visual_entry_gate": visual_entry_gate,
            "execution_preflight": execution_preflight,
            "execution_quality": execution_quality,
            "evaluation_timings": evaluation_timings,
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
        config["last_options_strategy_governance"] = deepcopy(order_copy.options_governance or {})
        config = self._record_visual_gate_metrics(
            state=config,
            gate=visual_entry_gate,
            order=order_copy,
            action=action,
        )
        config["last_decision"] = {
            "decision": decision,
            "allowed": allowed,
            "timestamp": payload["generated_at"],
            "reason": reasons[0] if reasons else "ok",
        }
        payload["evaluation_timings"]["total_sec"] = round(time.perf_counter() - started_at, 4)
        stage_timings = {
            key: float(value)
            for key, value in payload["evaluation_timings"].items()
            if key != "total_sec" and value is not None
        }
        dominant_stage = None
        dominant_stage_sec = 0.0
        if stage_timings:
            dominant_stage, dominant_stage_sec = max(stage_timings.items(), key=lambda item: item[1])
        total_sec = float(payload["evaluation_timings"]["total_sec"])
        payload["evaluation_profile"] = {
            "dominant_stage": dominant_stage,
            "dominant_stage_sec": round(dominant_stage_sec, 4) if dominant_stage is not None else 0.0,
            "dominant_stage_share_pct": round((dominant_stage_sec / total_sec) * 100.0, 2) if total_sec > 0 and dominant_stage is not None else 0.0,
            "stages_over_1s": [name for name, value in stage_timings.items() if value >= 1.0],
            "nonzero_stages": [name for name, value in stage_timings.items() if value > 0.0],
            "captured_stage_count": len(stage_timings),
        }
        self._save(config)
        payload["operation_status"] = self._lightweight_operation_status(
            scope=scope,
            config=config,
            monitor=monitor,
            vision_status=vision_status,
            executor_status=executor_status,
        )
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

    def _build_visual_entry_gate(
        self,
        *,
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
        capture_context: bool,
        vision_status: dict[str, Any],
    ) -> dict[str, Any]:
        chart_plan = order.chart_plan or {}
        camera_plan = order.camera_plan or {}
        chart_requested = bool(chart_plan.get("targets"))
        camera_required = bool(camera_plan.get("required"))
        gate = {
            "status": "not_requested",
            "applies": bool(chart_requested or camera_required),
            "degraded": False,
            "manual_required": False,
            "operator_review_required": False,
            "blocking_ready": True,
            "blocking_reason": None,
            "readiness_score_pct": 100.0 if not (chart_requested or camera_required) else 0.0,
            "readiness_components": {},
            "reasons": [],
            "warnings": [],
            "chart_requested": chart_requested,
            "camera_required": camera_required,
            "camera_provider": camera_plan.get("provider") or vision_status.get("provider"),
            "camera_provider_ready": bool(vision_status.get("provider_ready")),
            "chart_execution": {},
        }

        if not chart_requested and not camera_required:
            return gate

        chart_execution = self.chart_execution.ensure_chart_mission(
            chart_plan=chart_plan,
            camera_plan=camera_plan,
            symbol=str(order.symbol or ""),
        )
        gate["chart_execution"] = chart_execution
        chart_score = 100.0
        if chart_requested:
            if bool(chart_execution.get("open_ok")) and bool(chart_execution.get("symbol_match")) and bool(chart_execution.get("timeframe_match")):
                chart_score = float(chart_execution.get("readiness_score_pct") or 100.0)
            elif bool(chart_execution.get("manual_required")):
                chart_score = float(chart_execution.get("readiness_score_pct") or 35.0)
            else:
                chart_score = float(chart_execution.get("readiness_score_pct") or 20.0)

        camera_provider_ready = bool(vision_status.get("provider_ready"))
        camera_score = 100.0
        context_score = 100.0
        if camera_required:
            camera_score = 100.0 if camera_provider_ready else 0.0
            context_score = 100.0 if capture_context else 0.0

        weight_total = 0.0
        weighted_sum = 0.0
        if chart_requested:
            weight_total += 55.0
            weighted_sum += chart_score * 55.0
        if camera_required:
            weight_total += 25.0
            weighted_sum += camera_score * 25.0
            weight_total += 20.0
            weighted_sum += context_score * 20.0

        readiness_score = round((weighted_sum / weight_total), 2) if weight_total > 0 else 100.0
        gate["readiness_score_pct"] = readiness_score
        gate["readiness_components"] = {
            "chart_score_pct": round(chart_score, 2) if chart_requested else None,
            "camera_provider_score_pct": round(camera_score, 2) if camera_required else None,
            "context_capture_score_pct": round(context_score, 2) if camera_required else None,
        }

        if chart_requested and not chart_execution.get("open_ok"):
            gate["degraded"] = True
            gate["warnings"].append("chart mission is defined but chart opening is not confirmed.")
            gate["manual_required"] = True

        if camera_required and action in {"preview", "submit"} and not capture_context:
            gate["degraded"] = True
            gate["warnings"].append("camera validation requested but capture_context=False skipped visual confirmation.")
            gate["manual_required"] = True

        if camera_required and not camera_provider_ready:
            gate["degraded"] = True
            gate["warnings"].append("camera provider is not ready for visual confirmation.")
            gate["manual_required"] = True

        if chart_requested and not bool(chart_execution.get("symbol_match")):
            gate["degraded"] = True
            gate["warnings"].append("chart mission opened but symbol match is not confirmed.")
            gate["manual_required"] = True

        if chart_requested and not bool(chart_execution.get("timeframe_match")):
            gate["degraded"] = True
            gate["warnings"].append("chart mission opened but timeframe match is not confirmed.")
            gate["manual_required"] = True

        if bool(chart_execution.get("manual_required")):
            gate["manual_required"] = True

        if gate["degraded"] or gate["manual_required"]:
            gate["operator_review_required"] = True

        chart_state = str(chart_execution.get("execution_state") or chart_execution.get("open_mode") or "")
        if not gate["degraded"] and not gate["manual_required"]:
            gate["status"] = "visual_ready"
        elif camera_required and not camera_provider_ready:
            gate["status"] = "camera_unavailable"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "camera provider is not ready"
        elif camera_required and action in {"preview", "submit"} and not capture_context:
            gate["status"] = "manual_review"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "camera validation was skipped"
        elif chart_requested and chart_state == "manual_required":
            gate["status"] = "manual_required"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "chart mission requires manual opening"
        elif chart_requested and chart_state == "browser_unavailable":
            gate["status"] = "chart_pending"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "browser is unavailable for chart mission"
        elif chart_requested and not bool(chart_execution.get("open_ok")):
            gate["status"] = "chart_pending"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "chart opening is not confirmed"
        else:
            gate["status"] = "manual_review"
            gate["blocking_ready"] = False
            gate["blocking_reason"] = "visual checks require operator review"

        return gate

    def _finalize_visual_entry_gate(
        self,
        *,
        gate: dict[str, Any],
        order: OrderRequest,
        action: Literal["evaluate", "preview", "submit"],
        auton_mode: str,
        context_snapshot: dict[str, Any] | None,
        scope: str = "",
    ) -> dict[str, Any]:
        payload = deepcopy(gate)
        payload.setdefault("reasons", [])
        payload.setdefault("warnings", [])

        camera_plan = order.camera_plan or {}
        camera_required = bool(payload.get("camera_required"))
        context = context_snapshot or {}
        capture_requested = bool(payload.get("context_capture_requested"))
        capture_ok = bool(payload.get("context_capture_ok"))
        evidence_present = any(bool(context.get(key)) for key in ("capture_path", "resource_id", "meta_path"))
        ocr_payload = context.get("ocr") if isinstance(context.get("ocr"), dict) else {}
        ocr_confidence = _safe_float((ocr_payload or {}).get("confidence"), 0.0)
        if 0.0 < ocr_confidence <= 1.0:
            ocr_confidence *= 100.0
        gaze_payload = context.get("gaze") if isinstance(context.get("gaze"), dict) else {}
        gaze_ok = bool(gaze_payload) and not bool(gaze_payload.get("error"))
        camera_fit_pct = _safe_float(camera_plan.get("visual_fit_pct"), 0.0)
        visual_alignment = self._visual_alignment_snapshot(
            camera_plan=camera_plan,
            ocr_payload=ocr_payload or {},
        )
        validation_profile = camera_plan.get("validation_profile") if isinstance(camera_plan.get("validation_profile"), dict) else {}
        min_ocr_conf_pct = _safe_float(validation_profile.get("min_ocr_confidence_pct"), 0.0)
        min_alignment_score_pct = _safe_float(validation_profile.get("min_alignment_score_pct"), 70.0)
        require_pattern_confirmation = bool(validation_profile.get("require_pattern_confirmation"))
        require_price_evidence = bool(validation_profile.get("require_price_evidence"))

        evidence_score = 100.0
        if camera_required:
            if capture_requested and capture_ok:
                evidence_score = 100.0 if evidence_present else 70.0
            elif capture_requested and not capture_ok:
                evidence_score = 0.0
            elif not capture_requested:
                evidence_score = 0.0
            if ocr_confidence > 0:
                evidence_score = max(evidence_score, min(100.0, ocr_confidence))

        base_score = _safe_float(payload.get("readiness_score_pct"), 100.0)
        if camera_required:
            fit_score = 100.0 if camera_fit_pct <= 0 else min(100.0, camera_fit_pct)
            payload["readiness_score_pct"] = round((base_score * 0.60) + (fit_score * 0.15) + (evidence_score * 0.25), 2)
        else:
            payload["readiness_score_pct"] = round(base_score, 2)

        payload["context_evidence"] = {
            "provider": context.get("provider"),
            "capture_ok": capture_ok,
            "capture_path_present": bool(context.get("capture_path")),
            "resource_id_present": bool(context.get("resource_id")),
            "meta_path_present": bool(context.get("meta_path")),
            "evidence_present": evidence_present,
            "ocr_confidence_pct": round(ocr_confidence, 2) if ocr_confidence > 0 else None,
            "ocr_pattern": (ocr_payload or {}).get("pattern"),
            "ocr_chart_bias": (ocr_payload or {}).get("chart_color"),
            "gaze_ok": gaze_ok,
            "validation_profile": validation_profile,
            "visual_alignment": visual_alignment,
        }

        if camera_required and camera_fit_pct > 0 and camera_fit_pct < 70.0:
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["blocking_ready"] = False
            payload["status"] = "manual_review"
            payload["blocking_reason"] = (
                f"camera visual fit {camera_fit_pct:.1f}% is below the minimum robust threshold"
            )
            payload["warnings"].append(
                f"camera visual fit ({camera_fit_pct:.1f}%) is below the minimum robust threshold for entry timing."
            )

        if camera_required and capture_requested and not capture_ok:
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["blocking_ready"] = False
            payload["status"] = "camera_unavailable"
            payload["blocking_reason"] = "visual context capture failed"

        if camera_required and capture_requested and capture_ok and not evidence_present:
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["warnings"].append("visual capture succeeded but no persistent capture evidence was recorded.")

        if visual_alignment.get("checked") and not bool(visual_alignment.get("confirmed")):
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["warnings"].append(
                "OCR visual alignment does not confirm the expected directional thesis for this setup."
            )
            if _safe_float(visual_alignment.get("alignment_score_pct"), 0.0) < min_alignment_score_pct:
                payload["blocking_ready"] = False
                payload["status"] = "manual_review"
                payload["blocking_reason"] = "visual thesis is misaligned with OCR evidence"

        if camera_required and min_ocr_conf_pct > 0 and 0.0 < ocr_confidence < min_ocr_conf_pct:
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["blocking_ready"] = False
            payload["status"] = "manual_review"
            payload["blocking_reason"] = (
                f"OCR confidence {ocr_confidence:.2f}% is below the required threshold ({min_ocr_conf_pct:.2f}%)"
            )
            payload["warnings"].append(
                f"OCR confidence ({ocr_confidence:.2f}%) is below the required threshold for this setup."
            )

        if require_pattern_confirmation and camera_required and capture_requested and capture_ok and not str((ocr_payload or {}).get("pattern") or "").strip():
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["warnings"].append("Pattern confirmation is required for this setup but OCR did not return a pattern.")

        if require_price_evidence and camera_required and capture_requested and capture_ok and not list((ocr_payload or {}).get("prices") or []):
            payload["degraded"] = True
            payload["manual_required"] = True
            payload["operator_review_required"] = True
            payload["warnings"].append("Price evidence is required for this setup but OCR did not detect visible prices.")

        supervised_manual_chart_preview = (
            action == "preview"
            and str(auton_mode or "").lower() in {"paper_supervised", "paper_autonomous"}
            and bool(settings.visual_gate_supervised_manual_chart_preview)
            and str(payload.get("blocking_reason") or "") == "chart mission requires manual opening"
        )
        if supervised_manual_chart_preview:
            payload["blocking_ready"] = True
            payload["status"] = "manual_assist_preview"
            payload["warnings"].append(
                "chart mission requires manual opening, but supervised preview is allowed for operator-assisted validation."
            )

        if (
            action in {"preview", "submit"}
            and bool(settings.visual_gate_fail_closed)
            and bool(payload.get("applies"))
        ):
            if scope == "paper":
                payload["warnings"].append(
                    f"Visual gate would block {action} ({payload.get('blocking_reason') or 'not ready'}) — bypassed in paper mode."
                )
            else:
                threshold = _safe_float(settings.visual_gate_min_readiness_pct, 75.0)
                if not bool(payload.get("blocking_ready")):
                    reason = str(payload.get("blocking_reason") or "visual gate is not ready")
                    payload["reasons"].append(f"Visual gate blocked {action}: {reason}.")
                elif not supervised_manual_chart_preview and _safe_float(payload.get("readiness_score_pct"), 0.0) < threshold:
                    payload["degraded"] = True
                    payload["manual_required"] = True
                    payload["operator_review_required"] = True
                    payload["blocking_ready"] = False
                    payload["status"] = "manual_review"
                    payload["blocking_reason"] = (
                        f"visual readiness score {_safe_float(payload.get('readiness_score_pct'), 0.0):.2f}% "
                        f"is below the required threshold ({threshold:.2f}%)"
                    )
                    payload["reasons"].append(f"Visual gate blocked {action}: {payload['blocking_reason']}.")

        return payload

    @staticmethod
    def _visual_alignment_snapshot(
        *,
        camera_plan: dict[str, Any],
        ocr_payload: dict[str, Any],
    ) -> dict[str, Any]:
        expected = camera_plan.get("expected_visual") if isinstance(camera_plan.get("expected_visual"), dict) else {}
        pattern = str(ocr_payload.get("pattern") or "").lower()
        chart_bias = str(ocr_payload.get("chart_color") or "").lower()
        expected_bias = str(expected.get("expected_chart_bias") or "").lower()
        expected_patterns = [str(item).lower() for item in (expected.get("expected_patterns") or []) if str(item).strip()]
        if not expected or (not pattern and not chart_bias):
            return {
                "checked": False,
                "confirmed": None,
                "alignment_score_pct": None,
                "expected_chart_bias": expected_bias or None,
                "observed_chart_bias": chart_bias or None,
                "observed_pattern": pattern or None,
            }

        bias_score = 50.0
        if expected_bias:
            bias_score = 100.0 if chart_bias == expected_bias else 0.0 if chart_bias else 50.0

        pattern_score = 50.0
        if expected_patterns:
            pattern_score = 100.0 if any(token in pattern for token in expected_patterns) else 0.0 if pattern else 50.0

        weights = []
        weighted = []
        if expected_bias:
            weights.append(0.55)
            weighted.append(bias_score * 0.55)
        if expected_patterns:
            weights.append(0.45)
            weighted.append(pattern_score * 0.45)
        alignment_score = round(sum(weighted) / sum(weights), 2) if weights else 50.0

        return {
            "checked": True,
            "confirmed": alignment_score >= 70.0,
            "alignment_score_pct": alignment_score,
            "expected_chart_bias": expected_bias or None,
            "observed_chart_bias": chart_bias or None,
            "observed_pattern": pattern or None,
            "bias_score_pct": round(bias_score, 2),
            "pattern_score_pct": round(pattern_score, 2),
        }
