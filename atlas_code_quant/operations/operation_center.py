"""Operational control plane for Atlas Code-Quant."""
from __future__ import annotations

import json
import time
import threading
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

from api.schemas import OrderRequest
from backtesting.winning_probability import StrategyLeg, _capital_at_risk, _safe_float, get_winning_probability
from config.settings import settings
from learning.adaptive_policy import AdaptiveLearningService
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
    ) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "operation_center_state.json")
        self.tracker = tracker or StrategyTracker()
        self.journal = journal or JournalProService()
        self.vision = vision or SensorVisionService()
        self.executor = executor or AutonExecutorService()
        self.brain = brain or QuantBrainBridge()
        self.learning = learning or AdaptiveLearningService()
        self._monitor_summary_lock = threading.Lock()
        self._monitor_summary_cache: dict[str, dict[str, Any]] = {}
        self._monitor_summary_cache_at: dict[str, float] = {}
        self._monitor_summary_workers: dict[str, threading.Thread] = {}
        self._monitor_summary_errors: dict[str, str] = {}
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

    def _dispatch_brain_task(self, task_name: str, callback: Any, /, *args: Any, **kwargs: Any) -> None:
        def _runner() -> None:
            try:
                callback(*args, **kwargs)
            except Exception:
                pass

        threading.Thread(target=_runner, name=f"brain-{task_name}", daemon=True).start()

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
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        journal_snapshot = self.journal.snapshot(limit=3)
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
        monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
        balances = monitor.get("balances") or {}
        pdt_status = monitor.get("pdt_status") or {}
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

        risk_dollars = None
        bpr_pct = None
        if strategy_type:
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
        else:
            warnings.append("strategy_type is missing; autonomous gate is operating without Monte Carlo validation.")

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

        projected_position_count = len(monitor.get("strategies") or []) + (1 if action in {"preview", "submit"} and allowed else 0)
        option_bp = _safe_float(balances.get("option_buying_power"), 0.0)
        cash = _safe_float(balances.get("cash"), 0.0)
        equity = _safe_float(balances.get("equity"), 0.0)
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
            "monitor_summary": {
                "account_session": monitor.get("account_session"),
                "balances": balances,
                "totals": monitor.get("totals") or {},
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
