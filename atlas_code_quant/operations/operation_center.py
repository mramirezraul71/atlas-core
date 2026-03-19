"""Operational control plane for Atlas Code-Quant."""
from __future__ import annotations

import json
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

from api.schemas import OrderRequest
from backtesting.winning_probability import StrategyLeg, _capital_at_risk, _safe_float, get_winning_probability
from config.settings import settings
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
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
    "vision_mode": "manual",
    "require_operator_present": False,
    "operator_present": True,
    "screen_integrity_ok": True,
    "sentiment_score": 0.0,
    "sentiment_source": "manual",
    "min_auton_win_rate_pct": 65.0,
    "max_level4_bpr_pct": 20.0,
    "notes": "Plano de control orientado a simulada",
    "last_decision": None,
    "last_candidate": None,
}


class OperationCenter:
    def __init__(
        self,
        *,
        tracker: StrategyTracker | None = None,
        journal: JournalProService | None = None,
        vision: SensorVisionService | None = None,
        executor: AutonExecutorService | None = None,
        state_path: Path | None = None,
    ) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "operation_center_state.json")
        self.tracker = tracker or StrategyTracker()
        self.journal = journal or JournalProService()
        self.vision = vision or SensorVisionService()
        self.executor = executor or AutonExecutorService()
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
                return merged
        except Exception:
            pass
        return deepcopy(_DEFAULT_STATE)

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        self.state_path.write_text(json.dumps(merged, ensure_ascii=True, indent=2), encoding="utf-8")
        return merged

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
        monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
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
            "journal": {
                "recent_entries_count": journal_snapshot.get("recent_entries_count", 0),
                "recent_entries": journal_snapshot.get("recent_entries", []),
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
        return self.status()

    def clear_emergency_stop(self) -> dict[str, Any]:
        self.executor.clear_emergency_stop()
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
        allowed = not reasons
        execution_result: dict[str, Any] | None = None
        if allowed and action in {"preview", "submit"}:
            if scope != "paper" and bool(config.get("paper_only", True)):
                allowed = False
                reasons.append("La ejecucion real sigue deshabilitada mientras solo simulada este activa.")
            else:
                order_copy.preview = action != "submit"
                execution_result = self.executor.execute(order=order_copy, action=action, mode=str(config.get("executor_mode") or "disabled"))
                if execution_result.get("decision") == "blocked":
                    allowed = False
                    reasons.append(str(execution_result.get("reason") or "El ejecutor bloqueo la operacion."))

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
        state = self._load()
        state["last_candidate"] = {
            "symbol": order_copy.symbol,
            "strategy_type": strategy_type,
            "scope": scope,
            "action": action,
        }
        state["last_decision"] = {
            "decision": decision,
            "allowed": allowed,
            "timestamp": payload["generated_at"],
            "reason": reasons[0] if reasons else "ok",
        }
        self._save(state)
        payload["operation_status"] = self.status()
        return payload
