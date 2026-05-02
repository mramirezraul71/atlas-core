"""Operational control plane for Atlas Code-Quant."""
from __future__ import annotations

import json
import logging
import os
import time
import threading
import pytz
from copy import deepcopy
from datetime import datetime, timedelta
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
from learning.trading_self_audit_protocol import read_trading_self_audit_protocol
from monitoring.strategy_tracker import StrategyTracker
from operations.auton_executor import AutonExecutorService
from operations.brain_bridge import QuantBrainBridge
from operations.chart_execution import ChartExecutionService
from operations.kill_switch import kill_switch_health
from operations.live_order_routing import guarded_route_to_payload, resolve_guarded_order_route
from operations.live_switch import live_switch_state_to_payload, resolve_live_switch_state
from operations.runtime_config_v4 import validate_runtime_config_v4
from operations.runtime_mode import RuntimeMode, resolve_runtime_mode, runtime_resolution_to_event
from operations.operation_state_contract import (
    CORE_STATE_CONTRACT_KEYS,
    build_core_contract_payload,
    write_json_atomic as write_contract_json_atomic,
)
from operations.journal_pro import JournalProService
from operations.sensor_vision import SensorVisionService
from scanner.options_flow_provider import MarketTelemetryStore
from scanner.asset_classifier import AssetClass, classify_asset

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
    "signal_validator_enabled": True,
    "var_monitor_enabled": True,
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
    "post_journal_rebuild_guard": {
        "active": False,
        "scope": None,
        "activated_at": None,
        "expires_at": None,
        "reason": None,
    },
    # --- Paper-only governance toggles (aditivo / seguro) ---
    # Permite que ciertas guardas de riesgo que bloquean "submit" en live
    # se degraden a warning en paper para mantener el loop activo.
    "paper_bypass_exit_now_guard": False,
    # Permite degradar bloqueos de entry validation (spread/drift) a warning en paper.
    "paper_bypass_entry_validation_guard": False,
    # Si True (paper): bloquea submit fuera de sesión regular US (9:30–16:00 ET); no afecta preview.
    "paper_market_hours_strict": False,
    # Telegram al enviar órdenes paper y seguimiento de fills (si hay token/chat).
    "paper_telegram_trade_alerts": True,
    # ---- Runtime mode v4 / transition to live (live-capable, live-locked) ----
    "deployment_mode": "dev",
    "allowed_runtime_modes": [
        "paper_baseline",
        "paper_aggressive",
        "supervised_live",
        "guarded_live",
    ],
    "live_unlock_policy": "human_approval_required",
    "runtime_mode_requested": "paper_aggressive",
    "strategy_mode_overrides": {},
    "symbol_mode_overrides": {},
    "require_human_unlock": True,
    "require_dual_confirmation": False,
    "full_live_globally_locked": True,
    "live_capable": True,
    "live_ready": False,
    "live_unlock_requested": False,
    "live_unlock_approved": False,
    "live_guardrails_healthy": True,
    "live_switch_state": {},
    "runtime_mode_resolved": {},
}


def _paper_dry_run_enabled() -> bool:
    raw = os.getenv("ATLAS_TRADIER_DRY_RUN", "true").strip().lower()
    return raw not in {"0", "false", "no", "off"}

PDT_RULE_REMOVAL_DATE = "2026-04-14"
MIN_CAPITAL_OPERATIONAL = 500.0
MIN_CAPITAL_RECOMMENDED = 2000.0
MAX_DAILY_TRADES_LIVE = 50
MAX_DAILY_TRADES_PAPER = 100
MAX_INTRADAY_EXPOSURE_PCT = 0.50
MAX_INTRADAY_DRAWDOWN_PCT = 0.10
CONSECUTIVE_LOSS_COOLDOWN_STREAK = 3
CONSECUTIVE_LOSS_LOOKBACK = 5


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
        core_state_path: Path | None = None,
        reconciliation_provider: Callable[..., dict[str, Any]] | None = None,
        quote_provider: Callable[..., dict[str, Any] | None] | None = None,
        scorecard_provider: Callable[[], dict[str, Any]] | None = None,
        chart_execution: ChartExecutionService | None = None,
    ) -> None:
        base_dir = settings.data_dir.parent / "operation"
        base_dir.mkdir(parents=True, exist_ok=True)
        self.state_path = state_path or (base_dir / "operation_center_state.json")
        self.root_path = Path(__file__).resolve().parents[2]
        self.core_state_path = core_state_path
        if self.core_state_path is None and state_path is None:
            self.core_state_path = self.root_path / "data" / "operation" / "operation_center_state.json"
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
                normalized = self._normalize_runtime_state(merged)
                return self._merge_core_contract(normalized)
        except Exception:
            pass
        return self._merge_core_contract(self._normalize_runtime_state(deepcopy(_DEFAULT_STATE)))

    def _core_contract_path(self) -> Path | None:
        if self.core_state_path is None:
            return None
        try:
            if self.core_state_path.resolve() == self.state_path.resolve():
                return None
        except Exception:
            if self.core_state_path == self.state_path:
                return None
        return self.core_state_path

    def _load_core_contract(self) -> dict[str, Any]:
        path = self._core_contract_path()
        if path is None or not path.exists():
            return {}
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            logger.warning("Unable to read core operation contract: %s", path, exc_info=True)
            return {}
        if not isinstance(payload, dict):
            return {}
        if payload.get("source_module") != "atlas_core" or int(payload.get("contract_version") or 0) < 1:
            return {}
        return payload

    def _merge_core_contract(self, state: dict[str, Any]) -> dict[str, Any]:
        contract = self._load_core_contract()
        if not contract:
            return state
        merged = dict(state)
        for key in CORE_STATE_CONTRACT_KEYS:
            if key in contract and contract[key] is not None:
                merged[key] = contract[key]
        return self._normalize_runtime_state(merged)

    def _core_contract_payload(self, state: dict[str, Any]) -> dict[str, Any]:
        return build_core_contract_payload(state, source_module="atlas_code_quant")

    @staticmethod
    def _write_json_atomic(path: Path, payload: dict[str, Any]) -> None:
        write_contract_json_atomic(path, payload, ensure_ascii=True)

    def _publish_core_contract(self, state: dict[str, Any]) -> None:
        path = self._core_contract_path()
        if path is None:
            return
        try:
            self._write_json_atomic(path, self._core_contract_payload(state))
        except Exception:
            logger.warning("Unable to publish core operation contract: %s", path, exc_info=True)

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
        rebuild_guard = state.get("post_journal_rebuild_guard")
        if not isinstance(rebuild_guard, dict):
            rebuild_guard = {}
        normalized_rebuild_guard = deepcopy(_DEFAULT_STATE["post_journal_rebuild_guard"])
        normalized_rebuild_guard.update(rebuild_guard)
        normalized_rebuild_guard["active"] = bool(normalized_rebuild_guard.get("active"))
        state["post_journal_rebuild_guard"] = normalized_rebuild_guard

        # Paper-only governance toggles
        state["paper_bypass_exit_now_guard"] = bool(state.get("paper_bypass_exit_now_guard", False))
        state["paper_bypass_entry_validation_guard"] = bool(state.get("paper_bypass_entry_validation_guard", False))
        state["paper_market_hours_strict"] = bool(state.get("paper_market_hours_strict", False))
        state["paper_telegram_trade_alerts"] = bool(state.get("paper_telegram_trade_alerts", True))
        state["deployment_mode"] = str(state.get("deployment_mode") or "dev").strip().lower()
        state["allowed_runtime_modes"] = list(state.get("allowed_runtime_modes") or _DEFAULT_STATE["allowed_runtime_modes"])
        state["live_unlock_policy"] = str(state.get("live_unlock_policy") or "human_approval_required")
        state["runtime_mode_requested"] = str(state.get("runtime_mode_requested") or "paper_aggressive")
        state["strategy_mode_overrides"] = dict(state.get("strategy_mode_overrides") or {})
        state["symbol_mode_overrides"] = dict(state.get("symbol_mode_overrides") or {})
        state["require_human_unlock"] = bool(state.get("require_human_unlock", True))
        state["require_dual_confirmation"] = bool(state.get("require_dual_confirmation", False))
        state["full_live_globally_locked"] = bool(state.get("full_live_globally_locked", True))
        state["live_capable"] = bool(state.get("live_capable", True))
        state["live_ready"] = bool(state.get("live_ready", False))
        state["live_unlock_requested"] = bool(state.get("live_unlock_requested", False))
        state["live_unlock_approved"] = bool(state.get("live_unlock_approved", False))
        state["live_guardrails_healthy"] = bool(state.get("live_guardrails_healthy", True))
        state["runtime_mode_resolved"] = dict(state.get("runtime_mode_resolved") or {})
        state["live_switch_state"] = dict(state.get("live_switch_state") or {})
        try:
            validated_v4 = validate_runtime_config_v4(
                {
                    "deployment_mode": state["deployment_mode"],
                    "allowed_runtime_modes": state["allowed_runtime_modes"],
                    "live_unlock_policy": state["live_unlock_policy"],
                    "strategy_mode_overrides": state["strategy_mode_overrides"],
                    "symbol_mode_overrides": state["symbol_mode_overrides"],
                    "require_human_unlock": state["require_human_unlock"],
                    "require_dual_confirmation": state["require_dual_confirmation"],
                    "full_live_globally_locked": state["full_live_globally_locked"],
                }
            )
            state["allowed_runtime_modes"] = list(validated_v4.allowed_runtime_modes)
        except Exception:
            state["allowed_runtime_modes"] = list(_DEFAULT_STATE["allowed_runtime_modes"])
            state["deployment_mode"] = "dev"
        auton_mode = str(state.get("auton_mode") or "off").strip().lower()
        if auton_mode not in {"off", "paper_supervised", "paper_autonomous", "paper_aggressive"}:
            auton_mode = "off"
        state["auton_mode"] = auton_mode

        # ---- Autonomy Orchestrator compatibility (paper_autonomous) ----
        # Keys expected by atlas_core/autonomy/adapters/quant_adapter.py
        state.setdefault("autonomy_mode", "semi")
        if state.get("autonomy_mode") is None:
            state["autonomy_mode"] = "semi"

        max_risk_default = float(getattr(settings, "equity_kelly_max_risk_per_trade_pct", 0.02) or 0.02)
        raw_max_risk = state.get("max_risk_per_trade_pct", max_risk_default)
        try:
            state["max_risk_per_trade_pct"] = float(raw_max_risk)
        except Exception:
            state["max_risk_per_trade_pct"] = max_risk_default

        return state

    @staticmethod
    def _apply_paper_risk_guard_bypass(
        *,
        scope: str,
        action: str,
        config: dict[str, Any],
        portfolio_risk_guard: dict[str, Any],
        reasons: list[str],
        warnings: list[str],
    ) -> None:
        """
        En paper, permite degradar ciertos bloqueos a warnings para no congelar el loop.
        NO afecta a live.
        """
        if scope != "paper" or action != "submit":
            return
        if not bool(config.get("paper_bypass_exit_now_guard", False)):
            return
        guard_reasons = list(portfolio_risk_guard.get("reasons") or [])
        if not guard_reasons:
            return
        # Identifica el bloqueo específico por exit_now
        exit_now_reasons = [r for r in guard_reasons if "exit_now" in str(r).lower()]
        if not exit_now_reasons:
            return

        # Remueve del listado de razones finales y lo convierte en warning
        for r in exit_now_reasons:
            if r in reasons:
                reasons.remove(r)
            warnings.append(f"[paper bypass] {r}")

        portfolio_risk_guard["blocked"] = False
        portfolio_risk_guard["degraded"] = True
        portfolio_risk_guard["status"] = "warning"
        portfolio_risk_guard["paper_bypass_exit_now_guard"] = True

    @staticmethod
    def _apply_paper_entry_validation_bypass(
        *,
        scope: str,
        action: str,
        config: dict[str, Any],
        entry_validation: dict[str, Any],
        reasons: list[str],
        warnings: list[str],
    ) -> None:
        """
        En paper, degrada bloqueos de spread/drift a warning para mantener
        la continuidad de simulación en mercados volátiles.
        """
        if scope != "paper" or action != "submit":
            return
        if not bool(config.get("paper_bypass_entry_validation_guard", False)):
            return

        ev_reasons = list(entry_validation.get("reasons") or [])
        if not ev_reasons:
            return
        bypassable = [
            r for r in ev_reasons
            if ("spread is" in str(r).lower()) or ("adverse drift is" in str(r).lower())
        ]
        if not bypassable:
            return

        for r in bypassable:
            if r in reasons:
                reasons.remove(r)
            warnings.append(f"[paper bypass] {r}")

        entry_validation["blocked"] = False
        entry_validation["status"] = "warning"
        entry_validation["paper_bypass_entry_validation_guard"] = True

    def _save(self, payload: dict[str, Any]) -> dict[str, Any]:
        merged = deepcopy(_DEFAULT_STATE)
        merged.update(payload or {})
        merged = self._normalize_runtime_state(merged)
        self._write_json_atomic(self.state_path, merged)
        self._publish_core_contract(merged)
        self._invalidate_status_cache()
        return merged

    def _check_market_hours(self, symbol: str, account_scope: str) -> tuple[bool, str]:
        """Validate market hours for a symbol and scope."""
        symbol_upper = str(symbol or "").strip().upper()
        if not symbol_upper:
            return False, "missing_symbol"
        scope = str(account_scope or "paper").strip().lower() or "paper"
        if scope not in {"paper", "live"}:
            scope = "paper"

        if symbol_upper in {"BTC", "ETH", "SOL", "ADA", "DOT", "MATIC", "AVAX", "LINK", "UNI", "DOGE", "XRP", "BNB"}:
            return True, "crypto_market_24h"

        try:
            profile = classify_asset(symbol_upper)
            if profile.asset_class == AssetClass.CRYPTO:
                return True, "crypto_market_24h"
        except Exception:
            logger.debug("Asset classifier failed for market hours check: %s", symbol_upper, exc_info=True)

        et = pytz.timezone("America/New_York")
        now_et = datetime.now(et)
        weekday = now_et.weekday()
        if weekday > 4:
            return False, "market_closed_weekend"

        # Sesión regular US: 09:30–16:00 ET (sin extended hours).
        open_dt = now_et.replace(hour=9, minute=30, second=0, microsecond=0)
        close_dt = now_et.replace(hour=16, minute=0, second=0, microsecond=0)
        if now_et < open_dt:
            mins_to_open = int(max((open_dt - now_et).total_seconds(), 0) // 60)
            return False, f"market_not_open_yet ({mins_to_open}min_to_open)"
        if now_et > close_dt:
            return False, "market_closed_for_day"
        return True, "market_hours_ok"

    def market_hours_readiness(self, *, symbol: str | None = None, account_scope: str = "paper") -> dict[str, Any]:
        check_symbol = str(symbol or "SPY").strip().upper() or "SPY"
        normalized_scope = str(account_scope or "paper").strip().lower() or "paper"
        cfg = self._load()
        strict = bool(cfg.get("paper_market_hours_strict"))
        ok, reason = self._check_market_hours(check_symbol, normalized_scope)
        blocked = bool(
            (not ok)
            and (
                normalized_scope == "live"
                or (normalized_scope == "paper" and strict)
            )
        )
        degraded = bool((not ok) and normalized_scope == "paper" and not strict)
        status = "OK" if ok else ("DEGRADED" if degraded else "BLOCKED")
        return {
            "symbol": check_symbol,
            "account_scope": normalized_scope,
            "status": status,
            "market_hours_ok": ok,
            "market_hours_reason": reason,
            "blocked": blocked,
            "degraded": degraded,
            "paper_market_hours_strict": strict,
        }

    def is_us_equity_rth_open(self) -> bool:
        """True si la sesión regular US (referencia SPY) está abierta (ET)."""
        ok, _reason = self._check_market_hours("SPY", "paper")
        return ok

    @staticmethod
    def _extract_equity_value(account: Any) -> float:
        if isinstance(account, dict):
            return _safe_float(account.get("equity"), _safe_float(account.get("total_equity"), 0.0))
        return _safe_float(getattr(account, "equity", None), _safe_float(getattr(account, "total_equity", None), 0.0))

    def check_pdt_limit(self, account_id: str) -> tuple[bool, str]:
        """Deprecated gate kept for compatibility after PDT removal."""
        logger.warning(
            "check_pdt_limit() deprecated: PDT rule removed effective %s (account_id=%s)",
            PDT_RULE_REMOVAL_DATE,
            account_id,
        )
        return True, "pdt_limit_removed"

    def validate_account_capital(self, account: Any) -> tuple[bool, str]:
        """Operational capital validation (non-regulatory)."""
        equity = self._extract_equity_value(account)
        if equity < MIN_CAPITAL_OPERATIONAL:
            logger.warning(
                "Account capital %.2f below operational minimum %.2f",
                equity,
                MIN_CAPITAL_OPERATIONAL,
            )
        if equity < MIN_CAPITAL_RECOMMENDED:
            logger.warning(
                "Account capital %.2f below recommended minimum %.2f",
                equity,
                MIN_CAPITAL_RECOMMENDED,
            )
        return True, "capital_sufficient"

    @staticmethod
    def _entry_day(entry: dict[str, Any]) -> str | None:
        ts = str(entry.get("exit_time") or entry.get("entry_time") or "").strip()
        if len(ts) >= 10:
            return ts[:10]
        return None

    @staticmethod
    def _entry_realized_pnl(entry: dict[str, Any]) -> float:
        return _safe_float(entry.get("realized_pnl"), _safe_float(entry.get("pnl"), 0.0))

    def _journal_recent_items(self, *, limit: int = 400) -> list[dict[str, Any]]:
        try:
            snapshot = self.journal.snapshot(limit=limit)
        except Exception as exc:
            logger.warning("Unable to load journal snapshot for operational gates: %s", exc)
            return []
        items = snapshot.get("items")
        if not isinstance(items, list):
            items = snapshot.get("recent_entries")
        if not isinstance(items, list):
            return []
        return [item for item in items if isinstance(item, dict)]

    def _journal_items_today(self) -> list[dict[str, Any]]:
        today = datetime.utcnow().date().isoformat()
        return [item for item in self._journal_recent_items() if self._entry_day(item) == today]

    def check_daily_trade_limit(self, *, account_scope: str) -> tuple[bool, str, dict[str, Any]]:
        today_items = self._journal_items_today()
        trade_count = len(today_items)
        scope = str(account_scope or "paper").strip().lower()
        limit = MAX_DAILY_TRADES_PAPER if scope == "paper" else MAX_DAILY_TRADES_LIVE
        if trade_count > limit:
            return False, f"daily_trade_limit_reached_{limit}", {
                "trade_count_today": trade_count,
                "trade_limit": limit,
                "remaining": 0,
            }
        return True, "daily_trade_limit_ok", {
            "trade_count_today": trade_count,
            "trade_limit": limit,
            "remaining": max(limit - trade_count, 0),
        }

    def check_intraday_exposure(self, *, equity: float, current_exposure: float, proposed_exposure: float) -> tuple[bool, str, dict[str, Any]]:
        total_exposure = max(current_exposure, 0.0) + max(proposed_exposure, 0.0)
        max_exposure = max(equity, 0.0) * MAX_INTRADAY_EXPOSURE_PCT
        if total_exposure > max_exposure and max_exposure > 0:
            return (
                False,
                f"intraday_exposure_limit_exceeded_{total_exposure:.2f}_{max_exposure:.2f}",
                {
                    "current_exposure": round(current_exposure, 2),
                    "proposed_exposure": round(proposed_exposure, 2),
                    "total_exposure": round(total_exposure, 2),
                    "max_exposure": round(max_exposure, 2),
                },
            )
        return True, "intraday_exposure_ok", {
            "current_exposure": round(current_exposure, 2),
            "proposed_exposure": round(proposed_exposure, 2),
            "total_exposure": round(total_exposure, 2),
            "max_exposure": round(max_exposure, 2),
        }

    def check_intraday_drawdown(self, *, opening_equity: float | None, current_equity: float) -> tuple[bool, str, dict[str, Any]]:
        if opening_equity is None or opening_equity <= 0:
            return True, "intraday_drawdown_skipped_missing_opening_equity", {
                "opening_equity": opening_equity,
                "current_equity": round(current_equity, 2),
                "drawdown_pct": 0.0,
                "max_intraday_drawdown_pct": MAX_INTRADAY_DRAWDOWN_PCT * 100.0,
            }
        drawdown_pct = max((opening_equity - current_equity) / opening_equity, 0.0)
        if drawdown_pct > MAX_INTRADAY_DRAWDOWN_PCT:
            return (
                False,
                f"intraday_drawdown_exceeded_{drawdown_pct:.2%}_{MAX_INTRADAY_DRAWDOWN_PCT:.2%}",
                {
                    "opening_equity": round(opening_equity, 2),
                    "current_equity": round(current_equity, 2),
                    "drawdown_pct": round(drawdown_pct * 100.0, 2),
                    "max_intraday_drawdown_pct": MAX_INTRADAY_DRAWDOWN_PCT * 100.0,
                },
            )
        return True, "intraday_drawdown_ok", {
            "opening_equity": round(opening_equity, 2),
            "current_equity": round(current_equity, 2),
            "drawdown_pct": round(drawdown_pct * 100.0, 2),
            "max_intraday_drawdown_pct": MAX_INTRADAY_DRAWDOWN_PCT * 100.0,
        }

    def check_consecutive_loss_cooldown(self) -> tuple[bool, str, dict[str, Any]]:
        today_items = self._journal_items_today()
        recent = today_items[-CONSECUTIVE_LOSS_LOOKBACK:]
        losses = [self._entry_realized_pnl(item) for item in recent if self._entry_realized_pnl(item) < 0]
        consecutive_losses = 0
        for item in reversed(recent):
            if self._entry_realized_pnl(item) < 0:
                consecutive_losses += 1
            else:
                break
        if consecutive_losses >= CONSECUTIVE_LOSS_COOLDOWN_STREAK:
            logger.error("COOLDOWN: %s consecutive losses detected", consecutive_losses)
            return False, "consecutive_loss_cooldown_active", {
                "recent_trade_count": len(recent),
                "recent_losses_count": len(losses),
                "consecutive_losses": consecutive_losses,
                "cooldown_threshold": CONSECUTIVE_LOSS_COOLDOWN_STREAK,
            }
        return True, "loss_cooldown_ok", {
            "recent_trade_count": len(recent),
            "recent_losses_count": len(losses),
            "consecutive_losses": consecutive_losses,
            "cooldown_threshold": CONSECUTIVE_LOSS_COOLDOWN_STREAK,
        }

    def _resolve_intraday_opening_equity(self, *, state: dict[str, Any], account_scope: str, current_equity: float) -> float | None:
        if current_equity <= 0:
            return None
        scope = str(account_scope or "paper").strip().lower() or "paper"
        day_key = datetime.utcnow().date().isoformat()
        anchors = state.get("intraday_equity_anchor")
        if not isinstance(anchors, dict):
            anchors = {}
        anchor = anchors.get(scope)
        if not isinstance(anchor, dict) or str(anchor.get("day")) != day_key:
            anchor = {"day": day_key, "equity": round(current_equity, 2)}
            anchors[scope] = anchor
            state["intraday_equity_anchor"] = anchors
        return _safe_float(anchor.get("equity"), current_equity)

    def _invalidate_status_cache(self) -> None:
        with self._status_cache_lock:
            self._status_cache_payload = None
            self._status_cache_at = 0.0

    def reset_after_journal_rebuild(self, *, account_scope: str = "paper", guard_hours: float = 12.0) -> dict[str, Any]:
        state = self._load()
        state["last_decision"] = None
        state["last_candidate"] = None
        state["visual_gate_stats"] = deepcopy(_DEFAULT_STATE["visual_gate_stats"])
        now = datetime.utcnow()
        state["post_journal_rebuild_guard"] = {
            "active": str(account_scope or "").lower() == "paper",
            "scope": str(account_scope or "paper").lower() or "paper",
            "activated_at": now.isoformat(),
            "expires_at": (now + timedelta(hours=max(_safe_float(guard_hours, 12.0), 0.25))).isoformat(),
            "reason": "journal_rebuild_reset",
        }
        saved = self._save(state)
        with self._monitor_summary_lock:
            self._monitor_summary_cache.clear()
            self._monitor_summary_cache_at.clear()
            self._monitor_summary_errors.clear()
        with self._scorecard_lock:
            self._scorecard_cache = None
            self._scorecard_cache_at = 0.0
            self._scorecard_error = None
        return {
            "ok": True,
            "account_scope": str(account_scope or "paper").lower() or "paper",
            "post_journal_rebuild_guard": saved.get("post_journal_rebuild_guard"),
            "last_decision": saved.get("last_decision"),
            "last_candidate": saved.get("last_candidate"),
        }

    def paper_rebuild_guard_active(
        self,
        *,
        account_scope: str,
        reconciliation: dict[str, Any] | None = None,
    ) -> bool:
        scope = str(account_scope or "").lower()
        if scope != "paper":
            return False
        state = self._load()
        guard = state.get("post_journal_rebuild_guard") or {}
        if not isinstance(guard, dict):
            return False
        if not bool(guard.get("active")) or str(guard.get("scope") or "").lower() != "paper":
            return False
        try:
            expires_at = datetime.fromisoformat(str(guard.get("expires_at") or ""))
        except ValueError:
            return False
        if expires_at <= datetime.utcnow():
            state["post_journal_rebuild_guard"] = deepcopy(_DEFAULT_STATE["post_journal_rebuild_guard"])
            self._save(state)
            return False
        reconciliation_state = str((reconciliation or {}).get("state") or "").lower()
        return reconciliation_state != "healthy"

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

    def _paper_dry_run_monitor_summary(self, scope: str) -> dict[str, Any]:
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "refresh_interval_sec": settings.tradier_probability_refresh_sec,
            "account_session": {
                "scope": scope,
                "classification": "paper",
                "account_id": settings.tradier_paper_account_id or "PAPER-DRYRUN",
                "dry_run": True,
            },
            "balances": {
                "cash": 100000.0,
                "total_equity": 100000.0,
                "equity": 100000.0,
                "option_buying_power": 100000.0,
            },
            "alerts": [
                {
                    "level": "info",
                    "message": "ATLAS_TRADIER_DRY_RUN=true; using local virtual paper balances.",
                    "scope": scope,
                }
            ],
            "totals": {},
            "strategies": [],
            "pdt_status": {"status": "dry_run_virtual", "deprecated": True},
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
        if str(scope or "").strip().lower() == "paper" and _paper_dry_run_enabled():
            return self._paper_dry_run_monitor_summary(scope)
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
        broker_order_ids = route_payload.get("broker_order_ids_json") or {}
        expected_preview = action != "submit"
        expected_quantity = self._stringify_order_size(float(order.size))

        order_id = (
            tradier_response.get("id")
            or broker_order_ids.get("id")
            or broker_order_ids.get("order_id")
            or broker_order_ids.get("broker_order_id")
            or ((route_payload.get("order_response") or {}).get("id") if isinstance(route_payload.get("order_response"), dict) else None)
        )
        partner_id = tradier_response.get("partner_id") or broker_order_ids.get("partner_id")
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
            "broker_order_ids_json": broker_order_ids,
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
            "paper_bypass_exit_now_guard",
            "paper_bypass_entry_validation_guard",
            "paper_market_hours_strict",
            "paper_telegram_trade_alerts",
            "autonomy_mode",
            "max_risk_per_trade_pct",
            "deployment_mode",
            "allowed_runtime_modes",
            "live_unlock_policy",
            "runtime_mode_requested",
            "strategy_mode_overrides",
            "symbol_mode_overrides",
            "require_human_unlock",
            "require_dual_confirmation",
            "full_live_globally_locked",
            "live_capable",
            "live_ready",
            "live_unlock_requested",
            "live_unlock_approved",
            "live_guardrails_healthy",
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
                "deployment_mode": config.get("deployment_mode"),
                "live_capable": bool(config.get("live_capable", False)),
                "live_ready": bool(config.get("live_ready", False)),
            },
            "runtime_mode_resolved": config.get("runtime_mode_resolved") or {},
            "live_switch_state": config.get("live_switch_state") or {},
            "vision": vision_status,
            "executor": executor_status,
            "brain": self.brain.status(),
            "market_telemetry": self._market_telemetry_snapshot(),
            "monitor_summary": {
                "account_session": monitor.get("account_session"),
                "balances": monitor.get("balances") or {},
                "totals": monitor.get("totals") or {},
            },
        }

    def _market_telemetry_snapshot(self) -> dict[str, Any]:
        telemetry = MarketTelemetryStore().status()
        return {
            "scanner_options_flow_enabled": bool(settings.scanner_options_flow_enabled),
            "runtime_mode": "hybrid_options_intradia" if settings.scanner_options_flow_enabled else "proxy_intradia",
            "telemetry": telemetry,
            "config": {
                "min_dte": int(settings.scanner_options_flow_min_dte),
                "max_dte": int(settings.scanner_options_flow_max_dte),
                "expirations": int(settings.scanner_options_flow_expirations),
                "cache_ttl_sec": int(settings.scanner_options_flow_cache_ttl_sec),
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

    def _build_portfolio_risk_guard(
        self,
        *,
        scope: str,
        action: Literal["evaluate", "preview", "submit"],
        is_close_order: bool,
    ) -> dict[str, Any]:
        payload = {
            "status": "not_applicable" if is_close_order else "ok",
            "blocked": False,
            "degraded": False,
            "reasons": [],
            "warnings": [],
            "position_management": {},
            "exit_governance": {},
        }
        if is_close_order:
            return payload
        try:
            position_management = self.journal.position_management_snapshot(account_type=scope, limit=10)
        except Exception as exc:
            payload["status"] = "fallback"
            payload["warnings"].append(f"Portfolio risk guard no pudo cargar position management: {exc}")
            return payload
        try:
            exit_governance = self.journal.exit_governance_snapshot(account_type=scope, limit=10)
        except Exception as exc:
            exit_governance = {"enabled": False, "summary": {}, "alerts": [], "recommendations": []}
            payload["warnings"].append(f"Portfolio risk guard no pudo cargar exit governance: {exc}")

        payload["position_management"] = {
            "summary": deepcopy(position_management.get("summary") or {}),
            "var_monitor": deepcopy(position_management.get("var_monitor") or {}),
        }
        payload["exit_governance"] = {
            "summary": deepcopy(exit_governance.get("summary") or {}),
            "alerts": deepcopy(exit_governance.get("alerts") or []),
        }

        pm_summary = position_management.get("summary") or {}
        var_monitor = position_management.get("var_monitor") or {}
        var_status = str(var_monitor.get("status") or pm_summary.get("var_status") or "ok").lower()
        var_value = _safe_float(var_monitor.get("var_95_usd"), _safe_float(pm_summary.get("var_95_usd"), 0.0))
        exit_summary = exit_governance.get("summary") or {}
        exit_now_count = int(_safe_float(exit_summary.get("exit_now_count"), 0.0))
        de_risk_count = int(_safe_float(exit_summary.get("de_risk_count"), 0.0))

        if action == "submit":
            if exit_now_count > 0:
                payload["blocked"] = True
                payload["status"] = "blocked"
                payload["reasons"].append(
                    f"Portfolio risk guard blocked submit: hay {exit_now_count} posiciones con criterio de exit_now antes de sumar exposicion."
                )
            elif var_status == "critical":
                payload["blocked"] = True
                payload["status"] = "blocked"
                payload["reasons"].append(
                    f"Portfolio risk guard blocked submit: VaR Monte Carlo del libro esta en CRITICAL ({var_value:.2f} USD)."
                )
            elif var_status == "warning" and de_risk_count > 0:
                payload["blocked"] = True
                payload["status"] = "blocked"
                payload["reasons"].append(
                    f"Portfolio risk guard blocked submit: VaR en WARNING con {de_risk_count} recomendaciones activas de de-risk."
                )
        else:
            if exit_now_count > 0:
                payload["degraded"] = True
                payload["status"] = "warning"
                payload["warnings"].append(
                    f"Portfolio risk guard: hay {exit_now_count} posiciones en exit_now; revisar cierres antes de ejecutar."
                )
            elif var_status in {"warning", "critical"}:
                payload["degraded"] = True
                payload["status"] = "warning"
                payload["warnings"].append(
                    f"Portfolio risk guard: VaR del libro en {var_status.upper()} ({var_value:.2f} USD)."
                )
        return payload

    def _build_vision_pipeline_snapshot(
        self,
        *,
        vision_status: dict[str, Any],
        visual_gate_metrics: dict[str, Any],
    ) -> dict[str, Any]:
        provider = str(vision_status.get("provider") or "unknown")
        provider_ready = bool(vision_status.get("provider_ready"))
        provider_active = provider not in {"off", "manual", "unknown", ""}
        supported_modes = list(
            vision_status.get("supported_modes")
            or ["off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360"]
        )
        last_capture_at = vision_status.get("last_capture_at")
        last_capture_age_sec: float | None = None
        last_capture_fresh: bool | None = None
        if last_capture_at:
            try:
                capture_dt = datetime.fromisoformat(str(last_capture_at).replace("Z", "+00:00"))
                if capture_dt.tzinfo is not None:
                    capture_dt = capture_dt.astimezone().replace(tzinfo=None)
                last_capture_age_sec = round(max((datetime.utcnow() - capture_dt).total_seconds(), 0.0), 2)
                last_capture_fresh = last_capture_age_sec <= 900.0
            except ValueError:
                last_capture_age_sec = None
                last_capture_fresh = None

        evaluated_count = int(visual_gate_metrics.get("evaluated_count") or 0)
        applies_count = int(visual_gate_metrics.get("applies_count") or 0)
        blocked_count = int(visual_gate_metrics.get("blocked_count") or 0)
        passed_count = int(visual_gate_metrics.get("passed_count") or 0)
        manual_review_count = int(visual_gate_metrics.get("manual_review_count") or 0)

        trade_coverage_pct = round((applies_count / evaluated_count) * 100.0, 2) if evaluated_count > 0 else 0.0
        pass_rate_pct = round((passed_count / applies_count) * 100.0, 2) if applies_count > 0 else 0.0
        block_rate_pct = round((blocked_count / applies_count) * 100.0, 2) if applies_count > 0 else 0.0
        manual_review_rate_pct = round((manual_review_count / applies_count) * 100.0, 2) if applies_count > 0 else 0.0

        if not provider_active:
            activation_status = "disabled"
        elif not provider_ready:
            activation_status = "degraded"
        elif evaluated_count == 0:
            activation_status = "warming_up"
        else:
            activation_status = "active"

        if activation_status in {"disabled", "degraded"}:
            coverage_status = activation_status
        elif evaluated_count == 0:
            coverage_status = "warming_up"
        elif trade_coverage_pct >= 80.0:
            coverage_status = "healthy"
        else:
            coverage_status = "partial"

        return {
            "provider": provider,
            "provider_active": provider_active,
            "provider_ready": provider_ready,
            "supported_modes": supported_modes,
            "activation_status": activation_status,
            "coverage_status": coverage_status,
            "kpi_target_trade_coverage_pct": 80.0,
            "trade_coverage_pct": trade_coverage_pct,
            "pass_rate_pct": pass_rate_pct,
            "block_rate_pct": block_rate_pct,
            "manual_review_rate_pct": manual_review_rate_pct,
            "evaluated_count": evaluated_count,
            "applies_count": applies_count,
            "blocked_count": blocked_count,
            "passed_count": passed_count,
            "manual_review_count": manual_review_count,
            "last_capture_at": last_capture_at,
            "last_capture_age_sec": last_capture_age_sec,
            "last_capture_fresh": last_capture_fresh,
            "last_capture_note": vision_status.get("last_capture_note"),
            "notes": vision_status.get("notes"),
            "last_status": visual_gate_metrics.get("last_status"),
            "last_blocked": bool(visual_gate_metrics.get("last_blocked")),
            "last_manual_required": bool(visual_gate_metrics.get("last_manual_required")),
            "last_readiness_score_pct": visual_gate_metrics.get("last_readiness_score_pct"),
            "last_alignment_score_pct": visual_gate_metrics.get("last_alignment_score_pct"),
            "last_updated_at": visual_gate_metrics.get("last_updated_at"),
            "last_symbol": visual_gate_metrics.get("last_symbol"),
            "last_action": visual_gate_metrics.get("last_action"),
            "last_blocking_reason": visual_gate_metrics.get("last_blocking_reason"),
        }

    def _build_brain_council_snapshot(self, *, brain_status: dict[str, Any]) -> dict[str, Any]:
        claude_configured = any(
            str(os.getenv(key) or "").strip()
            for key in ("ANTHROPIC_API_KEY", "AWS_PROFILE", "AWS_REGION", "AWS_DEFAULT_REGION")
        )
        deepseek_local_url = str(os.getenv("ATLAS_OLLAMA_URL") or "").strip()
        deepseek_configured = bool(deepseek_local_url)
        atlas_brain_enabled = bool(brain_status.get("enabled", True))
        atlas_brain_available = atlas_brain_enabled and brain_status.get("last_memory_ok") is not False

        members = [
            {
                "member": "claude",
                "role": "reasoning",
                "configured": claude_configured,
                "available": claude_configured,
                "status": "ready" if claude_configured else "missing_config",
            },
            {
                "member": "deepseek_local",
                "role": "fast_local",
                "configured": deepseek_configured,
                "available": deepseek_configured,
                "status": "ready" if deepseek_configured else "missing_config",
                "endpoint": deepseek_local_url or None,
            },
            {
                "member": "atlas_brain",
                "role": "memory_execution_context",
                "configured": atlas_brain_enabled,
                "available": atlas_brain_available,
                "status": "ready" if atlas_brain_available else "degraded",
                "last_error": brain_status.get("last_error") or "",
            },
        ]
        available_members_count = sum(1 for item in members if item.get("available"))
        quorum_required = 2
        quorum_ready = available_members_count >= quorum_required
        if quorum_ready:
            activation_status = "active"
        elif available_members_count == 1:
            activation_status = "warming_up"
        else:
            activation_status = "degraded"

        return {
            "enabled": True,
            "consensus_mode": "2_of_3",
            "quorum_required": quorum_required,
            "available_members_count": available_members_count,
            "quorum_ready": quorum_ready,
            "activation_status": activation_status,
            "members": members,
            "last_event_kind": brain_status.get("last_event_kind"),
            "last_event_at": brain_status.get("last_event_at"),
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
        vision_pipeline = self._build_vision_pipeline_snapshot(
            vision_status=vision_status,
            visual_gate_metrics=visual_gate_metrics,
        )
        brain_status = self.brain.status()
        brain_council = self._build_brain_council_snapshot(brain_status=brain_status)
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
            "vision_pipeline": vision_pipeline,
            "executor": executor_status,
            "brain": brain_status,
            "brain_council": brain_council,
            "market_telemetry": self._market_telemetry_snapshot(),
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
        protocol = read_trading_self_audit_protocol(protocol_path)
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
        runtime_resolution = resolve_runtime_mode(
            requested_mode=str(config.get("runtime_mode_requested") or config.get("auton_mode") or "paper_baseline"),
            policy_variant=str(order_copy.market_context.get("policy_variant") or order_copy.strategy_type or "baseline_v1"),
            symbol=str(order_copy.symbol or ""),
            account_scope=scope,
            deployment_scope=str(config.get("deployment_mode") or "dev"),
            allowed_runtime_modes=list(config.get("allowed_runtime_modes") or []),
            strategy_mode_overrides=dict(config.get("strategy_mode_overrides") or {}),
            symbol_mode_overrides=dict(config.get("symbol_mode_overrides") or {}),
        )
        kill_state = kill_switch_health(
            active=bool(self.executor.status().get("kill_switch_active", False)),
            available=True,
            last_error=None,
        )
        live_switch_state = resolve_live_switch_state(
            runtime_mode=runtime_resolution.effective_mode,
            live_capable=bool(config.get("live_capable", True)),
            live_ready=bool(config.get("live_ready", False)),
            live_unlock_requested=bool(config.get("live_unlock_requested", False)),
            live_unlock_approved=bool(config.get("live_unlock_approved", False)),
            guardrails_healthy=bool(config.get("live_guardrails_healthy", True)),
            kill_switch_active=bool(kill_state.get("active", False)),
            require_human_unlock=bool(config.get("require_human_unlock", True)),
            full_live_globally_locked=bool(config.get("full_live_globally_locked", True)),
        )
        guarded_route = resolve_guarded_order_route(
            runtime_mode=runtime_resolution.effective_mode,
            effective_live_enabled=bool(live_switch_state.effective_live_enabled),
        )
        config["runtime_mode_resolved"] = runtime_resolution_to_event(runtime_resolution)
        config["live_switch_state"] = live_switch_state_to_payload(live_switch_state)
        opening_equity_order = self._is_opening_equity_order(order_copy)
        monitor_started = time.perf_counter()
        if scope == "paper" and _paper_dry_run_enabled():
            monitor = self._paper_dry_run_monitor_summary(scope)
        else:
            monitor = self.tracker.build_summary(account_scope=scope)  # type: ignore[arg-type]
        evaluation_timings["monitor_summary_sec"] = round(time.perf_counter() - monitor_started, 4)
        balances = monitor.get("balances") or {}
        option_bp = _safe_float(balances.get("option_buying_power"), 0.0)
        cash = _safe_float(balances.get("cash"), 0.0)
        pdt_status = monitor.get("pdt_status") or {}
        equity = _safe_float(balances.get("total_equity"), _safe_float(balances.get("equity"), 0.0))
        opening_equity_anchor = self._resolve_intraday_opening_equity(
            state=config,
            account_scope=scope,
            current_equity=equity,
        )
        _deprecated_pdt_ok, deprecated_pdt_reason = self.check_pdt_limit(order_copy.account_id or "unknown")
        _capital_ok, capital_reason = self.validate_account_capital({"equity": equity})
        reconciliation = self._load_reconciliation(account_scope=scope, account_id=order_copy.account_id)
        rebuild_guard_active = self.paper_rebuild_guard_active(account_scope=scope, reconciliation=reconciliation)
        open_symbols = self._extract_open_symbols(monitor)
        vision_status = self.vision.status()
        executor_status = self.executor.status()
        reasons: list[str] = []
        warnings: list[str] = []
        probability_payload: dict[str, Any] | None = None
        probability_source = "none"
        strategy_type = str(order_copy.strategy_type or order_copy.probability_gate.strategy_type) if order_copy.probability_gate and order_copy.strategy_type is None else (str(order_copy.strategy_type) if order_copy.strategy_type else None)

        is_close_order = str(order_copy.position_effect or "").lower() == "close"
        if bool(config.get("paper_only", True)) and scope != "paper" and guarded_route.route == "live_execution":
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
        warnings.append(
            f"PDT gate deprecated ({PDT_RULE_REMOVAL_DATE}): {deprecated_pdt_reason}."
        )
        warnings.append(f"Capital gate migrated to operational mode: {capital_reason}.")
        if opening_equity_order and not strategy_type:
            reasons.append("strategy_type is required for autonomous equity orders.")
        symbol_upper = str(order_copy.symbol or "").strip().upper()
        market_hours_ok = True
        market_hours_reason = "market_hours_not_checked"
        market_hours_blocked = False
        market_hours_degraded = False
        if action in {"preview", "submit"} and symbol_upper:
            if is_close_order and scope == "paper":
                market_hours_ok, market_hours_reason = True, "close_order_exempt_paper"
            else:
                market_hours_ok, market_hours_reason = self._check_market_hours(symbol_upper, scope)
            if not market_hours_ok:
                if scope == "paper":
                    strict_mh = bool(config.get("paper_market_hours_strict"))
                    if action == "submit" and strict_mh:
                        market_hours_blocked = True
                        reasons.append(
                            f"Market hours gate blocked submit (paper_market_hours_strict): {market_hours_reason}."
                        )
                    else:
                        market_hours_degraded = True
                        warnings.append(
                            f"[paper bypass] Market hours gate would block {action}: {market_hours_reason}."
                        )
                else:
                    market_hours_blocked = True
                    reasons.append(f"Market hours gate blocked {action}: {market_hours_reason}.")
        daily_trade_limit = {"status": "not_applicable", "blocked": False, "reason": "not_checked"}
        loss_cooldown = {"status": "not_applicable", "blocked": False, "reason": "not_checked"}
        intraday_drawdown_gate = {"status": "not_applicable", "blocked": False, "reason": "not_checked"}
        intraday_exposure_gate = {"status": "not_applicable", "blocked": False, "reason": "not_checked"}
        if not is_close_order and action in {"preview", "submit"}:
            daily_ok, daily_reason, daily_meta = self.check_daily_trade_limit(account_scope=scope)
            daily_trade_limit = {
                "status": "ok" if daily_ok else "blocked",
                "blocked": not daily_ok,
                "reason": daily_reason,
                **daily_meta,
            }
            if not daily_ok:
                reasons.append(f"Operational gate blocked {action}: {daily_reason}.")

            cooldown_ok, cooldown_reason, cooldown_meta = self.check_consecutive_loss_cooldown()
            loss_cooldown = {
                "status": "ok" if cooldown_ok else "blocked",
                "blocked": not cooldown_ok,
                "reason": cooldown_reason,
                **cooldown_meta,
            }
            if not cooldown_ok:
                reasons.append(f"Operational gate blocked {action}: {cooldown_reason}.")

            drawdown_ok, drawdown_reason, drawdown_meta = self.check_intraday_drawdown(
                opening_equity=opening_equity_anchor,
                current_equity=equity,
            )
            intraday_drawdown_gate = {
                "status": "ok" if drawdown_ok else "blocked",
                "blocked": not drawdown_ok,
                "reason": drawdown_reason,
                **drawdown_meta,
            }
            if not drawdown_ok:
                reasons.append(f"Operational gate blocked {action}: {drawdown_reason}.")
        if opening_equity_order and symbol_upper and symbol_upper in open_symbols:
            if rebuild_guard_active:
                warnings.append(
                    f"Open-symbol guard bypassed in paper mode for {symbol_upper} after journal rebuild while reconciliation stabilizes."
                )
            else:
                reasons.append(f"Open-symbol guard blocked re-entry for {symbol_upper}.")
        reconciliation_state = str((reconciliation or {}).get("state") or "").lower()
        if opening_equity_order and action == "submit" and reconciliation_state and reconciliation_state != "healthy":
            # In paper mode, the paper_local simulator always reports 0 positions,
            # causing a permanent reconciliation mismatch. Only block if we're NOT in paper mode.
            if scope != "paper":
                reasons.append(f"Reconciliation gate blocked submit because state is '{reconciliation_state}'.")
            else:
                if rebuild_guard_active:
                    warnings.append(
                        f"Reconciliation temporarily bypassed in paper mode after journal rebuild (state='{reconciliation_state}')."
                    )
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
        self._apply_paper_entry_validation_bypass(
            scope=scope,
            action=action,
            config=config,
            entry_validation=entry_validation,
            reasons=reasons,
            warnings=warnings,
        )
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
            current_exposure = 0.0
            for strategy in monitor.get("strategies") or []:
                if not isinstance(strategy, dict):
                    continue
                current_exposure += abs(
                    _safe_float(
                        strategy.get("open_notional"),
                        _safe_float(strategy.get("risk_at_entry"), 0.0),
                    )
                )
            exposure_ok, exposure_reason, exposure_meta = self.check_intraday_exposure(
                equity=equity,
                current_exposure=current_exposure,
                proposed_exposure=_safe_float(estimated_open_notional, 0.0),
            )
            intraday_exposure_gate = {
                "status": "ok" if exposure_ok else "blocked",
                "blocked": not exposure_ok,
                "reason": exposure_reason,
                **exposure_meta,
            }
            if not exposure_ok:
                reasons.append(f"Operational gate blocked {action}: {exposure_reason}.")

        portfolio_risk_guard = self._build_portfolio_risk_guard(
            scope=scope,
            action=action,
            is_close_order=is_close_order,
        )
        reasons.extend(list(portfolio_risk_guard.get("reasons") or []))
        warnings.extend(list(portfolio_risk_guard.get("warnings") or []))
        self._apply_paper_risk_guard_bypass(
            scope=scope,
            action=action,
            config=config,
            portfolio_risk_guard=portfolio_risk_guard,
            reasons=reasons,
            warnings=warnings,
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
            if guarded_route.route != "live_execution":
                if scope != "paper":
                    warnings.append(
                        f"[live-locked] Guarded routing activo: {guarded_route.route} ({guarded_route.reason})."
                    )
                scope = "paper"
                order_copy.account_scope = "paper"  # type: ignore[assignment]
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
                        elif (
                            scope == "paper"
                            and action == "submit"
                            and bool(config.get("paper_telegram_trade_alerts", True))
                            and execution_result.get("decision") == "paper_submit_sent"
                        ):
                            try:
                                from operations.paper_trade_telegram import notify_paper_submission

                                notify_paper_submission(order_copy, execution_result)
                            except Exception as _tg_err:
                                logger.debug("paper telegram notify skipped: %s", _tg_err)
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
            "runtime_mode_resolved": runtime_resolution_to_event(runtime_resolution),
            "live_switch_state": live_switch_state_to_payload(live_switch_state),
            "guarded_order_route": guarded_route_to_payload(guarded_route),
            "gates": {
                "scope": scope,
                "runtime_mode": runtime_resolution_to_event(runtime_resolution),
                "live_switch": live_switch_state_to_payload(live_switch_state),
                "guarded_order_route": guarded_route_to_payload(guarded_route),
                "pdt_status": pdt_status,
                "pdt_migration": {
                    "status": "deprecated",
                    "effective_date": PDT_RULE_REMOVAL_DATE,
                    "reason": deprecated_pdt_reason,
                },
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
                "portfolio_risk_guard": {
                    "status": portfolio_risk_guard.get("status"),
                    "blocked": bool(portfolio_risk_guard.get("blocked")),
                    "degraded": bool(portfolio_risk_guard.get("degraded")),
                },
                "market_context": {
                    "status": market_context_gate.get("status"),
                    "blocked": bool(market_context_gate.get("blocked")),
                    "degraded": bool(market_context_gate.get("degraded")),
                },
                "market_hours": {
                    "status": "ok" if market_hours_ok else ("degraded" if market_hours_degraded else "blocked"),
                    "blocked": market_hours_blocked,
                    "degraded": market_hours_degraded,
                    "reason": market_hours_reason,
                },
                "daily_trade_limit": daily_trade_limit,
                "intraday_exposure": intraday_exposure_gate,
                "intraday_drawdown": intraday_drawdown_gate,
                "loss_cooldown": loss_cooldown,
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
                "kill_switch_health": kill_state,
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
            "portfolio_risk_guard": portfolio_risk_guard,
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
        try:
            from atlas_code_quant.options.options_engine_metrics import record_live_transition_state

            record_live_transition_state(
                runtime_resolution=runtime_resolution_to_event(runtime_resolution),
                live_switch_state=live_switch_state_to_payload(live_switch_state),
            )
        except Exception:
            logger.debug("live transition metrics update skipped", exc_info=True)
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
            and str(auton_mode or "").lower() in {"paper_supervised", "paper_autonomous", "paper_aggressive"}
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
