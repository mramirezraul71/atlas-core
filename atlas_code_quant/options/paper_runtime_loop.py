"""Runtime loop paper-only del Options Engine.

Mantiene un ciclo liviano en segundo plano:
- session_plan periódico (briefing + intent + planner),
- entry_execution cuando el plan permite entrada,
- evaluación AutoClose periódica sobre posiciones paper abiertas,
- close_decision + close_execution al disparar reglas,
- refresco de métricas Prometheus/journal.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_code_quant.backtesting.winning_probability import TradierClient
from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine
from atlas_code_quant.execution.option_chain_cache import OptionChainCache
from atlas_code_quant.options.iv_rank_calculator import IVRankCalculator
from atlas_code_quant.options.options_engine_metrics import (
    default_journal_path,
    record_options_error,
    record_pipeline_module,
    refresh_journal_from_disk,
    tick_pipeline_ages,
)
from atlas_code_quant.options.options_intent_router import OptionsIntentRouter
from atlas_code_quant.options.options_paper_journal import OptionsPaperJournal
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
from atlas_code_quant.options.paper_session_orchestrator import PaperSessionOrchestrator
from atlas_code_quant.options.session_briefing import SessionBriefingEngine

logger = logging.getLogger("quant.options.paper_runtime")


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


@dataclass
class RuntimeOpenPosition:
    trace_id: str
    session_key: str | None
    symbol: str
    owner: str
    strategy_type: str
    gamma_regime: str
    opened_at_ts: float
    entry_credit: float
    dte_mode: str
    close_after_sec: float
    exit_rules: dict[str, Any]
    safety_flags: list[str]
    planned_entry: dict[str, Any]
    last_plan: dict[str, Any]
    position_size_units: int

    def to_autoclose_position(self, *, now_ts: float) -> dict[str, Any]:
        elapsed = max(0.0, now_ts - self.opened_at_ts)
        phase = min(1.0, elapsed / max(1.0, self.close_after_sec))
        current_value = max(0.05, self.entry_credit * (1.0 - phase))
        return {
            "position_id": self.trace_id,
            "strategy_type": self.strategy_type,
            "entry_credit": self.entry_credit,
            "current_value": current_value,
            "remaining_dte": 30,
            "is_0dte": self.dte_mode == "0dte",
            "is_credit": True,
            "position_size_units": self.position_size_units,
            "entry_executable": self.position_size_units >= 1,
            "owner": self.owner,
            "exit_rules": dict(self.exit_rules),
            "safety_flags": list(self.safety_flags),
        }


class OptionsPaperRuntimeLoop:
    """Loop periódico paper-only para mantener vivo el pipeline de opciones."""

    def __init__(
        self,
        *,
        symbol: str,
        capital: float,
        session_interval_sec: float,
        autoclose_interval_sec: float,
        close_after_sec: float,
        journal_path: Path | None = None,
        orchestrator: PaperSessionOrchestrator | None = None,
        journal: OptionsPaperJournal | None = None,
        autoclose_engine: AutoCloseEngine | None = None,
    ) -> None:
        sym = str(symbol or "").strip().upper()
        self._symbol = sym if sym else "IWM"
        self._capital = float(capital) if capital > 0 else 10_000.0
        self._session_interval_sec = max(15.0, float(session_interval_sec))
        self._autoclose_interval_sec = max(10.0, float(autoclose_interval_sec))
        self._close_after_sec = max(30.0, float(close_after_sec))
        self._journal_path = Path(journal_path) if journal_path is not None else default_journal_path()
        self._journal = journal or OptionsPaperJournal(path=self._journal_path)
        self._autoclose = autoclose_engine or AutoCloseEngine()
        self._orchestrator = orchestrator or self._build_default_orchestrator()
        self._open_positions: dict[str, RuntimeOpenPosition] = {}
        self._processed_session_keys: set[str] = set()
        self._last_session_ts = 0.0
        self._last_autoclose_ts = 0.0
        self._running = False

    @staticmethod
    def _build_default_orchestrator() -> PaperSessionOrchestrator:
        client = TradierClient(scope="paper")
        calc = IVRankCalculator(client, chain_cache=OptionChainCache(), scope="paper")
        briefing_engine = SessionBriefingEngine(calc)
        return PaperSessionOrchestrator(
            briefing_engine=briefing_engine,
            intent_router=OptionsIntentRouter(),
            entry_planner=PaperEntryPlanner(),
            journal=None,
        )

    async def run_forever(self) -> None:
        self._running = True
        self._bootstrap_open_positions_from_journal()
        refresh_journal_from_disk(self._journal_path)
        logger.info(
            "[options-runtime] started symbol=%s session_interval=%ss autoclose_interval=%ss close_after=%ss",
            self._symbol,
            self._session_interval_sec,
            self._autoclose_interval_sec,
            self._close_after_sec,
        )
        try:
            while self._running:
                now_ts = time.time()
                if now_ts - self._last_session_ts >= self._session_interval_sec:
                    await self._run_session_tick()
                    self._last_session_ts = now_ts
                if now_ts - self._last_autoclose_ts >= self._autoclose_interval_sec:
                    await self._run_autoclose_tick()
                    self._last_autoclose_ts = now_ts
                tick_pipeline_ages()
                await asyncio.sleep(3.0)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logger.exception("[options-runtime] loop error: %s", exc)
            record_options_error("options_runtime_loop_crash")
            raise
        finally:
            self._running = False
            tick_pipeline_ages()
            logger.info("[options-runtime] stopped")

    def stop(self) -> None:
        self._running = False

    def _bootstrap_open_positions_from_journal(self) -> None:
        p = self._journal_path
        if not p.is_file():
            return
        traces: dict[str, RuntimeOpenPosition] = {}
        closed: set[str] = set()
        for raw in p.read_text(encoding="utf-8", errors="ignore").splitlines():
            line = raw.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(row, dict):
                continue
            tid = str(row.get("trace_id") or "").strip()
            et = str(row.get("event_type") or "").strip()
            if not tid:
                continue
            payload = row.get("payload") if isinstance(row.get("payload"), dict) else {}
            session_key = self._extract_session_key_from_payload(payload)
            if session_key:
                self._processed_session_keys.add(session_key)
            if et == "entry_execution":
                planned_entry = payload.get("planned_entry") if isinstance(payload.get("planned_entry"), dict) else {}
                strategy_type = str(
                    row.get("structure_type")
                    or planned_entry.get("recommended_strategy")
                    or "paper_runtime_strategy"
                )
                entry_credit = float(row.get("entry_credit") or 1.0)
                dte_mode = str((payload.get("last_plan") or {}).get("briefing", {}).get("dte_mode") or "")
                gamma_regime = str((payload.get("last_plan") or {}).get("briefing", {}).get("gamma_regime") or "")
                traces[tid] = RuntimeOpenPosition(
                    trace_id=tid,
                    session_key=session_key,
                    symbol=str(row.get("symbol") or self._symbol).strip().upper() or self._symbol,
                    owner=str(planned_entry.get("entry_owner") or payload.get("entry_owner") or "autoclose_engine"),
                    strategy_type=strategy_type,
                    gamma_regime=gamma_regime,
                    opened_at_ts=time.time(),
                    entry_credit=max(0.1, entry_credit),
                    dte_mode=dte_mode,
                    close_after_sec=self._close_after_sec,
                    exit_rules=dict(planned_entry.get("exit_rules") or payload.get("exit_rules") or {}),
                    safety_flags=list(planned_entry.get("safety_flags") or payload.get("safety_flags") or []),
                    planned_entry=planned_entry,
                    last_plan=payload.get("last_plan") if isinstance(payload.get("last_plan"), dict) else {},
                    position_size_units=max(1, int(row.get("position_size_units") or 1)),
                )
            elif et == "close_execution":
                closed.add(tid)
        for tid, pos in traces.items():
            if tid not in closed:
                self._open_positions[tid] = pos

    async def _run_session_tick(self) -> None:
        try:
            plan = await asyncio.to_thread(
                self._orchestrator.build_session_plan,
                symbol=self._symbol,
                capital=self._capital,
            )
            trace_id = self._journal.log_session_plan(
                plan,
                notes=["paper_runtime_loop_tick"],
            )
            plan["trace_id"] = trace_id
            record_pipeline_module(module="briefing", status=1.0)
            record_pipeline_module(module="intent_router", status=1.0)
            record_pipeline_module(module="entry_planner", status=1.0)
            entry_plan = plan.get("entry_plan") if isinstance(plan.get("entry_plan"), dict) else {}
            session_key = self._resolve_session_key(plan, trace_id=trace_id)
            entry_owner = str(entry_plan.get("entry_owner") or "autoclose_engine").strip() or "autoclose_engine"
            if plan.get("entry_allowed"):
                if session_key and session_key in self._processed_session_keys:
                    logger.info(
                        "[options-runtime] session tick deduped symbol=%s session_key=%s",
                        self._symbol,
                        session_key,
                    )
                elif self._has_open_position(symbol=self._symbol, owner=entry_owner):
                    logger.info(
                        "[options-runtime] session tick skipped symbol=%s owner=%s reason=open_position_exists",
                        self._symbol,
                        entry_owner,
                    )
                else:
                    size_units = int(entry_plan.get("position_size_units") or 0)
                    if size_units >= 1:
                        self._log_entry_execution(plan, trace_id=trace_id, session_key=session_key)
                        if session_key:
                            self._processed_session_keys.add(session_key)
                    else:
                        briefing = plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {}
                        blocked_reason = str(entry_plan.get("size_blocked_reason") or "planner_no_executable_size")
                        self._journal.log_entry_blocked(
                            trace_id=trace_id,
                            symbol=self._symbol,
                            timestamp=_utc_now(),
                            blocked_reason=blocked_reason,
                            strategy_type=str(entry_plan.get("recommended_strategy") or ""),
                            gamma_regime=str(briefing.get("gamma_regime") or ""),
                            dte_mode=str(briefing.get("dte_mode") or ""),
                            planned_entry=entry_plan,
                            mode="paper",
                            source="runtime_loop",
                            status="canceled",
                            notes=["paper_runtime_loop_entry_blocked"],
                        )
                        if session_key:
                            self._processed_session_keys.add(session_key)
            refresh_journal_from_disk(self._journal_path)
            logger.info(
                "[options-runtime] session tick symbol=%s entry_allowed=%s open_positions=%d session_key=%s",
                self._symbol,
                bool(plan.get("entry_allowed")),
                len(self._open_positions),
                session_key,
            )
        except Exception as exc:
            logger.exception("[options-runtime] session tick error: %s", exc)
            record_pipeline_module(module="briefing", status=0.0)
            record_pipeline_module(module="intent_router", status=0.0)
            record_pipeline_module(module="entry_planner", status=0.0)
            record_options_error("options_runtime_session_tick")

    def _log_entry_execution(self, plan: dict[str, Any], *, trace_id: str, session_key: str | None) -> None:
        entry_plan = plan.get("entry_plan") if isinstance(plan.get("entry_plan"), dict) else {}
        size_units = int(entry_plan.get("position_size_units") or 0)
        strategy_type = str(entry_plan.get("recommended_strategy") or "paper_runtime_strategy")
        entry_owner = str(entry_plan.get("entry_owner") or "autoclose_engine").strip() or "autoclose_engine"
        exit_rules = entry_plan.get("exit_rules") if isinstance(entry_plan.get("exit_rules"), dict) else {}
        safety_flags = [str(x) for x in list(entry_plan.get("safety_flags") or []) if str(x).strip()]
        risk_hint = float(entry_plan.get("max_risk_budget_dollars") or 100.0)
        entry_credit = max(0.25, min(5.0, risk_hint / 200.0))
        executed_entry = {
            "structure": strategy_type,
            "entry_credit": round(entry_credit, 4),
            "entry_mid": round(entry_credit, 4),
            "dte": (plan.get("briefing") or {}).get("dte"),
            "position_size_units": size_units,
            "entry_owner": entry_owner,
            "exit_rules": dict(exit_rules),
            "safety_flags": list(safety_flags),
            "session_key": session_key,
        }
        self._journal.log_entry_execution(
            trace_id=trace_id,
            symbol=self._symbol,
            entry_timestamp=_utc_now(),
            planned_entry=entry_plan,
            executed_entry=executed_entry,
            mode="paper",
            source="runtime_loop",
            status="open",
            notes=["paper_runtime_loop_entry_execution"],
        )
        self._open_positions[trace_id] = RuntimeOpenPosition(
            trace_id=trace_id,
            session_key=session_key,
            symbol=self._symbol,
            owner=entry_owner,
            strategy_type=strategy_type,
            gamma_regime=str((plan.get("briefing") or {}).get("gamma_regime") or ""),
            opened_at_ts=time.time(),
            entry_credit=entry_credit,
            dte_mode=str((plan.get("briefing") or {}).get("dte_mode") or ""),
            close_after_sec=self._close_after_sec,
            exit_rules=dict(exit_rules),
            safety_flags=list(safety_flags),
            planned_entry=entry_plan,
            last_plan=plan,
            position_size_units=size_units,
        )

    async def _run_autoclose_tick(self) -> None:
        if not self._open_positions:
            record_pipeline_module(module="autoclose", status=1.0)
            refresh_journal_from_disk(self._journal_path)
            return
        now_ts = time.time()
        for tid, pos in list(self._open_positions.items()):
            try:
                position_payload = pos.to_autoclose_position(now_ts=now_ts)
                decision = self._autoclose.evaluate_position(position_payload, intraday_context={})
                record_pipeline_module(module="autoclose", status=1.0)
                if not bool(decision.get("should_close")):
                    continue
                reasons = list(decision.get("reasons") or [])
                reason = reasons[0] if reasons else "autoclose_rule"
                close_mid = float(position_payload.get("current_value") or 0.0)
                self._journal.log_close_decision(
                    trace_id=tid,
                    close_decision={
                        "symbol": pos.symbol,
                        "reason": reason,
                        "close_type": "full",
                        "close_mid": close_mid,
                        "debit": close_mid,
                    },
                    timestamp=_utc_now(),
                    mode="paper",
                    source="autoclose",
                    status="open",
                    autoclose_applied=True,
                    notes=["paper_runtime_loop_close_decision"],
                )
                pnl_realized = round(pos.entry_credit - close_mid, 6)
                self._journal.log_close_execution(
                    trace_id=tid,
                    symbol=pos.symbol,
                    close_timestamp=_utc_now(),
                    executed_close={
                        "close_type": "full",
                        "close_reason": reason,
                        "close_mid": close_mid,
                        "debit": close_mid,
                        "position_size_units": pos.position_size_units,
                        "entry_executable": True,
                        "entry_owner": pos.owner,
                        "exit_rules": dict(pos.exit_rules),
                        "safety_flags": list(pos.safety_flags),
                    },
                    pnl_realized=pnl_realized,
                    mode="paper",
                    source="autoclose",
                    status="closed",
                    autoclose_applied=True,
                    close_type="full",
                    close_reason=reason,
                    strategy_type=pos.strategy_type,
                    gamma_regime=pos.gamma_regime,
                    dte_mode=pos.dte_mode,
                    notes=["paper_runtime_loop_close_execution"],
                )
                self._open_positions.pop(tid, None)
                logger.info(
                    "[options-runtime] close execution trace_id=%s owner=%s reason=%s pnl=%.4f",
                    tid,
                    pos.owner,
                    reason,
                    pnl_realized,
                )
            except Exception as exc:
                logger.exception("[options-runtime] autoclose tick error trace=%s: %s", tid, exc)
                record_pipeline_module(module="autoclose", status=0.0)
                record_options_error("options_runtime_autoclose_tick")
        refresh_journal_from_disk(self._journal_path)

    @staticmethod
    def _extract_session_key_from_payload(payload: dict[str, Any]) -> str | None:
        session_plan = payload.get("session_plan") if isinstance(payload.get("session_plan"), dict) else {}
        planned_entry = payload.get("planned_entry") if isinstance(payload.get("planned_entry"), dict) else {}
        for source in (payload, session_plan, planned_entry):
            for key in ("session_id", "run_id", "cycle_id", "session_key"):
                raw = str(source.get(key) or "").strip()
                if raw:
                    return raw
        return None

    def _resolve_session_key(self, plan: dict[str, Any], *, trace_id: str) -> str | None:
        for source in (
            plan,
            plan.get("entry_plan") if isinstance(plan.get("entry_plan"), dict) else {},
            plan.get("briefing") if isinstance(plan.get("briefing"), dict) else {},
        ):
            if not isinstance(source, dict):
                continue
            for key in ("session_id", "run_id", "cycle_id", "session_key"):
                raw = str(source.get(key) or "").strip()
                if raw:
                    return raw
        return None

    def _has_open_position(self, *, symbol: str, owner: str) -> bool:
        symbol_norm = str(symbol or "").strip().upper()
        owner_norm = str(owner or "").strip()
        for pos in self._open_positions.values():
            if pos.symbol == symbol_norm and pos.owner == owner_norm:
                return True
        return False


def options_runtime_loop_enabled() -> bool:
    raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_LOOP_ENABLED", "true")).strip().lower()
    return raw not in {"0", "false", "no", "off"}


def build_default_options_runtime_loop() -> OptionsPaperRuntimeLoop:
    symbol = str(os.environ.get("QUANT_OPTIONS_RUNTIME_SYMBOL", "IWM")).strip().upper() or "IWM"
    capital_raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_CAPITAL", "10000")).strip()
    session_raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_SESSION_INTERVAL_SEC", "120")).strip()
    autoclose_raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_AUTOCLOSE_INTERVAL_SEC", "30")).strip()
    close_after_raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_CLOSE_AFTER_SEC", "90")).strip()
    journal_raw = str(os.environ.get("QUANT_OPTIONS_RUNTIME_JOURNAL_PATH", "")).strip()
    try:
        capital = max(1000.0, float(capital_raw))
    except Exception:
        capital = 10_000.0
    try:
        session_interval_sec = max(30.0, float(session_raw))
    except Exception:
        session_interval_sec = 120.0
    try:
        autoclose_interval_sec = max(10.0, float(autoclose_raw))
    except Exception:
        autoclose_interval_sec = 30.0
    try:
        close_after_sec = max(30.0, float(close_after_raw))
    except Exception:
        close_after_sec = 90.0
    journal_path = Path(journal_raw) if journal_raw else None
    return OptionsPaperRuntimeLoop(
        symbol=symbol,
        capital=capital,
        session_interval_sec=session_interval_sec,
        autoclose_interval_sec=autoclose_interval_sec,
        close_after_sec=close_after_sec,
        journal_path=journal_path,
    )
