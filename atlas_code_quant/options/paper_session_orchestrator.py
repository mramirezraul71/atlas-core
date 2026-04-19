"""Orquestador paper-only: briefing → intent → plan de entrada (sin órdenes)."""
from __future__ import annotations

from typing import Any

from atlas_code_quant.options.options_paper_journal import OptionsPaperJournal
from atlas_code_quant.options.paper_entry_planner import PaperEntryPlanner
from atlas_code_quant.options.options_intent_router import (
    KNOWN_STRATEGY_FAMILIES,
    OptionsIntentRouter,
)
from atlas_code_quant.options.session_briefing import SessionBriefingEngine


class PaperSessionOrchestrator:
    """Encadena ``SessionBriefingEngine``, ``OptionsIntentRouter`` y ``PaperEntryPlanner``."""

    def __init__(
        self,
        briefing_engine: SessionBriefingEngine,
        intent_router: OptionsIntentRouter,
        entry_planner: PaperEntryPlanner,
        journal: OptionsPaperJournal | None = None,
    ) -> None:
        self._briefing = briefing_engine
        self._intent = intent_router
        self._planner = entry_planner
        self._journal = journal

    def build_session_plan(
        self,
        *,
        symbol: str,
        direction: str | None = None,
        regime: str | None = None,
        gamma_regime: str | None = None,
        dte_mode: str | None = None,
        days_to_event: int | None = None,
        event_near: bool | None = None,
        spot: float | None = None,
        support_levels: list[float] | None = None,
        resistance_levels: list[float] | None = None,
        breach_context: dict[str, Any] | None = None,
        capital: float | None = None,
        manual_intent_overrides: dict[str, Any] | None = None,
        manual_entry_overrides: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        sym = str(symbol or "").strip().upper()
        pipeline_notes: list[str] = ["automation_paper_only_no_orders_no_live_broker"]
        pipeline_quality_flags: list[str] = []

        if not sym:
            pipeline_quality_flags.append("orchestrator:missing_symbol")

        dir_eff = (direction or "neutral").strip() or "neutral"
        reg_eff = (regime or "ranging").strip() or "ranging"
        event_eff = bool(event_near) if event_near is not None else False

        briefing: dict[str, Any] = {}
        if sym:
            briefing = self._briefing.build_briefing(
                sym,
                direction=dir_eff,
                regime=reg_eff,
                gamma_regime=gamma_regime,
                dte_mode=dte_mode,
                days_to_event=days_to_event,
                event_near=event_eff,
                spot=spot,
                support_levels=support_levels,
                resistance_levels=resistance_levels,
                breach_context=breach_context,
            )
        else:
            pipeline_quality_flags.append("orchestrator:briefing_skipped_empty_symbol")

        intent: dict[str, Any] = {}
        if sym:
            intent = self._intent.build_intent(
                briefing,
                manual_overrides=manual_intent_overrides,
            )
        else:
            intent = {
                "symbol": "",
                "allow_entry": False,
                "force_no_trade": True,
                "suppress_short_premium": True,
                "prefer_defined_risk": True,
                "entry_bias": "neutral",
                "market_bias": "neutral",
                "risk_posture": "blocked",
                "selector_inputs": {},
                "preferred_families": [],
                "blocked_families": list(KNOWN_STRATEGY_FAMILIES),
                "strategy_candidates": [],
                "recommended_strategy": None,
                "intent_notes": ["empty_symbol_pipeline_abort"],
                "intent_quality_flags": ["orchestrator:missing_symbol"],
            }

        entry_plan = self._planner.build_entry_plan(
            intent,
            briefing=briefing if sym else None,
            capital=capital,
            manual_entry_overrides=manual_entry_overrides,
        )

        self._merge_child_flags(
            briefing,
            intent,
            entry_plan,
            pipeline_quality_flags,
            pipeline_notes,
        )

        entry_allowed = (
            bool(intent.get("allow_entry"))
            and not bool(intent.get("force_no_trade"))
            and str(entry_plan.get("entry") or "").lower() == "proposed"
        )

        data: dict[str, Any] = {
            "automation_mode": "paper_only",
            "symbol": sym,
            "entry_allowed": entry_allowed,
            "briefing": briefing,
            "intent": intent,
            "entry_plan": entry_plan,
            "pipeline_notes": self._dedup(pipeline_notes),
            "pipeline_quality_flags": self._dedup(pipeline_quality_flags),
        }
        if self._journal is not None:
            data["trace_id"] = self._journal.log_session_plan(
                data,
                notes=["paper_session_plan_orchestrator"],
            )
        return data

    def _merge_child_flags(
        self,
        briefing: dict[str, Any],
        intent: dict[str, Any],
        entry_plan: dict[str, Any],
        pipeline_quality_flags: list[str],
        pipeline_notes: list[str],
    ) -> None:
        for f in briefing.get("quality_flags") or []:
            pipeline_quality_flags.append(f"briefing:{f}")
        for n in briefing.get("operational_notes") or []:
            pipeline_notes.append(f"briefing_op:{n}")

        for f in intent.get("intent_quality_flags") or []:
            pipeline_quality_flags.append(f"intent:{f}")
        for n in intent.get("intent_notes") or []:
            pipeline_notes.append(f"intent:{n}")

        for f in entry_plan.get("entry_quality_flags") or []:
            pipeline_quality_flags.append(f"entry:{f}")
        for n in entry_plan.get("entry_notes") or []:
            pipeline_notes.append(f"entry:{n}")

    @staticmethod
    def _dedup(items: list[Any]) -> list[Any]:
        out: list[Any] = []
        seen: set[Any] = set()
        for item in items:
            if item in seen:
                continue
            seen.add(item)
            out.append(item)
        return out