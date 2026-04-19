"""Planificador de entradas solo paper: propuesta analítica, sin órdenes ni broker."""
from __future__ import annotations

from copy import deepcopy
from typing import Any


class PaperEntryPlanner:
    """Traduce ``intent`` en un plan de entrada paper-only (no ejecuta órdenes)."""

    _DEFAULT_MAX_RISK_PCT = 2.0

    def build_entry_plan(
        self,
        intent: dict[str, Any],
        *,
        briefing: dict[str, Any] | None = None,
        capital: float | None = None,
        manual_entry_overrides: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        intent = intent or {}
        notes: list[str] = ["paper_only_proposal_not_an_order"]
        flags: list[str] = []

        symbol = str(intent.get("symbol") or "").upper()
        force_no = bool(intent.get("force_no_trade"))
        allow = bool(intent.get("allow_entry"))

        if manual_entry_overrides and bool(manual_entry_overrides.get("force_no_entry")):
            force_no = True
            allow = False
            notes.append("manual_force_no_entry")

        selector_inputs = deepcopy(intent.get("selector_inputs") or {})

        base: dict[str, Any] = {
            "mode": "paper_only",
            "symbol": symbol,
            "capital_reference": capital,
            "selector_inputs": selector_inputs,
            "suppress_short_premium": bool(intent.get("suppress_short_premium")),
            "prefer_defined_risk": bool(intent.get("prefer_defined_risk")),
            "preferred_families": list(intent.get("preferred_families") or []),
            "blocked_families": list(intent.get("blocked_families") or []),
            "strategy_candidates": list(intent.get("strategy_candidates") or []),
            "entry_notes": [],
            "entry_quality_flags": [],
        }

        if capital is None or (isinstance(capital, (int, float)) and float(capital) <= 0):
            flags.append("capital_missing_or_invalid")
            notes.append("capital no provisto o inválido; presupuesto de riesgo solo referencial.")

        if force_no or not allow:
            reason = "force_no_trade" if force_no else "intent_disallows_entry"
            base.update(
                {
                    "entry": "none",
                    "reason": reason,
                    "recommended_strategy": None,
                    "position_size_units": 0,
                    "max_risk_budget_pct": 0.0,
                    "max_risk_budget_dollars": 0.0,
                    "entry_notes": self._dedup(notes + list(intent.get("intent_notes") or [])[:3]),
                    "entry_quality_flags": self._dedup(flags),
                }
            )
            return base

        rec = intent.get("recommended_strategy")
        if not rec and base["strategy_candidates"]:
            rec = str(base["strategy_candidates"][0])
            notes.append("fallback_recommended_first_candidate")

        max_pct = float(self._DEFAULT_MAX_RISK_PCT)
        if manual_entry_overrides and manual_entry_overrides.get("max_risk_budget_pct") is not None:
            try:
                max_pct = max(0.0, min(100.0, float(manual_entry_overrides["max_risk_budget_pct"])))
                notes.append("max_risk_budget_pct_from_manual_override")
            except (TypeError, ValueError):
                flags.append("max_risk_budget_pct_override_invalid")

        max_dollars = None
        if capital is not None:
            try:
                c = float(capital)
                if c > 0:
                    max_dollars = round(c * (max_pct / 100.0), 2)
            except (TypeError, ValueError):
                flags.append("capital_parse_failed")

        if manual_entry_overrides and manual_entry_overrides.get("recommended_strategy"):
            rec = str(manual_entry_overrides["recommended_strategy"])
            notes.append("recommended_strategy_manual_override")

        if not rec:
            flags.append("no_recommended_strategy_resolved")

        extra_notes = manual_entry_overrides.get("entry_notes") if manual_entry_overrides else None
        if isinstance(extra_notes, list):
            notes.extend(str(n) for n in extra_notes)
        elif isinstance(extra_notes, str) and extra_notes.strip():
            notes.append(extra_notes.strip())

        briefing = briefing or {}
        if briefing.get("event_risk_flag"):
            notes.append("event_risk_flag_true_size_conservatively")

        base.update(
            {
                "entry": "proposed",
                "reason": None,
                "recommended_strategy": rec,
                "position_size_units": None,
                "max_risk_budget_pct": max_pct,
                "max_risk_budget_dollars": max_dollars,
                "entry_notes": self._dedup(notes),
                "entry_quality_flags": self._dedup(flags),
            }
        )
        return base

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
