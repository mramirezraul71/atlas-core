"""Planificador de entradas solo paper: propuesta analítica, sin órdenes ni broker."""
from __future__ import annotations

from copy import deepcopy
from typing import Any


class PaperEntryPlanner:
    """Traduce ``intent`` en un plan de entrada paper-only (no ejecuta órdenes)."""

    _DEFAULT_MAX_RISK_PCT = 2.0
    _DEFAULT_RISK_PER_UNIT = 200.0
    _RISK_PER_UNIT_BY_STRATEGY = {
        "bull_call_debit_spread": 150.0,
        "long_call": 120.0,
        "call_calendar_spread": 180.0,
        "iron_condor": 250.0,
        "iron_butterfly": 250.0,
        "bull_put_credit_spread": 220.0,
        "bear_call_credit_spread": 220.0,
    }

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
        capital_value = None
        if capital is not None:
            try:
                c = float(capital)
                if c > 0:
                    capital_value = c
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

        risk_per_unit = self._resolve_risk_per_unit(
            strategy=rec,
            manual_entry_overrides=manual_entry_overrides,
        )
        size, blocked_reason = self._compute_executable_size(
            capital_value=capital_value,
            max_risk_budget_dollars=max_dollars,
            risk_per_unit_dollars=risk_per_unit,
            manual_entry_overrides=manual_entry_overrides,
        )
        if blocked_reason:
            notes.append(f"entry_blocked:{blocked_reason}")
            flags.append(f"entry_blocked:{blocked_reason}")
        base.update(
            {
                "entry": "proposed" if size >= 1 else "blocked",
                "reason": None if size >= 1 else blocked_reason,
                "recommended_strategy": rec,
                "position_size_units": int(size) if size >= 1 else None,
                "size_blocked_reason": blocked_reason,
                "risk_per_unit_dollars": risk_per_unit,
                "max_risk_budget_pct": max_pct,
                "max_risk_budget_dollars": max_dollars,
                "entry_notes": self._dedup(notes),
                "entry_quality_flags": self._dedup(flags),
            }
        )
        return base

    def _resolve_risk_per_unit(
        self,
        *,
        strategy: Any,
        manual_entry_overrides: dict[str, Any] | None,
    ) -> float:
        if manual_entry_overrides and manual_entry_overrides.get("risk_per_unit_dollars") is not None:
            try:
                v = float(manual_entry_overrides["risk_per_unit_dollars"])
                if v > 0:
                    return v
            except (TypeError, ValueError):
                pass
        skey = str(strategy or "").strip().lower()
        return float(self._RISK_PER_UNIT_BY_STRATEGY.get(skey, self._DEFAULT_RISK_PER_UNIT))

    def _compute_executable_size(
        self,
        *,
        capital_value: float | None,
        max_risk_budget_dollars: float | None,
        risk_per_unit_dollars: float,
        manual_entry_overrides: dict[str, Any] | None,
    ) -> tuple[int, str | None]:
        if manual_entry_overrides and manual_entry_overrides.get("position_size_units") is not None:
            try:
                forced = int(float(manual_entry_overrides["position_size_units"]))
                if forced >= 1:
                    return forced, None
            except (TypeError, ValueError):
                return 0, "manual_position_size_invalid"
            return 0, "manual_position_size_invalid"
        if capital_value is None or capital_value <= 0:
            return 0, "insufficient_capital"
        if max_risk_budget_dollars is None or max_risk_budget_dollars <= 0:
            return 0, "risk_budget_unavailable"
        if risk_per_unit_dollars <= 0:
            return 0, "risk_model_invalid"
        max_by_risk = int(max_risk_budget_dollars // risk_per_unit_dollars)
        max_by_capital = int(capital_value // risk_per_unit_dollars)
        size = min(max_by_risk, max_by_capital)
        if size >= 1:
            return size, None
        if max_risk_budget_dollars < risk_per_unit_dollars:
            return 0, "min_contract_not_reached"
        return 0, "insufficient_capital"

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
