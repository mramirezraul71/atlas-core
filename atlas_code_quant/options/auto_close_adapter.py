"""Adaptador paper-only: produce reporte de candidatos a cierre desde portafolio."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from atlas_code_quant.execution.auto_close_engine import AutoCloseEngine
from atlas_code_quant.options.portfolio_analyzer import PortfolioAnalyzer


class AutoCloseReportBuilder:
    """Construye reporte estructurado de cierre sugerido.

    Criterio de orden en ``close_candidates``:
    1) prioridad (``high`` antes que ``normal``),
    2) mayor severidad económica relativa (pérdida o ganancia vs crédito de entrada).
    """

    def __init__(self, portfolio_analyzer: PortfolioAnalyzer, auto_close_engine: AutoCloseEngine) -> None:
        self._portfolio = portfolio_analyzer
        self._engine = auto_close_engine

    def build_report(
        self,
        *,
        include_hold: bool = False,
        intraday_context: dict[str, Any] | None = None,
        breach_context: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        entries = list(self._portfolio.get_open_entries())
        decisions = self._engine.scan_positions(
            entries,
            intraday_context=intraday_context,
            breach_context=breach_context,
        )

        close_candidates: list[dict[str, Any]] = []
        held_positions: list[dict[str, Any]] = []
        for entry, decision in zip(entries, decisions):
            row = self._compose_row(entry, decision)
            if decision.get("should_close"):
                close_candidates.append(row)
            elif include_hold:
                held_positions.append(
                    {
                        "position_id": row["position_id"],
                        "strategy_type": row["strategy_type"],
                        "priority": row["priority"],
                        "metrics": row["metrics"],
                    }
                )

        close_candidates.sort(key=self._candidate_sort_key)
        generated_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        return {
            "generated_at": generated_at,
            "total_positions": len(entries),
            "close_candidates": close_candidates,
            "held_positions": held_positions if include_hold else [],
        }

    def _compose_row(self, entry: Any, decision: dict[str, Any]) -> dict[str, Any]:
        strategy = self._obj_get(entry, "strategy", None)
        metadata = self._obj_get(strategy, "metadata", {}) if strategy is not None else {}
        if not isinstance(metadata, dict):
            metadata = {}
        return {
            "position_id": decision.get("position_id"),
            "strategy_type": decision.get("strategy_type"),
            "should_close": bool(decision.get("should_close")),
            "reasons": list(decision.get("reasons") or []),
            "priority": str(decision.get("priority") or "normal"),
            "recommended_action": str(decision.get("recommended_action") or "hold"),
            "metrics": dict(decision.get("metrics") or {}),
            "underlying": self._obj_get(entry, "underlying", None) or self._obj_get(strategy, "underlying", None),
            "opened_at": self._iso_or_none(self._obj_get(entry, "opened_at", None)),
            "tags": list(self._obj_get(entry, "tags", []) or []),
            "sync_key": metadata.get("sync_key"),
        }

    @staticmethod
    def _candidate_sort_key(row: dict[str, Any]) -> tuple[int, float]:
        priority = str(row.get("priority") or "normal").lower()
        priority_rank = 0 if priority == "high" else 1
        metrics = row.get("metrics") or {}
        entry_credit = AutoCloseReportBuilder._to_float(metrics.get("entry_credit"))
        pnl = AutoCloseReportBuilder._to_float(metrics.get("unrealized_pnl"))
        if entry_credit and entry_credit > 0 and pnl is not None:
            severity = abs(pnl) / entry_credit
        elif pnl is not None:
            severity = abs(pnl)
        else:
            severity = 0.0
        return (priority_rank, -severity)

    @staticmethod
    def _obj_get(obj: Any, key: str, default: Any = None) -> Any:
        if isinstance(obj, dict):
            return obj.get(key, default)
        return getattr(obj, key, default)

    @staticmethod
    def _iso_or_none(value: Any) -> str | None:
        if value is None:
            return None
        if isinstance(value, datetime):
            return value.isoformat()
        return str(value)

    @staticmethod
    def _to_float(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None
