"""Journal JSONL local para flujo de opciones en modo paper-only (sin broker ni sistemas externos)."""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso(dt: datetime) -> str:
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    else:
        dt = dt.astimezone(timezone.utc)
    return dt.isoformat().replace("+00:00", "Z")


def _json_dumps_line(obj: dict[str, Any]) -> str:
    return json.dumps(obj, ensure_ascii=False, separators=(",", ":"), default=str)


def _briefing_summary(briefing: dict[str, Any] | None) -> dict[str, Any]:
    if not briefing:
        return {}
    keys = (
        "direction",
        "regime",
        "gamma_regime",
        "dte_mode",
        "market_bias",
        "risk_posture",
        "recommended_strategy",
        "recommended_family",
        "event_risk_flag",
        "visual_breach_flag",
    )
    return {k: briefing[k] for k in keys if k in briefing}


def _intent_summary(intent: dict[str, Any] | None) -> dict[str, Any]:
    if not intent:
        return {}
    keys = (
        "allow_entry",
        "force_no_trade",
        "recommended_strategy",
        "risk_posture",
        "entry_bias",
        "market_bias",
        "suppress_short_premium",
        "prefer_defined_risk",
    )
    return {k: intent[k] for k in keys if k in intent}


def _entry_plan_summary(entry_plan: dict[str, Any] | None) -> dict[str, Any]:
    if not entry_plan:
        return {}
    keys = ("entry", "recommended_strategy", "reason", "mode", "max_risk_budget_dollars", "max_risk_budget_pct")
    return {k: entry_plan[k] for k in keys if k in entry_plan}


class OptionsPaperJournal:
    """Append-only JSONL: session_plan → entry_execution → close_decision → close_execution por ``trace_id``."""

    _DEFAULT_REL_PATH = Path("data") / "options_paper_journal.jsonl"

    def __init__(self, *, path: str | Path | None = None) -> None:
        self._path = Path(path) if path is not None else Path.cwd() / self._DEFAULT_REL_PATH

    @property
    def path(self) -> Path:
        return self._path

    def _append(self, record: dict[str, Any]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        line = _json_dumps_line(record) + "\n"
        with self._path.open("a", encoding="utf-8") as f:
            f.write(line)

    def log_session_plan(
        self,
        session_plan: dict[str, Any],
        *,
        notes: list[str] | None = None,
    ) -> str:
        """
        Registra un plan de sesión completo.
        Devuelve el ``trace_id`` asociado (existente en ``session_plan`` o UUID v4 nuevo).
        """
        raw_tid = session_plan.get("trace_id")
        trace_id = str(raw_tid).strip() if raw_tid is not None else ""
        if not trace_id:
            trace_id = str(uuid.uuid4())

        symbol = str(session_plan.get("symbol") or "")
        ts = _iso(_utc_now())
        briefing = session_plan.get("briefing") if isinstance(session_plan.get("briefing"), dict) else {}
        intent = session_plan.get("intent") if isinstance(session_plan.get("intent"), dict) else {}
        entry_plan = session_plan.get("entry_plan") if isinstance(session_plan.get("entry_plan"), dict) else {}

        plan_copy = dict(session_plan)
        plan_copy["trace_id"] = trace_id

        merged_notes = list(notes) if notes else []
        if not merged_notes:
            merged_notes = ["paper_session_plan"]

        payload: dict[str, Any] = {
            "session_plan": plan_copy,
            "briefing_summary": _briefing_summary(briefing),
            "intent_summary": _intent_summary(intent),
            "entry_plan_summary": _entry_plan_summary(entry_plan),
        }

        self._append(
            {
                "event_type": "session_plan",
                "trace_id": trace_id,
                "symbol": symbol,
                "timestamp": ts,
                "payload": payload,
                "notes": merged_notes,
            }
        )
        return trace_id

    def log_entry_execution(
        self,
        *,
        trace_id: str,
        symbol: str,
        entry_timestamp: datetime,
        planned_entry: dict[str, Any] | None = None,
        executed_entry: dict[str, Any] | None = None,
        notes: list[str] | None = None,
    ) -> None:
        self._append(
            {
                "event_type": "entry_execution",
                "trace_id": str(trace_id),
                "symbol": str(symbol or ""),
                "timestamp": _iso(entry_timestamp),
                "payload": {
                    "planned_entry": planned_entry,
                    "executed_entry": executed_entry,
                },
                "notes": list(notes) if notes else [],
            }
        )

    def log_close_decision(
        self,
        *,
        trace_id: str,
        close_decision: dict[str, Any],
        timestamp: datetime,
        notes: list[str] | None = None,
    ) -> None:
        sym = ""
        if isinstance(close_decision, dict):
            sym = str(close_decision.get("symbol") or close_decision.get("underlying") or "")
        self._append(
            {
                "event_type": "close_decision",
                "trace_id": str(trace_id),
                "symbol": sym,
                "timestamp": _iso(timestamp),
                "payload": {"close_decision": close_decision},
                "notes": list(notes) if notes else [],
            }
        )

    def log_close_execution(
        self,
        *,
        trace_id: str,
        symbol: str,
        close_timestamp: datetime,
        executed_close: dict[str, Any] | None = None,
        pnl_realized: float | None = None,
        notes: list[str] | None = None,
    ) -> None:
        self._append(
            {
                "event_type": "close_execution",
                "trace_id": str(trace_id),
                "symbol": str(symbol or ""),
                "timestamp": _iso(close_timestamp),
                "payload": {
                    "executed_close": executed_close,
                    "pnl_realized": pnl_realized,
                },
                "notes": list(notes) if notes else [],
            }
        )
