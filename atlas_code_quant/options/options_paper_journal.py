"""Journal JSONL local para flujo de opciones en modo paper-only (sin broker ni sistemas externos)."""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

_JOURNAL_VERSION = "1.0"
_ALLOWED_STATUS = {"open", "closed", "canceled", "phantom"}
_ALLOWED_MODE = {"paper"}


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


def _to_float_or_none(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int_or_none(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _normalize_mode(mode: str | None) -> str:
    raw = str(mode or "paper").strip().lower()
    return raw if raw in _ALLOWED_MODE else "paper"


def _normalize_status(status: str | None, *, default: str) -> str:
    raw = str(status or default).strip().lower()
    return raw if raw in _ALLOWED_STATUS else default


def _normalize_legs(raw_legs: Any) -> list[dict[str, Any]]:
    if not isinstance(raw_legs, list):
        return []
    legs: list[dict[str, Any]] = []
    for row in raw_legs:
        if not isinstance(row, dict):
            continue
        leg = {
            "kind": row.get("kind"),
            "side": row.get("side"),
            "strike": _to_float_or_none(row.get("strike")),
            "expiration": row.get("expiration"),
            "qty": _to_int_or_none(row.get("qty") if row.get("qty") is not None else row.get("quantity")),
        }
        legs.append(leg)
    return legs


def _legs_from_entry_payload(entry_payload: dict[str, Any]) -> list[dict[str, Any]]:
    legs = _normalize_legs(entry_payload.get("legs"))
    if legs:
        return legs
    mapped: list[dict[str, Any]] = []
    expiry = entry_payload.get("expiration")
    qty = _to_int_or_none(entry_payload.get("qty") if entry_payload.get("qty") is not None else 1) or 1
    strike_map = (
        ("short_put", "put", "short"),
        ("long_put", "put", "long"),
        ("short_call", "call", "short"),
        ("long_call", "call", "long"),
    )
    for key, kind, side in strike_map:
        strike = _to_float_or_none(entry_payload.get(key))
        if strike is None:
            continue
        mapped.append(
            {
                "kind": kind,
                "side": side,
                "strike": strike,
                "expiration": expiry,
                "qty": qty,
            }
        )
    return mapped


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

    def _build_base_record(
        self,
        *,
        event_type: str,
        trace_id: str,
        timestamp: datetime,
        symbol: str,
        mode: str | None,
        source: str | None,
        status: str | None,
        autoclose_applied: bool,
        notes: list[str] | None,
    ) -> dict[str, Any]:
        ts = _iso(timestamp)
        underlying = str(symbol or "").strip().upper()
        base_status = "closed" if event_type == "close_execution" else "open"
        return {
            "event_type": event_type,
            "trace_id": str(trace_id).strip(),
            "timestamp_utc": ts,
            "timestamp": ts,  # compat hacia atrás para readers existentes.
            "mode": _normalize_mode(mode),
            "source": str(source or "").strip() or "unknown",
            "underlying": underlying,
            "symbol": underlying,  # compat hacia atrás.
            "status": _normalize_status(status, default=base_status),
            "autoclose_applied": bool(autoclose_applied),
            "journal_version": _JOURNAL_VERSION,
            "notes": list(notes) if notes else [],
            "structure_type": None,
            "dte_at_entry": None,
            "legs": [],
            "entry_credit": None,
            "entry_debit": None,
            "entry_mid": None,
            "max_loss": None,
            "max_profit": None,
            "close_type": None,
            "close_reason": None,
            "close_debit": None,
            "close_credit": None,
            "close_mid": None,
            "pnl_usd": None,
            "pnl_pct": None,
        }

    def _append(self, record: dict[str, Any]) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        line = _json_dumps_line(record) + "\n"
        with self._path.open("a", encoding="utf-8") as f:
            f.write(line)
        try:
            from atlas_code_quant.options.options_engine_metrics import on_journal_record

            on_journal_record(self._path, record)
        except Exception:
            pass

    def log_session_plan(
        self,
        session_plan: dict[str, Any],
        *,
        mode: str = "paper",
        source: str = "planner",
        status: str = "open",
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

        symbol = str(session_plan.get("symbol") or "").strip().upper()
        now = _utc_now()
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

        rec = self._build_base_record(
            event_type="session_plan",
            trace_id=trace_id,
            timestamp=now,
            symbol=symbol,
            mode=mode,
            source=source,
            status=status,
            autoclose_applied=False,
            notes=merged_notes,
        )
        rec["structure_type"] = entry_plan.get("recommended_strategy")
        rec["dte_at_entry"] = _to_int_or_none(briefing.get("dte"))
        rec["payload"] = payload
        self._append(rec)
        return trace_id

    def log_entry_execution(
        self,
        *,
        trace_id: str,
        symbol: str,
        entry_timestamp: datetime,
        planned_entry: dict[str, Any] | None = None,
        executed_entry: dict[str, Any] | None = None,
        mode: str = "paper",
        source: str = "manual",
        status: str = "open",
        autoclose_applied: bool = False,
        notes: list[str] | None = None,
    ) -> None:
        sym = str(symbol or "").strip().upper()
        exec_row = executed_entry if isinstance(executed_entry, dict) else {}
        planned_row = planned_entry if isinstance(planned_entry, dict) else {}
        rec = self._build_base_record(
            event_type="entry_execution",
            trace_id=str(trace_id),
            timestamp=entry_timestamp,
            symbol=sym,
            mode=mode,
            source=source,
            status=status,
            autoclose_applied=autoclose_applied,
            notes=notes,
        )
        rec["structure_type"] = exec_row.get("structure") or planned_row.get("structure") or planned_row.get(
            "recommended_strategy"
        )
        rec["dte_at_entry"] = _to_int_or_none(exec_row.get("dte") if exec_row.get("dte") is not None else planned_row.get("dte"))
        rec["legs"] = _legs_from_entry_payload(exec_row) or _legs_from_entry_payload(planned_row)
        rec["entry_credit"] = _to_float_or_none(
            exec_row.get("entry_credit") if exec_row.get("entry_credit") is not None else exec_row.get("credit")
        )
        rec["entry_debit"] = _to_float_or_none(
            exec_row.get("entry_debit") if exec_row.get("entry_debit") is not None else exec_row.get("debit")
        )
        rec["entry_mid"] = _to_float_or_none(exec_row.get("entry_mid") if exec_row.get("entry_mid") is not None else exec_row.get("mid"))
        rec["max_loss"] = _to_float_or_none(
            exec_row.get("max_loss") if exec_row.get("max_loss") is not None else planned_row.get("max_loss")
        )
        rec["max_profit"] = _to_float_or_none(
            exec_row.get("max_profit") if exec_row.get("max_profit") is not None else planned_row.get("max_profit")
        )
        rec["payload"] = {
            "planned_entry": planned_entry,
            "executed_entry": executed_entry,
        }
        self._append(rec)

    def log_close_decision(
        self,
        *,
        trace_id: str,
        close_decision: dict[str, Any],
        timestamp: datetime,
        mode: str = "paper",
        source: str = "autoclose",
        status: str = "open",
        autoclose_applied: bool = False,
        notes: list[str] | None = None,
    ) -> None:
        sym = ""
        if isinstance(close_decision, dict):
            sym = str(close_decision.get("symbol") or close_decision.get("underlying") or "")
        rec = self._build_base_record(
            event_type="close_decision",
            trace_id=str(trace_id),
            timestamp=timestamp,
            symbol=sym,
            mode=mode,
            source=source,
            status=status,
            autoclose_applied=autoclose_applied,
            notes=notes,
        )
        decision = close_decision if isinstance(close_decision, dict) else {}
        rec["close_reason"] = decision.get("reason")
        rec["close_type"] = decision.get("close_type") or "full"
        rec["close_debit"] = _to_float_or_none(decision.get("debit"))
        rec["close_credit"] = _to_float_or_none(decision.get("credit"))
        rec["close_mid"] = _to_float_or_none(decision.get("close_mid") if decision.get("close_mid") is not None else decision.get("exit_mid"))
        rec["payload"] = {"close_decision": close_decision}
        self._append(rec)

    def log_close_execution(
        self,
        *,
        trace_id: str,
        symbol: str,
        close_timestamp: datetime,
        executed_close: dict[str, Any] | None = None,
        pnl_realized: float | None = None,
        mode: str = "paper",
        source: str = "autoclose",
        status: str = "closed",
        autoclose_applied: bool = False,
        close_type: str | None = None,
        close_reason: str | None = None,
        pnl_pct: float | None = None,
        notes: list[str] | None = None,
    ) -> None:
        sym = str(symbol or "").strip().upper()
        close_row = executed_close if isinstance(executed_close, dict) else {}
        rec = self._build_base_record(
            event_type="close_execution",
            trace_id=str(trace_id),
            timestamp=close_timestamp,
            symbol=sym,
            mode=mode,
            source=source,
            status=status,
            autoclose_applied=autoclose_applied,
            notes=notes,
        )
        rec["close_type"] = close_type or close_row.get("close_type") or "full"
        rec["close_reason"] = close_reason or close_row.get("close_reason") or close_row.get("reason")
        rec["close_debit"] = _to_float_or_none(close_row.get("close_debit") if close_row.get("close_debit") is not None else close_row.get("debit"))
        rec["close_credit"] = _to_float_or_none(close_row.get("close_credit") if close_row.get("close_credit") is not None else close_row.get("credit"))
        rec["close_mid"] = _to_float_or_none(close_row.get("close_mid") if close_row.get("close_mid") is not None else close_row.get("exit_mid"))
        rec["pnl_usd"] = _to_float_or_none(pnl_realized if pnl_realized is not None else close_row.get("pnl_usd"))
        rec["pnl_pct"] = _to_float_or_none(pnl_pct if pnl_pct is not None else close_row.get("pnl_pct"))
        rec["payload"] = {
            "executed_close": executed_close,
            "pnl_realized": rec["pnl_usd"],
        }
        self._append(rec)

    def read_events(self) -> list[dict[str, Any]]:
        """Reader mínimo para reconstrucción offline (paper) sin dependencias externas."""
        if not self._path.is_file():
            return []
        rows: list[dict[str, Any]] = []
        for raw_line in self._path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(row, dict):
                rows.append(row)
        return rows
