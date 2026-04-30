"""Local intraday ledger for conservative PDT guard rails."""
from __future__ import annotations

import json
from datetime import date, datetime, timedelta
from pathlib import Path
from typing import Any

from atlas_code_quant.api.schemas import OrderRequest
from atlas_code_quant.config.settings import settings


def _ledger_path() -> Path:
    path = settings.logs_dir / "tradier_pdt_ledger.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _last_business_days(end_day: date, count: int) -> set[date]:
    days: list[date] = []
    cursor = end_day
    while len(days) < count:
        if cursor.weekday() < 5:
            days.append(cursor)
        cursor -= timedelta(days=1)
    return set(days)


def _stringify_qty(value: float | int | None) -> str:
    quantity = float(value or 0)
    if quantity.is_integer():
        return str(int(quantity))
    return f"{quantity:.8f}".rstrip("0").rstrip(".")


def _security_key(body: OrderRequest, order_class: str) -> str:
    if order_class == "equity":
        return f"EQUITY:{body.symbol.upper()}"
    if order_class == "option":
        return f"OPTION:{str(body.option_symbol or '').upper()}"
    tokens: list[str] = []
    for leg in body.legs:
        if leg.instrument_type == "equity":
            tokens.append(f"EQ:{body.symbol.upper()}:{_stringify_qty(abs(leg.quantity))}")
        else:
            tokens.append(f"OPT:{str(leg.option_symbol or '').upper()}:{_stringify_qty(abs(leg.quantity))}")
    strategy = str(body.strategy_type or order_class).upper()
    return f"{order_class.upper()}:{strategy}:{'|'.join(sorted(tokens))}"


def _response_is_accepted(response: dict[str, Any]) -> bool:
    status = str((response or {}).get("status") or "").strip().lower()
    if status in {"error", "rejected", "failed", "canceled", "cancelled"}:
        return False
    result = (response or {}).get("result")
    if isinstance(result, bool):
        return result
    return bool(status or response)


def record_live_order_intent(
    *,
    account_id: str,
    scope: str,
    body: OrderRequest,
    order_class: str,
    position_effect: str,
    broker_response: dict[str, Any],
) -> dict[str, Any] | None:
    if scope != "live" or body.preview:
        return None
    if not _response_is_accepted(broker_response):
        return None

    now = datetime.utcnow()
    entry = {
        "recorded_at": now.isoformat(),
        "trade_date": now.date().isoformat(),
        "account_id": account_id,
        "scope": scope,
        "symbol": body.symbol.upper(),
        "strategy_type": body.strategy_type,
        "order_class": order_class,
        "position_effect": position_effect,
        "security_key": _security_key(body, order_class),
        "preview": False,
        "broker_order_id": broker_response.get("id") or broker_response.get("order_id"),
        "broker_status": broker_response.get("status") or broker_response.get("order_status"),
    }
    with _ledger_path().open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(entry, ensure_ascii=True) + "\n")
    return entry


def _load_entries(*, account_id: str, reference_day: date) -> list[dict[str, Any]]:
    path = _ledger_path()
    if not path.exists():
        return []
    business_window = _last_business_days(reference_day, settings.tradier_pdt_window_days)
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            text = line.strip()
            if not text:
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError:
                continue
            if str(item.get("account_id") or "").strip() != account_id:
                continue
            raw_day = str(item.get("trade_date") or "")[:10]
            try:
                trade_day = datetime.strptime(raw_day, "%Y-%m-%d").date()
            except ValueError:
                continue
            if trade_day not in business_window:
                continue
            rows.append(item)
    return rows


def count_intraday_day_trades(
    *,
    account_id: str,
    reference_day: date,
) -> tuple[int, list[dict[str, Any]]]:
    tracker: dict[tuple[str, str], set[str]] = {}
    for item in _load_entries(account_id=account_id, reference_day=reference_day):
        trade_day = str(item.get("trade_date") or "")[:10]
        security_key = str(item.get("security_key") or "").strip()
        position_effect = str(item.get("position_effect") or "").strip().lower()
        if not trade_day or not security_key or position_effect not in {"open", "close"}:
            continue
        key = (trade_day, security_key)
        tracker.setdefault(key, set()).add(position_effect)
    details = [
        {
            "date": trade_day,
            "security_key": security_key,
            "actions": sorted(list(actions)),
            "source": "atlas_intraday_ledger",
        }
        for (trade_day, security_key), actions in sorted(tracker.items())
        if {"open", "close"}.issubset(actions)
    ]
    return len(details), details
