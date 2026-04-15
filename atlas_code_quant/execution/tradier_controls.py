"""Tradier account resolution and PDT guard rails."""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from datetime import date, datetime, timedelta, timezone
from typing import Any

from atlas_code_quant.backtesting.winning_probability import TradierClient, TradierScope, _safe_float
from atlas_code_quant.config.settings import settings
from atlas_code_quant.execution.tradier_pdt_ledger import count_intraday_day_trades


logger = logging.getLogger("quant.execution.tradier_controls")

_SESSION_CACHE: dict[tuple[str, str], "TradierAccountSession"] = {}
_SESSION_CACHE_TS: dict[tuple[str, str], float] = {}
_SESSION_CACHE_TTL_SEC = max(
    0.0,
    float(getattr(settings, "tradier_account_session_cache_ttl_sec", 5.0) or 5.0),
)


@dataclass
class TradierAccountSession:
    scope: TradierScope
    classification: str
    account_id: str
    account_name: str | None
    base_url: str
    total_equity: float | None
    resolved_at: str
    raw_account: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "scope": self.scope,
            "classification": self.classification,
            "account_id": self.account_id,
            "account_name": self.account_name,
            "base_url": self.base_url,
            "total_equity": self.total_equity,
            "resolved_at": self.resolved_at,
        }


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _pick_account_id(scope: TradierScope, account_id: str | None, accounts: list[dict[str, Any]]) -> str:
    preferred = (account_id or (settings.tradier_live_account_id if scope == "live" else settings.tradier_paper_account_id) or "").strip()
    if preferred:
        return preferred
    if not accounts:
        raise ValueError(f"No Tradier accounts available for scope '{scope}'")
    candidate = str(accounts[0].get("account_number") or accounts[0].get("account_id") or "").strip()
    if not candidate:
        raise ValueError(f"Unable to resolve account_id for scope '{scope}'")
    return candidate


def _find_account(accounts: list[dict[str, Any]], account_id: str) -> dict[str, Any]:
    for account in accounts:
        candidate = str(account.get("account_number") or account.get("account_id") or "").strip()
        if candidate == account_id:
            return account
    return {}


def _account_name(account: dict[str, Any]) -> str | None:
    for key in ("nickname", "name", "label", "type"):
        value = str(account.get(key) or "").strip()
        if value:
            return value
    return None


def _parse_total_equity(balances: dict[str, Any]) -> float | None:
    for key in (
        "total_equity",
        "equity",
        "account_equity",
        "net_liquidating_value",
        "net_liq",
        "option_short_value",
        "total_value",
    ):
        value = _safe_float(balances.get(key), float("nan"))
        if value == value and value > 0:
            return value
    return None


def resolve_account_session(
    *,
    account_scope: TradierScope | None = None,
    account_id: str | None = None,
    tradier_token: str | None = None,
    tradier_base_url: str | None = None,
    force_refresh: bool = False,
) -> tuple[TradierClient, TradierAccountSession]:
    scope: TradierScope = account_scope or settings.tradier_default_scope  # type: ignore[assignment]
    cache_key = (scope, (account_id or "").strip())
    if not force_refresh and cache_key in _SESSION_CACHE:
        cache_age = time.monotonic() - _SESSION_CACHE_TS.get(cache_key, 0.0)
        if cache_age <= _SESSION_CACHE_TTL_SEC:
            client = TradierClient(scope=scope, token=tradier_token, base_url=tradier_base_url)
            return client, _SESSION_CACHE[cache_key]

    client = TradierClient(scope=scope, token=tradier_token, base_url=tradier_base_url)
    accounts = client.accounts()
    resolved_account_id = _pick_account_id(scope, account_id, accounts)
    account = _find_account(accounts, resolved_account_id)
    balances = {}
    try:
        balances = client.balances(resolved_account_id)
    except Exception:
        logger.exception("Unable to fetch balances for Tradier account %s", resolved_account_id)
    session = TradierAccountSession(
        scope=scope,
        classification="paper" if scope == "paper" or "sandbox.tradier.com" in client.base_url else "live",
        account_id=resolved_account_id,
        account_name=_account_name(account),
        base_url=client.base_url,
        total_equity=_parse_total_equity(balances),
        resolved_at=datetime.utcnow().isoformat(),
        raw_account=account,
    )
    _SESSION_CACHE[cache_key] = session
    _SESSION_CACHE_TS[cache_key] = time.monotonic()
    return client, session


def _last_business_days(end_day: date, count: int) -> set[date]:
    days: list[date] = []
    cursor = end_day
    while len(days) < count:
        if cursor.weekday() < 5:
            days.append(cursor)
        cursor -= timedelta(days=1)
    return set(days)


def _event_timestamp(item: dict[str, Any]) -> datetime | None:
    for key in ("date", "transaction_date", "trade_date", "created_at", "executed_at"):
        value = item.get(key)
        if not value:
            continue
        text = str(value)
        try:
            return datetime.fromisoformat(text.replace("Z", "+00:00"))
        except ValueError:
            try:
                return datetime.strptime(text[:10], "%Y-%m-%d").replace(tzinfo=timezone.utc)
            except ValueError:
                continue
    return None


def _event_date(item: dict[str, Any]) -> date | None:
    timestamp = _event_timestamp(item)
    return timestamp.date() if timestamp else None


def _event_symbol(item: dict[str, Any]) -> str:
    for key in ("option_symbol", "occ_symbol", "symbol", "description"):
        value = str(item.get(key) or "").strip()
        if value:
            return value
    return "UNKNOWN"


def _event_action(item: dict[str, Any]) -> str:
    for key in ("side", "transaction_type", "type", "action", "description"):
        value = str(item.get(key) or "").strip().upper()
        if value:
            return value
    return ""


def _event_status(item: dict[str, Any]) -> str:
    for key in ("status", "order_status", "exec_status"):
        value = str(item.get(key) or "").strip().lower()
        if value:
            return value
    return ""


def _event_exec_quantity(item: dict[str, Any]) -> float:
    for key in ("exec_quantity", "executed_quantity", "filled_quantity", "filled_qty"):
        value = _safe_float(item.get(key), float("nan"))
        if value == value and value > 0:
            return value
    return 0.0


def _is_executed_event(item: dict[str, Any]) -> bool:
    if str(item.get("type") or "").strip().lower() == "trade":
        return True
    status = _event_status(item)
    if status in {"filled", "partially_filled", "executed"}:
        return True
    return _event_exec_quantity(item) > 0


def _is_buy_action(action: str) -> bool:
    return any(token in action for token in ("BUY", "BTO", "BTC"))


def _is_sell_action(action: str) -> bool:
    return any(token in action for token in ("SELL", "STO", "STC"))


def _action_bucket(action: str) -> str | None:
    normalized = action.strip().upper()
    if "BUY_TO_OPEN" in normalized or "BTO" in normalized:
        return "open_long"
    if "SELL_TO_CLOSE" in normalized or "STC" in normalized:
        return "close_long"
    if "SELL_TO_OPEN" in normalized or "STO" in normalized or "SELL_SHORT" in normalized:
        return "open_short"
    if "BUY_TO_CLOSE" in normalized or "BTC" in normalized or "BUY_TO_COVER" in normalized:
        return "close_short"
    if normalized == "BUY":
        return "buy"
    if normalized == "SELL":
        return "sell"
    return None


def _recent_filled_events(events: list[dict[str, Any]], *, limit: int) -> list[dict[str, Any]]:
    eligible: list[tuple[datetime, dict[str, Any]]] = []
    for event in events:
        if not _is_executed_event(event):
            continue
        timestamp = _event_timestamp(event)
        if timestamp is None:
            continue
        eligible.append((timestamp, event))
    eligible.sort(key=lambda item: item[0], reverse=True)
    return [item[1] for item in eligible[:limit]]


def _count_day_trades(events: list[dict[str, Any]], *, reference_day: date) -> tuple[int, list[dict[str, Any]], bool]:
    business_window = _last_business_days(reference_day, settings.tradier_pdt_window_days)
    tracker: dict[tuple[date, str], set[str]] = {}
    parse_ok = False
    for event in events:
        trade_day = _event_date(event)
        if trade_day is None or trade_day not in business_window:
            continue
        action = _event_action(event)
        if not action:
            continue
        bucket = _action_bucket(action)
        if bucket is None:
            continue
        parse_ok = True
        key = (trade_day, _event_symbol(event))
        sides = tracker.setdefault(key, set())
        sides.add(bucket)
    details = [
        {"date": trade_day.isoformat(), "symbol": symbol, "actions": sorted(list(sides))}
        for (trade_day, symbol), sides in sorted(tracker.items(), key=lambda item: item[0])
        if (
            {"open_long", "close_long"}.issubset(sides)
            or {"open_short", "close_short"}.issubset(sides)
            or {"buy", "sell"}.issubset(sides)
        )
    ]
    return len(details), details, parse_ok


def check_pdt_status(
    client: TradierClient,
    session: TradierAccountSession,
    *,
    reference_day: date | None = None,
) -> dict[str, Any]:
    today = reference_day or datetime.utcnow().date()
    result = {
        "applicable": session.classification == "live",
        "scope": session.scope,
        "account_id": session.account_id,
        "total_equity": session.total_equity,
        "min_equity_required": settings.tradier_pdt_min_equity,
        "max_day_trades_before_block": settings.tradier_pdt_max_day_trades,
        "window_business_days": settings.tradier_pdt_window_days,
        "day_trades_last_window": 0,
        "day_trades_remaining": settings.tradier_pdt_max_day_trades,
        "day_trade_details": [],
        "broker_day_trades_last_window": 0,
        "broker_day_trade_details": [],
        "filled_events_analyzed": 0,
        "ledger_day_trades_today": 0,
        "ledger_day_trade_details": [],
        "combined_day_trades_last_window": 0,
        "blocked_opening": False,
        "can_open": True,
        "reason": None,
        "fail_closed": settings.tradier_pdt_fail_closed,
    }
    if session.classification != "live":
        result["reason"] = "PDT guard rail only applies to live accounts"
        return result

    if session.total_equity is None:
        if settings.tradier_pdt_fail_closed:
            result["blocked_opening"] = True
            result["can_open"] = False
            result["reason"] = "Unable to determine live account equity"
        else:
            result["reason"] = "Unable to determine live account equity"
        return result

    if session.total_equity >= settings.tradier_pdt_min_equity:
        result["reason"] = "Account equity is above PDT minimum"
        return result

    start_day = today - timedelta(days=max(60, settings.tradier_pdt_window_days * 4))
    try:
        history_events = client.account_history(
            session.account_id,
            history_type="trade",
            start=start_day,
            end=today,
        )
        current_orders = client.orders(session.account_id)
    except Exception as exc:
        logger.exception("Unable to evaluate PDT status for %s", session.account_id)
        if settings.tradier_pdt_fail_closed:
            result["blocked_opening"] = True
            result["can_open"] = False
        result["reason"] = f"PDT evaluation failed: {exc}"
        return result

    recent_filled_events = _recent_filled_events(
        history_events + current_orders,
        limit=settings.tradier_pdt_history_fill_limit,
    )
    broker_day_trade_count, broker_day_trade_details, parse_ok = _count_day_trades(
        recent_filled_events,
        reference_day=today,
    )
    ledger_day_trade_count, ledger_day_trade_details = count_intraday_day_trades(
        account_id=session.account_id,
        reference_day=today,
    )
    broker_prior_days = sum(1 for item in broker_day_trade_details if str(item.get("date") or "") != today.isoformat())
    broker_today = sum(1 for item in broker_day_trade_details if str(item.get("date") or "") == today.isoformat())
    combined_day_trade_count = broker_prior_days + max(broker_today, ledger_day_trade_count)

    result["broker_day_trades_last_window"] = broker_day_trade_count
    result["broker_day_trade_details"] = broker_day_trade_details
    result["filled_events_analyzed"] = len(recent_filled_events)
    result["ledger_day_trades_today"] = ledger_day_trade_count
    result["ledger_day_trade_details"] = ledger_day_trade_details
    result["combined_day_trades_last_window"] = combined_day_trade_count
    result["day_trades_last_window"] = combined_day_trade_count
    result["day_trades_remaining"] = max(0, settings.tradier_pdt_max_day_trades - combined_day_trade_count)
    result["day_trade_details"] = broker_day_trade_details + ledger_day_trade_details

    if not parse_ok and settings.tradier_pdt_fail_closed:
        result["blocked_opening"] = True
        result["can_open"] = False
        result["reason"] = "Unable to parse Tradier trade history safely"
        return result

    if combined_day_trade_count >= settings.tradier_pdt_max_day_trades:
        result["blocked_opening"] = True
        result["can_open"] = False
        result["reason"] = (
            f"Live account below {settings.tradier_pdt_min_equity:.0f} with "
            f"{combined_day_trade_count} day trades in the last {settings.tradier_pdt_window_days} business days"
        )
        return result

    result["reason"] = "PDT threshold not reached"
    return result
