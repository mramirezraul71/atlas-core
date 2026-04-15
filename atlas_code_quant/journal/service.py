"""Trading journal synchronization and analytics services."""
from __future__ import annotations

import hashlib
import json
import logging
import math
import threading
from collections import Counter, defaultdict
from datetime import date, datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
from sqlalchemy import delete, func, or_, select

from backtesting.winning_probability import TradierClient, _compute_realized_vol, _safe_float
from execution.tradier_controls import (
    TradierAccountSession,
    _action_bucket,
    _event_action,
    _event_symbol,
    _event_timestamp,
    resolve_account_session,
)
try:
    from atlas_code_quant.data.signal_validator import SignalValidator, ValidationResult
except ModuleNotFoundError:  # pragma: no cover - runtime fallback for package imports
    from data.signal_validator import SignalValidator, ValidationResult
try:
    from atlas_code_quant.data.var_monitor import VarMonitor
except ModuleNotFoundError:  # pragma: no cover - runtime fallback for package imports
    from data.var_monitor import VarMonitor
from atlas_code_quant.journal.db import init_db, session_scope
from atlas_code_quant.journal.models import TradingJournal
from config.settings import settings
from learning.adaptive_policy import AdaptiveLearningService
from learning.ic_signal_tracker import ICSignalTracker, get_ic_tracker
from learning.journal_data_quality import build_journal_quality_scorecard, filter_closed_entries, row_quality_flags
from monitoring.strategy_tracker import StrategyTracker
from operations.brain_bridge import QuantBrainBridge

logger = logging.getLogger("quant.journal")

_SAFE_ADAPTIVE_REBUILD_DATE = date(2026, 4, 11)

LEVEL4_STRATEGIES = {
    "bear_call_credit_spread",
    "bull_put_credit_spread",
    "call_credit_butterfly",
    "put_credit_butterfly",
    "call_credit_condor",
    "put_credit_condor",
    "call_debit_butterfly",
    "put_debit_butterfly",
    "call_debit_condor",
    "put_debit_condor",
    "iron_condor",
    "iron_butterfly",
    "calendar_spread",
    "call_calendar_spread",
    "put_calendar_spread",
    "call_diagonal_debit_spread",
    "put_diagonal_debit_spread",
}

OPTION_SINGLE_LEG_STRATEGIES = {
    "long_call",
    "long_put",
}

OPTION_DIRECTIONAL_VERTICAL_STRATEGIES = {
    "bull_call_debit_spread",
    "bear_put_debit_spread",
    "bull_put_credit_spread",
    "bear_call_credit_spread",
}

OPTION_NEUTRAL_THETA_STRATEGIES = {
    "iron_condor",
    "iron_butterfly",
    "call_credit_condor",
    "put_credit_condor",
    "call_credit_butterfly",
    "put_credit_butterfly",
}

OPTION_TIME_SPREAD_STRATEGIES = {
    "calendar_spread",
    "call_calendar_spread",
    "put_calendar_spread",
    "call_diagonal_debit_spread",
    "put_diagonal_debit_spread",
}

OPTION_STRATEGY_FAMILY_MAP = {
    **{name: "single_leg_directional" for name in OPTION_SINGLE_LEG_STRATEGIES},
    **{name: "directional_vertical" for name in OPTION_DIRECTIONAL_VERTICAL_STRATEGIES},
    **{name: "neutral_theta" for name in OPTION_NEUTRAL_THETA_STRATEGIES},
    **{name: "term_structure_time_spread" for name in OPTION_TIME_SPREAD_STRATEGIES},
}


def _json_dump(value: Any) -> str:
    return json.dumps(value or {}, ensure_ascii=True, default=str)


def _json_load(value: str | None, fallback: Any) -> Any:
    if not value:
        return fallback
    try:
        return json.loads(value)
    except Exception:
        return fallback


def _coerce_list_payload(value: Any) -> list[Any]:
    if value is None or value == "" or value == {}:
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, dict):
        return [value] if value else []
    return [value]


def _extract_broker_order_id(value: Any) -> str | None:
    if isinstance(value, dict):
        candidate = value.get("id") or value.get("order_id") or value.get("broker_order_id")
        if candidate not in {None, ""}:
            return str(candidate)
    elif isinstance(value, list):
        for item in value:
            if item in {None, ""}:
                continue
            return str(item)
    elif value not in {None, ""}:
        return str(value)
    return None


def _normalize_broker_position(position: dict[str, Any]) -> dict[str, Any]:
    symbol = str(position.get("symbol") or position.get("underlying") or "").strip().upper()
    quantity = abs(
        _safe_float(
            position.get("quantity")
            or position.get("qty")
            or position.get("size")
            or position.get("shares"),
            0.0,
        )
    )
    broker_id = _extract_broker_order_id(
        position.get("id")
        or position.get("order_id")
        or position.get("broker_order_id")
        or position.get("broker_id")
    )
    return {
        "symbol": symbol,
        "quantity": quantity,
        "broker_id": broker_id,
        "raw": position,
    }


def _is_recent_entry(entry_time: datetime | None, days: int = 1) -> bool:
    """True when the entry was created within the last `days`."""
    if entry_time is None:
        return False
    if entry_time.tzinfo is not None:
        now = datetime.now(entry_time.tzinfo)
    else:
        now = datetime.utcnow()
    age = now - entry_time
    return age <= timedelta(days=max(days, 0))


def _entry_quantity(entry: TradingJournal) -> float:
    legs = _json_load(entry.legs_details, [])
    if isinstance(legs, list) and legs:
        total = 0.0
        for leg in legs:
            if not isinstance(leg, dict):
                continue
            signed_qty = _safe_float(leg.get("signed_qty"), float("nan"))
            if signed_qty == signed_qty and signed_qty != 0:
                total += abs(signed_qty)
                continue
            qty = _safe_float(leg.get("quantity"), float("nan"))
            if qty == qty and qty != 0:
                total += abs(qty)
        if total > 0:
            return total
    return 1.0


def _sanitize_rebuilt_rows(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    valid_rows: list[dict[str, Any]] = []
    invalid_rows: list[dict[str, Any]] = []
    for row in rows:
        flags = row_quality_flags(SimpleNamespace(**row))
        negative_flags = [flag for flag in flags if flag in {"negative_entry_price", "negative_exit_price"}]
        if negative_flags:
            invalid_rows.append(
                {
                    "journal_key": row.get("journal_key"),
                    "symbol": row.get("symbol"),
                    "strategy_type": row.get("strategy_type"),
                    "flags": negative_flags,
                }
            )
            continue
        valid_rows.append(row)
    return valid_rows, invalid_rows


def _refresh_adaptive_snapshot_after_rebuild(*, min_cutoff_date: date) -> dict[str, Any]:
    return AdaptiveLearningService().refresh(
        force=True,
        min_cutoff=datetime.combine(min_cutoff_date, datetime.min.time()),
    )


def _reset_post_rebuild_operational_state(*, account_scope: str) -> dict[str, Any]:
    try:
        from operations.operation_center import OperationCenter
    except ModuleNotFoundError:  # pragma: no cover - runtime fallback for package imports
        from atlas_code_quant.operations.operation_center import OperationCenter

    return OperationCenter().reset_after_journal_rebuild(account_scope=account_scope)


def _is_bad_strategy_type(value: Any) -> bool:
    normalized = str(value or "").strip().lower()
    return normalized in {"", "unknown", "untracked", "none", "null"}


def _option_strategy_family(strategy_type: Any) -> str | None:
    normalized = str(strategy_type or "").strip().lower()
    return OPTION_STRATEGY_FAMILY_MAP.get(normalized)


def _strategy_leg_tokens(strategy: dict[str, Any]) -> list[str]:
    tokens: list[str] = []
    for position in sorted(
        strategy.get("positions") or [],
        key=lambda item: (
            item.get("symbol") or "",
            item.get("expiration") or "",
            float(item.get("strike") or 0.0),
            item.get("side") or "",
        ),
    ):
        if str(position.get("asset_class") or "") == "equity":
            tokens.append(f"equity:{strategy.get('underlying')}")
        else:
            tokens.append(
                f"option:{position.get('symbol')}:{position.get('expiration')}:{position.get('side')}:{position.get('option_type')}:{position.get('strike')}"
            )
    return tokens


def _position_match_tokens(positions: list[dict[str, Any]]) -> list[str]:
    tokens: list[str] = []
    for position in sorted(
        positions or [],
        key=lambda item: (
            str(item.get("symbol") or ""),
            str(item.get("expiration") or ""),
            float(item.get("strike") or 0.0),
            str(item.get("side") or ""),
            float(item.get("signed_qty") or 0.0),
        ),
    ):
        asset_class = str(position.get("asset_class") or "").lower()
        symbol = str(position.get("symbol") or "").upper()
        side = str(position.get("side") or "").lower()
        signed_qty = round(_safe_float(position.get("signed_qty"), 0.0), 6)
        if asset_class == "equity":
            tokens.append(f"equity:{symbol}:{side}:{signed_qty}")
        else:
            expiration = str(position.get("expiration") or "")
            option_type = str(position.get("option_type") or "")
            strike = round(_safe_float(position.get("strike"), 0.0), 6)
            tokens.append(
                f"option:{symbol}:{expiration}:{side}:{option_type}:{strike}:{signed_qty}"
            )
    return tokens


def _strategy_match_signature(strategy: dict[str, Any]) -> str:
    return "|".join(_position_match_tokens(strategy.get("positions") or []))


def _entry_match_signature(entry: TradingJournal) -> str:
    return "|".join(_position_match_tokens(_json_load(entry.legs_details, [])))


def _select_matching_open_entry(
    open_entries: list[TradingJournal],
    *,
    strategy: dict[str, Any],
    strategy_id: str,
) -> TradingJournal | None:
    tracker_strategy_id = str(strategy.get("strategy_id") or "").strip()
    strategy_match_signature = _strategy_match_signature(strategy)
    incoming_strategy_type = str(strategy.get("strategy_type") or "").strip()

    for entry in open_entries:
        if entry.strategy_id == strategy_id:
            return entry

    if tracker_strategy_id:
        for entry in open_entries:
            if str(entry.tracker_strategy_id or "").strip() == tracker_strategy_id:
                return entry

    if not strategy_match_signature:
        return None

    # If the incoming snapshot degraded to unknown/untracked, prefer an already
    # attributed open entry with the same structure so we do not churn the journal
    # by closing the attributed row and reopening it as untracked.
    if _is_bad_strategy_type(incoming_strategy_type):
        for entry in open_entries:
            if _is_bad_strategy_type(entry.strategy_type):
                continue
            if _entry_match_signature(entry) == strategy_match_signature:
                return entry

    for entry in open_entries:
        if not _is_bad_strategy_type(entry.strategy_type):
            continue
        if _entry_match_signature(entry) == strategy_match_signature:
            return entry
    return None


def _stable_strategy_id(strategy: dict[str, Any]) -> tuple[str, str]:
    signature = "|".join(_strategy_leg_tokens(strategy))
    digest = hashlib.sha1()
    digest.update(str(strategy.get("strategy_type") or "").encode("utf-8"))
    digest.update(str(strategy.get("underlying") or "").encode("utf-8"))
    digest.update(signature.encode("utf-8"))
    short = digest.hexdigest()[:14]
    return f"{strategy.get('strategy_type', 'strategy')}:{strategy.get('underlying', 'UNK')}:{short}", signature


def _position_notional(position: dict[str, Any], price_key: str) -> float:
    price = _safe_float(position.get(price_key), 0.0)
    qty = abs(_safe_float(position.get("signed_qty"), 0.0))
    if price <= 0 or qty <= 0:
        return 0.0
    multiplier = 100.0 if str(position.get("asset_class") or "option") == "option" else 1.0
    sign = 1.0 if str(position.get("side") or "").lower() in {"long", "buy"} else -1.0
    return price * qty * multiplier * sign


def _strategy_notional(strategy: dict[str, Any], price_key: str) -> float:
    return round(float(sum(_position_notional(position, price_key) for position in strategy.get("positions") or [])), 4)


def _extract_avg_iv(strategy: dict[str, Any]) -> float | None:
    details = strategy.get("win_rate_details") or {}
    legs = details.get("selected_legs") or []
    values = [_safe_float(leg.get("implied_volatility"), float("nan")) for leg in legs]
    valid = [value for value in values if value == value and value > 0]
    return float(sum(valid) / len(valid)) if valid else None


def _estimate_iv_rank(client: TradierClient, symbol: str, current_iv: float | None) -> float | None:
    if current_iv is None or current_iv <= 0:
        return None
    try:
        end_date = datetime.utcnow().date()
        start_date = end_date - timedelta(days=420)
        history = client.history(symbol, start=start_date, end=end_date, interval="daily")
        _, log_returns = _compute_realized_vol(history)
        if log_returns.size < 40:
            return None
        rolling = [float(np.std(log_returns[idx - 20:idx], ddof=1) * math.sqrt(252)) for idx in range(20, log_returns.size + 1)]
        if not rolling:
            return None
        return round(float(sum(1.0 for value in rolling if value <= current_iv) / len(rolling) * 100.0), 2)
    except Exception:
        logger.exception("Unable to estimate IV rank for %s", symbol)
        return None


def _recorded_price(value: Any) -> float | None:
    out = _safe_float(value, float("nan"))
    if out != out:
        return None
    return round(abs(out), 4)


def _normalized_symbol_filter(symbols: list[str] | None) -> set[str]:
    return {str(symbol).strip().upper() for symbol in (symbols or []) if str(symbol).strip()}


def _rebuild_journal_rows_from_events(
    events: list[dict[str, Any]],
    *,
    account_scope: str,
    account_id: str,
    symbols: list[str] | None = None,
) -> list[dict[str, Any]]:
    allowed_symbols = _normalized_symbol_filter(symbols)
    open_lots: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    closed_rows: list[dict[str, Any]] = []
    counters: Counter[str] = Counter()

    for event in sorted(events, key=lambda item: item.get("timestamp") or datetime.min):
        symbol = str(event.get("symbol") or "").strip().upper()
        action = str(event.get("action") or "").strip().lower()
        timestamp = event.get("timestamp")
        quantity = abs(_safe_float(event.get("quantity"), 0.0))
        price = _recorded_price(event.get("price"))
        commission = round(_safe_float(event.get("commission"), 0.0), 4)
        if not symbol or (allowed_symbols and symbol not in allowed_symbols):
            continue
        if not isinstance(timestamp, datetime) or quantity <= 0 or price is None or price <= 0:
            counters["ignored_invalid_event"] += 1
            continue

        long_key = (symbol, "long")
        short_key = (symbol, "short")

        def _append_open(side: str) -> None:
            open_lots[(symbol, side)].append(
                {
                    "symbol": symbol,
                    "side": side,
                    "timestamp": timestamp,
                    "price": price,
                    "quantity": quantity,
                    "commission": commission,
                }
            )
            counters[f"open_{side}"] += 1

        def _close_fifo(side: str) -> bool:
            queue = open_lots[(symbol, side)]
            if not queue:
                return False
            remaining = quantity
            close_commission_per_share = commission / quantity if quantity > 0 else 0.0
            while remaining > 0 and queue:
                lot = queue[0]
                matched_qty = min(remaining, float(lot["quantity"]))
                open_commission_per_share = float(lot["commission"]) / float(lot["quantity"]) if float(lot["quantity"]) > 0 else 0.0
                total_fees = round((open_commission_per_share + close_commission_per_share) * matched_qty, 4)
                entry_price = float(lot["price"])
                exit_price = float(price)
                realized_pnl = (
                    (exit_price - entry_price) * matched_qty
                    if side == "long"
                    else (entry_price - exit_price) * matched_qty
                ) - total_fees
                strategy_type = "equity_long" if side == "long" else "equity_short"
                ts_key = int(float(lot["timestamp"].timestamp()))
                exit_key = int(float(timestamp.timestamp()))
                closed_rows.append(
                    {
                        "journal_key": f"rebuild:{account_scope}:{account_id}:{strategy_type}:{symbol}:{ts_key}:{exit_key}:{len(closed_rows) + 1}",
                        "account_type": account_scope,
                        "account_id": account_id,
                        "strategy_id": f"rebuild:{strategy_type}:{symbol}",
                        "tracker_strategy_id": f"rebuild:{strategy_type}:{symbol}",
                        "strategy_type": strategy_type,
                        "symbol": symbol,
                        "legs_signature": f"equity:{symbol}:{side}",
                        "legs_details": _json_dump(
                            [
                                {
                                    "symbol": symbol,
                                    "asset_class": "equity",
                                    "side": side,
                                    "signed_qty": matched_qty if side == "long" else -matched_qty,
                                    "entry_price": round(entry_price, 4),
                                    "current_price": round(exit_price, 4),
                                }
                            ]
                        ),
                        "entry_price": round(entry_price, 4),
                        "exit_price": round(exit_price, 4),
                        "fees": total_fees,
                        "entry_time": lot["timestamp"],
                        "exit_time": timestamp,
                        "status": "closed",
                        "is_level4": False,
                        "realized_pnl": round(realized_pnl, 4),
                        "unrealized_pnl": 0.0,
                        "mark_price": round(exit_price, 4),
                        "spot_price": round(exit_price, 4),
                        "entry_notional": round(entry_price * matched_qty, 4),
                        "risk_at_entry": round(entry_price * matched_qty, 4),
                        "thesis_rich_text": "Rebuilt from broker trade history.",
                        "greeks_json": "{}",
                        "attribution_json": "{}",
                        "post_mortem_json": "{}",
                        "post_mortem_text": "Rebuilt from broker trade history.",
                        "broker_order_ids_json": "[]",
                        "raw_entry_payload_json": _json_dump({"source": "broker_history_rebuild"}),
                        "raw_exit_payload_json": _json_dump([event.get("raw") or {}]),
                    }
                )
                remaining -= matched_qty
                lot["quantity"] = round(float(lot["quantity"]) - matched_qty, 8)
                if float(lot["quantity"]) <= 0:
                    queue.pop(0)
            counters[f"close_{side}"] += 1
            return True

        if action == "open_long":
            _append_open("long")
            continue
        if action == "open_short":
            _append_open("short")
            continue
        if action == "close_long":
            if not _close_fifo("long"):
                counters["orphan_close_long"] += 1
            continue
        if action == "close_short":
            if not _close_fifo("short"):
                counters["orphan_close_short"] += 1
            continue
        if action == "buy":
            if not _close_fifo("short"):
                _append_open("long")
            continue
        if action == "sell":
            if not _close_fifo("long"):
                _append_open("short")
            continue

    return closed_rows


def _parse_broker_datetime(value: Any) -> datetime | None:
    text = str(value or "").strip()
    if not text:
        return None
    try:
        return datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError:
        return None


def _rebuild_journal_rows_from_closed_positions(
    closed_positions: list[dict[str, Any]],
    *,
    account_scope: str,
    account_id: str,
    symbols: list[str] | None = None,
) -> list[dict[str, Any]]:
    allowed_symbols = _normalized_symbol_filter(symbols)
    rows: list[dict[str, Any]] = []

    for index, item in enumerate(closed_positions, start=1):
        symbol = str(item.get("symbol") or "").strip().upper()
        quantity = abs(_safe_float(item.get("quantity"), 0.0))
        cost = _recorded_price(item.get("cost"))
        proceeds = _recorded_price(item.get("proceeds"))
        realized_pnl = round(_safe_float(item.get("gain_loss"), 0.0), 4)
        entry_time = _parse_broker_datetime(item.get("open_date"))
        exit_time = _parse_broker_datetime(item.get("close_date"))

        if not symbol or (allowed_symbols and symbol not in allowed_symbols):
            continue
        if quantity <= 0 or cost is None or cost <= 0 or proceeds is None or proceeds <= 0:
            continue
        if entry_time is None or exit_time is None or exit_time < entry_time:
            continue

        entry_price = round(cost / quantity, 4)
        exit_price = round(proceeds / quantity, 4)
        strategy_type = "rebuild_option_closed" if len(symbol) > 10 else "rebuild_equity_closed"
        ts_key = int(float(entry_time.timestamp()))
        exit_key = int(float(exit_time.timestamp()))

        rows.append(
            {
                "journal_key": f"rebuild:gainloss:{account_scope}:{account_id}:{symbol}:{ts_key}:{exit_key}:{index}",
                "account_type": account_scope,
                "account_id": account_id,
                "strategy_id": f"rebuild:gainloss:{strategy_type}:{symbol}",
                "tracker_strategy_id": f"rebuild:gainloss:{strategy_type}:{symbol}",
                "strategy_type": strategy_type,
                "symbol": symbol,
                "legs_signature": f"rebuild:{symbol}",
                "legs_details": _json_dump(
                    [
                        {
                            "symbol": symbol,
                            "asset_class": "option" if len(symbol) > 10 else "equity",
                            "side": "unknown",
                            "signed_qty": quantity,
                            "entry_price": entry_price,
                            "current_price": exit_price,
                        }
                    ]
                ),
                "entry_price": entry_price,
                "exit_price": exit_price,
                "fees": 0.0,
                "entry_time": entry_time,
                "exit_time": exit_time,
                "status": "closed",
                "is_level4": False,
                "realized_pnl": realized_pnl,
                "unrealized_pnl": 0.0,
                "mark_price": exit_price,
                "spot_price": exit_price,
                "entry_notional": cost,
                "risk_at_entry": cost,
                "thesis_rich_text": "Rebuilt from broker gain/loss closed positions.",
                "greeks_json": "{}",
                "attribution_json": "{}",
                "post_mortem_json": "{}",
                "post_mortem_text": "Rebuilt from broker gain/loss closed positions.",
                "broker_order_ids_json": "[]",
                "raw_entry_payload_json": _json_dump({"source": "broker_gainloss_rebuild"}),
                "raw_exit_payload_json": _json_dump([item]),
            }
        )

    return rows


def _build_thesis(strategy: dict[str, Any], iv_rank: float | None) -> str:
    win_rate = _safe_float(strategy.get("win_rate_pct"), 0.0)
    driver = str((strategy.get("attribution") or {}).get("dominant_driver") or "mixto")
    iv_line = f"<li>IV rank proxy: {iv_rank:.2f}%.</li>" if iv_rank is not None else "<li>IV rank proxy: no disponible.</li>"
    return (
        f"<p><strong>Setup:</strong> {strategy.get('strategy_type')} sobre {strategy.get('underlying')}.</p>"
        f"<ul>"
        f"<li>Probabilidad al entrar: {win_rate:.2f}%.</li>"
        f"{iv_line}"
        f"<li>Driver de riesgo dominante: {driver}.</li>"
        f"<li>Tesis automÃ¡tica: conservar la estructura mientras el perfil de probabilidad siga a favor y el driver dominante no se acelere en contra.</li>"
        f"<li>Sentimiento Grok: pendiente de enriquecimiento externo; el campo queda listo para persistir esa explicaciÃ³n.</li>"
        f"</ul>"
    )


def _normalize_trade_events(events: list[dict[str, Any]]) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    for item in events:
        if not isinstance(item, dict):
            continue
        action = _action_bucket(_event_action(item))
        if action is None:
            continue
        timestamp = _event_timestamp(item)
        if timestamp is None:
            continue
        quantity = max(
            _safe_float(item.get("exec_quantity"), 0.0),
            _safe_float(item.get("filled_quantity"), 0.0),
            _safe_float(item.get("quantity"), 0.0),
        )
        normalized.append(
            {
                "timestamp": timestamp,
                "symbol": _event_symbol(item),
                "underlying": str(item.get("symbol") or "").strip().upper(),
                "action": action,
                "price": _safe_float(item.get("price"), 0.0),
                "quantity": quantity,
                "commission": _safe_float(item.get("commission"), 0.0),
                "raw": item,
            }
        )
    normalized.sort(key=lambda item: item["timestamp"])
    return normalized


def _event_symbols(strategy: dict[str, Any]) -> set[str]:
    values: set[str] = set()
    for position in strategy.get("positions") or []:
        symbol = str(position.get("symbol") or "").strip().upper()
        if symbol:
            values.add(symbol)
        if str(position.get("asset_class") or "") == "equity":
            underlying = str(strategy.get("underlying") or "").strip().upper()
            if underlying:
                values.add(underlying)
    if not values:
        underlying = str(strategy.get("underlying") or "").strip().upper()
        if underlying:
            values.add(underlying)
    return values


def _matching_events(events: list[dict[str, Any]], strategy: dict[str, Any]) -> list[dict[str, Any]]:
    symbols = _event_symbols(strategy)
    return [
        event
        for event in events
        if str(event.get("symbol") or "").upper() in symbols or str(event.get("underlying") or "").upper() in symbols
    ]


def _event_flow_notional(event: dict[str, Any]) -> float:
    price = _safe_float(event.get("price"), 0.0)
    quantity = abs(_safe_float(event.get("quantity"), 0.0))
    if price <= 0 or quantity <= 0:
        return 0.0
    multiplier = 100.0 if len(str(event.get("symbol") or "")) > 10 else 1.0
    sign = 1.0 if str(event.get("action") or "") in {"open_long", "close_short", "buy"} else -1.0
    return round(price * quantity * multiplier * sign, 4)


def _open_event_details(events: list[dict[str, Any]]) -> tuple[datetime | None, float | None, float]:
    open_events = [event for event in events if str(event.get("action") or "").startswith("open") or str(event.get("action") or "") == "buy"]
    if not open_events:
        return None, None, 0.0
    ts = min(event["timestamp"] for event in open_events)
    return ts, round(sum(_event_flow_notional(event) for event in open_events), 4), round(sum(_safe_float(event.get("commission"), 0.0) for event in open_events), 4)


def _close_event_details(events: list[dict[str, Any]]) -> tuple[datetime | None, float | None, float, list[dict[str, Any]]]:
    close_events = [event for event in events if str(event.get("action") or "").startswith("close") or str(event.get("action") or "") == "sell"]
    if not close_events:
        return None, None, 0.0, []
    ts = max(event["timestamp"] for event in close_events)
    return ts, round(sum(_event_flow_notional(event) for event in close_events), 4), round(sum(_safe_float(event.get("commission"), 0.0) for event in close_events), 4), close_events


def _trade_return(entry: TradingJournal) -> float | None:
    base = max(abs(_safe_float(entry.risk_at_entry, 0.0)), abs(_safe_float(entry.entry_notional, 0.0)), 1.0)
    return _safe_float(entry.realized_pnl, float("nan")) / base if base > 0 else None


def _risk_budget(entry: TradingJournal) -> float:
    risk_at_entry = abs(_safe_float(entry.risk_at_entry, 0.0))
    if risk_at_entry > 0:
        return risk_at_entry
    entry_notional = abs(_safe_float(entry.entry_notional, 0.0))
    if entry_notional > 0:
        return entry_notional
    return 1.0


def _position_open_r_multiple(entry: TradingJournal) -> float:
    return round(_safe_float(entry.unrealized_pnl, 0.0) / _risk_budget(entry), 4)


def _thesis_drift_pct(entry: TradingJournal) -> float | None:
    current_win = _safe_float(entry.current_win_rate_pct, float("nan"))
    entry_win = _safe_float(entry.win_rate_at_entry, float("nan"))
    if current_win != current_win or entry_win != entry_win:
        return None
    return round(current_win - entry_win, 4)


def _holding_hours(entry: TradingJournal, *, now: datetime) -> float:
    if not entry.entry_time:
        return 0.0
    return round(max((now - entry.entry_time).total_seconds() / 3600.0, 0.0), 4)


def _ratio_pct(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return 0.0
    return float(numerator / denominator * 100.0)


def _bucket_concentration(
    open_entries: list[TradingJournal],
    *,
    key_name: str,
    key_builder: Any,
    total_risk_budget: float,
) -> list[dict[str, Any]]:
    buckets: dict[str, dict[str, Any]] = {}
    for entry in open_entries:
        key = str(key_builder(entry) or "unknown")
        bucket = buckets.setdefault(
            key,
            {
                key_name: key,
                "open_positions": 0,
                "risk_budget": 0.0,
                "entry_notional": 0.0,
                "unrealized_pnl": 0.0,
            },
        )
        bucket["open_positions"] += 1
        bucket["risk_budget"] += _risk_budget(entry)
        bucket["entry_notional"] += abs(_safe_float(entry.entry_notional, 0.0))
        bucket["unrealized_pnl"] += _safe_float(entry.unrealized_pnl, 0.0)

    rows: list[dict[str, Any]] = []
    for bucket in buckets.values():
        risk_share_pct = _ratio_pct(bucket["risk_budget"], total_risk_budget) if total_risk_budget > 0 else 0.0
        rows.append(
            {
                key_name: bucket[key_name],
                "open_positions": int(bucket["open_positions"]),
                "risk_budget": round(bucket["risk_budget"], 4),
                "risk_share_pct": round(risk_share_pct, 4),
                "entry_notional": round(bucket["entry_notional"], 4),
                "unrealized_pnl": round(bucket["unrealized_pnl"], 4),
            }
        )
    return sorted(rows, key=lambda item: (-_safe_float(item.get("risk_share_pct"), 0.0), str(item.get(key_name) or "")))


def build_position_management_snapshot(
    open_entries: list[TradingJournal],
    *,
    account_type: str | None = None,
    limit: int = 12,
    now: datetime | None = None,
) -> dict[str, Any]:
    generated_at = (now or datetime.utcnow()).isoformat()
    active_now = now or datetime.utcnow()
    var_monitor = VarMonitor()
    filtered = [
        entry
        for entry in open_entries
        if entry.status == "open" and (account_type in {None, "", "all"} or entry.account_type == account_type)
    ]
    capped_limit = max(1, min(int(limit), 25))
    thresholds = {
        "position_management_enabled": bool(settings.position_management_enabled),
        "max_symbol_heat_pct": float(settings.position_management_max_symbol_heat_pct),
        "max_unrealized_loss_r": float(settings.position_management_max_unrealized_loss_r),
        "max_thesis_drift_pct": float(settings.position_management_max_thesis_drift_pct),
        "max_stale_hours": int(settings.position_management_max_stale_hours),
        "var_monitor_enabled": bool(var_monitor.is_enabled()),
    }
    if not filtered:
        return {
            "generated_at": generated_at,
            "account_type": account_type or "all",
            "enabled": thresholds["position_management_enabled"],
            "summary": {
                "open_positions": 0,
                "total_risk_budget": 0.0,
                "total_entry_notional": 0.0,
                "total_unrealized_pnl": 0.0,
                "avg_holding_hours": 0.0,
                "median_holding_hours": 0.0,
                "adverse_positions_count": 0,
                "stale_positions_count": 0,
                "thesis_deteriorated_count": 0,
                "watchlist_count": 0,
                "max_symbol_heat_pct": 0.0,
                "max_strategy_heat_pct": 0.0,
                "var_95_usd": 0.0,
                "cvar_95_usd": 0.0,
                "var_95_pct_of_book": 0.0,
                "var_status": "disabled" if not thresholds["var_monitor_enabled"] else "ok",
                "var_method": "disabled" if not thresholds["var_monitor_enabled"] else "monte_carlo_portfolio",
            },
            "alerts": [],
            "concentrations": {"by_symbol": [], "by_strategy_type": []},
            "var_monitor": {
                "enabled": thresholds["var_monitor_enabled"],
                "method": "disabled" if not thresholds["var_monitor_enabled"] else "monte_carlo_portfolio",
                "simulation_count": int(settings.position_management_var_mc_scenarios),
                "confidence_level_pct": float(settings.position_management_var_confidence_pct),
                "horizon_days": int(settings.position_management_var_horizon_days),
                "var_95_usd": 0.0,
                "cvar_95_usd": 0.0,
                "var_95_pct_of_book": 0.0,
                "threshold_usd": 0.0,
                "status": "disabled" if not thresholds["var_monitor_enabled"] else "ok",
                "drivers": [],
                "diversified_risk_usd": 0.0,
                "gross_risk_usd": 0.0,
                "concentration_multiplier": 1.0,
                "loss_multiplier": 1.0,
                "monte_carlo_var_usd": 0.0,
                "monte_carlo_cvar_usd": 0.0,
                "expected_loss_usd": 0.0,
                "worst_case_loss_usd": 0.0,
                "net_directional_exposure_pct": 0.0,
            },
            "watchlist": [],
            "thresholds": thresholds,
        }

    position_rows: list[dict[str, Any]] = []
    holding_hours_values: list[float] = []
    adverse_count = 0
    stale_count = 0
    thesis_deteriorated_count = 0
    total_risk_budget = round(sum(_risk_budget(entry) for entry in filtered), 4)
    total_entry_notional = round(sum(abs(_safe_float(entry.entry_notional, 0.0)) for entry in filtered), 4)
    total_unrealized_pnl = round(sum(_safe_float(entry.unrealized_pnl, 0.0) for entry in filtered), 4)

    for entry in filtered:
        holding_hours = _holding_hours(entry, now=active_now)
        holding_hours_values.append(holding_hours)
        thesis_drift_pct = _thesis_drift_pct(entry)
        open_r_multiple = _position_open_r_multiple(entry)
        symbol_heat_pct = _ratio_pct(_risk_budget(entry), total_risk_budget) if total_risk_budget > 0 else 0.0
        reasons: list[str] = []
        if open_r_multiple <= -float(settings.position_management_max_unrealized_loss_r):
            adverse_count += 1
            reasons.append("adverse_loss_r")
        if holding_hours >= int(settings.position_management_max_stale_hours) and _safe_float(entry.unrealized_pnl, 0.0) <= 0.0:
            stale_count += 1
            reasons.append("stale_loser")
        if thesis_drift_pct is not None and thesis_drift_pct <= -float(settings.position_management_max_thesis_drift_pct):
            thesis_deteriorated_count += 1
            reasons.append("thesis_drift")
        if symbol_heat_pct > float(settings.position_management_max_symbol_heat_pct):
            reasons.append("symbol_heat")
        abs_loss = _safe_float(entry.unrealized_pnl, 0.0)
        if abs_loss <= -float(settings.exit_governance_hard_exit_loss_usd):
            reasons.append("hard_dollar_loss")

        position_rows.append(
            {
                "symbol": entry.symbol,
                "strategy_type": entry.strategy_type,
                "account_type": entry.account_type,
                "entry_time": entry.entry_time.isoformat() if entry.entry_time else None,
                "holding_hours": round(holding_hours, 4),
                "unrealized_pnl": round(_safe_float(entry.unrealized_pnl, 0.0), 4),
                "open_r_multiple": open_r_multiple,
                "thesis_drift_pct": thesis_drift_pct,
                "risk_budget": round(_risk_budget(entry), 4),
                "entry_notional": round(abs(_safe_float(entry.entry_notional, 0.0)), 4),
                "symbol_heat_pct": round(symbol_heat_pct, 4),
                "alert_reasons": reasons,
                "status": "watch" if reasons else "normal",
                "journal_key": entry.journal_key,
                "strategy_id": entry.strategy_id,
            }
        )

    symbol_concentrations = _bucket_concentration(
        filtered,
        key_name="symbol",
        key_builder=lambda entry: entry.symbol,
        total_risk_budget=total_risk_budget,
    )
    strategy_concentrations = _bucket_concentration(
        filtered,
        key_name="strategy_type",
        key_builder=lambda entry: entry.strategy_type,
        total_risk_budget=total_risk_budget,
    )
    max_symbol_heat_pct = round(max((_safe_float(row.get("risk_share_pct"), 0.0) for row in symbol_concentrations), default=0.0), 4)
    max_strategy_heat_pct = round(max((_safe_float(row.get("risk_share_pct"), 0.0) for row in strategy_concentrations), default=0.0), 4)
    var_snapshot = var_monitor.compute(
        position_rows,
        total_risk_budget=total_risk_budget,
        total_entry_notional=total_entry_notional,
        max_symbol_heat_pct=max_symbol_heat_pct,
        adverse_positions_count=adverse_count,
        total_unrealized_pnl=total_unrealized_pnl,
    )

    alerts: list[dict[str, Any]] = []
    if max_symbol_heat_pct > float(settings.position_management_max_symbol_heat_pct):
        top_symbol = symbol_concentrations[0]["symbol"] if symbol_concentrations else "unknown"
        alerts.append(
            {
                "level": "warning",
                "code": "symbol_heat_exceeded",
                "message": (
                    f"El calor por simbolo excede el umbral configurado: {top_symbol} concentra {max_symbol_heat_pct:.2f}% "
                    f"del riesgo abierto."
                ),
                "value": max_symbol_heat_pct,
                "threshold": float(settings.position_management_max_symbol_heat_pct),
            }
        )
    if adverse_count > 0:
        alerts.append(
            {
                "level": "warning",
                "code": "adverse_positions_present",
                "message": f"Hay {adverse_count} posiciones abiertas con perdida superior al umbral de R no realizado.",
                "value": adverse_count,
                "threshold": float(settings.position_management_max_unrealized_loss_r),
            }
        )
    if stale_count > 0:
        alerts.append(
            {
                "level": "warning",
                "code": "stale_losers_present",
                "message": f"Hay {stale_count} posiciones estancadas o perdedoras mas alla del hold time tolerado.",
                "value": stale_count,
                "threshold": int(settings.position_management_max_stale_hours),
            }
        )
    if thesis_deteriorated_count > 0:
        alerts.append(
            {
                "level": "warning",
                "code": "thesis_drift_present",
                "message": (
                    f"Hay {thesis_deteriorated_count} posiciones cuya probabilidad o tesis se deterioro mas alla del umbral permitido."
                ),
                "value": thesis_deteriorated_count,
                "threshold": float(settings.position_management_max_thesis_drift_pct),
            }
        )
    if var_snapshot.enabled and var_snapshot.status in {"warning", "critical"}:
        alerts.append(
            {
                "level": "critical" if var_snapshot.status == "critical" else "warning",
                "code": "portfolio_var_limit",
                "message": (
                    f"VaR dinamico 95% estimado en {var_snapshot.var_95_usd:.2f} USD "
                    f"({var_snapshot.var_95_pct_of_book:.2f}% del libro abierto)."
                ),
                "value": var_snapshot.var_95_usd,
                "threshold": var_snapshot.threshold_usd,
                "drivers": list(var_snapshot.drivers),
            }
        )

    watchlist = [row for row in position_rows if row["alert_reasons"]]
    watchlist.sort(
        key=lambda row: (
            -len(row["alert_reasons"]),
            _safe_float(row.get("open_r_multiple"), 0.0),
            -_safe_float(row.get("holding_hours"), 0.0),
        )
    )

    return {
        "generated_at": generated_at,
        "account_type": account_type or "all",
        "enabled": thresholds["position_management_enabled"],
        "summary": {
            "open_positions": len(filtered),
            "total_risk_budget": total_risk_budget,
            "total_entry_notional": total_entry_notional,
            "total_unrealized_pnl": total_unrealized_pnl,
            "avg_holding_hours": round(float(np.mean(holding_hours_values)) if holding_hours_values else 0.0, 4),
            "median_holding_hours": round(float(np.median(holding_hours_values)) if holding_hours_values else 0.0, 4),
            "adverse_positions_count": adverse_count,
            "stale_positions_count": stale_count,
            "thesis_deteriorated_count": thesis_deteriorated_count,
            "watchlist_count": len(watchlist),
            "max_symbol_heat_pct": max_symbol_heat_pct,
            "max_strategy_heat_pct": max_strategy_heat_pct,
            "var_95_usd": var_snapshot.var_95_usd,
            "cvar_95_usd": var_snapshot.cvar_95_usd,
            "var_95_pct_of_book": var_snapshot.var_95_pct_of_book,
            "var_status": var_snapshot.status,
            "var_method": var_snapshot.method,
        },
        "alerts": alerts,
        "concentrations": {
            "by_symbol": symbol_concentrations[: capped_limit // 2 if capped_limit > 1 else 1],
            "by_strategy_type": strategy_concentrations[: capped_limit // 2 if capped_limit > 1 else 1],
        },
        "var_monitor": {
            "enabled": var_snapshot.enabled,
            "method": var_snapshot.method,
            "simulation_count": var_snapshot.simulation_count,
            "confidence_level_pct": var_snapshot.confidence_level_pct,
            "horizon_days": var_snapshot.horizon_days,
            "var_95_usd": var_snapshot.var_95_usd,
            "cvar_95_usd": var_snapshot.cvar_95_usd,
            "var_95_pct_of_book": var_snapshot.var_95_pct_of_book,
            "threshold_usd": var_snapshot.threshold_usd,
            "status": var_snapshot.status,
            "drivers": list(var_snapshot.drivers),
            "diversified_risk_usd": var_snapshot.diversified_risk_usd,
            "gross_risk_usd": var_snapshot.gross_risk_usd,
            "concentration_multiplier": var_snapshot.concentration_multiplier,
            "loss_multiplier": var_snapshot.loss_multiplier,
            "monte_carlo_var_usd": var_snapshot.monte_carlo_var_usd,
            "monte_carlo_cvar_usd": var_snapshot.monte_carlo_cvar_usd,
            "expected_loss_usd": var_snapshot.expected_loss_usd,
            "worst_case_loss_usd": var_snapshot.worst_case_loss_usd,
            "net_directional_exposure_pct": var_snapshot.net_directional_exposure_pct,
        },
        "watchlist": watchlist[:capped_limit],
        "thresholds": thresholds,
    }


def build_exit_governance_snapshot(
    open_entries: list[TradingJournal],
    *,
    account_type: str | None = None,
    limit: int = 10,
    now: datetime | None = None,
) -> dict[str, Any]:
    position_management = build_position_management_snapshot(
        open_entries,
        account_type=account_type,
        limit=max(limit, 10),
        now=now,
    )
    thresholds = {
        "exit_governance_enabled": bool(settings.exit_governance_enabled),
        "hard_exit_loss_r": float(settings.exit_governance_hard_exit_loss_r),
        "take_profit_r": float(settings.exit_governance_take_profit_r),
        "time_stop_hours": int(settings.exit_governance_time_stop_hours),
        "trailing_profit_floor_r": float(settings.exit_governance_trailing_profit_floor_r),
        "thesis_drift_guard_pct": float(settings.position_management_max_thesis_drift_pct),
        "symbol_heat_guard_pct": float(settings.position_management_max_symbol_heat_pct),
        "hard_exit_loss_usd": float(settings.exit_governance_hard_exit_loss_usd),
    }
    candidates: list[dict[str, Any]] = []
    action_counts = {
        "exit_now": 0,
        "de_risk": 0,
        "take_profit": 0,
        "hold": 0,
    }
    portfolio_var = dict(position_management.get("var_monitor") or {})
    portfolio_var_status = str(portfolio_var.get("status") or "ok")
    portfolio_var_enabled = bool(portfolio_var.get("enabled"))
    portfolio_var_95_usd = _safe_float(portfolio_var.get("var_95_usd"), 0.0)
    portfolio_var_method = str(portfolio_var.get("method") or "unknown")

    for row in position_management.get("watchlist") or []:
        reasons = set(str(reason) for reason in row.get("alert_reasons") or [])
        open_r_multiple = _safe_float(row.get("open_r_multiple"), 0.0)
        holding_hours = _safe_float(row.get("holding_hours"), 0.0)
        thesis_drift_pct = _safe_float(row.get("thesis_drift_pct"), float("nan"))
        symbol_heat_pct = _safe_float(row.get("symbol_heat_pct"), 0.0)

        recommendation = "hold"
        exit_reason = "monitor_only"
        urgency = "low"

        abs_loss_usd = _safe_float(row.get("unrealized_pnl"), 0.0)
        if open_r_multiple <= -float(settings.exit_governance_hard_exit_loss_r):
            recommendation = "exit_now"
            exit_reason = "hard_stop_loss_r"
            urgency = "high"
        elif abs_loss_usd <= -float(settings.exit_governance_hard_exit_loss_usd):
            recommendation = "exit_now"
            exit_reason = "hard_dollar_loss"
            urgency = "high"
        elif "thesis_drift" in reasons and open_r_multiple < 0:
            recommendation = "exit_now"
            exit_reason = "thesis_invalidated"
            urgency = "high"
        elif holding_hours >= int(settings.exit_governance_time_stop_hours) and open_r_multiple <= 0:
            recommendation = "exit_now"
            exit_reason = "time_stop"
            urgency = "medium"
        elif open_r_multiple >= float(settings.exit_governance_take_profit_r):
            recommendation = "take_profit"
            exit_reason = "profit_target"
            urgency = "medium"
        elif symbol_heat_pct > float(settings.position_management_max_symbol_heat_pct):
            recommendation = "de_risk"
            exit_reason = "book_concentration"
            urgency = "medium"
        elif open_r_multiple >= float(settings.exit_governance_trailing_profit_floor_r) and thesis_drift_pct == thesis_drift_pct and thesis_drift_pct < 0:
            recommendation = "de_risk"
            exit_reason = "protect_open_profit"
            urgency = "medium"
        elif portfolio_var_enabled and portfolio_var_status == "critical":
            recommendation = "de_risk"
            exit_reason = "portfolio_var_limit"
            urgency = "high"
        elif portfolio_var_enabled and portfolio_var_status == "warning" and open_r_multiple > 0:
            recommendation = "de_risk"
            exit_reason = "portfolio_var_pressure"
            urgency = "medium"

        action_counts[recommendation] += 1
        candidates.append(
            {
                **row,
                "recommendation": recommendation,
                "exit_reason": exit_reason,
                "urgency": urgency,
            }
        )

    candidates.sort(
        key=lambda row: (
            {"exit_now": 0, "de_risk": 1, "take_profit": 2, "hold": 3}.get(str(row.get("recommendation")), 4),
            {"high": 0, "medium": 1, "low": 2}.get(str(row.get("urgency")), 3),
            _safe_float(row.get("open_r_multiple"), 0.0),
            -_safe_float(row.get("holding_hours"), 0.0),
        )
    )

    alerts: list[dict[str, Any]] = []
    if action_counts["exit_now"] > 0:
        alerts.append(
            {
                "level": "warning",
                "code": "exit_now_candidates_present",
                "message": f"Hay {action_counts['exit_now']} posiciones que ya cumplen criterio estructurado de salida inmediata.",
                "value": action_counts["exit_now"],
            }
        )
    if action_counts["de_risk"] > 0:
        alerts.append(
            {
                "level": "warning",
                "code": "de_risk_candidates_present",
                "message": f"Hay {action_counts['de_risk']} posiciones que conviene reducir o proteger.",
                "value": action_counts["de_risk"],
            }
        )
    if action_counts["take_profit"] > 0:
        alerts.append(
            {
                "level": "info",
                "code": "take_profit_candidates_present",
                "message": f"Hay {action_counts['take_profit']} posiciones con criterio de toma parcial o proteccion de beneficio.",
                "value": action_counts["take_profit"],
            }
        )
    if portfolio_var_enabled and portfolio_var_status in {"warning", "critical"}:
        alerts.append(
            {
                "level": "critical" if portfolio_var_status == "critical" else "warning",
                "code": "portfolio_var_exit_pressure",
                "message": (
                    f"Exit governance entra en modo defensivo porque el libro marca VaR {portfolio_var_status} "
                    f"({portfolio_var_95_usd:.2f} USD)."
                ),
                "value": portfolio_var_95_usd,
                "method": portfolio_var_method,
            }
        )

    return {
        "generated_at": position_management.get("generated_at"),
        "account_type": account_type or "all",
        "enabled": thresholds["exit_governance_enabled"],
        "summary": {
            "open_positions": position_management.get("summary", {}).get("open_positions", 0),
            "exit_now_count": action_counts["exit_now"],
            "de_risk_count": action_counts["de_risk"],
            "take_profit_count": action_counts["take_profit"],
            "hold_count": action_counts["hold"],
            "var_status": portfolio_var_status,
            "var_method": portfolio_var_method,
            "var_95_usd": portfolio_var_95_usd,
        },
        "alerts": alerts,
        "recommendations": candidates[: max(1, min(int(limit), 20))],
        "position_management_summary": position_management.get("summary", {}),
        "thresholds": thresholds,
    }


def build_post_trade_learning_snapshot(
    closed_entries: list[TradingJournal],
    *,
    account_type: str | None = None,
    limit: int = 10,
) -> dict[str, Any]:
    filtered = [
        entry
        for entry in closed_entries
        if entry.status == "closed" and (account_type in {None, "", "all"} or entry.account_type == account_type)
    ]
    capped_limit = max(1, min(int(limit), 20))
    if not filtered:
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "account_type": account_type or "all",
            "enabled": True,
            "summary": {
                "closed_trades": 0,
                "win_rate_pct": 0.0,
                "realized_pnl": 0.0,
                "post_mortem_coverage_pct": 0.0,
                "policy_candidate_count": 0,
            },
            "root_cause_breakdown": [],
            "strategy_learning": [],
            "policy_candidates": [],
        }

    wins = sum(1 for entry in filtered if _safe_float(entry.realized_pnl, 0.0) > 0)
    realized_pnl = round(sum(_safe_float(entry.realized_pnl, 0.0) for entry in filtered), 4)
    root_causes: dict[str, dict[str, Any]] = {}
    strategy_map: dict[str, dict[str, Any]] = {}
    coverage = 0

    for entry in filtered:
        post_mortem = _json_load(entry.post_mortem_json, {})
        if post_mortem:
            coverage += 1
        root_cause = str(post_mortem.get("root_cause") or "unknown")
        cause_bucket = root_causes.setdefault(
            root_cause,
            {"root_cause": root_cause, "count": 0, "realized_pnl": 0.0, "loss_count": 0},
        )
        cause_bucket["count"] += 1
        pnl = _safe_float(entry.realized_pnl, 0.0)
        cause_bucket["realized_pnl"] += pnl
        if pnl < 0:
            cause_bucket["loss_count"] += 1

        strategy_type = str(entry.strategy_type or "unknown")
        strat_bucket = strategy_map.setdefault(
            strategy_type,
            {"strategy_type": strategy_type, "count": 0, "realized_pnl": 0.0, "wins": 0},
        )
        strat_bucket["count"] += 1
        strat_bucket["realized_pnl"] += pnl
        if pnl > 0:
            strat_bucket["wins"] += 1

    root_cause_breakdown = []
    for bucket in root_causes.values():
        root_cause_breakdown.append(
            {
                "root_cause": bucket["root_cause"],
                "count": int(bucket["count"]),
                "loss_count": int(bucket["loss_count"]),
                "realized_pnl": round(bucket["realized_pnl"], 4),
            }
        )
    root_cause_breakdown.sort(key=lambda item: (-item["loss_count"], item["realized_pnl"]))

    strategy_learning = []
    policy_candidates = []
    for bucket in strategy_map.values():
        count = int(bucket["count"])
        win_rate_pct = round((int(bucket["wins"]) / count) * 100.0, 2) if count else 0.0
        realized = round(bucket["realized_pnl"], 4)
        row = {
            "strategy_type": bucket["strategy_type"],
            "count": count,
            "win_rate_pct": win_rate_pct,
            "realized_pnl": realized,
        }
        strategy_learning.append(row)
        if count >= 2 and realized < 0:
            policy_candidates.append(
                {
                    "kind": "strategy_review",
                    "strategy_type": bucket["strategy_type"],
                    "sample_count": count,
                    "realized_pnl": realized,
                    "recommendation": "review_reduce_or_disable",
                }
            )
    strategy_learning.sort(key=lambda item: (item["realized_pnl"], item["win_rate_pct"]))
    policy_candidates.sort(key=lambda item: (item["realized_pnl"], -item["sample_count"]))

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "account_type": account_type or "all",
        "enabled": True,
        "summary": {
            "closed_trades": len(filtered),
            "win_rate_pct": round((wins / len(filtered)) * 100.0, 2),
            "realized_pnl": realized_pnl,
            "post_mortem_coverage_pct": round(_ratio_pct(coverage, len(filtered)), 2),
            "policy_candidate_count": len(policy_candidates),
        },
        "root_cause_breakdown": root_cause_breakdown[:capped_limit],
        "strategy_learning": strategy_learning[:capped_limit],
        "policy_candidates": policy_candidates[:capped_limit],
    }


def build_attribution_integrity_snapshot(
    entries: list[TradingJournal],
    *,
    account_type: str | None = None,
    limit: int = 10,
) -> dict[str, Any]:
    capped_limit = max(1, min(int(limit), 25))
    filtered = [
        entry
        for entry in entries
        if account_type in {None, "", "all"} or entry.account_type == account_type
    ]
    open_entries = [entry for entry in filtered if entry.status == "open"]
    flagged_open = [entry for entry in open_entries if _is_bad_strategy_type(entry.strategy_type)]
    flagged_recent = [
        entry
        for entry in sorted(
            filtered,
            key=lambda item: (item.updated_at or item.last_synced_at or item.entry_time),
            reverse=True,
        )
        if _is_bad_strategy_type(entry.strategy_type)
    ][:capped_limit]

    attributed_open_positions_pct = round(
        ((len(open_entries) - len(flagged_open)) / len(open_entries)) * 100.0,
        2,
    ) if open_entries else 100.0
    open_untracked_ratio_pct = round((len(flagged_open) / len(open_entries)) * 100.0, 2) if open_entries else 0.0

    alerts: list[dict[str, Any]] = []
    if flagged_open:
        alerts.append(
            {
                "level": "critical",
                "code": "open_positions_unattributed",
                "message": f"{len(flagged_open)} open positions remain unattributed (strategy_type unknown/untracked).",
                "count": len(flagged_open),
            }
        )
    if flagged_recent:
        alerts.append(
            {
                "level": "warning",
                "code": "recent_entries_flagged",
                "message": f"{len(flagged_recent)} recent journal entries are flagged for attribution integrity review.",
                "count": len(flagged_recent),
            }
        )

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "account_type": account_type or "all",
        "enabled": True,
        "summary": {
            "entries_total": len(filtered),
            "open_positions": len(open_entries),
            "open_untracked_count": len(flagged_open),
            "recent_flagged_count": len(flagged_recent),
            "attributed_open_positions_pct": attributed_open_positions_pct,
            "open_untracked_ratio_pct": open_untracked_ratio_pct,
        },
        "alerts": alerts,
        "flagged_entries": [_entry_payload(entry) for entry in flagged_recent],
        "limit": capped_limit,
    }


def build_options_governance_adoption_snapshot(
    entries: list[TradingJournal],
    *,
    account_type: str | None = None,
    limit: int = 10,
) -> dict[str, Any]:
    capped_limit = max(1, min(int(limit), 20))
    filtered = [
        entry
        for entry in entries
        if account_type in {None, "", "all"} or entry.account_type == account_type
    ]
    option_entries = [
        entry
        for entry in filtered
        if _option_strategy_family(entry.strategy_type) is not None
    ]
    if not option_entries:
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "account_type": account_type or "all",
            "enabled": True,
            "summary": {
                "option_entries_total": 0,
                "open_option_positions": 0,
                "closed_option_trades": 0,
                "time_spread_count": 0,
                "vertical_count": 0,
                "neutral_theta_count": 0,
                "single_leg_count": 0,
                "strategy_diversity_count": 0,
                "time_spread_share_pct": 0.0,
                "vertical_share_pct": 0.0,
            },
            "alerts": [
                {
                    "level": "warning",
                    "code": "options_governance_not_exercised",
                    "message": "Options governance is implemented, but no option trades exist yet in the journal for this account scope.",
                }
            ],
            "family_mix": [],
            "strategy_mix": [],
            "limit": capped_limit,
        }

    open_entries = [entry for entry in option_entries if entry.status == "open"]
    closed_entries = [entry for entry in option_entries if entry.status == "closed"]
    family_counts: dict[str, int] = {}
    strategy_counts: dict[str, dict[str, Any]] = {}
    for entry in option_entries:
        family = str(_option_strategy_family(entry.strategy_type) or "unknown")
        family_counts[family] = family_counts.get(family, 0) + 1
        strategy_key = str(entry.strategy_type or "unknown")
        bucket = strategy_counts.setdefault(
            strategy_key,
            {"strategy_type": strategy_key, "count": 0, "status_open": 0, "status_closed": 0},
        )
        bucket["count"] += 1
        if entry.status == "open":
            bucket["status_open"] += 1
        elif entry.status == "closed":
            bucket["status_closed"] += 1

    option_total = len(option_entries)
    time_spread_count = family_counts.get("term_structure_time_spread", 0)
    vertical_count = family_counts.get("directional_vertical", 0)
    neutral_theta_count = family_counts.get("neutral_theta", 0)
    single_leg_count = family_counts.get("single_leg_directional", 0)
    time_spread_share_pct = round(_ratio_pct(time_spread_count, option_total), 2)
    vertical_share_pct = round(_ratio_pct(vertical_count, option_total), 2)

    alerts: list[dict[str, Any]] = []
    if vertical_share_pct >= 80.0 and option_total >= 5:
        alerts.append(
            {
                "level": "warning",
                "code": "options_mix_concentrated_in_verticals",
                "message": "Option usage is still heavily concentrated in directional verticals; calendars/diagonals are not yet being exercised much.",
                "vertical_share_pct": vertical_share_pct,
            }
        )
    if time_spread_count == 0 and option_total >= 5:
        alerts.append(
            {
                "level": "watch",
                "code": "time_spread_not_exercised",
                "message": "No calendar/diagonal structures have been exercised yet in the observed option sample.",
            }
        )

    family_mix = [
        {
            "family": family,
            "count": count,
            "share_pct": round(_ratio_pct(count, option_total), 2),
        }
        for family, count in family_counts.items()
    ]
    family_mix.sort(key=lambda item: (-item["count"], item["family"]))

    strategy_mix = [
        {
            **bucket,
            "share_pct": round(_ratio_pct(bucket["count"], option_total), 2),
        }
        for bucket in strategy_counts.values()
    ]
    strategy_mix.sort(key=lambda item: (-item["count"], item["strategy_type"]))

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "account_type": account_type or "all",
        "enabled": True,
        "summary": {
            "option_entries_total": option_total,
            "open_option_positions": len(open_entries),
            "closed_option_trades": len(closed_entries),
            "time_spread_count": time_spread_count,
            "vertical_count": vertical_count,
            "neutral_theta_count": neutral_theta_count,
            "single_leg_count": single_leg_count,
            "strategy_diversity_count": len(strategy_counts),
            "time_spread_share_pct": time_spread_share_pct,
            "vertical_share_pct": vertical_share_pct,
        },
        "alerts": alerts,
        "family_mix": family_mix[:capped_limit],
        "strategy_mix": strategy_mix[:capped_limit],
        "limit": capped_limit,
    }


def _make_aware(dt: datetime | None) -> datetime | None:
    """Ensure a datetime is timezone-aware (UTC) for safe comparison."""
    if dt is None:
        return None
    if dt.tzinfo is None:
        return dt.replace(tzinfo=timezone.utc)
    return dt


def _equity_curve(entries: list[TradingJournal]) -> list[dict[str, Any]]:
    running = 0.0
    points: list[dict[str, Any]] = []
    for entry in sorted(
        (item for item in entries if item.exit_time),
        key=lambda item: _make_aware(item.exit_time) or _make_aware(item.entry_time) or datetime.min.replace(tzinfo=timezone.utc),
    ):
        running += _safe_float(entry.realized_pnl, 0.0)
        points.append({"timestamp": (entry.exit_time or entry.entry_time).isoformat(), "equity": round(running, 4), "pnl": round(_safe_float(entry.realized_pnl, 0.0), 4)})
    return points


def _heatmap(entries: list[TradingJournal]) -> list[dict[str, Any]]:
    matrix: dict[tuple[int, int], dict[str, Any]] = {}
    for entry in entries:
        if not entry.entry_time:
            continue
        key = (entry.entry_time.weekday(), entry.entry_time.hour)
        bucket = matrix.setdefault(key, {"weekday": key[0], "hour": key[1], "trades": 0, "wins": 0})
        bucket["trades"] += 1
        if entry.realized_pnl > 0:
            bucket["wins"] += 1
    return [
        {**bucket, "success_rate_pct": round(bucket["wins"] / max(bucket["trades"], 1) * 100.0, 2)}
        for bucket in sorted(matrix.values(), key=lambda item: (item["weekday"], item["hour"]))
    ]


def _account_stats(entries: list[TradingJournal]) -> dict[str, Any]:
    closed = [entry for entry in entries if entry.status == "closed"]
    open_entries = [entry for entry in entries if entry.status == "open"]
    gross_wins = sum(max(_safe_float(entry.realized_pnl, 0.0), 0.0) for entry in closed)
    gross_losses = sum(min(_safe_float(entry.realized_pnl, 0.0), 0.0) for entry in closed)
    profit_factor = round(gross_wins / abs(gross_losses), 4) if gross_losses < 0 else None
    returns = [value for value in (_trade_return(entry) for entry in closed) if value is not None and value == value]
    sharpe_ratio = None
    if len(returns) >= 2:
        arr = np.array(returns, dtype=float)
        sigma = float(np.std(arr, ddof=1))
        if sigma > 1e-9:
            sharpe_ratio = round(float(np.mean(arr) / sigma * math.sqrt(len(arr))), 4)
    wins = [entry for entry in closed if _safe_float(entry.realized_pnl, 0.0) > 0]
    losses = [entry for entry in closed if _safe_float(entry.realized_pnl, 0.0) < 0]
    avg_win = float(np.mean([entry.realized_pnl for entry in wins])) if wins else 0.0
    avg_loss = abs(float(np.mean([entry.realized_pnl for entry in losses]))) if losses else 0.0
    win_rate = (len(wins) / len(closed) * 100.0) if closed else 0.0
    expectancy = round((len(wins) / len(closed) * avg_win - len(losses) / len(closed) * avg_loss), 4) if closed else 0.0
    return {
        "trades_closed": len(closed),
        "trades_open": len(open_entries),
        "win_rate_pct": round(win_rate, 2),
        "profit_factor": profit_factor,
        "sharpe_ratio": sharpe_ratio,
        "expectancy": expectancy,
        "gross_wins": round(gross_wins, 4),
        "gross_losses": round(gross_losses, 4),
        "realized_pnl": round(sum(_safe_float(entry.realized_pnl, 0.0) for entry in closed), 4),
        "unrealized_pnl": round(sum(_safe_float(entry.unrealized_pnl, 0.0) for entry in open_entries), 4),
        "equity_curve": _equity_curve(closed),
        "heatmap": _heatmap(closed),
    }


def _entry_payload(entry: TradingJournal) -> dict[str, Any]:
    return {
        "id": entry.id,
        "journal_key": entry.journal_key,
        "account_type": entry.account_type,
        "account_id": entry.account_id,
        "strategy_id": entry.strategy_id,
        "tracker_strategy_id": entry.tracker_strategy_id,
        "strategy_type": entry.strategy_type,
        "symbol": entry.symbol,
        "status": entry.status,
        "is_level4": bool(entry.is_level4),
        "entry_time": entry.entry_time.isoformat(),
        "exit_time": entry.exit_time.isoformat() if entry.exit_time else None,
        "entry_price": entry.entry_price,
        "exit_price": entry.exit_price,
        "fees": round(_safe_float(entry.fees, 0.0), 4),
        "win_rate_at_entry": entry.win_rate_at_entry,
        "current_win_rate_pct": entry.current_win_rate_pct,
        "iv_rank": entry.iv_rank,
        "realized_pnl": round(_safe_float(entry.realized_pnl, 0.0), 4),
        "unrealized_pnl": round(_safe_float(entry.unrealized_pnl, 0.0), 4),
        "mark_price": entry.mark_price,
        "spot_price": entry.spot_price,
        "entry_notional": entry.entry_notional,
        "risk_at_entry": entry.risk_at_entry,
        "thesis_rich_text": entry.thesis_rich_text or "",
        "legs": _json_load(entry.legs_details, []),
        "greeks": _json_load(entry.greeks_json, {}),
        "attribution": _json_load(entry.attribution_json, {}),
        "post_mortem": _json_load(entry.post_mortem_json, {}),
        "post_mortem_text": entry.post_mortem_text or "",
        # Normaliza payloads heredados del journal SQLite donde algunas columnas
        # historicas quedaron serializadas como {} en vez de [].
        "broker_order_ids": _coerce_list_payload(_json_load(entry.broker_order_ids_json, [])),
        "raw_entry_payload": _json_load(entry.raw_entry_payload_json, {}),
        "raw_exit_payload": _coerce_list_payload(_json_load(entry.raw_exit_payload_json, [])),
        "updated_at": entry.updated_at.isoformat() if entry.updated_at else None,
        "last_synced_at": entry.last_synced_at.isoformat() if entry.last_synced_at else None,
    }


class TradingJournalService:
    def __init__(
        self,
        tracker: StrategyTracker | None = None,
        brain: QuantBrainBridge | None = None,
        learning: AdaptiveLearningService | None = None,
        ic_tracker: ICSignalTracker | None = None,
        signal_validator: SignalValidator | None = None,
    ) -> None:
        self.tracker = tracker or StrategyTracker()
        self.brain = brain or QuantBrainBridge()
        self.learning = learning or AdaptiveLearningService()
        self.ic_tracker = ic_tracker or get_ic_tracker()
        self.signal_validator = signal_validator or SignalValidator()
        self._sync_lock_guard = threading.Lock()
        self._sync_locks: dict[str, threading.Lock] = {}
        self._last_close_attempt_at: dict[str, datetime] = {}

    def _scope_sync_lock(self, scope: str) -> threading.Lock:
        normalized = str(scope or "paper").strip().lower() or "paper"
        with self._sync_lock_guard:
            return self._sync_locks.setdefault(normalized, threading.Lock())

    def init_db(self) -> None:
        init_db()

    def _trade_events(self, client: TradierClient, account_id: str) -> list[dict[str, Any]]:
        start = date.today() - timedelta(days=settings.tradier_journal_history_days)
        return _normalize_trade_events(client.account_history(account_id, history_type="trade", start=start, end=date.today()))

    def _closed_positions(self, client: TradierClient, account_id: str) -> list[dict[str, Any]]:
        gainloss_fetch = getattr(client, "gainloss", None)
        if not callable(gainloss_fetch):
            return []
        try:
            payload = gainloss_fetch(account_id) or {}
        except Exception:
            logger.exception("Unable to fetch closed positions from broker gain/loss for %s", account_id)
            return []
        closed_positions = payload.get("closed_position") or []
        if isinstance(closed_positions, dict):
            return [closed_positions]
        if isinstance(closed_positions, list):
            return [item for item in closed_positions if isinstance(item, dict)]
        return []

    @staticmethod
    def _log_validation_result(result: ValidationResult, *, context: str, payload: dict[str, Any]) -> None:
        level = logging.WARNING if result.severity == "warn" else logging.ERROR
        logger.log(level, "%s rejected by signal validator: %s | payload=%s", context, result.reason, payload)

    @staticmethod
    def _validation_trade_payload(
        *,
        strategy: dict[str, Any],
        strategy_type: str,
        entry_price: float | None,
    ) -> dict[str, Any]:
        return {
            "symbol": str(strategy.get("underlying") or strategy.get("symbol") or ""),
            "underlying": str(strategy.get("underlying") or strategy.get("symbol") or ""),
            "strategy_type": strategy_type,
            "signed_qty": strategy.get("signed_qty"),
            "quantity": strategy.get("quantity"),
            "positions": strategy.get("positions") or [],
            "entry_price": entry_price,
        }

    def _upsert_open_entry(self, db: Any, account: TradierAccountSession, client: TradierClient, strategy: dict[str, Any], matched_events: list[dict[str, Any]]) -> tuple[str, bool, bool]:
        strategy_id, signature = _stable_strategy_id(strategy)
        # PATCH: skip sandbox-restricted symbols (OS, AA) — phantom prevention
        _SYNC_BLOCKED: frozenset = frozenset({"OS", "AA"})
        if str(strategy.get("underlying") or "") in _SYNC_BLOCKED:
            return strategy_id, False, True  # skip, not created, skipped=True
        incoming_strategy_type = str(strategy.get("strategy_type") or "unknown")
        existing_open_entries = db.execute(
            select(TradingJournal).where(
                TradingJournal.account_type == account.scope,
                TradingJournal.account_id == account.account_id,
                TradingJournal.status == "open",
                TradingJournal.symbol == str(strategy.get("underlying") or ""),
            )
        ).scalars().all()
        entry = _select_matching_open_entry(
            existing_open_entries,
            strategy=strategy,
            strategy_id=strategy_id,
        )
        if (
            entry is not None
            and _is_bad_strategy_type(incoming_strategy_type)
            and not _is_bad_strategy_type(entry.strategy_type)
        ):
            # Preserve the attributed identity when the live snapshot momentarily
            # regresses to "untracked"/unknown for the same open structure.
            strategy_id = str(entry.strategy_id or strategy_id)
            signature = str(entry.legs_signature or signature)
            incoming_strategy_type = str(entry.strategy_type or incoming_strategy_type)
        opened_at, entry_flow, open_fees = _open_event_details(matched_events)
        iv_rank = _estimate_iv_rank(client, str(strategy.get("underlying") or ""), _extract_avg_iv(strategy))
        normalized_entry_price = _recorded_price(
            entry_flow if entry_flow not in {None, 0.0} else _strategy_notional(strategy, "entry_price")
        )
        normalized_entry_notional = _recorded_price(_strategy_notional(strategy, "entry_price"))
        trade_payload = self._validation_trade_payload(
            strategy=strategy,
            strategy_type=incoming_strategy_type,
            entry_price=normalized_entry_price,
        )
        trade_result = self.signal_validator.validate_trade_entry(trade_payload)
        if not trade_result.is_valid:
            self._log_validation_result(trade_result, context="journal_trade_entry", payload=trade_payload)
            return strategy_id, False, True
        journal_result = self.signal_validator.validate_journal_write(
            {
                "symbol": trade_payload["symbol"],
                "underlying": trade_payload["underlying"],
                "entry_price": normalized_entry_price,
            }
        )
        if not journal_result.is_valid:
            self._log_validation_result(
                journal_result,
                context="journal_write",
                payload={
                    "strategy_id": strategy_id,
                    "symbol": trade_payload["symbol"],
                    "entry_price": normalized_entry_price,
                },
            )
            return strategy_id, False, True
        if entry is None:
            entry = TradingJournal(
                journal_key=f"{account.scope}:{account.account_id}:{strategy_id}:{int(datetime.utcnow().timestamp())}",
                account_type=account.scope,
                account_id=account.account_id,
                strategy_id=strategy_id,
                tracker_strategy_id=str(strategy.get("strategy_id") or ""),
                strategy_type=incoming_strategy_type,
                symbol=str(strategy.get("underlying") or ""),
                legs_signature=signature,
                legs_details=_json_dump(strategy.get("positions") or []),
                entry_price=normalized_entry_price,
                fees=open_fees,
                win_rate_at_entry=_safe_float(strategy.get("win_rate_pct"), float("nan")),
                current_win_rate_pct=_safe_float(strategy.get("win_rate_pct"), float("nan")),
                iv_rank=iv_rank,
                thesis_rich_text=_build_thesis(strategy, iv_rank),
                entry_time=opened_at or datetime.utcnow(),
                status="open",
                is_level4=str(strategy.get("strategy_type") or "") in LEVEL4_STRATEGIES,
                realized_pnl=0.0,
                unrealized_pnl=_safe_float(strategy.get("open_pnl"), 0.0),
                mark_price=_strategy_notional(strategy, "current_price"),
                spot_price=_safe_float(strategy.get("spot"), 0.0),
                entry_notional=normalized_entry_notional,
                risk_at_entry=abs(_safe_float(strategy.get("bpr_model"), 0.0)),
                greeks_json=_json_dump({"delta": strategy.get("net_delta"), "gamma": strategy.get("net_gamma"), "theta": strategy.get("theta_daily"), "vega": strategy.get("net_vega")}),
                attribution_json=_json_dump(strategy.get("attribution") or {}),
                broker_order_ids_json=_json_dump(sorted({event["raw"].get("id") for event in matched_events if event.get("raw") and event["raw"].get("id")})),
                raw_entry_payload_json=_json_dump(strategy.get("win_rate_details") or {}),
                last_synced_at=datetime.utcnow(),
            )
            db.add(entry)
            db.flush()
            return strategy_id, True, False

        entry.strategy_id = strategy_id
        entry.tracker_strategy_id = str(strategy.get("strategy_id") or entry.tracker_strategy_id or "")
        entry.strategy_type = incoming_strategy_type or str(entry.strategy_type or "unknown")
        entry.symbol = str(strategy.get("underlying") or entry.symbol or "")
        entry.legs_signature = signature
        entry.legs_details = _json_dump(strategy.get("positions") or [])
        entry.current_win_rate_pct = _safe_float(strategy.get("win_rate_pct"), entry.current_win_rate_pct or float("nan"))
        entry.unrealized_pnl = _safe_float(strategy.get("open_pnl"), 0.0)
        entry.mark_price = _strategy_notional(strategy, "current_price")
        entry.spot_price = _safe_float(strategy.get("spot"), 0.0)
        entry.entry_price = normalized_entry_price if normalized_entry_price is not None else entry.entry_price
        entry.entry_notional = normalized_entry_notional if normalized_entry_notional is not None else entry.entry_notional
        entry.iv_rank = iv_rank if iv_rank is not None else entry.iv_rank
        entry.fees = max(_safe_float(entry.fees, 0.0), open_fees)
        entry.is_level4 = str(strategy.get("strategy_type") or "") in LEVEL4_STRATEGIES
        entry.greeks_json = _json_dump({"delta": strategy.get("net_delta"), "gamma": strategy.get("net_gamma"), "theta": strategy.get("theta_daily"), "vega": strategy.get("net_vega")})
        entry.attribution_json = _json_dump(strategy.get("attribution") or {})
        entry.raw_entry_payload_json = _json_dump(strategy.get("win_rate_details") or {})
        entry.last_synced_at = datetime.utcnow()
        return strategy_id, False, False

    def _close_entry(self, entry: TradingJournal, matched_events: list[dict[str, Any]]) -> bool:
        closed_at, exit_flow, close_fees, close_events = _close_event_details(matched_events)
        if not close_events:
            entry.last_synced_at = datetime.utcnow()
            logger.warning(
                "Journal close skipped without broker close evidence for %s (%s)",
                entry.journal_key,
                entry.symbol,
            )
            return False
        entry.status = "closed"
        entry.exit_time = closed_at or datetime.utcnow()
        entry.exit_price = _recorded_price(exit_flow if exit_flow not in {None, 0.0} else entry.mark_price)
        entry.fees = round(_safe_float(entry.fees, 0.0) + close_fees, 4)
        if entry.entry_price is not None and entry.exit_price is not None:
            # FIX 2026-03-30: fórmula corregida — direction * (|exit| - |entry_per_share|) * qty
            # La formula anterior -(entry+exit) sumaba los precios en vez de restarlos,
            # produciendo PnL siempre negativo para longs y siempre positivo para shorts.
            # Usamos abs() para ser independientes de la convencion de signo del broker.
            entry_px = abs(_safe_float(entry.entry_notional or entry.entry_price, 0.0))
            exit_px  = abs(_safe_float(entry.exit_price, 0.0))
            _st = str(entry.strategy_type or "").lower()
            _is_bullish = any(kw in _st for kw in ("bull_call", "bull_put", "covered_call", "cash_secured_put"))
            is_short = not _is_bullish and any(kw in _st for kw in ("short", "bear_put", "bear_call", "long_put"))
            direction = -1.0 if is_short else 1.0
            _raw_ep = abs(_safe_float(entry.entry_price, 0.0))
            qty = round(entry_px / _raw_ep, 0) if _raw_ep > 0 else 1.0
            qty = max(qty, 1.0)
            entry.realized_pnl = round(
                direction * (exit_px - entry_px / qty) * qty - _safe_float(entry.fees, 0.0),
                4,
            )
        else:
            entry.realized_pnl = round(_safe_float(entry.unrealized_pnl, 0.0) - close_fees, 4)
        entry.unrealized_pnl = 0.0
        entry.raw_exit_payload_json = _json_dump([event.get("raw") for event in close_events if event.get("raw")])
        if entry.is_level4:
            attribution = _json_load(entry.attribution_json, {})
            driver = str((attribution or {}).get("dominant_driver") or "mixto")
            final_win = _safe_float(entry.current_win_rate_pct, float("nan"))
            root_cause = {"delta": "direccion", "vega": "volatilidad", "theta": "tiempo"}.get(driver, "mixto")
            entry.post_mortem_json = _json_dump({"entry_win_rate_pct": entry.win_rate_at_entry, "final_win_rate_pct": final_win if final_win == final_win else None, "realized_pnl": entry.realized_pnl, "driver": driver, "root_cause": root_cause})
            entry.post_mortem_text = (
                f"Post-mortem {entry.strategy_type}: win rate inicial {entry.win_rate_at_entry or 0:.2f}% vs final "
                f"{final_win if final_win == final_win else 0:.2f}%. Resultado {entry.realized_pnl:.2f}. "
                f"Driver dominante: {driver}. Diagnostico principal: {root_cause}."
            )
        entry.last_synced_at = datetime.utcnow()
        return True

    def _sync_closed_entry_learning(self, entry_payload: dict[str, Any]) -> dict[str, Any]:
        symbol = str(entry_payload.get("symbol") or "").upper().strip()
        entry_time = str(entry_payload.get("entry_time") or "")
        strategy_type = str(entry_payload.get("strategy_type") or "").strip() or None
        entry_price = abs(_safe_float(entry_payload.get("entry_price"), 0.0))
        exit_price = abs(_safe_float(entry_payload.get("exit_price"), 0.0))
        signal_id = None

        if symbol and exit_price > 0:
            try:
                signal_id = self.ic_tracker.update_pending_outcome(
                    symbol=symbol,
                    method=strategy_type,
                    entry_price=entry_price if entry_price > 0 else None,
                    recorded_near=entry_time or None,
                    exit_price=exit_price,
                )
            except Exception:
                logger.exception("Unable to update IC outcome for %s", entry_payload.get("journal_key"))

        return {
            "journal_key": entry_payload.get("journal_key"),
            "symbol": symbol,
            "signal_id": signal_id,
            "ic_updated": bool(signal_id),
        }

    def _process_closed_entry_payloads(self, scope: str, closed_entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
        outcomes: list[dict[str, Any]] = []
        if not closed_entries:
            return outcomes

        for entry_payload in closed_entries:
            outcomes.append(self._sync_closed_entry_learning(entry_payload))
            try:
                self.brain.record_journal_outcome(entry_payload)
            except Exception:
                logger.exception("Unable to publish journal outcome for %s", entry_payload.get("journal_key"))

        try:
            self.learning.refresh(force=True)
        except Exception:
            logger.exception("Unable to refresh adaptive learning after journal sync for scope=%s", scope)
        return outcomes

    def _should_guard_mass_close(
        self,
        *,
        scope: str,
        strategies_count: int,
        open_entries_count: int,
        unseen_entries_count: int,
    ) -> bool:
        if not settings.journal_sync_partial_snapshot_guard_enabled:
            return False
        if open_entries_count < settings.journal_sync_mass_close_min_positions:
            return False
        if unseen_entries_count <= 0:
            return False
        ratio_pct = (unseen_entries_count / max(open_entries_count, 1)) * 100.0
        if ratio_pct < settings.journal_sync_mass_close_ratio_pct:
            return False
        if strategies_count >= open_entries_count:
            return False
        last_attempt = self._last_close_attempt_at.get(scope)
        if last_attempt is not None:
            elapsed = (datetime.utcnow() - last_attempt).total_seconds()
            if elapsed < settings.journal_sync_close_debounce_sec:
                return True
        self._last_close_attempt_at[scope] = datetime.utcnow()
        return True

    def sync_scope(self, scope: str) -> dict[str, Any]:
        sync_lock = self._scope_sync_lock(scope)
        if not sync_lock.acquire(blocking=False):
            return {
                "scope": scope,
                "account_id": None,
                "created": 0,
                "updated": 0,
                "closed": 0,
                "active_strategies": 0,
                "trade_events_seen": 0,
                "learning_outcomes": [],
                "skipped": True,
                "reason": "sync_already_running",
            }

        client, account = resolve_account_session(account_scope=scope)  # type: ignore[arg-type]
        try:
            summary = self.tracker.build_summary(account_scope=scope, account_id=account.account_id)  # type: ignore[arg-type]
            strategies = summary.get("strategies") or []

            with session_scope() as db:
                existing_open_entries = db.execute(
                    select(TradingJournal).where(
                        TradingJournal.account_type == account.scope,
                        TradingJournal.account_id == account.account_id,
                        TradingJournal.status == "open",
                    )
                ).scalars().all()

            if not strategies and existing_open_entries:
                return {
                    "scope": scope,
                    "account_id": account.account_id,
                    "created": 0,
                    "updated": 0,
                    "closed": 0,
                    "active_strategies": 0,
                    "trade_events_seen": 0,
                    "learning_outcomes": [],
                    "skipped": True,
                    "reason": "empty_strategy_snapshot",
                    "open_entries_preserved": len(existing_open_entries),
                }

            trade_events = self._trade_events(client, account.account_id)
            try:
                broker_positions = [
                    _normalize_broker_position(position)
                    for position in (client.positions(account.account_id) or [])
                ]
            except Exception:
                logger.exception("Unable to fetch broker positions for scope=%s account=%s", scope, account.account_id)
                broker_positions = []
            created = 0
            updated = 0
            closed = 0
            invalid_skipped = 0
            seen_ids: set[str] = set()
            closed_entries: list[dict[str, Any]] = []

            with session_scope() as db:
                for strategy in strategies:
                    matched = _matching_events(trade_events, strategy)
                    strategy_id, created_now, skipped_now = self._upsert_open_entry(db, account, client, strategy, matched)
                    seen_ids.add(strategy_id)
                    if skipped_now:
                        invalid_skipped += 1
                        continue
                    if created_now:
                        created += 1
                    else:
                        updated += 1

                open_entries = db.execute(
                    select(TradingJournal).where(
                        TradingJournal.account_type == account.scope,
                        TradingJournal.account_id == account.account_id,
                        TradingJournal.status == "open",
                    )
                ).scalars().all()
                unseen_entries = [entry for entry in open_entries if entry.strategy_id not in seen_ids]
                if self._should_guard_mass_close(
                    scope=scope,
                    strategies_count=len(strategies),
                    open_entries_count=len(open_entries),
                    unseen_entries_count=len(unseen_entries),
                ):
                    logger.warning(
                        "Journal partial snapshot guard preserved %s/%s open entries for scope=%s (strategies=%s)",
                        len(unseen_entries),
                        len(open_entries),
                        scope,
                        len(strategies),
                    )
                    return {
                        "scope": scope,
                        "account_id": account.account_id,
                        "created": created,
                        "updated": updated,
                        "closed": 0,
                        "active_strategies": len(strategies),
                        "trade_events_seen": len(trade_events),
                        "learning_outcomes": [],
                        "skipped": True,
                        "reason": "partial_strategy_snapshot_guard",
                        "open_entries_preserved": len(unseen_entries),
                        "open_entries_total": len(open_entries),
                    }
                for entry in unseen_entries:
                    strategy_stub = {"underlying": entry.symbol, "positions": _json_load(entry.legs_details, [])}
                    if self._close_entry(entry, _matching_events(trade_events, strategy_stub)):
                        closed_entries.append(_entry_payload(entry))
                        closed += 1

                open_entries_after_sync = db.execute(
                    select(TradingJournal).where(
                        TradingJournal.account_type == account.scope,
                        TradingJournal.account_id == account.account_id,
                        TradingJournal.status == "open",
                    )
                ).scalars().all()

            learning_outcomes = self._process_closed_entry_payloads(scope, closed_entries)
            matched: list[dict[str, Any]] = []
            unmatched_journal: list[dict[str, Any]] = []
            unmatched_broker: list[dict[str, Any]] = []
            used_broker_indexes: set[int] = set()

            for entry in open_entries_after_sync:
                entry_order_id = _extract_broker_order_id(_json_load(entry.broker_order_ids_json, []))
                entry_symbol = str(entry.symbol or "").strip().upper()
                entry_qty = _entry_quantity(entry)
                resolved_match: dict[str, Any] | None = None
                resolved_idx: int | None = None
                match_method = "none"

                if entry_order_id:
                    for idx, broker_pos in enumerate(broker_positions):
                        if idx in used_broker_indexes:
                            continue
                        broker_id = str(broker_pos.get("broker_id") or "").strip()
                        if broker_id and broker_id == entry_order_id:
                            resolved_match = broker_pos
                            resolved_idx = idx
                            match_method = "broker_order_id"
                            break

                if resolved_match is None:
                    for idx, broker_pos in enumerate(broker_positions):
                        if idx in used_broker_indexes:
                            continue
                        if str(broker_pos.get("symbol") or "") != entry_symbol:
                            continue
                        broker_qty = abs(_safe_float(broker_pos.get("quantity"), 0.0))
                        qty_close = entry_qty <= 0 or broker_qty <= 0 or abs(entry_qty - broker_qty) <= 1e-6
                        if qty_close and _is_recent_entry(entry.entry_time, days=1):
                            resolved_match = broker_pos
                            resolved_idx = idx
                            match_method = "symbol_entry_time_fallback"
                            logger.info(
                                "Matched %s by (symbol, entry_date) fallback (broker_order_id was missing)",
                                entry_symbol,
                            )
                            break

                if resolved_match is None:
                    unmatched_journal.append(
                        {
                            "journal_key": entry.journal_key,
                            "symbol": entry_symbol,
                            "broker_order_id": entry_order_id,
                            "entry_time": entry.entry_time.isoformat() if entry.entry_time else None,
                        }
                    )
                    continue

                if resolved_idx is not None:
                    used_broker_indexes.add(resolved_idx)
                matched.append(
                    {
                        "journal_key": entry.journal_key,
                        "symbol": entry_symbol,
                        "broker_order_id": entry_order_id,
                        "broker_symbol": resolved_match.get("symbol"),
                        "broker_id": resolved_match.get("broker_id"),
                        "match_method": match_method,
                    }
                )

            for idx, broker_pos in enumerate(broker_positions):
                if idx in used_broker_indexes:
                    continue
                unmatched_broker.append(broker_pos)
                logger.error(
                    "PHANTOM DETECTED: Broker position %s qty=%s not in journal",
                    broker_pos.get("symbol"),
                    broker_pos.get("quantity"),
                )

            if not unmatched_broker and not unmatched_journal:
                reconciliation_status = "OK"
            elif unmatched_broker:
                reconciliation_status = "PARTIAL_UNMATCHED_BROKER"
            else:
                reconciliation_status = "PARTIAL_UNMATCHED_JOURNAL"

            return {
                "scope": scope,
                "account_id": account.account_id,
                "created": created,
                "updated": updated,
                "closed": closed,
                "active_strategies": len(strategies),
                "trade_events_seen": len(trade_events),
                "invalid_skipped": invalid_skipped,
                "learning_outcomes": learning_outcomes,
                "broker_positions": broker_positions,
                "journal_entries": [
                    {
                        "journal_key": entry.journal_key,
                        "symbol": str(entry.symbol or "").strip().upper(),
                        "entry_time": entry.entry_time.isoformat() if entry.entry_time else None,
                        "broker_order_id": _extract_broker_order_id(_json_load(entry.broker_order_ids_json, [])),
                    }
                    for entry in open_entries_after_sync
                ],
                "matched": matched,
                "unmatched_broker": unmatched_broker,
                "unmatched_journal": unmatched_journal,
                "reconciliation_status": reconciliation_status,
            }
        finally:
            sync_lock.release()

    def stats(self) -> dict[str, Any]:
        with session_scope() as db:
            entries = db.execute(select(TradingJournal)).scalars().all()
        buckets = {"live": [], "paper": []}
        for entry in entries:
            buckets.setdefault(entry.account_type, []).append(entry)
        live_stats = _account_stats(buckets.get("live", []))
        paper_stats = _account_stats(buckets.get("paper", []))
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "accounts": {"live": live_stats, "paper": paper_stats},
            "comparison": {
                "realized_pnl_gap": round(_safe_float(live_stats.get("realized_pnl"), 0.0) - _safe_float(paper_stats.get("realized_pnl"), 0.0), 4),
                "win_rate_gap_pct": round(_safe_float(live_stats.get("win_rate_pct"), 0.0) - _safe_float(paper_stats.get("win_rate_pct"), 0.0), 4),
                "expectancy_gap": round(_safe_float(live_stats.get("expectancy"), 0.0) - _safe_float(paper_stats.get("expectancy"), 0.0), 4),
            },
        }

    def account_summary(self, *, account_scope: str = "paper") -> dict[str, Any]:
        scope = str(account_scope or "paper").strip().lower() or "paper"
        with session_scope() as db:
            entries = db.execute(
                select(TradingJournal).where(TradingJournal.account_type == scope)
            ).scalars().all()
        payload = _account_stats(entries)
        payload["generated_at"] = datetime.utcnow().isoformat()
        payload["account_scope"] = scope
        return payload

    def entries(self, *, limit: int = 24, status: str | None = None) -> dict[str, Any]:
        capped_limit = max(1, min(int(limit), 100))
        with session_scope() as db:
            query = select(TradingJournal)
            if status in {"open", "closed"}:
                query = query.where(TradingJournal.status == status)
            rows = db.execute(
                query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc()).limit(capped_limit)
            ).scalars().all()
        return {
            "generated_at": datetime.utcnow().isoformat(),
            "count": len(rows),
            "items": [_entry_payload(entry) for entry in rows],
        }

    def chart_data(self, *, account_scope: str = "paper", limit: int = 500) -> dict[str, Any]:
        """Datos optimizados para el modulo de charts avanzados del dashboard."""
        capped = max(1, min(int(limit), 2000))
        with session_scope() as db:
            rows = db.execute(
                select(TradingJournal)
                .where(TradingJournal.status == "closed")
                .where(TradingJournal.account_type == account_scope)
                .order_by(TradingJournal.exit_time.desc(), TradingJournal.id.desc())
                .limit(capped)
            ).scalars().all()
        quality = build_journal_quality_scorecard(rows)
        rows = list(reversed(filter_closed_entries(rows, scorecard=quality, include_outlier_days=False)))

        trades: list[dict] = []
        equity = 10_000.0
        peak = equity
        for row in rows:
            pnl = _safe_float(row.realized_pnl, 0.0)
            equity += pnl
            peak = max(peak, equity)
            dd = ((equity - peak) / peak * 100.0) if peak > 0 else 0.0
            risk = abs(_safe_float(row.risk_at_entry, 0.0))
            r_mult = round(pnl / risk, 3) if risk > 0 else 0.0
            exit_ts = row.exit_time.isoformat() if row.exit_time else None
            entry_ts = row.entry_time.isoformat() if row.entry_time else None
            trades.append({
                "n": len(trades) + 1,
                "symbol": row.symbol or "",
                "strategy_type": row.strategy_type or "",
                "entry_time": entry_ts,
                "exit_time": exit_ts,
                "pnl": round(pnl, 2),
                "equity": round(equity, 2),
                "drawdown_pct": round(dd, 2),
                "r_multiple": r_mult,
                "win": pnl > 0,
            })

        # Daily PnL aggregation for calendar
        daily_pnl: dict[str, dict] = {}
        for t in trades:
            day = (t["exit_time"] or "")[:10]
            if not day:
                continue
            if day not in daily_pnl:
                daily_pnl[day] = {"pnl": 0.0, "trades": 0, "strategies": {}}
            daily_pnl[day]["pnl"] = round(daily_pnl[day]["pnl"] + t["pnl"], 2)
            daily_pnl[day]["trades"] += 1
            st = t["strategy_type"]
            if st:
                daily_pnl[day]["strategies"][st] = daily_pnl[day]["strategies"].get(st, 0) + 1

        calendar = [
            {"date": k, "pnl": v["pnl"], "trades": v["trades"],
             "dominant_strategy": max(v["strategies"], key=v["strategies"].get) if v["strategies"] else ""}
            for k, v in sorted(daily_pnl.items())
        ]

        return {
            "generated_at": datetime.utcnow().isoformat(),
            "account_scope": account_scope,
            "quality": quality,
            "total_trades": len(trades),
            "source_closed_trades": int(quality.get("total_closed_count") or 0),
            "excluded_trades": max(0, int(quality.get("total_closed_count") or 0) - len(trades)),
            "window_limit": capped,
            "window_mode": "latest_closed_trades",
            "trades": trades,
            "calendar": calendar,
        }

    def position_management_snapshot(self, *, account_type: str | None = None, limit: int = 12) -> dict[str, Any]:
        with session_scope() as db:
            query = select(TradingJournal).where(TradingJournal.status == "open")
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())).scalars().all()
        return build_position_management_snapshot(rows, account_type=account_type, limit=limit)

    def exit_governance_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        with session_scope() as db:
            query = select(TradingJournal).where(TradingJournal.status == "open")
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())).scalars().all()
        return build_exit_governance_snapshot(rows, account_type=account_type, limit=limit)

    def post_trade_learning_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        with session_scope() as db:
            query = select(TradingJournal).where(TradingJournal.status == "closed")
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())).scalars().all()
        return build_post_trade_learning_snapshot(rows, account_type=account_type, limit=limit)

    def attribution_integrity_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        with session_scope() as db:
            query = select(TradingJournal)
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())).scalars().all()
        return build_attribution_integrity_snapshot(rows, account_type=account_type, limit=limit)

    def options_governance_adoption_snapshot(self, *, account_type: str | None = None, limit: int = 10) -> dict[str, Any]:
        with session_scope() as db:
            query = select(TradingJournal)
            if account_type not in {None, "", "all"}:
                query = query.where(TradingJournal.account_type == account_type)
            rows = db.execute(query.order_by(TradingJournal.updated_at.desc(), TradingJournal.id.desc())).scalars().all()
        return build_options_governance_adoption_snapshot(rows, account_type=account_type, limit=limit)


def rebuild_from_broker_history(
    *,
    start_date: str,
    end_date: str | None = None,
    symbols: list[str] | None = None,
    account_scope: str = "paper",
    validate_quality: bool = True,
    quality_threshold_pct: float = 80.0,
    persist: bool = True,
    export_dir: Path | None = None,
) -> dict[str, Any]:
    service = TradingJournalService()
    service.init_db()
    start = datetime.fromisoformat(start_date).date()
    end = datetime.fromisoformat(end_date).date() if end_date else datetime.utcnow().date()
    client, account = resolve_account_session(account_scope=account_scope)  # type: ignore[arg-type]
    raw_events = service._trade_events(client, account.account_id)
    filtered_events = []
    allowed_symbols = _normalized_symbol_filter(symbols)
    for event in raw_events:
        timestamp = event.get("timestamp")
        symbol = str(event.get("symbol") or "").strip().upper()
        if not isinstance(timestamp, datetime):
            continue
        if timestamp.date() < start or timestamp.date() > end:
            continue
        if allowed_symbols and symbol not in allowed_symbols:
            continue
        filtered_events.append(event)

    closed_positions = service._closed_positions(client, account.account_id)
    filtered_closed_positions: list[dict[str, Any]] = []
    for item in closed_positions:
        symbol = str(item.get("symbol") or "").strip().upper()
        close_time = _parse_broker_datetime(item.get("close_date"))
        if close_time is None:
            continue
        if close_time.date() < start or close_time.date() > end:
            continue
        if allowed_symbols and symbol not in allowed_symbols:
            continue
        filtered_closed_positions.append(item)

    source_mode = "trade_events"
    rebuilt_rows = _rebuild_journal_rows_from_events(
        filtered_events,
        account_scope=account.scope,
        account_id=account.account_id,
        symbols=sorted(allowed_symbols) if allowed_symbols else None,
    )
    if not rebuilt_rows and filtered_closed_positions:
        rebuilt_rows = _rebuild_journal_rows_from_closed_positions(
            filtered_closed_positions,
            account_scope=account.scope,
            account_id=account.account_id,
            symbols=sorted(allowed_symbols) if allowed_symbols else None,
        )
        source_mode = "gainloss_closed_positions"
    rebuilt_rows, invalid_rebuilt_rows = _sanitize_rebuilt_rows(rebuilt_rows)
    quality_input = [SimpleNamespace(**row) for row in rebuilt_rows]
    quality = build_journal_quality_scorecard(quality_input)
    quality_score = float(quality.get("score_pct") or 0.0)
    if validate_quality and quality_score < quality_threshold_pct:
        raise RuntimeError(
            f"Rebuild quality score {quality_score:.2f} is below required threshold {quality_threshold_pct:.2f}"
        )

    deleted_existing = 0
    invalidated_negative_price_rows = 0
    adaptive_snapshot: dict[str, Any] | None = None
    operation_reset: dict[str, Any] | None = None
    if persist:
        with session_scope() as db:
            delete_query = delete(TradingJournal).where(
                TradingJournal.account_type == account.scope,
                TradingJournal.status == "closed",
                TradingJournal.entry_time >= datetime.combine(start, datetime.min.time()),
                TradingJournal.entry_time <= datetime.combine(end, datetime.max.time()),
            )
            if allowed_symbols:
                delete_query = delete_query.where(TradingJournal.symbol.in_(sorted(allowed_symbols)))
            result = db.execute(delete_query)
            deleted_existing = int(getattr(result, "rowcount", 0) or 0)
            invalid_query = delete(TradingJournal).where(
                TradingJournal.account_type == account.scope,
                TradingJournal.status == "closed",
                or_(
                    TradingJournal.entry_price < 0,
                    TradingJournal.exit_price < 0,
                ),
            )
            if allowed_symbols:
                invalid_query = invalid_query.where(TradingJournal.symbol.in_(sorted(allowed_symbols)))
            invalid_result = db.execute(invalid_query)
            invalidated_negative_price_rows = int(getattr(invalid_result, "rowcount", 0) or 0)
            for row in rebuilt_rows:
                db.add(TradingJournal(**row))
        adaptive_snapshot = _refresh_adaptive_snapshot_after_rebuild(min_cutoff_date=_SAFE_ADAPTIVE_REBUILD_DATE)
        operation_reset = _reset_post_rebuild_operational_state(account_scope=account.scope)

    export_root = export_dir or (settings.data_dir / "journal_rebuild")
    export_root.mkdir(parents=True, exist_ok=True)
    export_path = export_root / f"journal_rebuild_{account.scope}_{start.isoformat()}_{end.isoformat()}.json"
    ml_enabled = bool(rebuilt_rows) and quality_score >= quality_threshold_pct and bool(quality.get("learning_allowed", False))
    payload = {
        "account_scope": account.scope,
        "account_id": account.account_id,
        "start_date": start.isoformat(),
        "end_date": end.isoformat(),
        "symbols": sorted(allowed_symbols),
        "events_considered": len(filtered_events),
        "closed_positions_considered": len(filtered_closed_positions),
        "history_source": source_mode,
        "journal_rebuilt": len(rebuilt_rows),
        "invalid_rebuilt_rows": len(invalid_rebuilt_rows),
        "invalid_rebuilt_row_samples": invalid_rebuilt_rows[:10],
        "deleted_existing": deleted_existing,
        "invalidated_negative_price_rows": invalidated_negative_price_rows,
        "score_calidad": quality_score,
        "ml_habilitado": ml_enabled,
        "quality": quality,
        "adaptive_snapshot_cutoff": _SAFE_ADAPTIVE_REBUILD_DATE.isoformat(),
        "adaptive_snapshot_sample_count": int((adaptive_snapshot or {}).get("sample_count") or 0),
        "operation_reset": operation_reset,
        "export_path": str(export_path),
    }
    export_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, default=str), encoding="utf-8")
    return payload
