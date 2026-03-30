"""Trading journal synchronization and analytics services."""
from __future__ import annotations

import hashlib
import json
import logging
import math
import threading
from datetime import date, datetime, timedelta
from typing import Any

import numpy as np
from sqlalchemy import select

from backtesting.winning_probability import TradierClient, _compute_realized_vol, _safe_float
from execution.tradier_controls import (
    TradierAccountSession,
    _action_bucket,
    _event_action,
    _event_symbol,
    _event_timestamp,
    resolve_account_session,
)
from atlas_code_quant.journal.db import init_db, session_scope
from atlas_code_quant.journal.models import TradingJournal
from config.settings import settings
from learning.adaptive_policy import AdaptiveLearningService
from learning.ic_signal_tracker import ICSignalTracker, get_ic_tracker
from monitoring.strategy_tracker import StrategyTracker
from operations.brain_bridge import QuantBrainBridge

logger = logging.getLogger("quant.journal")

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
            },
            "alerts": [],
            "concentrations": {"by_symbol": [], "by_strategy_type": []},
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
        },
        "alerts": alerts,
        "concentrations": {
            "by_symbol": symbol_concentrations[: capped_limit // 2 if capped_limit > 1 else 1],
            "by_strategy_type": strategy_concentrations[: capped_limit // 2 if capped_limit > 1 else 1],
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
    }
    candidates: list[dict[str, Any]] = []
    action_counts = {
        "exit_now": 0,
        "de_risk": 0,
        "take_profit": 0,
        "hold": 0,
    }

    for row in position_management.get("watchlist") or []:
        reasons = set(str(reason) for reason in row.get("alert_reasons") or [])
        open_r_multiple = _safe_float(row.get("open_r_multiple"), 0.0)
        holding_hours = _safe_float(row.get("holding_hours"), 0.0)
        thesis_drift_pct = _safe_float(row.get("thesis_drift_pct"), float("nan"))
        symbol_heat_pct = _safe_float(row.get("symbol_heat_pct"), 0.0)

        recommendation = "hold"
        exit_reason = "monitor_only"
        urgency = "low"

        if open_r_multiple <= -float(settings.exit_governance_hard_exit_loss_r):
            recommendation = "exit_now"
            exit_reason = "hard_stop_loss_r"
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


def _equity_curve(entries: list[TradingJournal]) -> list[dict[str, Any]]:
    running = 0.0
    points: list[dict[str, Any]] = []
    for entry in sorted((item for item in entries if item.exit_time), key=lambda item: item.exit_time or item.entry_time):
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
    ) -> None:
        self.tracker = tracker or StrategyTracker()
        self.brain = brain or QuantBrainBridge()
        self.learning = learning or AdaptiveLearningService()
        self.ic_tracker = ic_tracker or get_ic_tracker()
        self._sync_lock_guard = threading.Lock()
        self._sync_locks: dict[str, threading.Lock] = {}

    def _scope_sync_lock(self, scope: str) -> threading.Lock:
        normalized = str(scope or "paper").strip().lower() or "paper"
        with self._sync_lock_guard:
            return self._sync_locks.setdefault(normalized, threading.Lock())

    def init_db(self) -> None:
        init_db()

    def _trade_events(self, client: TradierClient, account_id: str) -> list[dict[str, Any]]:
        start = date.today() - timedelta(days=settings.tradier_journal_history_days)
        return _normalize_trade_events(client.account_history(account_id, history_type="trade", start=start, end=date.today()))

    def _upsert_open_entry(self, db: Any, account: TradierAccountSession, client: TradierClient, strategy: dict[str, Any], matched_events: list[dict[str, Any]]) -> tuple[str, bool]:
        strategy_id, signature = _stable_strategy_id(strategy)
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
                entry_price=entry_flow if entry_flow not in {None, 0.0} else _strategy_notional(strategy, "entry_price"),
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
                entry_notional=abs(_strategy_notional(strategy, "entry_price")),
                risk_at_entry=abs(_safe_float(strategy.get("bpr_model"), 0.0)),
                greeks_json=_json_dump({"delta": strategy.get("net_delta"), "gamma": strategy.get("net_gamma"), "theta": strategy.get("theta_daily"), "vega": strategy.get("net_vega")}),
                attribution_json=_json_dump(strategy.get("attribution") or {}),
                broker_order_ids_json=_json_dump(sorted({event["raw"].get("id") for event in matched_events if event.get("raw") and event["raw"].get("id")})),
                raw_entry_payload_json=_json_dump(strategy.get("win_rate_details") or {}),
                last_synced_at=datetime.utcnow(),
            )
            db.add(entry)
            db.flush()
            return strategy_id, True

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
        entry.iv_rank = iv_rank if iv_rank is not None else entry.iv_rank
        entry.fees = max(_safe_float(entry.fees, 0.0), open_fees)
        entry.is_level4 = str(strategy.get("strategy_type") or "") in LEVEL4_STRATEGIES
        entry.greeks_json = _json_dump({"delta": strategy.get("net_delta"), "gamma": strategy.get("net_gamma"), "theta": strategy.get("theta_daily"), "vega": strategy.get("net_vega")})
        entry.attribution_json = _json_dump(strategy.get("attribution") or {})
        entry.raw_entry_payload_json = _json_dump(strategy.get("win_rate_details") or {})
        entry.last_synced_at = datetime.utcnow()
        return strategy_id, False

    def _close_entry(self, entry: TradingJournal, matched_events: list[dict[str, Any]]) -> None:
        closed_at, exit_flow, close_fees, close_events = _close_event_details(matched_events)
        entry.status = "closed"
        entry.exit_time = closed_at or datetime.utcnow()
        entry.exit_price = exit_flow if exit_flow not in {None, 0.0} else entry.mark_price
        entry.fees = round(_safe_float(entry.fees, 0.0) + close_fees, 4)
        if entry.entry_price is not None and entry.exit_price is not None:
            # FIX 2026-03-30: fórmula corregida — direction * (|exit| - |entry_per_share|) * qty
            # La formula anterior -(entry+exit) sumaba los precios en vez de restarlos,
            # produciendo PnL siempre negativo para longs y siempre positivo para shorts.
            # Usamos abs() para ser independientes de la convencion de signo del broker.
            entry_px = abs(_safe_float(entry.entry_notional or entry.entry_price, 0.0))
            exit_px  = abs(_safe_float(entry.exit_price, 0.0))
            _st = str(entry.strategy_type or "").lower()
            is_short = any(kw in _st for kw in ("short", "bear_put", "bear_call", "long_put"))
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
            created = 0
            updated = 0
            closed = 0
            seen_ids: set[str] = set()
            closed_entries: list[dict[str, Any]] = []

            with session_scope() as db:
                for strategy in strategies:
                    matched = _matching_events(trade_events, strategy)
                    strategy_id, created_now = self._upsert_open_entry(db, account, client, strategy, matched)
                    seen_ids.add(strategy_id)
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
                for entry in open_entries:
                    if entry.strategy_id in seen_ids:
                        continue
                    strategy_stub = {"underlying": entry.symbol, "positions": _json_load(entry.legs_details, [])}
                    self._close_entry(entry, _matching_events(trade_events, strategy_stub))
                    closed_entries.append(_entry_payload(entry))
                    closed += 1

            learning_outcomes = self._process_closed_entry_payloads(scope, closed_entries)

            return {
                "scope": scope,
                "account_id": account.account_id,
                "created": created,
                "updated": updated,
                "closed": closed,
                "active_strategies": len(strategies),
                "trade_events_seen": len(trade_events),
                "learning_outcomes": learning_outcomes,
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
