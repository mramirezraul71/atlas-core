"""Trading journal synchronization and analytics services."""
from __future__ import annotations

import hashlib
import json
import logging
import math
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
from journal.db import init_db, session_scope
from journal.models import TradingJournal
from monitoring.strategy_tracker import StrategyTracker
from config.settings import settings

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


def _json_dump(value: Any) -> str:
    return json.dumps(value or {}, ensure_ascii=True, default=str)


def _json_load(value: str | None, fallback: Any) -> Any:
    if not value:
        return fallback
    try:
        return json.loads(value)
    except Exception:
        return fallback


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
        f"<li>Tesis automática: conservar la estructura mientras el perfil de probabilidad siga a favor y el driver dominante no se acelere en contra.</li>"
        f"<li>Sentimiento Grok: pendiente de enriquecimiento externo; el campo queda listo para persistir esa explicación.</li>"
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
        "broker_order_ids": _json_load(entry.broker_order_ids_json, []),
        "raw_entry_payload": _json_load(entry.raw_entry_payload_json, {}),
        "raw_exit_payload": _json_load(entry.raw_exit_payload_json, []),
        "updated_at": entry.updated_at.isoformat() if entry.updated_at else None,
        "last_synced_at": entry.last_synced_at.isoformat() if entry.last_synced_at else None,
    }


class TradingJournalService:
    def __init__(self, tracker: StrategyTracker | None = None) -> None:
        self.tracker = tracker or StrategyTracker()

    def init_db(self) -> None:
        init_db()

    def _trade_events(self, client: TradierClient, account_id: str) -> list[dict[str, Any]]:
        start = date.today() - timedelta(days=settings.tradier_journal_history_days)
        return _normalize_trade_events(client.account_history(account_id, history_type="trade", start=start, end=date.today()))

    def _upsert_open_entry(self, db: Any, account: TradierAccountSession, client: TradierClient, strategy: dict[str, Any], matched_events: list[dict[str, Any]]) -> tuple[str, bool]:
        strategy_id, signature = _stable_strategy_id(strategy)
        entry = db.execute(
            select(TradingJournal).where(
                TradingJournal.account_type == account.scope,
                TradingJournal.account_id == account.account_id,
                TradingJournal.strategy_id == strategy_id,
                TradingJournal.status == "open",
            ).limit(1)
        ).scalar_one_or_none()
        opened_at, entry_flow, open_fees = _open_event_details(matched_events)
        iv_rank = _estimate_iv_rank(client, str(strategy.get("underlying") or ""), _extract_avg_iv(strategy))
        if entry is None:
            entry = TradingJournal(
                journal_key=f"{account.scope}:{account.account_id}:{strategy_id}:{int(datetime.utcnow().timestamp())}",
                account_type=account.scope,
                account_id=account.account_id,
                strategy_id=strategy_id,
                tracker_strategy_id=str(strategy.get("strategy_id") or ""),
                strategy_type=str(strategy.get("strategy_type") or "unknown"),
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

        entry.tracker_strategy_id = str(strategy.get("strategy_id") or entry.tracker_strategy_id or "")
        entry.legs_signature = signature
        entry.legs_details = _json_dump(strategy.get("positions") or [])
        entry.current_win_rate_pct = _safe_float(strategy.get("win_rate_pct"), entry.current_win_rate_pct or float("nan"))
        entry.unrealized_pnl = _safe_float(strategy.get("open_pnl"), 0.0)
        entry.mark_price = _strategy_notional(strategy, "current_price")
        entry.spot_price = _safe_float(strategy.get("spot"), 0.0)
        entry.iv_rank = iv_rank if iv_rank is not None else entry.iv_rank
        entry.fees = max(_safe_float(entry.fees, 0.0), open_fees)
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
            entry.realized_pnl = round(-(entry.entry_price + entry.exit_price) - _safe_float(entry.fees, 0.0), 4)
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

    def sync_scope(self, scope: str) -> dict[str, Any]:
        client, account = resolve_account_session(account_scope=scope)  # type: ignore[arg-type]
        summary = self.tracker.build_summary(account_scope=scope, account_id=account.account_id)  # type: ignore[arg-type]
        strategies = summary.get("strategies") or []
        trade_events = self._trade_events(client, account.account_id)
        created = 0
        updated = 0
        closed = 0
        seen_ids: set[str] = set()

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
                closed += 1

        return {"scope": scope, "account_id": account.account_id, "created": created, "updated": updated, "closed": closed, "active_strategies": len(strategies), "trade_events_seen": len(trade_events)}

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
