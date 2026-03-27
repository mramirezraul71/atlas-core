"""Advanced monitoring for live Tradier option positions."""
from __future__ import annotations

import hashlib
import logging
import math
import re
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any

from atlas_code_quant.backtesting.winning_probability import (
    SUPPORTED_STRATEGIES,
    StrategyLeg,
    StrategyType,
    TradierClient,
    _safe_float,
    estimate_probability_from_legs,
)
from atlas_code_quant.config.settings import settings
from atlas_code_quant.execution.tradier_controls import (
    TradierAccountSession,
    check_pdt_status,
    resolve_account_session,
)


logger = logging.getLogger("quant.monitor.advanced")

OPTION_RE = re.compile(r"^(?P<underlying>[A-Z]{1,6})(?P<expiry>\d{6})(?P<cp>[CP])(?P<strike>\d{8})$")
_PROBABILITY_CACHE: dict[str, tuple[float, dict[str, Any]]] = {}


@dataclass
class NormalizedPosition:
    symbol: str
    underlying: str
    asset_class: str
    signed_qty: float
    quantity_abs: float
    side: str
    strike: float | None
    option_type: str | None
    expiration: str | None
    dte: int | None
    entry_price: float
    current_price: float
    current_pnl: float
    prev_close: float | None
    greeks: dict[str, float]


def _as_list(value: Any) -> list[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _parse_option_symbol(symbol: str) -> dict[str, Any] | None:
    candidate = symbol.replace(" ", "").upper()
    match = OPTION_RE.match(candidate)
    if not match:
        return None
    expiry = datetime.strptime(match.group("expiry"), "%y%m%d").date()
    strike = int(match.group("strike")) / 1000.0
    return {
        "underlying": match.group("underlying"),
        "expiration": expiry.isoformat(),
        "option_type": "call" if match.group("cp") == "C" else "put",
        "strike": strike,
        "dte": max((expiry - datetime.utcnow().date()).days, 0),
    }


def _signed_quantity(raw: dict[str, Any]) -> float:
    quantity = _safe_float(raw.get("quantity") or raw.get("qty") or raw.get("quantity_open"), 0.0)
    side = str(raw.get("side") or raw.get("position_type") or "").lower()
    if quantity == 0:
        return 0.0
    if side in {"short", "sell"} and quantity > 0:
        return -quantity
    return quantity


def _entry_price(raw: dict[str, Any], *, multiplier: float, quantity_abs: float) -> float:
    for key in ("cost_basis", "cost", "average_buy_price", "average_open_price", "basis"):
        value = _safe_float(raw.get(key), float("nan"))
        if value == value and quantity_abs > 0:
            if key in {"cost_basis", "cost", "basis"}:
                return abs(value) / max(quantity_abs * multiplier, 1e-6)
            return abs(value)
    return _safe_float(raw.get("price"), 0.0)


def _current_price(raw: dict[str, Any], *, multiplier: float, quantity_abs: float) -> float:
    for key in ("last", "mark", "current_price", "close", "price"):
        value = _safe_float(raw.get(key), float("nan"))
        if value == value and value > 0:
            return value
    market_value = _safe_float(raw.get("market_value"), float("nan"))
    if market_value == market_value and quantity_abs > 0:
        return abs(market_value) / max(quantity_abs * multiplier, 1e-6)
    return 0.0


def _current_pnl(raw: dict[str, Any], *, entry_price: float, current_price: float, signed_qty: float, multiplier: float) -> float:
    for key in ("gain_loss", "gainloss", "open_pnl", "unrealized_gain_loss", "unrealized_pl"):
        value = _safe_float(raw.get(key), float("nan"))
        if value == value:
            return value
    return (current_price - entry_price) * signed_qty * multiplier


def _normalize_positions(positions: list[dict[str, Any]]) -> list[NormalizedPosition]:
    normalized: list[NormalizedPosition] = []
    for raw in positions:
        symbol = str(raw.get("symbol") or raw.get("option_symbol") or "").strip().upper()
        if not symbol:
            continue
        option_meta = _parse_option_symbol(symbol)
        multiplier = 100.0 if option_meta else 1.0
        signed_qty = _signed_quantity(raw)
        quantity_abs = abs(signed_qty)
        if quantity_abs <= 0:
            continue
        entry_price = _entry_price(raw, multiplier=multiplier, quantity_abs=quantity_abs)
        current_price = _current_price(raw, multiplier=multiplier, quantity_abs=quantity_abs)
        normalized.append(
            NormalizedPosition(
                symbol=symbol,
                underlying=(option_meta or {}).get("underlying") or symbol,
                asset_class="option" if option_meta else "equity",
                signed_qty=signed_qty,
                quantity_abs=quantity_abs,
                side="short" if signed_qty < 0 else "long",
                strike=(option_meta or {}).get("strike"),
                option_type=(option_meta or {}).get("option_type"),
                expiration=(option_meta or {}).get("expiration"),
                dte=(option_meta or {}).get("dte"),
                entry_price=entry_price,
                current_price=current_price,
                current_pnl=_current_pnl(raw, entry_price=entry_price, current_price=current_price, signed_qty=signed_qty, multiplier=multiplier),
                prev_close=_safe_float(raw.get("prevclose"), float("nan")),
                greeks={},
            )
        )
    return normalized


def _option_key(position: NormalizedPosition) -> tuple[str, str]:
    return position.underlying, position.expiration or ""


def _enrich_option_greeks(client: TradierClient, positions: list[NormalizedPosition]) -> None:
    targets = {(position.underlying, position.expiration) for position in positions if position.asset_class == "option" and position.expiration}
    chain_index: dict[str, dict[str, Any]] = {}
    for underlying, expiration in targets:
        try:
            for contract in client.chain(underlying, expiration):
                symbol = str(contract.get("symbol") or "").strip().upper()
                if symbol:
                    chain_index[symbol] = contract
        except Exception:
            logger.exception("Unable to fetch chain for %s %s", underlying, expiration)
    for position in positions:
        if position.asset_class != "option":
            continue
        contract = chain_index.get(position.symbol, {})
        greeks = contract.get("greeks") or {}
        position.greeks = {
            "delta": _safe_float(greeks.get("delta"), 0.0),
            "theta": _safe_float(greeks.get("theta"), 0.0),
            "vega": _safe_float(greeks.get("vega"), 0.0),
            "gamma": _safe_float(greeks.get("gamma"), 0.0),
            "iv": _safe_float(greeks.get("mid_iv") or greeks.get("smv_vol") or contract.get("iv"), 0.0),
        }
        bid = _safe_float(contract.get("bid"), 0.0)
        ask = _safe_float(contract.get("ask"), 0.0)
        mark = (bid + ask) / 2.0 if bid > 0 and ask > 0 else _safe_float(contract.get("last"), position.current_price)
        if mark > 0:
            position.current_price = mark
            position.current_pnl = (position.current_price - position.entry_price) * position.signed_qty * 100.0


def _enrich_equity_quotes(client: TradierClient, positions: list[NormalizedPosition]) -> dict[str, dict[str, Any]]:
    underlyings = sorted({position.underlying for position in positions})
    quotes = client.quotes(underlyings, greeks=False) if underlyings else []
    quote_index = {str(quote.get("symbol") or "").upper(): quote for quote in quotes}
    for position in positions:
        quote = quote_index.get(position.underlying, {})
        if not quote:
            continue
        if position.asset_class == "equity":
            position.current_price = _safe_float(quote.get("last") or quote.get("close"), position.current_price)
            position.prev_close = _safe_float(quote.get("prevclose"), float("nan"))
            position.current_pnl = (position.current_price - position.entry_price) * position.signed_qty
        elif position.prev_close is None or position.prev_close != position.prev_close:
            position.prev_close = _safe_float(quote.get("prevclose"), float("nan"))
    return quote_index


def _strategy_fingerprint(strategy_type: str, positions: list[NormalizedPosition]) -> str:
    digest = hashlib.sha1()
    digest.update(strategy_type.encode("utf-8"))
    for position in sorted(positions, key=lambda item: (item.symbol, item.signed_qty)):
        digest.update(f"{position.symbol}:{position.signed_qty}:{position.entry_price}:{position.current_price}".encode("utf-8"))
    return digest.hexdigest()


def _group_single(position: NormalizedPosition, strategy_type: str) -> dict[str, Any]:
    return {
        "group_id": f"{strategy_type}:{position.symbol}",
        "underlying": position.underlying,
        "strategy_type": strategy_type,
        "positions": [position],
    }


def _infer_groups(positions: list[NormalizedPosition]) -> list[dict[str, Any]]:
    groups: list[dict[str, Any]] = []
    by_underlying: dict[str, list[NormalizedPosition]] = {}
    for position in positions:
        by_underlying.setdefault(position.underlying, []).append(position)

    for underlying, bucket in by_underlying.items():
        equities = [position for position in bucket if position.asset_class == "equity"]
        options = [position for position in bucket if position.asset_class == "option"]
        if equities and len(options) == 1 and options[0].option_type == "call" and options[0].signed_qty < 0:
            groups.append({"group_id": f"covered_call:{underlying}", "underlying": underlying, "strategy_type": "covered_call", "positions": equities + options})
            continue
        if len(options) == 1:
            option = options[0]
            if option.option_type == "call" and option.signed_qty > 0:
                groups.append(_group_single(option, "long_call"))
                continue
            if option.option_type == "put" and option.signed_qty > 0:
                groups.append(_group_single(option, "long_put"))
                continue
            if option.option_type == "put" and option.signed_qty < 0:
                groups.append(_group_single(option, "cash_secured_put"))
                continue

        if len(options) == 2:
            a, b = sorted(options, key=lambda item: (item.expiration or "", item.strike or 0.0, item.option_type or ""))
            if a.option_type == "call" and b.option_type == "call" and a.expiration == b.expiration:
                if a.signed_qty > 0 and b.signed_qty < 0:
                    groups.append({"group_id": f"bull_call_debit_spread:{underlying}", "underlying": underlying, "strategy_type": "bull_call_debit_spread", "positions": [a, b]})
                    continue
                if a.signed_qty < 0 and b.signed_qty > 0:
                    groups.append({"group_id": f"bear_call_credit_spread:{underlying}", "underlying": underlying, "strategy_type": "bear_call_credit_spread", "positions": [a, b]})
                    continue
            if a.option_type == "put" and b.option_type == "put" and a.expiration == b.expiration:
                if a.signed_qty < 0 and b.signed_qty > 0:
                    groups.append({"group_id": f"bull_put_credit_spread:{underlying}", "underlying": underlying, "strategy_type": "bull_put_credit_spread", "positions": [a, b]})
                    continue
                if a.signed_qty > 0 and b.signed_qty < 0:
                    groups.append({"group_id": f"bear_put_debit_spread:{underlying}", "underlying": underlying, "strategy_type": "bear_put_debit_spread", "positions": [a, b]})
                    continue
            if a.option_type != b.option_type and a.expiration == b.expiration:
                if a.signed_qty > 0 and b.signed_qty > 0 and math.isclose(a.strike or 0.0, b.strike or 0.0, abs_tol=1e-6):
                    groups.append({"group_id": f"long_straddle:{underlying}", "underlying": underlying, "strategy_type": "long_straddle", "positions": [a, b]})
                    continue
                if a.signed_qty > 0 and b.signed_qty > 0:
                    groups.append({"group_id": f"long_strangle:{underlying}", "underlying": underlying, "strategy_type": "long_strangle", "positions": [a, b]})
                    continue
            if a.option_type == b.option_type and a.expiration != b.expiration:
                long_leg = a if a.signed_qty > 0 else b
                short_leg = b if long_leg is a else a
                if long_leg.signed_qty > 0 and short_leg.signed_qty < 0:
                    same_strike = math.isclose(a.strike or 0.0, b.strike or 0.0, abs_tol=1e-6)
                    option_type = a.option_type or "call"
                    if same_strike:
                        strategy_type = "call_calendar_spread" if option_type == "call" else "put_calendar_spread"
                        groups.append({"group_id": f"{strategy_type}:{underlying}", "underlying": underlying, "strategy_type": strategy_type, "positions": [long_leg, short_leg]})
                        continue
                    if option_type == "call" and (long_leg.strike or 0.0) < (short_leg.strike or 0.0):
                        groups.append({"group_id": f"call_diagonal_debit_spread:{underlying}", "underlying": underlying, "strategy_type": "call_diagonal_debit_spread", "positions": [long_leg, short_leg]})
                        continue
                    if option_type == "put" and (long_leg.strike or 0.0) > (short_leg.strike or 0.0):
                        groups.append({"group_id": f"put_diagonal_debit_spread:{underlying}", "underlying": underlying, "strategy_type": "put_diagonal_debit_spread", "positions": [long_leg, short_leg]})
                        continue

        if len(options) == 3:
            trio = sorted(options, key=lambda item: (item.expiration or "", item.option_type or "", item.strike or 0.0))
            expirations = {position.expiration for position in trio}
            option_types = {position.option_type for position in trio}
            if len(expirations) == 1 and len(option_types) == 1:
                low, mid, high = trio
                if (
                    math.isclose(mid.quantity_abs, 2.0, abs_tol=1e-6)
                    and math.isclose(low.quantity_abs, 1.0, abs_tol=1e-6)
                    and math.isclose(high.quantity_abs, 1.0, abs_tol=1e-6)
                ):
                    option_type = low.option_type or "call"
                    if low.signed_qty > 0 and mid.signed_qty < 0 and high.signed_qty > 0:
                        strategy_type = "call_debit_butterfly" if option_type == "call" else "put_debit_butterfly"
                        groups.append({"group_id": f"{strategy_type}:{underlying}", "underlying": underlying, "strategy_type": strategy_type, "positions": trio})
                        continue
                    if low.signed_qty < 0 and mid.signed_qty > 0 and high.signed_qty < 0:
                        strategy_type = "call_credit_butterfly" if option_type == "call" else "put_credit_butterfly"
                        groups.append({"group_id": f"{strategy_type}:{underlying}", "underlying": underlying, "strategy_type": strategy_type, "positions": trio})
                        continue

        if len(options) == 4:
            calls = sorted([position for position in options if position.option_type == "call"], key=lambda item: item.strike or 0.0)
            puts = sorted([position for position in options if position.option_type == "put"], key=lambda item: item.strike or 0.0)
            if len(calls) == 4 and len({position.expiration for position in calls}) == 1:
                signs = [1 if position.signed_qty > 0 else -1 for position in calls]
                strikes = [position.strike or 0.0 for position in calls]
                unique_strikes = len({round(strike, 6) for strike in strikes})
                if unique_strikes == 3 and signs == [1, -1, -1, 1]:
                    groups.append({"group_id": f"call_debit_butterfly:{underlying}", "underlying": underlying, "strategy_type": "call_debit_butterfly", "positions": calls})
                    continue
                if unique_strikes == 3 and signs == [-1, 1, 1, -1]:
                    groups.append({"group_id": f"call_credit_butterfly:{underlying}", "underlying": underlying, "strategy_type": "call_credit_butterfly", "positions": calls})
                    continue
                if unique_strikes == 4 and signs == [1, -1, -1, 1]:
                    groups.append({"group_id": f"call_debit_condor:{underlying}", "underlying": underlying, "strategy_type": "call_debit_condor", "positions": calls})
                    continue
                if unique_strikes == 4 and signs == [-1, 1, 1, -1]:
                    groups.append({"group_id": f"call_credit_condor:{underlying}", "underlying": underlying, "strategy_type": "call_credit_condor", "positions": calls})
                    continue
            if len(puts) == 4 and len({position.expiration for position in puts}) == 1:
                signs = [1 if position.signed_qty > 0 else -1 for position in puts]
                strikes = [position.strike or 0.0 for position in puts]
                unique_strikes = len({round(strike, 6) for strike in strikes})
                if unique_strikes == 3 and signs == [1, -1, -1, 1]:
                    groups.append({"group_id": f"put_debit_butterfly:{underlying}", "underlying": underlying, "strategy_type": "put_debit_butterfly", "positions": puts})
                    continue
                if unique_strikes == 3 and signs == [-1, 1, 1, -1]:
                    groups.append({"group_id": f"put_credit_butterfly:{underlying}", "underlying": underlying, "strategy_type": "put_credit_butterfly", "positions": puts})
                    continue
                if unique_strikes == 4 and signs == [1, -1, -1, 1]:
                    groups.append({"group_id": f"put_debit_condor:{underlying}", "underlying": underlying, "strategy_type": "put_debit_condor", "positions": puts})
                    continue
                if unique_strikes == 4 and signs == [-1, 1, 1, -1]:
                    groups.append({"group_id": f"put_credit_condor:{underlying}", "underlying": underlying, "strategy_type": "put_credit_condor", "positions": puts})
                    continue
            if len(calls) == 2 and len(puts) == 2:
                short_calls = [position for position in calls if position.signed_qty < 0]
                short_puts = [position for position in puts if position.signed_qty < 0]
                if len(short_calls) == 1 and len(short_puts) == 1:
                    if math.isclose(short_calls[0].strike or 0.0, short_puts[0].strike or 1.0, abs_tol=1e-6):
                        groups.append({"group_id": f"iron_butterfly:{underlying}", "underlying": underlying, "strategy_type": "iron_butterfly", "positions": options})
                    else:
                        groups.append({"group_id": f"iron_condor:{underlying}", "underlying": underlying, "strategy_type": "iron_condor", "positions": options})
                    continue

        for position in bucket:
            groups.append(_group_single(position, "untracked"))
    return groups


def _to_probability_legs(group: dict[str, Any]) -> list[StrategyLeg]:
    legs: list[StrategyLeg] = []
    for position in group["positions"]:
        if position.asset_class != "option":
            continue
        quantity = max(int(round(abs(position.signed_qty))), 1)
        for _ in range(quantity):
            legs.append(
                StrategyLeg(
                    side="short" if position.signed_qty < 0 else "long",
                    option_type=position.option_type or "call",
                    strike=float(position.strike or 0.0),
                    premium_mid=position.entry_price,
                    expiration=position.expiration,
                    dte=position.dte,
                    symbol=position.symbol,
                    bid=position.current_price,
                    ask=position.current_price,
                    volume=0.0,
                    open_interest=0.0,
                    implied_volatility=position.greeks.get("iv") or None,
                )
            )
    return legs


def _group_metrics(group: dict[str, Any], spot: float, prevclose: float | None) -> dict[str, Any]:
    positions: list[NormalizedPosition] = group["positions"]
    open_pnl = float(sum(position.current_pnl for position in positions))
    net_delta = float(sum((position.greeks.get("delta", 0.0) * position.signed_qty * 100.0) if position.asset_class == "option" else position.signed_qty for position in positions))
    theta_daily = float(sum(position.greeks.get("theta", 0.0) * position.signed_qty * 100.0 for position in positions if position.asset_class == "option"))
    net_vega = float(sum(position.greeks.get("vega", 0.0) * position.signed_qty * 100.0 for position in positions if position.asset_class == "option"))
    delta_pnl = net_delta * (spot - (prevclose or spot))
    theta_pnl = theta_daily
    vega_pnl = open_pnl - delta_pnl - theta_pnl
    return {
        "open_pnl": round(open_pnl, 4),
        "net_delta": round(net_delta, 4),
        "theta_daily": round(theta_daily, 4),
        "net_vega": round(net_vega, 4),
        "attribution": {
            "delta_pnl_est": round(delta_pnl, 4),
            "theta_pnl_est": round(theta_pnl, 4),
            "vega_pnl_est": round(vega_pnl, 4),
            "dominant_driver": max(
                {
                    "delta": abs(delta_pnl),
                    "theta": abs(theta_pnl),
                    "vega": abs(vega_pnl),
                }.items(),
                key=lambda item: item[1],
            )[0],
        },
    }


def _leg_payoff(position: NormalizedPosition, price: float) -> float:
    if position.asset_class == "equity":
        return (price - position.entry_price) * position.signed_qty
    intrinsic = max(price - (position.strike or 0.0), 0.0) if position.option_type == "call" else max((position.strike or 0.0) - price, 0.0)
    per_contract = (intrinsic - position.entry_price) if position.signed_qty > 0 else (position.entry_price - intrinsic)
    return per_contract * abs(position.signed_qty) * 100.0


def _payoff_curve(group: dict[str, Any], spot: float) -> list[dict[str, float]]:
    if spot <= 0:
        return []
    start = spot * 0.7
    end = spot * 1.3
    points: list[dict[str, float]] = []
    for idx in range(41):
        price = start + ((end - start) * idx / 40.0)
        profit = float(sum(_leg_payoff(position, price) for position in group["positions"]))
        points.append({"price": round(price, 4), "profit": round(profit, 4)})
    return points


def _cached_probability(
    client: TradierClient,
    session: TradierAccountSession,
    group: dict[str, Any],
) -> tuple[float | None, dict[str, Any] | None]:
    strategy_type = group["strategy_type"]
    if strategy_type not in SUPPORTED_STRATEGIES:
        return None, None
    legs = _to_probability_legs(group)
    if not legs:
        return None, None
    key = f"{session.account_id}:{_strategy_fingerprint(strategy_type, group['positions'])}"
    now = time.time()
    cached = _PROBABILITY_CACHE.get(key)
    if cached and now - cached[0] < settings.tradier_probability_refresh_sec:
        return cached[1].get("win_rate_pct"), cached[1]
    result = estimate_probability_from_legs(
        symbol=group["underlying"],
        strategy_type=strategy_type,  # type: ignore[arg-type]
        legs=legs,
        account_scope=session.scope,
        tradier_base_url=client.base_url,
        tradier_token=client.token,
        n_paths=6000,
    ).to_dict()
    result["updated_at"] = datetime.utcnow().isoformat()
    _PROBABILITY_CACHE[key] = (now, result)
    return result.get("win_rate_pct"), result


def build_monitor_summary(*, account_scope: TradierScope | None = None, account_id: str | None = None) -> dict[str, Any]:
    client, session = resolve_account_session(account_scope=account_scope, account_id=account_id)
    positions_raw = client.positions(session.account_id)
    balances = client.balances(session.account_id)
    normalized = _normalize_positions(positions_raw)
    _enrich_option_greeks(client, normalized)
    quote_index = _enrich_equity_quotes(client, normalized)
    groups = _infer_groups(normalized)
    pdt_status = check_pdt_status(client, session)

    strategy_summaries: list[dict[str, Any]] = []
    alerts: list[dict[str, Any]] = []
    total_open_pnl = 0.0

    for group in groups:
        underlying = group["underlying"]
        quote = quote_index.get(underlying, {})
        spot = _safe_float(quote.get("last") or quote.get("close"), 0.0)
        prevclose = _safe_float(quote.get("prevclose"), float("nan"))
        metrics = _group_metrics(group, spot=spot, prevclose=None if prevclose != prevclose else prevclose)
        probability_pct, probability_payload = _cached_probability(client, session, group)
        summary = {
            "group_id": group["group_id"],
            "underlying": underlying,
            "strategy_type": group["strategy_type"],
            "spot": round(spot, 4),
            "positions": [
                {
                    "symbol": position.symbol,
                    "asset_class": position.asset_class,
                    "side": position.side,
                    "signed_qty": position.signed_qty,
                    "entry_price": round(position.entry_price, 4),
                    "current_price": round(position.current_price, 4),
                    "current_pnl": round(position.current_pnl, 4),
                    "strike": position.strike,
                    "option_type": position.option_type,
                    "expiration": position.expiration,
                    "dte": position.dte,
                }
                for position in group["positions"]
            ],
            **metrics,
            "win_rate_pct": probability_pct,
            "win_rate_details": probability_payload,
            "payoff_curve": _payoff_curve(group, spot=spot),
            "probability_updated_at": (probability_payload or {}).get("updated_at"),
        }
        if probability_pct is not None and probability_pct < 50.0:
            alert = {
                "level": "warning",
                "strategy_type": group["strategy_type"],
                "underlying": underlying,
                "message": f"Winning probability below 50% for {underlying} {group['strategy_type']}",
                "win_rate_pct": probability_pct,
            }
            summary["alert"] = alert
            alerts.append(alert)
        strategy_summaries.append(summary)
        total_open_pnl += metrics["open_pnl"]

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "refresh_interval_sec": settings.tradier_probability_refresh_sec,
        "account_session": session.to_dict(),
        "balances": {
            "total_equity": session.total_equity,
            "cash": _safe_float(balances.get("cash") or balances.get("total_cash"), 0.0),
            "margin": _safe_float(balances.get("margin") or balances.get("margin_balance"), 0.0),
            "option_buying_power": _safe_float(balances.get("option_buying_power"), 0.0),
        },
        "pdt_status": pdt_status,
        "totals": {
            "strategies": len(strategy_summaries),
            "positions": len(normalized),
            "open_pnl": round(total_open_pnl, 4),
        },
        "strategies": strategy_summaries,
        "alerts": alerts,
    }
