"""Strategy tracking and payoff services for Atlas Code-Quant."""
from __future__ import annotations

import asyncio
import hashlib
from dataclasses import dataclass
from datetime import datetime
from typing import Any

from api.schemas import PayoffPoint, QuantPayoffPayload
from backtesting.winning_probability import (
    StrategyType,
    TradierScope,
    _black_scholes_price,
    _capital_at_risk,
    _safe_float,
)
from config.settings import settings
from execution.tradier_controls import TradierAccountSession, check_pdt_status, resolve_account_session
from monitoring.advanced_monitor import (
    NormalizedPosition,
    _cached_probability,
    _enrich_equity_quotes,
    _enrich_option_greeks,
    _group_metrics,
    _infer_groups,
    _leg_payoff,
    _normalize_positions,
    _to_probability_legs,
)


_WIN_RATE_MEMORY: dict[str, tuple[float, str]] = {}


@dataclass
class TrackerSnapshot:
    session: TradierAccountSession
    client: Any
    balances: dict[str, Any]
    groups: list[dict[str, Any]]
    quote_index: dict[str, dict[str, Any]]
    pdt_status: dict[str, Any]
    normalized_positions: list[NormalizedPosition]


def _strategy_id(group: dict[str, Any]) -> str:
    digest = hashlib.sha1()
    digest.update(str(group.get("underlying") or "").encode("utf-8"))
    digest.update(str(group.get("strategy_type") or "").encode("utf-8"))
    for position in sorted(group.get("positions") or [], key=lambda item: (item.symbol, item.expiration or "", item.strike or 0.0, item.signed_qty)):
        digest.update(
            f"{position.symbol}:{position.expiration}:{position.strike}:{position.signed_qty}:{position.entry_price}".encode("utf-8")
        )
    short = digest.hexdigest()[:12]
    return f"{group.get('strategy_type', 'strategy')}:{group.get('underlying', 'UNK')}:{short}"


def _group_expirations(group: dict[str, Any]) -> list[str]:
    values = sorted({str(position.expiration) for position in group.get("positions") or [] if position.expiration})
    return values


def _group_spot(group: dict[str, Any], quote_index: dict[str, dict[str, Any]]) -> tuple[float, float | None]:
    quote = quote_index.get(str(group.get("underlying") or "").upper(), {})
    spot = _safe_float(quote.get("last") or quote.get("close"), 0.0)
    prevclose = _safe_float(quote.get("prevclose"), float("nan"))
    return spot, None if prevclose != prevclose else prevclose


def _account_requirement(balances: dict[str, Any]) -> float:
    for key in (
        "current_requirement",
        "option_requirement",
        "maintenance_requirement",
        "margin_requirement",
        "initial_requirement",
    ):
        value = _safe_float(balances.get(key), float("nan"))
        if value == value and value > 0:
            return value
    return 0.0


def _group_model_bpr(group: dict[str, Any], spot: float) -> float:
    legs = _to_probability_legs(group)
    strategy_type = str(group.get("strategy_type") or "")
    if legs and strategy_type:
        try:
            return float(_capital_at_risk(strategy_type, legs, spot) * 100.0)  # type: ignore[arg-type]
        except Exception:
            pass
    return float(
        sum(
            abs(position.current_price * position.signed_qty * (100.0 if position.asset_class == "option" else 1.0))
            for position in group.get("positions") or []
        )
    )


def _group_greeks(group: dict[str, Any]) -> dict[str, float]:
    positions: list[NormalizedPosition] = group.get("positions") or []
    return {
        "delta": round(float(sum((position.greeks.get("delta", 0.0) * position.signed_qty * 100.0) if position.asset_class == "option" else position.signed_qty for position in positions)), 4),
        "gamma": round(float(sum(position.greeks.get("gamma", 0.0) * position.signed_qty * 100.0 for position in positions if position.asset_class == "option")), 4),
        "theta": round(float(sum(position.greeks.get("theta", 0.0) * position.signed_qty * 100.0 for position in positions if position.asset_class == "option")), 4),
        "vega": round(float(sum(position.greeks.get("vega", 0.0) * position.signed_qty * 100.0 for position in positions if position.asset_class == "option")), 4),
    }


def _theoretical_today_leg_pnl(position: NormalizedPosition, underlying_price: float) -> float:
    if position.asset_class == "equity":
        return (underlying_price - position.entry_price) * position.signed_qty
    implied_vol = max(_safe_float(position.greeks.get("iv"), 0.25), 0.05)
    time_years = max(float(position.dte or 0), 0.0) / 365.0
    theoretical = _black_scholes_price(
        spot=max(underlying_price, 1e-6),
        strike=float(position.strike or 0.0),
        time_years=time_years,
        annual_vol=implied_vol,
        option_type=str(position.option_type or "call"),
    )
    if position.signed_qty > 0:
        return (theoretical - position.entry_price) * abs(position.signed_qty) * 100.0
    return (position.entry_price - theoretical) * abs(position.signed_qty) * 100.0


def _payoff_curve_100(group: dict[str, Any], spot: float) -> list[PayoffPoint]:
    positions: list[NormalizedPosition] = group.get("positions") or []
    strikes = [float(position.strike or 0.0) for position in positions if position.strike]
    base = spot if spot > 0 else max(strikes or [1.0])
    start = min(base * 0.6, (min(strikes) * 0.8) if strikes else base * 0.6)
    end = max(base * 1.4, (max(strikes) * 1.2) if strikes else base * 1.4)
    if start <= 0:
        start = max(base * 0.3, 0.01)
    points: list[PayoffPoint] = []
    for idx in range(100):
        price = start + ((end - start) * idx / 99.0)
        expiry_pnl = float(sum(_leg_payoff(position, price) for position in positions))
        theoretical_today = float(sum(_theoretical_today_leg_pnl(position, price) for position in positions))
        points.append(
            PayoffPoint(
                underlying_price=round(price, 4),
                pnl_at_expiration=round(expiry_pnl, 4),
                pnl_theoretical_today=round(theoretical_today, 4),
            )
        )
    return points


def _break_even_points(points: list[PayoffPoint]) -> list[float]:
    values: list[float] = []
    for prev, curr in zip(points[:-1], points[1:]):
        y1 = prev.pnl_at_expiration
        y2 = curr.pnl_at_expiration
        if y1 == 0:
            values.append(prev.underlying_price)
            continue
        if y1 * y2 > 0:
            continue
        dx = curr.underlying_price - prev.underlying_price
        dy = y2 - y1
        if abs(dy) < 1e-9:
            values.append(curr.underlying_price)
            continue
        t = -y1 / dy
        values.append(round(prev.underlying_price + (dx * t), 4))
    return sorted({round(value, 4) for value in values})


class StrategyTracker:
    """Aggregate Tradier positions into strategy-level tracking objects."""

    def snapshot(
        self,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
    ) -> TrackerSnapshot:
        client, session = resolve_account_session(account_scope=account_scope, account_id=account_id)
        positions_raw = client.positions(session.account_id)
        balances = client.balances(session.account_id)
        normalized = _normalize_positions(positions_raw)
        _enrich_option_greeks(client, normalized)
        quote_index = _enrich_equity_quotes(client, normalized)
        groups = _infer_groups(normalized)
        for group in groups:
            strategy_id = _strategy_id(group)
            group["strategy_id"] = strategy_id
            group["group_id"] = strategy_id
            group["expirations"] = _group_expirations(group)
        pdt_status = check_pdt_status(client, session)
        return TrackerSnapshot(
            session=session,
            client=client,
            balances=balances,
            groups=groups,
            quote_index=quote_index,
            pdt_status=pdt_status,
            normalized_positions=normalized,
        )

    def build_summary(
        self,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
    ) -> dict[str, Any]:
        snapshot = self.snapshot(account_scope=account_scope, account_id=account_id)
        account_requirement = _account_requirement(snapshot.balances)
        model_bprs: dict[str, float] = {}
        total_model_bpr = 0.0
        spots: dict[str, float] = {}
        prevcloses: dict[str, float | None] = {}

        for group in snapshot.groups:
            spot, prevclose = _group_spot(group, snapshot.quote_index)
            spots[group["strategy_id"]] = spot
            prevcloses[group["strategy_id"]] = prevclose
            model_bpr = _group_model_bpr(group, spot=spot)
            model_bprs[group["strategy_id"]] = model_bpr
            total_model_bpr += model_bpr

        strategies: list[dict[str, Any]] = []
        alerts: list[dict[str, Any]] = []
        total_open_pnl = 0.0
        now_iso = datetime.utcnow().isoformat()

        for group in snapshot.groups:
            strategy_id = group["strategy_id"]
            spot = spots.get(strategy_id, 0.0)
            prevclose = prevcloses.get(strategy_id)
            metrics = _group_metrics(group, spot=spot, prevclose=prevclose)
            greeks = _group_greeks(group)
            probability_pct, probability_payload = _cached_probability(snapshot.client, snapshot.session, group)
            previous_probability = _WIN_RATE_MEMORY.get(strategy_id)
            probability_change_pct = (
                round(float(probability_pct) - previous_probability[0], 4)
                if probability_pct is not None and previous_probability is not None
                else None
            )
            if probability_pct is not None:
                _WIN_RATE_MEMORY[strategy_id] = (float(probability_pct), now_iso)

            model_bpr = model_bprs.get(strategy_id, 0.0)
            allocated_bpr = (
                round(account_requirement * (model_bpr / total_model_bpr), 4)
                if account_requirement > 0 and total_model_bpr > 0
                else round(model_bpr, 4)
            )
            bpr_source = "tradier_requirement_allocated" if account_requirement > 0 and total_model_bpr > 0 else "risk_model"

            summary = {
                "strategy_id": strategy_id,
                "group_id": strategy_id,
                "underlying": group["underlying"],
                "strategy_type": group["strategy_type"],
                "expirations": group.get("expirations") or [],
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
                "net_delta": greeks["delta"],
                "net_gamma": greeks["gamma"],
                "theta_daily": greeks["theta"],
                "net_vega": greeks["vega"],
                "bpr": allocated_bpr,
                "bpr_model": round(model_bpr, 4),
                "bpr_source": bpr_source,
                "account_requirement_total": round(account_requirement, 4),
                "win_rate_pct": probability_pct,
                "win_rate_change_pct": probability_change_pct,
                "win_rate_details": probability_payload,
                "payoff_curve": [
                    {
                        "price": point.underlying_price,
                        "profit": point.pnl_at_expiration,
                        "today": point.pnl_theoretical_today,
                    }
                    for point in _payoff_curve_100(group, spot=spot)[::3]
                ],
                "probability_updated_at": (probability_payload or {}).get("updated_at"),
            }
            if probability_pct is not None and probability_pct < 50.0:
                alert = {
                    "level": "warning",
                    "strategy_id": strategy_id,
                    "strategy_type": group["strategy_type"],
                    "underlying": group["underlying"],
                    "message": f"Winning probability below 50% for {group['underlying']} {group['strategy_type']}",
                    "win_rate_pct": probability_pct,
                }
                summary["alert"] = alert
                alerts.append(alert)
            strategies.append(summary)
            total_open_pnl += float(metrics.get("open_pnl") or 0.0)

        return {
            "generated_at": now_iso,
            "refresh_interval_sec": settings.tradier_probability_refresh_sec,
            "account_session": snapshot.session.to_dict(),
            "balances": {
                "total_equity": snapshot.session.total_equity,
                "cash": _safe_float(snapshot.balances.get("cash") or snapshot.balances.get("total_cash"), 0.0),
                "margin": _safe_float(snapshot.balances.get("margin") or snapshot.balances.get("margin_balance"), 0.0),
                "option_buying_power": _safe_float(snapshot.balances.get("option_buying_power"), 0.0),
                "current_requirement": round(account_requirement, 4),
            },
            "pdt_status": snapshot.pdt_status,
            "totals": {
                "strategies": len(strategies),
                "positions": len(snapshot.normalized_positions),
                "open_pnl": round(total_open_pnl, 4),
            },
            "strategies": strategies,
            "alerts": alerts,
        }

    def payoff(
        self,
        strategy_id: str,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
    ) -> QuantPayoffPayload:
        snapshot = self.snapshot(account_scope=account_scope, account_id=account_id)
        group = next((item for item in snapshot.groups if item.get("strategy_id") == strategy_id), None)
        if group is None:
            raise ValueError(f"Strategy '{strategy_id}' not found")
        spot, _ = _group_spot(group, snapshot.quote_index)
        points = _payoff_curve_100(group, spot=spot)
        return QuantPayoffPayload(
            generated_at=datetime.utcnow().isoformat(),
            strategy_id=strategy_id,
            strategy_type=str(group.get("strategy_type") or "unknown"),
            underlying=str(group.get("underlying") or ""),
            spot=round(spot, 4),
            points=points,
            break_even_points=_break_even_points(points),
        )

    async def live_update(
        self,
        *,
        account_scope: TradierScope | None = None,
        account_id: str | None = None,
    ) -> dict[str, Any]:
        summary = await asyncio.to_thread(
            self.build_summary,
            account_scope=account_scope,
            account_id=account_id,
        )
        return {
            "type": "quant.live_update",
            "generated_at": datetime.utcnow().isoformat(),
            "monitor_summary": summary,
        }
