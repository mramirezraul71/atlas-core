"""Tradier order routing for Atlas Code-Quant."""
from __future__ import annotations

from typing import Any, Literal

from api.schemas import OrderRequest, TradierOrderLeg
from config.settings import settings
from execution.tradier_controls import check_pdt_status, resolve_account_session
from execution.tradier_pdt_ledger import record_live_order_intent
from backtesting.winning_probability import _safe_float


TradierOrderClass = Literal["equity", "option", "multileg", "combo"]

_OPENING_SIDES = {"buy", "buy_to_open", "sell_to_open", "sell_short"}
_CLOSING_SIDES = {"sell", "buy_to_close", "sell_to_close", "buy_to_cover"}
_EXPLICIT_OPTION_SIDES = {"buy_to_open", "sell_to_open", "buy_to_close", "sell_to_close"}
_EXPLICIT_EQUITY_SIDES = {"buy", "sell", "sell_short", "buy_to_cover"}


class TradierOrderBlocked(Exception):
    def __init__(self, message: str, *, payload: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.payload = payload or {}


def _extract_preview_day_trades(preview_response: dict[str, Any]) -> int | None:
    value = _safe_float((preview_response or {}).get("day_trades"), float("nan"))
    if value == value and value >= 0:
        return int(value)
    return None


def should_route_to_tradier(body: OrderRequest) -> bool:
    return any(
        (
            body.account_scope is not None,
            body.account_id is not None,
            body.option_symbol is not None,
            bool(body.legs),
            body.asset_class != "auto",
            body.tradier_class is not None,
            body.preview is False,
            body.extended_hours,
            body.tag is not None,
            body.stop_price is not None,
            body.order_type not in {"market", "limit"},
            body.side not in {"buy", "sell"},
        )
    )


def is_opening_order(body: OrderRequest) -> bool:
    if body.position_effect == "open":
        return True
    if body.position_effect == "close":
        return False
    return body.side.lower() in _OPENING_SIDES


def _stringify_number(value: float) -> str:
    numeric = float(value)
    if numeric.is_integer():
        return str(int(numeric))
    return f"{numeric:.8f}".rstrip("0").rstrip(".")


def _effective_position_effect(body: OrderRequest) -> Literal["open", "close"]:
    return "open" if is_opening_order(body) else "close"


def _normalize_option_side(side: str, *, position_effect: Literal["open", "close"]) -> str:
    normalized = side.lower()
    if normalized in _EXPLICIT_OPTION_SIDES:
        return normalized
    if normalized == "buy":
        return "buy_to_open" if position_effect == "open" else "buy_to_close"
    if normalized == "sell":
        return "sell_to_open" if position_effect == "open" else "sell_to_close"
    raise ValueError(f"Unsupported option side '{side}'")


def _normalize_equity_side(side: str) -> str:
    normalized = side.lower()
    if normalized not in _EXPLICIT_EQUITY_SIDES:
        raise ValueError(f"Unsupported equity side '{side}'")
    return normalized


def _resolve_order_class(body: OrderRequest) -> TradierOrderClass:
    if body.tradier_class is not None:
        return body.tradier_class
    if body.asset_class != "auto":
        return body.asset_class
    if body.legs:
        return "combo" if any(leg.instrument_type == "equity" for leg in body.legs) else "multileg"
    if body.option_symbol:
        return "option"
    return "equity"


def _validate_order_shape(body: OrderRequest, order_class: TradierOrderClass) -> None:
    if order_class == "equity":
        if body.option_symbol or body.legs:
            raise ValueError("Equity orders cannot include option_symbol or legs")
    elif order_class == "option":
        if not body.option_symbol:
            raise ValueError("Option orders require option_symbol")
        if body.legs:
            raise ValueError("Single option orders cannot include legs")
    elif order_class == "multileg":
        if len(body.legs) < 2:
            raise ValueError("Multileg orders require at least 2 legs")
        if any(leg.instrument_type == "equity" for leg in body.legs):
            raise ValueError("Multileg orders only support option legs")
    elif order_class == "combo":
        if len(body.legs) < 2:
            raise ValueError("Combo orders require at least 2 legs")
        if not any(leg.instrument_type == "equity" for leg in body.legs):
            raise ValueError("Combo orders require at least one equity leg")

    if body.order_type in {"limit", "debit", "credit", "even", "stop_limit"} and body.price is None:
        raise ValueError(f"Order type '{body.order_type}' requires price")
    if body.order_type in {"stop", "stop_limit"} and body.stop_price is None:
        raise ValueError(f"Order type '{body.order_type}' requires stop_price")


def _serialize_leg(
    leg: TradierOrderLeg,
    *,
    index: int,
    position_effect: Literal["open", "close"],
) -> dict[str, str]:
    payload: dict[str, str] = {f"quantity[{index}]": _stringify_number(leg.quantity)}
    if leg.instrument_type == "equity":
        payload[f"side[{index}]"] = _normalize_equity_side(leg.side)
        payload[f"option_symbol[{index}]"] = "null"
        return payload
    if not leg.option_symbol:
        raise ValueError("Option legs require option_symbol")
    payload[f"side[{index}]"] = _normalize_option_side(leg.side, position_effect=position_effect)
    payload[f"option_symbol[{index}]"] = leg.option_symbol
    return payload


def build_tradier_order_payload(body: OrderRequest) -> tuple[TradierOrderClass, Literal["open", "close"], dict[str, str]]:
    order_class = _resolve_order_class(body)
    position_effect = _effective_position_effect(body)
    _validate_order_shape(body, order_class)

    payload: dict[str, str] = {
        "class": order_class,
        "symbol": body.symbol,
        "type": body.order_type,
        "duration": body.duration,
        "preview": str(body.preview).lower(),
    }
    if body.tag:
        payload["tag"] = body.tag
    if body.extended_hours:
        payload["extended_hours"] = "true"
    if body.price is not None and body.order_type in {"limit", "debit", "credit", "even", "stop_limit"}:
        payload["price"] = _stringify_number(body.price)
    if body.stop_price is not None and body.order_type in {"stop", "stop_limit"}:
        payload["stop"] = _stringify_number(body.stop_price)

    if order_class == "equity":
        payload["side"] = _normalize_equity_side(body.side)
        payload["quantity"] = _stringify_number(body.size)
    elif order_class == "option":
        payload["side"] = _normalize_option_side(body.side, position_effect=position_effect)
        payload["quantity"] = _stringify_number(body.size)
        payload["option_symbol"] = str(body.option_symbol)
    else:
        for index, leg in enumerate(body.legs):
            payload.update(_serialize_leg(leg, index=index, position_effect=position_effect))

    return order_class, position_effect, payload


def route_order_to_tradier(body: OrderRequest) -> dict[str, Any]:
    effective_scope = body.account_scope or settings.tradier_default_scope
    client, session = resolve_account_session(
        account_scope=effective_scope,  # type: ignore[arg-type]
        account_id=body.account_id,
    )
    order_class, position_effect, payload = build_tradier_order_payload(body)

    pdt_status = None
    preview_probe = None
    if position_effect == "open" and session.classification == "live":
        pdt_status = check_pdt_status(client, session)
        if pdt_status.get("blocked_opening"):
            reason = str(pdt_status.get("reason") or "PDT guard rail blocked opening order")
            raise TradierOrderBlocked(
                reason,
                payload={
                    "account_session": session.to_dict(),
                    "pdt_status": pdt_status,
                },
            )
        if not body.preview and session.total_equity is not None and session.total_equity < settings.tradier_pdt_min_equity:
            preview_probe = client.place_order(session.account_id, {**payload, "preview": "true"})
            broker_day_trades = _extract_preview_day_trades(preview_probe)
            if broker_day_trades is not None:
                pdt_status = {
                    **(pdt_status or {}),
                    "broker_preview_day_trades": broker_day_trades,
                }
                if broker_day_trades >= settings.tradier_pdt_max_day_trades:
                    raise TradierOrderBlocked(
                        (
                            f"Tradier preview reports {broker_day_trades} day trades "
                            f"for a live account below {settings.tradier_pdt_min_equity:.0f}"
                        ),
                        payload={
                            "account_session": session.to_dict(),
                            "pdt_status": pdt_status,
                            "tradier_preview": preview_probe,
                        },
                    )

    response = client.place_order(session.account_id, payload)
    ledger_entry = record_live_order_intent(
        account_id=session.account_id,
        scope=session.scope,
        body=body,
        order_class=order_class,
        position_effect=position_effect,
        broker_response=response,
    )
    return {
        "provider": "tradier",
        "route": "tradier",
        "mode": "preview" if body.preview else "submit",
        "preview": body.preview,
        "order_class": order_class,
        "position_effect": position_effect,
        "account_session": session.to_dict(),
        "pdt_status": pdt_status,
        "request_payload": payload,
        "tradier_preview": preview_probe,
        "pdt_ledger_entry": ledger_entry,
        "tradier_response": response,
    }
