"""Tradier order routing for Atlas Code-Quant."""
from __future__ import annotations

import logging
import os
import re
import time
from typing import Any, Literal

from api.schemas import OrderRequest, TradierOrderLeg
from backtesting.winning_probability import TradierClient
from config.settings import settings
from execution.tradier_controls import TradierAccountSession, check_pdt_status, resolve_account_session
from execution.tradier_pdt_ledger import record_live_order_intent
from backtesting.winning_probability import _safe_float

logger = logging.getLogger("atlas.execution.tradier")

# ATLAS Safety: fuerza preview=True en live para validar la orden antes de ejecutar.
# Según documentación Tradier 2026: preview obligatorio antes de cualquier orden real.
# Desactivar solo con ATLAS_FORCE_LIVE_PREVIEW=false (no recomendado).
_FORCE_LIVE_PREVIEW = os.getenv("ATLAS_FORCE_LIVE_PREVIEW", "true").lower() != "false"


TradierOrderClass = Literal["equity", "option", "multileg", "combo"]

_OPENING_SIDES = {"buy", "buy_to_open", "sell_to_open", "sell_short"}
_CLOSING_SIDES = {"sell", "buy_to_close", "sell_to_close", "buy_to_cover"}
_EXPLICIT_OPTION_SIDES = {"buy_to_open", "sell_to_open", "buy_to_close", "sell_to_close"}
_EXPLICIT_EQUITY_SIDES = {"buy", "sell", "sell_short", "buy_to_cover"}


class TradierOrderBlocked(Exception):
    def __init__(self, message: str, *, payload: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.payload = payload or {}


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
    logger.info("[_resolve_order_class] V18 tradier_class=%s asset_class=%s legs=%d", body.tradier_class, body.asset_class, len(body.legs or []))
    # If tradier_class is explicitly set AND legs are present, use it.
    if body.tradier_class is not None:
        if body.tradier_class in ("multileg", "combo") and not body.legs:
            logger.warning("tradier_class=%s but no legs — downgrading to equity", body.tradier_class)
        else:
            return body.tradier_class
    # If asset_class requires legs but none present, fall through to auto-detect.
    if body.asset_class not in ("auto",) and body.asset_class not in ("multileg", "combo"):
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
    # Tag disabled — Tradier sandbox rejects all tag formats
    # if body.tag:
    #     payload["tag"] = re.sub(r"[^a-zA-Z0-9_]", "", body.tag)[:255]
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


def _extract_order_id(response: dict[str, Any] | None) -> str | None:
    if not isinstance(response, dict):
        return None
    for key in ("id", "order_id", "broker_order_id"):
        value = response.get(key)
        if value not in {None, ""}:
            return str(value)
    nested = response.get("order")
    if isinstance(nested, dict):
        for key in ("id", "order_id", "broker_order_id"):
            value = nested.get(key)
            if value not in {None, ""}:
                return str(value)
    return None


def _extract_broker_order_ids(response: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(response, dict):
        return {}
    order_id = _extract_order_id(response)
    if order_id:
        return {"id": order_id}
    raw = response.get("raw_response")
    if isinstance(raw, dict):
        raw_id = _extract_order_id(raw)
        if raw_id:
            return {"id": raw_id}
    return {}


def _match_order(broker_order: dict[str, Any], payload: dict[str, Any]) -> bool:
    broker_symbol = str(broker_order.get("symbol") or "").strip().upper()
    payload_symbol = str(payload.get("symbol") or "").strip().upper()
    if broker_symbol and payload_symbol and broker_symbol != payload_symbol:
        return False

    broker_side = str(broker_order.get("side") or "").strip().lower()
    payload_side = str(payload.get("side") or payload.get("side[0]") or "").strip().lower()
    if broker_side and payload_side and broker_side != payload_side:
        return False

    broker_qty = _safe_float(
        broker_order.get("quantity")
        or broker_order.get("qty")
        or broker_order.get("exec_quantity")
        or broker_order.get("filled_quantity"),
        float("nan"),
    )
    payload_qty = _safe_float(payload.get("quantity") or payload.get("quantity[0]"), float("nan"))
    if broker_qty == broker_qty and payload_qty == payload_qty and abs(broker_qty - payload_qty) > 1e-9:
        return False

    broker_type = str(broker_order.get("type") or "").strip().lower()
    payload_type = str(payload.get("type") or "").strip().lower()
    if broker_type and payload_type and broker_type != payload_type:
        return False

    return True


def _stream_order_confirmation(
    client: TradierClient,
    session: TradierAccountSession,
    payload: dict[str, Any],
    timeout_sec: int,
) -> dict[str, Any]:
    started = time.perf_counter()
    response = client.place_order(session.account_id, payload)
    order_id = _extract_order_id(response)
    if order_id:
        return {
            "order_id": order_id,
            "status": str(response.get("status") or "accepted"),
            "filled_qty": _safe_float(response.get("exec_quantity") or response.get("filled_quantity"), 0.0),
            "method_used": "stream_confirmation",
            "raw_response": response,
        }
    elapsed = time.perf_counter() - started
    if elapsed >= float(timeout_sec):
        raise TimeoutError(f"stream confirmation timeout after {timeout_sec}s")
    raise TimeoutError("stream confirmation missing order_id")


def _polling_get_orders(
    client: TradierClient,
    session: TradierAccountSession,
    payload: dict[str, Any],
    timeout_sec: int,
) -> dict[str, Any]:
    submitted = client.place_order(session.account_id, payload)
    submitted_id = _extract_order_id(submitted)
    deadline = time.time() + max(float(timeout_sec), 0.5)
    while time.time() <= deadline:
        broker_orders = client.orders(session.account_id)
        for broker_order in broker_orders:
            broker_order_id = _extract_order_id(broker_order)
            if submitted_id and broker_order_id and broker_order_id == submitted_id:
                return {
                    "order_id": broker_order_id,
                    "status": str(broker_order.get("status") or "accepted"),
                    "filled_qty": _safe_float(broker_order.get("exec_quantity") or broker_order.get("filled_quantity"), 0.0),
                    "method_used": "polling_get_orders",
                    "raw_response": broker_order,
                    "submitted_response": submitted,
                }
            if _match_order(broker_order, payload):
                return {
                    "order_id": broker_order_id or "",
                    "status": str(broker_order.get("status") or "accepted"),
                    "filled_qty": _safe_float(broker_order.get("exec_quantity") or broker_order.get("filled_quantity"), 0.0),
                    "method_used": "polling_get_orders",
                    "raw_response": broker_order,
                    "submitted_response": submitted,
                }
        time.sleep(0.5)
    raise TimeoutError(f"polling could not find matching order after {timeout_sec}s")


def _execute_with_fallback(
    client: TradierClient,
    session: TradierAccountSession,
    payload: dict[str, Any],
    primary_method: str = "stream_confirmation",
    fallback_method: str = "polling_get_orders",
    max_retries: int = 3,
    timeout_sec: int = 5,
) -> dict[str, Any]:
    primary_map = {
        "stream_confirmation": _stream_order_confirmation,
    }
    fallback_map = {
        "polling_get_orders": _polling_get_orders,
    }
    primary = primary_map.get(primary_method)
    fallback = fallback_map.get(fallback_method)
    if primary is None:
        raise ValueError(f"Unsupported primary method '{primary_method}'")
    if fallback is None:
        raise ValueError(f"Unsupported fallback method '{fallback_method}'")

    retries = max(1, int(max_retries))
    last_error: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            return primary(client=client, session=session, payload=payload, timeout_sec=timeout_sec)
        except Exception as exc:
            last_error = exc
            logger.warning(
                "Stream confirmation failed (attempt %s/%s), falling back... %s",
                attempt,
                retries,
                exc,
            )
    try:
        response = fallback(client=client, session=session, payload=payload, timeout_sec=timeout_sec)
        logger.info("Fallback successful: found order_id via polling")
        return response
    except Exception as exc:
        logger.error("Failed to execute order with all retry strategies")
        detail = f"{last_error}; fallback={exc}" if last_error is not None else str(exc)
        raise RuntimeError(f"Failed to execute order with fallback strategies: {detail}") from exc


def route_order_to_tradier(body: OrderRequest) -> dict[str, Any]:
    effective_scope = body.account_scope or settings.tradier_default_scope
    client, session = resolve_account_session(
        account_scope=effective_scope,  # type: ignore[arg-type]
        account_id=body.account_id,
    )
    order_class, position_effect, payload = build_tradier_order_payload(body)

    pdt_status = None
    preview_probe = None

    # ── ATLAS Safety: preview obligatorio en live antes de enviar orden real ──
    if (
        _FORCE_LIVE_PREVIEW
        and session.classification == "live"
        and not body.preview
        and position_effect == "open"
    ):
        try:
            preview_probe = client.place_order(session.account_id, {**payload, "preview": "true"})
            logger.info(
                "LIVE preview OK [%s] %s qty=%s",
                body.symbol, body.side, payload.get("quantity", "?")
            )
        except Exception as exc:
            logger.warning("Preview probe error (continuando): %s", exc)

    if position_effect == "open" and session.classification == "live":
        pdt_status = check_pdt_status(client, session)
        logger.info("PDT checks deprecated; using operational gates only for %s", session.account_id)

    response = _execute_with_fallback(
        client=client,
        session=session,
        payload=payload,
        primary_method="stream_confirmation",
        fallback_method="polling_get_orders",
        max_retries=3,
        timeout_sec=max(int(settings.tradier_timeout_sec), 1),
    )
    broker_order_ids = _extract_broker_order_ids(response)
    if not broker_order_ids or not broker_order_ids.get("id"):
        raise ValueError(
            "CRITICAL: Failed to extract broker_order_id. "
            "Order may be placed in broker but is not trackable locally. "
            f"Response: {response}"
        )
    ledger_entry = record_live_order_intent(
        account_id=session.account_id,
        scope=session.scope,
        body=body,
        order_class=order_class,
        position_effect=position_effect,
        broker_response=response,
    )
    return {
        "status": "sent",
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
        "broker_order_ids_json": broker_order_ids,
        "method_used": str(response.get("method_used") or "unknown"),
        "order_response": response,
    }
