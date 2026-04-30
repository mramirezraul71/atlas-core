"""
Ejecución live/paper de órdenes opciones vía Tradier **solo sandbox**.

``OptionsLiveExecutor`` no conoce producción: rechaza ejecutores con ``sandbox=False`` o
``allow_production=True``.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Literal

from atlas_options_brain.broker.tradier_executor import TradierOrderExecutor
from atlas_options_brain.broker.tradier_live import LiveOrder, TradierOrderBuilder
from atlas_options_brain.simulator.paper import Position

from .options_client import AtlasOptionsService

LiveMode = Literal["off", "dry_run", "sandbox"]


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


class OptionsLiveExecutor:
    """
    - ``off``: ``send_order`` prohibido.
    - ``dry_run``: sin HTTP; ``order.status='simulated'``.
    - ``sandbox``: delega en ``TradierOrderExecutor`` (debe ser sandbox puro).
    """

    def __init__(
        self,
        *,
        mode: LiveMode = "off",
        tradier_executor: TradierOrderExecutor | None = None,
        order_builder: TradierOrderBuilder | None = None,
    ) -> None:
        self._mode = mode
        self._executor = tradier_executor
        self._builder = order_builder or TradierOrderBuilder()
        self._last_send_result: dict[str, Any] | None = None
        if self._executor is not None:
            if not self._executor.sandbox:
                raise ValueError(
                    "OptionsLiveExecutor solo admite TradierOrderExecutor con sandbox=True"
                )
            if self._executor.allow_production:
                raise ValueError(
                    "OptionsLiveExecutor no admite allow_production=True en el ejecutor Tradier"
                )

    @property
    def mode(self) -> LiveMode:
        return self._mode

    @property
    def last_send_result(self) -> dict[str, Any] | None:
        return self._last_send_result

    def prepare_open_order(
        self,
        position: Position,
        strategy_type: str | None = None,
    ) -> LiveOrder:
        return self._builder.build_open_order(position, strategy_type)

    def prepare_close_order(
        self,
        position: Position,
        strategy_type: str | None = None,
    ) -> LiveOrder:
        return self._builder.build_close_order(position, strategy_type)

    def send_order(self, order: LiveOrder, *, preview: bool = True) -> LiveOrder:
        self._last_send_result = None
        if self._mode == "off":
            raise RuntimeError("OptionsLiveExecutor: send_order no permitido con mode=off")
        if self._mode == "dry_run":
            order.status = "simulated"
            order.last_update = _utc_now()
            order.error = None
            self._last_send_result = {
                "mode": "dry_run",
                "preview": preview,
                "simulated": True,
                "note": "sin HTTP",
            }
            return order
        if self._mode == "sandbox":
            if self._executor is None:
                raise RuntimeError(
                    "OptionsLiveExecutor: mode=sandbox requiere tradier_executor"
                )
            resp = self._executor.place_multileg_order(
                order,
                preview=preview,
                multileg_type="market",
            )
            self._last_send_result = dict(resp)
            order.last_update = _utc_now()
            if preview:
                return order
            if resp.get("ok"):
                order.status = "sent"
                order.error = None
            else:
                order.status = "rejected"
                order.error = _format_tradier_error(resp)
            return order
        raise RuntimeError(f"mode desconocido: {self._mode!r}")


def _format_tradier_error(resp: dict[str, Any]) -> str:
    err = resp.get("error")
    if err:
        return str(err)
    body = resp.get("body")
    if isinstance(body, dict):
        try:
            return json.dumps(body)[:2000]
        except (TypeError, ValueError):
            return str(body)[:2000]
    if body is not None:
        return str(body)[:2000]
    return "tradier_error"


class AtlasOptionsLiveService:
    """Orquesta ``AtlasOptionsService`` + ``OptionsLiveExecutor`` para sandbox."""

    def __init__(
        self,
        options_service: AtlasOptionsService,
        live_executor: OptionsLiveExecutor,
    ) -> None:
        self._options = options_service
        self._live = live_executor

    def _resolve_position(self, position_id: str) -> Position | None:
        for p in self._options.list_open_positions():
            if p.position_id == position_id:
                return p
        for p in self._options.list_closed_positions():
            if p.position_id == position_id:
                return p
        return None

    def send_open_sandbox(self, position_id: str, *, preview: bool = True) -> dict[str, Any]:
        pos = self._resolve_position(position_id)
        if pos is None:
            return {
                "status": "error",
                "error": "position_not_found",
                "position_id": position_id,
            }
        if not pos.is_open:
            return {
                "status": "error",
                "error": "position_not_open",
                "position_id": position_id,
                "message": "La acción open en sandbox requiere una posición paper abierta",
            }
        meta = self._options.client.get_position_meta(position_id)
        st = meta.get("strategy_type") or None
        order = self._live.prepare_open_order(pos, st)
        try:
            self._live.send_order(order, preview=preview)
        except RuntimeError as e:
            return {
                "status": "error",
                "error": "send_failed",
                "position_id": position_id,
                "message": str(e),
            }
        return self._result_dict(position_id, order, preview, action="open")

    def send_close_sandbox(self, position_id: str, *, preview: bool = True) -> dict[str, Any]:
        pos = self._resolve_position(position_id)
        if pos is None:
            return {
                "status": "error",
                "error": "position_not_found",
                "position_id": position_id,
            }
        if not pos.is_open:
            return {
                "status": "error",
                "error": "position_not_open",
                "position_id": position_id,
                "message": "Solo se puede cerrar en sandbox si la posición paper sigue abierta",
            }
        meta = self._options.client.get_position_meta(position_id)
        st = meta.get("strategy_type") or None
        order = self._live.prepare_close_order(pos, st)
        try:
            self._live.send_order(order, preview=preview)
        except RuntimeError as e:
            return {
                "status": "error",
                "error": "send_failed",
                "position_id": position_id,
                "message": str(e),
            }
        return self._result_dict(position_id, order, preview, action="close")

    def _result_dict(
        self,
        position_id: str,
        order: LiveOrder,
        preview: bool,
        *,
        action: str,
    ) -> dict[str, Any]:
        ok = order.status != "rejected"
        out: dict[str, Any] = {
            "status": "ok" if ok else "error",
            "position_id": position_id,
            "order_id": order.order_id,
            "order_status": order.status,
            "preview": preview,
            "action": action,
        }
        if not ok:
            out["error"] = "order_rejected"
        if order.error:
            out["order_error"] = order.error
        lr = self._live.last_send_result
        if lr is not None:
            out["tradier"] = lr
        return out
