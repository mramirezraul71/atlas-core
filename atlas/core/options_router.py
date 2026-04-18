"""
Routing de intents JSON del brain principal hacia planner/servicio de opciones.

Sin efectos globales: la instancia se construye con ``AtlasOptionsPlannerService``
y ``AtlasOptionsService`` ya configurados.
"""
from __future__ import annotations

from typing import Any

from .options_client import AtlasOptionsRiskError, AtlasOptionsService
from .options_live import AtlasOptionsLiveService
from .options_planner import AtlasOptionsPlannerService, MarketContext, Trend

_VALID_TRENDS: frozenset[str] = frozenset(
    {"bull", "bear", "sideways", "slightly_bull"}
)

_KNOWN_INTENTS: frozenset[str] = frozenset(
    {
        "options_plan_and_open",
        "options_build_and_open",
        "options_mark_all",
        "options_get_position",
        "options_send_sandbox_order",
    }
)


class OptionsIntentRouter:
    """
    Mapea ``payload["intent"]`` + campos a llamadas del stack de opciones.

    Errores de riesgo se devuelven como ``status: "risk_error"`` (no se relanzan).
    """

    def __init__(
        self,
        planner_service: AtlasOptionsPlannerService,
        options_service: AtlasOptionsService,
        live_service: AtlasOptionsLiveService | None = None,
    ) -> None:
        self._planner = planner_service
        self._options = options_service
        self._live = live_service

    def handle_intent(self, payload: dict[str, Any]) -> dict[str, Any]:
        intent = payload.get("intent")
        if intent not in _KNOWN_INTENTS:
            return {
                "status": "error",
                "error": "unknown_intent",
                "intent": intent,
            }

        try:
            if intent == "options_plan_and_open":
                return self._handle_plan_and_open(payload)
            if intent == "options_build_and_open":
                return self._handle_build_and_open(payload)
            if intent == "options_mark_all":
                return self._handle_mark_all()
            if intent == "options_get_position":
                return self._handle_get_position(payload)
            if intent == "options_send_sandbox_order":
                return self._handle_send_sandbox_order(payload)
        except AtlasOptionsRiskError as e:
            return {
                "status": "risk_error",
                "code": e.code,
                "detail": e.detail,
            }

        raise RuntimeError(f"intent interno no manejado: {intent!r}")

    def _handle_plan_and_open(self, payload: dict[str, Any]) -> dict[str, Any]:
        err = self._validate_required(payload, ("symbol", "spot", "trend"))
        if err:
            return err
        trend_raw = payload["trend"]
        if trend_raw not in _VALID_TRENDS:
            return {
                "status": "error",
                "error": "invalid_trend",
                "trend": trend_raw,
                "allowed": sorted(_VALID_TRENDS),
            }
        trend: Trend = trend_raw  # type: ignore[assignment]
        try:
            spot = float(payload["spot"])
        except (TypeError, ValueError):
            return {
                "status": "error",
                "error": "invalid_spot",
                "field": "spot",
            }
        iv_rank: float | None
        if "iv_rank" not in payload or payload["iv_rank"] is None:
            iv_rank = None
        else:
            try:
                iv_rank = float(payload["iv_rank"])
            except (TypeError, ValueError):
                return {
                    "status": "error",
                    "error": "invalid_iv_rank",
                    "field": "iv_rank",
                }
        regime = payload.get("regime")
        if regime is not None and not isinstance(regime, str):
            regime = str(regime)

        ctx = MarketContext(
            symbol=str(payload["symbol"]),
            spot=spot,
            trend=trend,
            iv_rank=iv_rank,
            regime=regime,
        )
        position_id = self._planner.plan_and_open(ctx)
        meta = self._options.client.get_position_meta(position_id)
        return {
            "status": "ok",
            "position_id": position_id,
            "strategy_type": meta.get("strategy_type", ""),
            "symbol": meta.get("symbol", ctx.symbol),
        }

    def _handle_build_and_open(self, payload: dict[str, Any]) -> dict[str, Any]:
        err = self._validate_required(payload, ("symbol", "strategy_type"))
        if err:
            return err
        symbol = str(payload["symbol"])
        strategy_type = str(payload["strategy_type"])
        params = payload.get("params")
        if params is not None and not isinstance(params, dict):
            return {
                "status": "error",
                "error": "invalid_params",
                "message": "params debe ser un dict u omitirse",
            }
        p = dict(params) if params else None
        position_id = self._options.build_and_open(symbol, strategy_type, p)  # type: ignore[arg-type]
        meta = self._options.client.get_position_meta(position_id)
        return {
            "status": "ok",
            "position_id": position_id,
            "strategy_type": meta.get("strategy_type", strategy_type),
            "symbol": meta.get("symbol", symbol),
        }

    def _handle_mark_all(self) -> dict[str, Any]:
        positions = self._options.mark_all()
        return {"status": "ok", "positions": positions}

    def _handle_get_position(self, payload: dict[str, Any]) -> dict[str, Any]:
        err = self._validate_required(payload, ("position_id",))
        if err:
            return err
        position_id = str(payload["position_id"])
        try:
            position = self._options.get_position(position_id)
        except KeyError:
            return {
                "status": "error",
                "error": "position_not_found",
                "position_id": position_id,
            }
        return {"status": "ok", "position": position}

    def _handle_send_sandbox_order(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._live is None:
            return {
                "status": "error",
                "error": "live_service_not_configured",
                "message": "AtlasOptionsLiveService no inyectado en OptionsIntentRouter",
            }
        err = self._validate_required(payload, ("position_id", "action"))
        if err:
            return err
        position_id = str(payload["position_id"])
        action = str(payload["action"]).lower()
        if action not in ("open", "close"):
            return {
                "status": "error",
                "error": "invalid_action",
                "action": action,
                "allowed": ["open", "close"],
            }
        preview = payload.get("preview", True)
        if not isinstance(preview, bool):
            preview = bool(preview)
        if action == "open":
            return self._live.send_open_sandbox(position_id, preview=preview)
        return self._live.send_close_sandbox(position_id, preview=preview)

    @staticmethod
    def _validate_required(
        payload: dict[str, Any],
        fields: tuple[str, ...],
    ) -> dict[str, Any] | None:
        for f in fields:
            if f not in payload or payload[f] is None:
                return {
                    "status": "error",
                    "error": "missing_field",
                    "field": f,
                }
        return None
