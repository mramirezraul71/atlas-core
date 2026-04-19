"""AutoCloseEngine (Fase B inicial): decisiones de cierre para opciones en paper/sandbox.

Este módulo NO envía órdenes. Solo evalúa si una posición debería cerrarse y por qué.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import date, datetime
from typing import Any


@dataclass(frozen=True)
class AutoClosePolicy:
    """Política mínima configurable para reglas de salida."""

    profit_take_pct_credit: float = 0.50
    stop_loss_multiple_credit: float = 2.0
    min_remaining_dte: int = 21
    enable_0dte_breach_hook: bool = True


class AutoCloseEngine:
    """Motor de decisión de cierre (sin ejecución).

    Reglas implementadas:
    - Crédito: TP cuando PnL >= ``profit_take_pct_credit * entry_credit``.
    - Crédito: SL cuando pérdida >= ``stop_loss_multiple_credit * entry_credit``.
    - DTE gate: no-0DTE con ``remaining_dte <= min_remaining_dte``.
    - Hook 0DTE: cierre defensivo por ruptura crítica vía contexto externo
      (típicamente ``breach_context`` emitido por ``signals.VisualSignalAdapter``).
    """

    def __init__(
        self,
        *,
        profit_take_pct_credit: float = 0.50,
        stop_loss_multiple_credit: float = 2.0,
        min_remaining_dte: int = 21,
        enable_0dte_breach_hook: bool = True,
    ) -> None:
        self.policy = AutoClosePolicy(
            profit_take_pct_credit=float(profit_take_pct_credit),
            stop_loss_multiple_credit=float(stop_loss_multiple_credit),
            min_remaining_dte=int(min_remaining_dte),
            enable_0dte_breach_hook=bool(enable_0dte_breach_hook),
        )

    def evaluate_position(
        self,
        position: Any,
        *,
        intraday_context: dict[str, Any] | None = None,
        breach_context: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Evalúa una posición abierta y devuelve una decisión estructurada.

        ``position`` puede ser un ``dict`` normalizado o un objeto real del portafolio
        (por ejemplo ``PortfolioEntry``) con campos equivalentes.
        """
        normalized = self._normalize_position(position)
        entry_credit = normalized["entry_credit"]
        current_value = normalized["current_value"]
        unrealized_pnl = normalized["unrealized_pnl"]
        remaining_dte = normalized["remaining_dte"]
        is_0dte = normalized["is_0dte"]
        is_credit = normalized["is_credit"]

        reasons: list[str] = []
        metrics: dict[str, Any] = {
            "entry_credit": entry_credit,
            "current_value": current_value,
            "unrealized_pnl": unrealized_pnl,
            "remaining_dte": remaining_dte,
            "profit_take_threshold": None,
            "stop_loss_threshold": None,
        }

        # TP/SL para estructuras de crédito.
        if is_credit and entry_credit is not None and entry_credit > 0 and unrealized_pnl is not None:
            tp = entry_credit * self.policy.profit_take_pct_credit
            sl = entry_credit * self.policy.stop_loss_multiple_credit
            metrics["profit_take_threshold"] = round(tp, 6)
            metrics["stop_loss_threshold"] = round(sl, 6)
            if unrealized_pnl >= tp:
                reasons.append("take_profit")
            if (-unrealized_pnl) >= sl:
                reasons.append("stop_loss")

        # DTE gate estándar (aplica a no-0DTE).
        if (not is_0dte) and remaining_dte is not None and remaining_dte <= self.policy.min_remaining_dte:
            reasons.append("dte_gate")

        # Hook explícito 0DTE defensivo.
        if is_0dte and self.policy.enable_0dte_breach_hook:
            if self._critical_breach_detected(intraday_context=intraday_context, breach_context=breach_context):
                reasons.append("0dte_critical_breach")

        should_close = len(reasons) > 0
        high_priority_reasons = {"stop_loss", "0dte_critical_breach"}
        priority = "high" if any(reason in high_priority_reasons for reason in reasons) else "normal"
        recommended_action = "close" if should_close else "hold"

        return {
            "position_id": normalized["position_id"],
            "should_close": should_close,
            "reasons": reasons,
            "priority": priority,
            "recommended_action": recommended_action,
            "strategy_type": normalized["strategy_type"],
            "is_0dte": is_0dte,
            "metrics": metrics,
        }

    def scan_positions(
        self,
        positions: list[Any],
        *,
        intraday_context: dict[str, Any] | None = None,
        breach_context: dict[str, Any] | None = None,
    ) -> list[dict[str, Any]]:
        """Evalúa múltiples posiciones sin romper el flujo por una entrada defectuosa."""
        decisions: list[dict[str, Any]] = []
        for idx, position in enumerate(positions):
            try:
                decision = self.evaluate_position(
                    position,
                    intraday_context=intraday_context,
                    breach_context=breach_context,
                )
            except Exception as exc:  # pragma: no cover - camino defensivo
                pid = self._position_identifier(position) or f"scan_idx_{idx}"
                decision = {
                    "position_id": pid,
                    "should_close": False,
                    "reasons": ["evaluation_error"],
                    "priority": "normal",
                    "recommended_action": "hold",
                    "strategy_type": "unknown",
                    "is_0dte": False,
                    "metrics": {"error": str(exc)},
                }
            decisions.append(decision)
        return decisions

    def _normalize_position(self, position: Any) -> dict[str, Any]:
        strategy = self._obj_get(position, "strategy", None)
        metadata = self._extract_metadata(position, strategy=strategy)

        position_id = (
            self._obj_get(position, "position_id", None)
            or self._obj_get(position, "id", None)
            or self._obj_get(position, "name", None)
            or metadata.get("sync_key")
            or metadata.get("strategy_id")
            or "unknown_position"
        )
        strategy_type = (
            self._obj_get(position, "strategy_type", None)
            or metadata.get("strategy_type")
            or self._obj_get(strategy, "name", None)
            or "unknown_strategy"
        )

        # Entry credit: preferir explícito; fallback a Strategy.net_premium.
        entry_credit_raw = (
            self._obj_get(position, "entry_credit", None)
            or self._obj_get(position, "credit_received", None)
            or self._obj_get(position, "net_credit", None)
        )
        if entry_credit_raw is None and strategy is not None:
            net_premium = self._obj_get(strategy, "net_premium", None)
            if isinstance(net_premium, (int, float)) and float(net_premium) > 0:
                entry_credit_raw = float(net_premium)
        entry_credit = self._to_float_or_none(entry_credit_raw)

        current_value = self._to_float_or_none(
            self._obj_get(position, "current_value", None)
            or self._obj_get(position, "mark_value", None)
            or self._obj_get(position, "close_cost", None)
        )
        unrealized_pnl = self._to_float_or_none(
            self._obj_get(position, "unrealized_pnl", None)
            or self._obj_get(position, "current_pnl", None)
            or self._obj_get(position, "pnl", None)
        )
        if unrealized_pnl is None and entry_credit is not None and current_value is not None:
            # Convención crédito: PnL = crédito inicial - costo de cierre.
            unrealized_pnl = entry_credit - current_value

        remaining_dte = self._to_int_or_none(
            self._obj_get(position, "remaining_dte", None) or self._obj_get(position, "dte", None)
        )
        if remaining_dte is None:
            remaining_dte = self._infer_remaining_dte_from_legs(position, strategy=strategy)

        dte_mode = str(self._obj_get(position, "dte_mode", "") or metadata.get("dte_mode") or "").strip().lower()
        is_0dte = bool(
            self._obj_get(position, "is_0dte", False)
            or dte_mode == "0dte"
            or (remaining_dte is not None and remaining_dte <= 0)
        )

        is_credit_raw = self._obj_get(position, "is_credit", None)
        if is_credit_raw is None and strategy is not None:
            is_credit_raw = self._obj_get(strategy, "is_credit", None)
        is_credit = bool(is_credit_raw) if is_credit_raw is not None else bool(entry_credit and entry_credit > 0)

        return {
            "position_id": str(position_id),
            "strategy_type": str(strategy_type),
            "entry_credit": entry_credit,
            "current_value": current_value,
            "unrealized_pnl": unrealized_pnl,
            "remaining_dte": remaining_dte,
            "is_0dte": is_0dte,
            "is_credit": is_credit,
        }

    def _infer_remaining_dte_from_legs(self, position: Any, *, strategy: Any) -> int | None:
        legs = self._obj_get(position, "legs", None)
        if legs is None and strategy is not None:
            legs = self._obj_get(strategy, "legs", None) or self._obj_get(strategy, "option_legs", None)
        if not legs:
            return None
        today = date.today()
        dtes: list[int] = []
        for leg in legs:
            expiry = self._obj_get(leg, "expiry", None) or self._obj_get(leg, "expiration", None)
            dte = self._parse_dte(expiry, today=today)
            if dte is not None:
                dtes.append(dte)
        return min(dtes) if dtes else None

    def _critical_breach_detected(
        self,
        *,
        intraday_context: dict[str, Any] | None,
        breach_context: dict[str, Any] | None,
    ) -> bool:
        ctxs = [intraday_context or {}, breach_context or {}]
        trigger_keys = {
            "critical_breach",
            "short_strike_breached",
            "breach_detected",
            "level_broken",
            "risk_break",
        }
        for ctx in ctxs:
            for key in trigger_keys:
                if bool(ctx.get(key)):
                    return True
        return False

    def _position_identifier(self, position: Any) -> str | None:
        return (
            self._obj_get(position, "position_id", None)
            or self._obj_get(position, "id", None)
            or self._obj_get(position, "name", None)
        )

    @staticmethod
    def _extract_metadata(position: Any, *, strategy: Any) -> dict[str, Any]:
        md = AutoCloseEngine._obj_get(position, "metadata", None)
        if isinstance(md, dict):
            return md
        md = AutoCloseEngine._obj_get(strategy, "metadata", None)
        return md if isinstance(md, dict) else {}

    @staticmethod
    def _obj_get(obj: Any, key: str, default: Any = None) -> Any:
        if isinstance(obj, dict):
            return obj.get(key, default)
        return getattr(obj, key, default)

    @staticmethod
    def _to_float_or_none(value: Any) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _to_int_or_none(value: Any) -> int | None:
        if value is None:
            return None
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _parse_dte(expiry: Any, *, today: date) -> int | None:
        if expiry is None:
            return None
        if isinstance(expiry, date):
            return (expiry - today).days
        value = str(expiry).strip()
        if not value:
            return None
        if "T" in value:
            value = value.split("T", 1)[0]
        try:
            exp_date = datetime.strptime(value, "%Y-%m-%d").date()
        except ValueError:
            return None
        return (exp_date - today).days
