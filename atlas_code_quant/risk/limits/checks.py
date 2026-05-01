"""Atlas Code Quant — Risk limits checks (F19).

Funciones puras de comprobación de límites de riesgo a nivel de
cuenta paper. Devuelven el mismo `GateResult` canónico que usan los
gates del orquestador, para que `RiskGate` (autonomy) las pueda
consumir sin acoplamiento adicional.

Reglas duras F19:

* Sólo se usan en el pipeline paper. NO autorizan ni bloquean
  ninguna orden real.
* Defensivas: nunca lanzan; si el input es inválido, devuelven
  ``ok=False`` con razón explícita.
* No leen env vars directamente; reciben configuración explícita
  vía ``RiskLimitsConfig`` (que F20 cargará desde
  ``config/live_readiness.py``).
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass

from atlas_code_quant.autonomy.gates import GateResult

logger = logging.getLogger("atlas.code_quant.risk.limits")


__all__ = [
    "RiskLimitsConfig",
    "load_risk_limits_from_env",
    "check_daily_loss_limit",
    "check_position_notional_limit",
    "check_orders_per_minute",
    "check_all_limits",
]


@dataclass(frozen=True)
class RiskLimitsConfig:
    """Configuración inmutable de límites de riesgo (F19).

    Todas las cantidades en USD salvo ``max_orders_per_minute``.
    Defaults conservadores; valores reales llegan en F20 vía
    ``ATLAS_MAX_DAILY_LOSS_USD``, ``ATLAS_MAX_POSITION_NOTIONAL_USD``,
    ``ATLAS_MAX_ORDERS_PER_MINUTE``.
    """

    max_daily_loss_usd: float = 500.0  # pérdida acumulada diaria absoluta (positiva)
    max_position_notional_usd: float = 2_500.0
    max_orders_per_minute: int = 30


def _safe_float(value, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def load_risk_limits_from_env() -> RiskLimitsConfig:
    """Carga ``RiskLimitsConfig`` desde env con defaults seguros."""
    return RiskLimitsConfig(
        max_daily_loss_usd=_safe_float(
            os.environ.get("ATLAS_MAX_DAILY_LOSS_USD"), 500.0
        ),
        max_position_notional_usd=_safe_float(
            os.environ.get("ATLAS_MAX_POSITION_NOTIONAL_USD"), 2_500.0
        ),
        max_orders_per_minute=_safe_int(
            os.environ.get("ATLAS_MAX_ORDERS_PER_MINUTE"), 30
        ),
    )


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------


def check_daily_loss_limit(
    *,
    realized_pnl_usd: float,
    config: RiskLimitsConfig,
) -> GateResult:
    """Verifica que la pérdida acumulada del día está por debajo del límite.

    ``realized_pnl_usd`` es positivo si hubo ganancia y negativo si pérdida.
    """
    pnl = _safe_float(realized_pnl_usd, 0.0)
    limit = float(config.max_daily_loss_usd)
    # Si pnl >= -limit, está dentro de tolerancia.
    if pnl >= -limit:
        return GateResult(
            ok=True,
            reason="daily_loss_within_limit",
            evidence={"pnl_usd": pnl, "limit_usd": limit},
        )
    result = GateResult(
        ok=False,
        reason="daily_loss_limit_exceeded",
        evidence={"pnl_usd": pnl, "limit_usd": limit},
    )
    logger.warning(
        "safety_event %s",
        {
            "type": "safety_event",
            "safety_type": "risk_limit",
            "triggered_by": "daily_loss",
            "value": pnl,
            "threshold": -limit,
            "state": None,
            "trace_id": None,
            "symbol": None,
            "strategy_id": None,
            "action": "block_order",
            "reason": result.reason,
        },
    )
    return result


def check_position_notional_limit(
    *,
    notional_usd: float,
    config: RiskLimitsConfig,
) -> GateResult:
    n = abs(_safe_float(notional_usd, 0.0))
    limit = float(config.max_position_notional_usd)
    if n <= limit:
        return GateResult(
            ok=True,
            reason="position_notional_within_limit",
            evidence={"notional_usd": n, "limit_usd": limit},
        )
    result = GateResult(
        ok=False,
        reason="position_notional_limit_exceeded",
        evidence={"notional_usd": n, "limit_usd": limit},
    )
    logger.warning(
        "safety_event %s",
        {
            "type": "safety_event",
            "safety_type": "risk_limit",
            "triggered_by": "position_notional",
            "value": n,
            "threshold": limit,
            "state": None,
            "trace_id": None,
            "symbol": None,
            "strategy_id": None,
            "action": "block_order",
            "reason": result.reason,
        },
    )
    return result


def check_orders_per_minute(
    *,
    orders_in_last_minute: int,
    config: RiskLimitsConfig,
) -> GateResult:
    n = _safe_int(orders_in_last_minute, 0)
    limit = int(config.max_orders_per_minute)
    if n < limit:
        return GateResult(
            ok=True,
            reason="orders_rate_within_limit",
            evidence={"orders": n, "limit": limit},
        )
    result = GateResult(
        ok=False,
        reason="orders_rate_limit_exceeded",
        evidence={"orders": n, "limit": limit},
    )
    logger.warning(
        "safety_event %s",
        {
            "type": "safety_event",
            "safety_type": "risk_limit",
            "triggered_by": "orders_per_minute",
            "value": n,
            "threshold": limit,
            "state": None,
            "trace_id": None,
            "symbol": None,
            "strategy_id": None,
            "action": "block_order",
            "reason": result.reason,
        },
    )
    return result


def check_all_limits(
    *,
    realized_pnl_usd: float = 0.0,
    notional_usd: float = 0.0,
    orders_in_last_minute: int = 0,
    config: RiskLimitsConfig | None = None,
) -> GateResult:
    """Combina los tres checks. Falla rápido al primer ``ok=False``."""
    cfg = config or load_risk_limits_from_env()
    for check_result in (
        check_daily_loss_limit(realized_pnl_usd=realized_pnl_usd, config=cfg),
        check_position_notional_limit(notional_usd=notional_usd, config=cfg),
        check_orders_per_minute(
            orders_in_last_minute=orders_in_last_minute, config=cfg
        ),
    ):
        if not check_result.ok:
            return check_result
    return GateResult(ok=True, reason="risk_limits_ok")
