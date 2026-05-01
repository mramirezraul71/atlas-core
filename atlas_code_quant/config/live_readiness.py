"""Atlas Code Quant — Live readiness env helpers (F20).

Helpers puros para leer las variables de entorno que controlan la
preparación para live trading. **Ningún módulo de F17–F20 activa live
realmente**: estos helpers sólo permiten describir el estado actual y,
en F20, alimentan la checklist de ``autonomy/live_checklist.py``.

Reglas duras (F17–F20):

* No tocar valores por defecto del proceso ni mutar env. Sólo leer.
* No autorizar live: las funciones devuelven bool/dataclasses para que
  un humano decida fuera de Atlas.
* Defensivo: env vars ausentes o corruptas → comportamiento conservador
  (live deshabilitado, dry-run forzado).

Variables soportadas:

* ``ATLAS_ENV`` — ``paper`` | ``live`` (default ``paper``).
* ``ATLAS_LIVETRADINGENABLED`` — bool (default ``false``).
* ``ATLAS_TRADIERDRYRUN`` — bool (default ``true``).
* ``ATLAS_VISION_REQUIRED_FOR_LIVE`` — bool (default ``true``).
* ``ATLAS_KILLSWITCH_FILE`` — path (default ``/tmp/atlas_killswitch``).
* ``ATLAS_MAX_DAILY_LOSS_USD`` — float (default 500).
* ``ATLAS_MAX_POSITION_NOTIONAL_USD`` — float (default 2500).
* ``ATLAS_MAX_ORDERS_PER_MINUTE`` — int (default 30).
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass

logger = logging.getLogger("atlas.code_quant.config.live_readiness")


__all__ = [
    "LiveReadinessEnv",
    "load_live_readiness_env",
    "is_live_mode_requested",
    "is_live_mode_safe_to_arm",
]


_TRUE = {"1", "true", "yes", "on", "y", "t"}
_FALSE = {"0", "false", "no", "off", "n", "f"}


def _bool_env(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    norm = raw.strip().lower()
    if norm in _TRUE:
        return True
    if norm in _FALSE:
        return False
    return default


def _str_env(name: str, default: str) -> str:
    raw = os.environ.get(name)
    if raw is None:
        return default
    s = raw.strip()
    return s or default


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def _int_env(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None:
        return default
    try:
        return int(raw)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class LiveReadinessEnv:
    """Snapshot inmutable de la configuración env relevante para live."""

    atlas_env: str
    live_trading_enabled: bool
    tradier_dry_run: bool
    vision_required_for_live: bool
    killswitch_file: str
    max_daily_loss_usd: float
    max_position_notional_usd: float
    max_orders_per_minute: int


def load_live_readiness_env() -> LiveReadinessEnv:
    """Carga ``LiveReadinessEnv`` desde ``os.environ`` con defaults seguros."""
    return LiveReadinessEnv(
        atlas_env=_str_env("ATLAS_ENV", "paper").lower(),
        live_trading_enabled=_bool_env("ATLAS_LIVETRADINGENABLED", False),
        tradier_dry_run=_bool_env("ATLAS_TRADIERDRYRUN", True),
        vision_required_for_live=_bool_env("ATLAS_VISION_REQUIRED_FOR_LIVE", True),
        killswitch_file=_str_env("ATLAS_KILLSWITCH_FILE", "/tmp/atlas_killswitch"),
        max_daily_loss_usd=_float_env("ATLAS_MAX_DAILY_LOSS_USD", 500.0),
        max_position_notional_usd=_float_env(
            "ATLAS_MAX_POSITION_NOTIONAL_USD", 2_500.0
        ),
        max_orders_per_minute=_int_env("ATLAS_MAX_ORDERS_PER_MINUTE", 30),
    )


def is_live_mode_requested(env: LiveReadinessEnv | None = None) -> bool:
    """¿La configuración expresa intención de operar en live?

    Es **declarativa**: no implica que esté permitido. Devuelve True si
    ``ATLAS_ENV=live`` y ``ATLAS_LIVETRADINGENABLED`` es verdadero.
    """
    e = env or load_live_readiness_env()
    return e.atlas_env == "live" and e.live_trading_enabled


def is_live_mode_safe_to_arm(env: LiveReadinessEnv | None = None) -> bool:
    """¿La configuración cumple condiciones mínimas para *armar* live?

    En F20 esto es sólo informativo: aunque devuelva True, el orquestador
    F17 sigue rechazando transiciones a ``LIVE_ARMED`` /
    ``LIVE_EXECUTING`` (``allow_live=False``).

    Reglas:
        * ``atlas_env == "live"``.
        * ``live_trading_enabled == True``.
        * ``tradier_dry_run == False``.
        * ``vision_required_for_live`` no se evalúa aquí; el checklist
          de ``autonomy/live_checklist.py`` lo cruza con el estado real
          de la cámara.
    """
    e = env or load_live_readiness_env()
    return (
        e.atlas_env == "live"
        and e.live_trading_enabled
        and not e.tradier_dry_run
    )
