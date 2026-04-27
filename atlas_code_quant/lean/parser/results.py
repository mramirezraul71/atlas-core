"""Atlas Code Quant — LEAN results parser (F13).

Parsea los artefactos típicos de una corrida LEAN (``statistics.json``
y ``orders.json``) en una representación interna mínima
(``LeanRunArtifacts``) que el adapter convierte después en
``StrategyFitnessResult``.

Reglas duras:

    * Defensivo: cualquier campo faltante o malformado se traduce a
      valor neutral (0.0 / 0 / None / lista vacía). NUNCA lanza.
    * No depende del filesystem en su núcleo: las funciones reciben
      contenido (dict / str). El loader de ficheros vive en
      ``atlas_code_quant/lean/runner/launcher.py``.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping


__all__ = [
    "LeanRunArtifacts",
    "parse_statistics_payload",
    "parse_orders_payload",
    "parse_run_artifacts",
]


logger = logging.getLogger("atlas.code_quant.lean.parser")


def _to_dict(obj: Any) -> dict[str, Any]:
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return dict(obj)
    if isinstance(obj, str):
        try:
            data = json.loads(obj)
            if isinstance(data, dict):
                return data
        except Exception:  # noqa: BLE001
            return {}
    if isinstance(obj, Mapping):
        return dict(obj)
    return {}


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    if f != f:  # NaN
        return default
    return f


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


@dataclass(frozen=True)
class LeanRunArtifacts:
    """Snapshot canónico interno de una corrida LEAN."""

    sharpe: float
    win_rate: float
    max_drawdown: float
    total_return: float
    expectancy: float
    num_orders: int
    raw_statistics: dict[str, Any] = field(repr=False, compare=False)
    raw_orders: list[dict[str, Any]] = field(repr=False, compare=False)
    parse_warnings: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "sharpe": self.sharpe,
            "win_rate": self.win_rate,
            "max_drawdown": self.max_drawdown,
            "total_return": self.total_return,
            "expectancy": self.expectancy,
            "num_orders": self.num_orders,
            "parse_warnings": list(self.parse_warnings),
        }


# Mapa de claves canónicas LEAN → campo interno. Acepta variaciones
# habituales en los nombres (LEAN cambia entre versiones / runtimes).
_STAT_KEYS_SHARPE = (
    "Sharpe Ratio",
    "SharpeRatio",
    "sharpe",
    "sharpe_ratio",
)
_STAT_KEYS_WINRATE = (
    "Win Rate",
    "WinRate",
    "win_rate",
)
_STAT_KEYS_DRAWDOWN = (
    "Drawdown",
    "Maximum Drawdown",
    "MaxDrawdown",
    "max_drawdown",
)
_STAT_KEYS_RETURN = (
    "Net Profit",
    "NetProfit",
    "total_return",
    "TotalReturn",
)
_STAT_KEYS_EXPECTANCY = (
    "Expectancy",
    "expectancy",
)


def _first_match(d: Mapping[str, Any], keys: Iterable[str]) -> Any:
    for k in keys:
        if k in d:
            return d[k]
    return None


def _parse_pct_or_number(value: Any) -> float:
    """LEAN suele dar números como ``"3.45%"`` o ``"-12.7%"``.

    Convertimos a float en escala 0-100 cuando hay '%', y a número
    bruto en otro caso.
    """
    if value is None:
        return 0.0
    if isinstance(value, (int, float)):
        return _safe_float(value)
    s = str(value).strip()
    if s.endswith("%"):
        return _safe_float(s[:-1])
    return _safe_float(s)


def parse_statistics_payload(payload: Any) -> tuple[dict[str, float], list[str]]:
    """Extrae métricas canónicas del payload de statistics.

    Devuelve ``(metricas, warnings)``. Nunca lanza.
    """
    warnings: list[str] = []
    raw = _to_dict(payload)
    if not raw:
        warnings.append("statistics_payload_empty_or_invalid")

    # LEAN suele anidar bajo "Statistics".
    stats = _to_dict(raw.get("Statistics")) if "Statistics" in raw else raw

    sharpe = _safe_float(_first_match(stats, _STAT_KEYS_SHARPE))
    win_rate = _parse_pct_or_number(_first_match(stats, _STAT_KEYS_WINRATE))
    max_dd = _parse_pct_or_number(_first_match(stats, _STAT_KEYS_DRAWDOWN))
    total_ret = _parse_pct_or_number(_first_match(stats, _STAT_KEYS_RETURN))
    expectancy = _safe_float(_first_match(stats, _STAT_KEYS_EXPECTANCY))

    return (
        {
            "sharpe": sharpe,
            "win_rate": win_rate,
            "max_drawdown": max_dd,
            "total_return": total_ret,
            "expectancy": expectancy,
        },
        warnings,
    )


def parse_orders_payload(payload: Any) -> tuple[list[dict[str, Any]], list[str]]:
    """Normaliza el listado de órdenes a una lista de dicts.

    Acepta:
        * dict con key ``"orders"`` → lista
        * lista directa de dicts
        * string JSON
    """
    warnings: list[str] = []
    raw: Any = payload
    if isinstance(payload, str):
        try:
            raw = json.loads(payload)
        except Exception:  # noqa: BLE001
            warnings.append("orders_payload_invalid_json")
            return [], warnings
    if isinstance(raw, dict):
        candidate = raw.get("orders") or raw.get("Orders") or []
        raw = candidate
    if not isinstance(raw, list):
        warnings.append("orders_payload_not_list")
        return [], warnings
    out: list[dict[str, Any]] = []
    for item in raw:
        if isinstance(item, Mapping):
            out.append(dict(item))
    return out, warnings


def parse_run_artifacts(
    *,
    statistics: Any,
    orders: Any,
) -> LeanRunArtifacts:
    """Pipeline completo: statistics + orders → ``LeanRunArtifacts``."""
    stats, w_stats = parse_statistics_payload(statistics)
    orders_list, w_orders = parse_orders_payload(orders)
    return LeanRunArtifacts(
        sharpe=stats["sharpe"],
        win_rate=stats["win_rate"],
        max_drawdown=stats["max_drawdown"],
        total_return=stats["total_return"],
        expectancy=stats["expectancy"],
        num_orders=len(orders_list),
        raw_statistics=_to_dict(statistics),
        raw_orders=orders_list,
        parse_warnings=tuple(w_stats + w_orders),
    )
