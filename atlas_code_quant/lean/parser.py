"""Parser de artefactos LEAN (F4).

Soporta lectura tolerante a versión de:
- ``statistics.json``: métricas agregadas (sharpe, drawdown, total_return, ...).
- ``orders.json``: lista de órdenes ejecutadas durante el backtest.
- ``equity.csv``: curva de capital ``timestamp,equity``.

Diseño defensivo: ningún parser lanza excepción ante artefactos faltantes o
malformados. Devuelve estructuras vacías o normalizadas auditables.
"""
from __future__ import annotations

import csv
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

logger = logging.getLogger("atlas.lean.parser")


# ── statistics.json ────────────────────────────────────────────────────────
def parse_statistics(path: str | Path) -> dict[str, Any]:
    """Parsea estadísticas LEAN. Devuelve dict vacío si no existe/inválido."""
    p = Path(path)
    if not p.exists():
        return {}
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict):
            return data
        return {}
    except json.JSONDecodeError as e:
        logger.warning("lean_statistics_invalid_json path=%s err=%s", p, e)
        return {}
    except OSError as e:
        logger.warning("lean_statistics_read_error path=%s err=%s", p, e)
        return {}


def fitness_from_statistics(stats: dict[str, Any]) -> dict[str, float]:
    """Extrae métricas estándar para cribado de estrategias.

    Las claves son tolerantes a variantes (LEAN cambia naming entre versiones).
    """
    if not stats:
        return {}
    # las versiones LEAN usan tanto camelCase como Title Case en claves
    def pick(*keys: str, default: float = 0.0) -> float:
        for k in keys:
            v = stats.get(k)
            if v is None:
                continue
            try:
                return float(v)
            except (TypeError, ValueError):
                continue
        return default

    return {
        "sharpe": pick("Sharpe Ratio", "SharpeRatio", "sharpe_ratio", "sharpe"),
        "total_return": pick("Total Return", "TotalReturn", "total_return"),
        "drawdown": pick("Drawdown", "MaxDrawdown", "max_drawdown", "drawdown"),
        "win_rate": pick("Win Rate", "WinRate", "win_rate"),
        "profit_factor": pick("Profit Factor", "ProfitFactor", "profit_factor"),
        "trades": pick("Total Trades", "TotalTrades", "trades"),
    }


# ── orders.json ────────────────────────────────────────────────────────────
@dataclass(slots=True)
class LeanOrder:
    id: str
    symbol: str
    side: str            # buy/sell
    quantity: float
    price: float
    time: str
    status: str = "filled"
    type: str = "market"
    raw: dict[str, Any] = field(default_factory=dict)


def parse_orders(path: str | Path) -> list[LeanOrder]:
    """Parsea órdenes LEAN. Lista vacía si no existe/inválido."""
    p = Path(path)
    if not p.exists():
        return []
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        logger.warning("lean_orders_invalid_json path=%s err=%s", p, e)
        return []
    except OSError as e:
        logger.warning("lean_orders_read_error path=%s err=%s", p, e)
        return []

    items: list[dict[str, Any]] = []
    if isinstance(data, list):
        items = [d for d in data if isinstance(d, dict)]
    elif isinstance(data, dict):
        # algunas versiones envuelven en {"Orders": [...]}
        for k in ("Orders", "orders", "items"):
            if isinstance(data.get(k), list):
                items = [d for d in data[k] if isinstance(d, dict)]
                break
        else:
            items = [data] if data else []

    out: list[LeanOrder] = []
    for d in items:
        out.append(
            LeanOrder(
                id=str(d.get("Id") or d.get("id") or d.get("OrderId") or ""),
                symbol=str(d.get("Symbol") or d.get("symbol") or "").upper(),
                side=str(d.get("Direction") or d.get("side") or "").lower() or "buy",
                quantity=float(d.get("Quantity") or d.get("quantity") or 0) or 0.0,
                price=float(d.get("Price") or d.get("price") or 0) or 0.0,
                time=str(d.get("Time") or d.get("time") or ""),
                status=str(d.get("Status") or d.get("status") or "filled").lower(),
                type=str(d.get("Type") or d.get("type") or "market").lower(),
                raw=d,
            )
        )
    return out


# ── equity.csv ─────────────────────────────────────────────────────────────
@dataclass(slots=True)
class EquityPoint:
    timestamp: str
    equity: float


def parse_equity_curve(path: str | Path) -> list[EquityPoint]:
    """Parsea curva de equity como lista de puntos."""
    p = Path(path)
    if not p.exists():
        return []
    points: list[EquityPoint] = []
    try:
        with p.open("r", encoding="utf-8", newline="") as f:
            reader = csv.reader(f)
            header_skipped = False
            for row in reader:
                if not row:
                    continue
                if not header_skipped and not _is_number(row[-1]):
                    header_skipped = True
                    continue
                try:
                    ts = row[0]
                    equity = float(row[-1])
                except (ValueError, IndexError):
                    continue
                points.append(EquityPoint(timestamp=ts, equity=equity))
    except OSError as e:
        logger.warning("lean_equity_read_error path=%s err=%s", p, e)
        return []
    return points


def _is_number(s: str) -> bool:
    try:
        float(s)
        return True
    except (TypeError, ValueError):
        return False
