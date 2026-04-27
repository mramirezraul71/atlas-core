"""Selection policy — F5.

Filtros conservadores sobre cadenas de opciones (greeks, OI, vol, IV, spread).
Útil para descartar strikes inviables antes de pedir cotización al broker.

API:
    decide(option, policy) -> SelectionResult(accept: bool, reasons: list[str])

Estructuras de entrada son ``dict``-like; no asumimos un proveedor concreto.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(slots=True)
class SelectionPolicy:
    delta_min: float = 0.30
    delta_max: float = 0.50
    open_interest_min: int = 100
    volume_min: int = 50
    bid_ask_spread_pct_max: float = 0.05  # 5% del mid
    iv_rank_min: float = 30.0
    iv_rank_max: float = 80.0
    dte_min: int = 7
    dte_max: int = 45


@dataclass(slots=True)
class SelectionResult:
    accept: bool
    reasons: list[str] = field(default_factory=list)


def decide(option: dict[str, Any], policy: SelectionPolicy | None = None) -> SelectionResult:
    """Aplica filtros greeks/IV/OI/vol/spread/DTE.

    ``option`` debe contener al menos:
        delta, open_interest, volume, bid, ask, iv_rank, dte
    Campos faltantes son tratados como 0 (rechazo conservador si dependen).
    """
    p = policy or SelectionPolicy()
    reasons: list[str] = []

    delta = abs(float(option.get("delta", 0.0) or 0.0))
    if not (p.delta_min <= delta <= p.delta_max):
        reasons.append(f"delta_out_of_range delta={delta:.2f}")

    oi = int(option.get("open_interest", 0) or 0)
    if oi < p.open_interest_min:
        reasons.append(f"open_interest_low oi={oi}")

    vol = int(option.get("volume", 0) or 0)
    if vol < p.volume_min:
        reasons.append(f"volume_low vol={vol}")

    bid = float(option.get("bid", 0.0) or 0.0)
    ask = float(option.get("ask", 0.0) or 0.0)
    mid = (bid + ask) / 2 if (bid > 0 and ask > 0) else 0.0
    if mid > 0:
        spread_pct = (ask - bid) / mid
        if spread_pct > p.bid_ask_spread_pct_max:
            reasons.append(f"spread_wide pct={spread_pct:.3f}")
    else:
        reasons.append("no_quote_mid")

    iv_rank = float(option.get("iv_rank", 0.0) or 0.0)
    if not (p.iv_rank_min <= iv_rank <= p.iv_rank_max):
        reasons.append(f"iv_rank_out_of_range iv_rank={iv_rank:.1f}")

    dte = int(option.get("dte", 0) or 0)
    if not (p.dte_min <= dte <= p.dte_max):
        reasons.append(f"dte_out_of_range dte={dte}")

    return SelectionResult(accept=not reasons, reasons=reasons)
