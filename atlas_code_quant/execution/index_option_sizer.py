"""Sizing para opciones de índices — Fase 3.

El KellySizer estándar trabaja en shares. Para índices (SPX, NDX, RUT)
el sizing se basa en BPR (Buying Power Reduction) como % del capital.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

logger = logging.getLogger("atlas.execution.index_option_sizer")


@dataclass
class IndexOptionSize:
    contracts:            int
    bpr_per_contract:     float    # Buying Power Reduction por contrato (USD)
    total_bpr:            float
    max_loss:             float
    max_gain:             float
    bpr_pct_of_capital:   float
    warnings:             list[str] = field(default_factory=list)
    is_approved:          bool = True


def size_index_option(
    strategy_type:   str,
    legs:            list[dict],    # patas del option_selector
    capital:         float,
    multiplier:      int   = 100,
    max_bpr_pct:     float = 0.02,  # 2% del capital por defecto — conservador
    max_contracts:   int   = 2,
) -> IndexOptionSize:
    """
    Calcula el número de contratos para una posición de opciones de índice.

    BPR = ancho del spread × multiplicador (para credit spreads)
    BPR = debit pagado × multiplicador (para debit spreads)
    """
    warnings: list[str] = []

    if not legs:
        return IndexOptionSize(
            contracts=0, bpr_per_contract=0, total_bpr=0,
            max_loss=0, max_gain=0, bpr_pct_of_capital=0,
            warnings=["Sin patas — no se puede calcular BPR"],
            is_approved=False,
        )

    # ── BPR según estrategia ───────────────────────────────────────────────
    bpr = _estimate_bpr(strategy_type, legs, multiplier)

    if bpr <= 0:
        return IndexOptionSize(
            contracts=0, bpr_per_contract=0, total_bpr=0,
            max_loss=0, max_gain=0, bpr_pct_of_capital=0,
            warnings=["BPR calculado = 0 — spread demasiado pequeño"],
            is_approved=False,
        )

    # ── Número de contratos ────────────────────────────────────────────────
    max_bpr_total = capital * max_bpr_pct
    contracts = max(1, min(
        int(max_bpr_total / bpr),
        max_contracts,
    ))

    total_bpr = bpr * contracts
    bpr_pct   = total_bpr / capital if capital > 0 else 0

    if bpr_pct > max_bpr_pct * 1.1:
        warnings.append(
            f"BPR {bpr_pct:.1%} supera límite {max_bpr_pct:.1%}. "
            f"Reduciendo a 1 contrato."
        )
        contracts = 1
        total_bpr = bpr
        bpr_pct   = total_bpr / capital

    # ── Max loss / max gain ────────────────────────────────────────────────
    net_premium = _net_premium(legs)
    is_credit = net_premium > 0
    if is_credit:
        max_loss = bpr - net_premium * multiplier * contracts
        max_gain = net_premium * multiplier * contracts
    else:
        max_loss = abs(net_premium) * multiplier * contracts
        max_gain = _estimate_max_gain(strategy_type, legs, multiplier, contracts)

    logger.info(
        "[IDX SIZER] %s | contracts=%d BPR/c=$%.0f total_BPR=$%.0f (%.1f%% capital)",
        strategy_type, contracts, bpr, total_bpr, bpr_pct * 100,
    )

    return IndexOptionSize(
        contracts          = contracts,
        bpr_per_contract   = round(bpr, 2),
        total_bpr          = round(total_bpr, 2),
        max_loss           = round(max_loss, 2),
        max_gain           = round(max_gain, 2),
        bpr_pct_of_capital = round(bpr_pct, 4),
        warnings           = warnings,
        is_approved        = contracts >= 1,
    )


# ── Helpers internos ───────────────────────────────────────────────────────────

def _estimate_bpr(strategy_type: str, legs: list[dict], multiplier: int) -> float:
    """BPR estimado en USD por contrato."""
    strikes = sorted(set(leg.get("strike", 0) for leg in legs))
    net = _net_premium(legs)

    # Credit strategies: BPR = width - credit received
    if strategy_type in ("bull_put_credit_spread", "bear_call_credit_spread"):
        width = abs(strikes[-1] - strikes[0]) if len(strikes) >= 2 else 0
        return max(0.0, (width - net) * multiplier)

    # Iron Condor / Iron Butterfly: BPR = narrower wing - credit
    if strategy_type in ("iron_condor", "iron_butterfly"):
        if len(strikes) >= 4:
            put_width  = strikes[1] - strikes[0]
            call_width = strikes[3] - strikes[2]
            width = min(put_width, call_width)
        else:
            width = abs(strikes[-1] - strikes[0]) if strikes else 0
        return max(0.0, (width - net) * multiplier)

    # Debit strategies: BPR = debit paid
    return max(0.0, abs(net) * multiplier)


def _net_premium(legs: list[dict]) -> float:
    """Prima neta de la posición (positivo = credit recibido)."""
    total = 0.0
    for leg in legs:
        mid = leg.get("mid", 0.0)
        if leg.get("side") == "short":
            total += mid    # cobramos
        else:
            total -= mid    # pagamos
    return total


def _estimate_max_gain(
    strategy_type: str, legs: list[dict],
    multiplier: int, contracts: int,
) -> float:
    """Max gain estimado para estrategias de débito."""
    strikes = sorted(leg.get("strike", 0) for leg in legs)
    net     = abs(_net_premium(legs))
    if strategy_type in ("bull_call_debit_spread", "bear_put_debit_spread"):
        width = abs(strikes[-1] - strikes[0]) if len(strikes) >= 2 else 0
        return max(0.0, (width - net) * multiplier * contracts)
    # Single leg: ganancia ilimitada en teoría — acotamos a 3× debit
    return net * multiplier * contracts * 3
