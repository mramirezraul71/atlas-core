"""Selector de contratos de opciones — Fase 1 core.

Dado un TradeSignal (dirección + IV metrics + régimen), selecciona:
  1. La estrategia óptima (long call, bull spread, iron condor, etc.)
  2. La expiración más adecuada (DTE target)
  3. El strike ATM/OTM óptimo con filtro de liquidez

Pure functions — sin I/O directo (el caller inyecta el chain dict).
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger("atlas.execution.option_selector")

# Reutilizamos StrategyType y StrategyLeg del módulo de backtesting
from backtesting.winning_probability import StrategyLeg, StrategyType


# ── Resultado de selección ─────────────────────────────────────────────────────

@dataclass
class ContractSelection:
    """Contrato(s) seleccionados para una señal."""
    strategy_type:          str               # StrategyType
    legs:                   list[dict]        # [{side, option_type, strike, symbol, expiration,...}]
    option_symbol:          str = ""          # para single-leg
    estimated_debit:        float = 0.0       # costo neto (positivo = pago)
    estimated_credit:       float = 0.0       # ingreso neto
    selected_dte:           int = 0
    anchor_strike:          float = 0.0
    width_pct:              float = 0.0
    iv_rank_at_selection:   float = 0.0
    selection_score:        float = 0.0       # 0-1 confianza de la selección
    warnings:               list[str] = field(default_factory=list)
    is_valid:               bool = True


# ── Lógica de selección de estrategia ─────────────────────────────────────────

def pick_strategy(
    direction:    str,        # "BUY" | "SELL"
    regime:       str,        # "BULL" | "BEAR" | "SIDEWAYS" | "TREND"
    iv_rank:      float,      # 0-100
    iv_hv_ratio:  float,      # IV / HV histórica
) -> str:
    """
    Selecciona la estrategia óptima según régimen y nivel de IV.

    Reglas:
      IV Rank > 70 + SIDEWAYS  → iron_condor (sell premium)
      IV Rank > 50 + BULL      → bull_put_credit_spread
      IV Rank > 50 + BEAR      → bear_call_credit_spread
      IV Rank < 30 + BULL      → bull_call_debit_spread o long_call
      IV Rank < 30 + BEAR      → bear_put_debit_spread o long_put
      IV Rank 30-70 + cualquier → spread con delta moderado
    """
    direction = direction.upper()
    regime    = regime.upper() if regime else "SIDEWAYS"
    is_bull   = direction == "BUY" or regime in ("BULL", "TREND")
    is_bear   = direction == "SELL" or regime == "BEAR"
    is_side   = regime == "SIDEWAYS"

    # ── Alta IV: vender prima ──────────────────────────────────────────────
    if iv_rank >= 70:
        if is_side:
            return "iron_condor"
        if is_bull:
            return "bull_put_credit_spread"
        return "bear_call_credit_spread"

    if iv_rank >= 50:
        if is_side:
            return "iron_condor"
        if is_bull:
            return "bull_put_credit_spread"
        return "bear_call_credit_spread"

    # ── IV moderada ────────────────────────────────────────────────────────
    if iv_rank >= 30:
        if is_side:
            return "iron_condor"
        if is_bull:
            return "bull_call_debit_spread"
        return "bear_put_debit_spread"

    # ── Baja IV: comprar prima ─────────────────────────────────────────────
    if is_bull:
        return "long_call" if iv_rank < 20 else "bull_call_debit_spread"
    if is_bear:
        return "long_put" if iv_rank < 20 else "bear_put_debit_spread"

    return "iron_condor"    # fallback neutral


# ── Selección de expiración ────────────────────────────────────────────────────

def pick_expiration(
    expirations: list[str],   # ["2025-02-21", "2025-03-21", ...] ordenadas
    target_dte:  int = 30,
    min_dte:     int = 7,
    max_dte:     int = 60,
) -> str | None:
    """Selecciona la expiración más cercana al target_dte dentro del rango."""
    from datetime import date, datetime
    today = date.today()
    candidates = []
    for exp in expirations:
        try:
            exp_date = datetime.strptime(exp, "%Y-%m-%d").date()
            dte = (exp_date - today).days
            if min_dte <= dte <= max_dte:
                candidates.append((abs(dte - target_dte), dte, exp))
        except ValueError:
            continue
    if not candidates:
        return None
    candidates.sort()
    return candidates[0][2]


# ── Selección de strike ────────────────────────────────────────────────────────

def pick_strike(
    chain_options: list[dict],   # lista de contratos del chain Tradier
    spot:          float,
    option_type:   str,          # "call" | "put"
    target_delta:  float = 0.40, # delta absoluto objetivo (0.30-0.50 = ATM-ish)
    min_oi:        int   = 100,
    min_volume:    int   = 5,
    max_spread_pct: float = 0.20,
) -> dict | None:
    """
    Encuentra el contrato con delta más cercano al target, cumpliendo
    filtros de liquidez (OI, volumen, spread).
    """
    filtered = []
    for opt in chain_options:
        try:
            oi     = float(opt.get("open_interest") or 0)
            vol    = float(opt.get("volume") or 0)
            bid    = float(opt.get("bid") or 0)
            ask    = float(opt.get("ask") or 0)
            mid    = (bid + ask) / 2
            spread = (ask - bid) / mid if mid > 0 else 1.0
            delta  = abs(float(opt.get("greeks", {}).get("delta") or 0))
            otype  = (opt.get("option_type") or "").lower()

            if otype != option_type.lower():
                continue
            if oi < min_oi or vol < min_volume or spread > max_spread_pct:
                continue

            filtered.append((abs(delta - target_delta), opt))
        except (TypeError, ValueError):
            continue

    if not filtered:
        return None
    filtered.sort(key=lambda x: x[0])
    return filtered[0][1]


# ── Construcción de patas ──────────────────────────────────────────────────────

def build_legs_for_strategy(
    strategy_type: str,
    chain_calls:   list[dict],
    chain_puts:    list[dict],
    spot:          float,
    expiration:    str,
    width_pct:     float = 0.03,    # ancho del spread como % del spot
    min_oi:        int   = 100,
) -> list[dict]:
    """Construye la lista de patas para la estrategia dada."""
    width = spot * width_pct

    def _leg(opt: dict, side: str) -> dict:
        return {
            "side":        side,
            "option_type": opt.get("option_type", ""),
            "strike":      float(opt.get("strike") or 0),
            "symbol":      opt.get("symbol", ""),
            "expiration":  expiration,
            "bid":         float(opt.get("bid") or 0),
            "ask":         float(opt.get("ask") or 0),
            "mid":         (float(opt.get("bid") or 0) + float(opt.get("ask") or 0)) / 2,
            "iv":          float((opt.get("greeks") or {}).get("mid_iv") or 0),
            "delta":       float((opt.get("greeks") or {}).get("delta") or 0),
            "open_interest": float(opt.get("open_interest") or 0),
        }

    # ── Single leg ────────────────────────────────────────────────────────
    if strategy_type == "long_call":
        c = pick_strike(chain_calls, spot, "call", target_delta=0.40, min_oi=min_oi)
        return [_leg(c, "long")] if c else []

    if strategy_type == "long_put":
        p = pick_strike(chain_puts, spot, "put", target_delta=0.40, min_oi=min_oi)
        return [_leg(p, "long")] if p else []

    # ── Bull Call Debit Spread ────────────────────────────────────────────
    if strategy_type == "bull_call_debit_spread":
        long_c  = pick_strike(chain_calls, spot, "call", target_delta=0.45, min_oi=min_oi)
        short_c = pick_strike(chain_calls, spot, "call", target_delta=0.25, min_oi=min_oi)
        if long_c and short_c and long_c["symbol"] != short_c["symbol"]:
            return [_leg(long_c, "long"), _leg(short_c, "short")]
        return [_leg(long_c, "long")] if long_c else []

    # ── Bear Put Debit Spread ─────────────────────────────────────────────
    if strategy_type == "bear_put_debit_spread":
        long_p  = pick_strike(chain_puts, spot, "put", target_delta=0.45, min_oi=min_oi)
        short_p = pick_strike(chain_puts, spot, "put", target_delta=0.25, min_oi=min_oi)
        if long_p and short_p and long_p["symbol"] != short_p["symbol"]:
            return [_leg(long_p, "long"), _leg(short_p, "short")]
        return [_leg(long_p, "long")] if long_p else []

    # ── Bull Put Credit Spread ────────────────────────────────────────────
    if strategy_type == "bull_put_credit_spread":
        short_p = pick_strike(chain_puts, spot, "put", target_delta=0.30, min_oi=min_oi)
        long_p  = pick_strike(chain_puts, spot, "put", target_delta=0.15, min_oi=min_oi)
        if short_p and long_p and short_p["symbol"] != long_p["symbol"]:
            return [_leg(short_p, "short"), _leg(long_p, "long")]
        return []

    # ── Bear Call Credit Spread ───────────────────────────────────────────
    if strategy_type == "bear_call_credit_spread":
        short_c = pick_strike(chain_calls, spot, "call", target_delta=0.30, min_oi=min_oi)
        long_c  = pick_strike(chain_calls, spot, "call", target_delta=0.15, min_oi=min_oi)
        if short_c and long_c and short_c["symbol"] != long_c["symbol"]:
            return [_leg(short_c, "short"), _leg(long_c, "long")]
        return []

    # ── Iron Condor ───────────────────────────────────────────────────────
    if strategy_type == "iron_condor":
        sp  = pick_strike(chain_puts,  spot, "put",  target_delta=0.25, min_oi=min_oi)
        lp  = pick_strike(chain_puts,  spot, "put",  target_delta=0.12, min_oi=min_oi)
        sc  = pick_strike(chain_calls, spot, "call", target_delta=0.25, min_oi=min_oi)
        lc  = pick_strike(chain_calls, spot, "call", target_delta=0.12, min_oi=min_oi)
        legs = []
        if sp: legs.append(_leg(sp, "short"))
        if lp: legs.append(_leg(lp, "long"))
        if sc: legs.append(_leg(sc, "short"))
        if lc: legs.append(_leg(lc, "long"))
        return legs if len(legs) == 4 else []

    # ── Iron Butterfly ────────────────────────────────────────────────────
    if strategy_type == "iron_butterfly":
        atm_c = pick_strike(chain_calls, spot, "call", target_delta=0.50, min_oi=min_oi)
        atm_p = pick_strike(chain_puts,  spot, "put",  target_delta=0.50, min_oi=min_oi)
        wing_c = pick_strike(chain_calls, spot, "call", target_delta=0.15, min_oi=min_oi)
        wing_p = pick_strike(chain_puts,  spot, "put",  target_delta=0.15, min_oi=min_oi)
        legs = []
        if atm_c: legs.append(_leg(atm_c, "short"))
        if atm_p: legs.append(_leg(atm_p, "short"))
        if wing_c: legs.append(_leg(wing_c, "long"))
        if wing_p: legs.append(_leg(wing_p, "long"))
        return legs if len(legs) == 4 else []

    logger.warning("Estrategia no implementada en build_legs: %s", strategy_type)
    return []


# ── Función principal ──────────────────────────────────────────────────────────

def select_option_contract(
    symbol:        str,
    direction:     str,           # "BUY" | "SELL"
    spot:          float,
    iv_rank:       float,
    iv_hv_ratio:   float,
    regime:        str,
    expirations:   list[str],
    chain_by_exp:  dict[str, dict],  # {exp: {calls: [...], puts: [...]}}
    strategy_hint: str | None = None,
    min_dte:       int = 14,
    max_dte:       int = 45,
    target_dte:    int = 30,
    min_oi:        int = 100,
    min_volume:    int = 5,
    max_spread_pct: float = 0.20,
    width_pct:     float = 0.03,
) -> ContractSelection:
    """Selección completa: estrategia → expiración → strikes → patas."""

    warnings: list[str] = []

    # 1. Estrategia
    strategy = strategy_hint or pick_strategy(direction, regime, iv_rank, iv_hv_ratio)
    logger.info("[OPT] %s → estrategia=%s iv_rank=%.1f regime=%s",
                symbol, strategy, iv_rank, regime)

    # 2. Expiración
    exp = pick_expiration(expirations, target_dte=target_dte,
                          min_dte=min_dte, max_dte=max_dte)
    if not exp:
        warnings.append(f"No hay expiración entre {min_dte}-{max_dte} DTE")
        return ContractSelection(
            strategy_type=strategy, legs=[], warnings=warnings, is_valid=False,
        )

    # 3. Chain para esa expiración
    chain = chain_by_exp.get(exp, {})
    chain_calls = chain.get("calls") or chain.get("options", {}).get("option", [])
    chain_puts  = chain.get("puts")  or []
    # Separar calls/puts si vienen mezclados
    if not chain_puts and chain_calls:
        chain_puts  = [o for o in chain_calls if (o.get("option_type") or "").lower() == "put"]
        chain_calls = [o for o in chain_calls if (o.get("option_type") or "").lower() == "call"]

    if not chain_calls and not chain_puts:
        warnings.append(f"Chain vacío para {symbol} exp={exp}")
        return ContractSelection(
            strategy_type=strategy, legs=[], warnings=warnings, is_valid=False,
        )

    # 4. Construir patas
    legs = build_legs_for_strategy(
        strategy, chain_calls, chain_puts, spot, exp,
        width_pct=width_pct, min_oi=min_oi,
    )

    if not legs:
        warnings.append(f"No se encontraron contratos líquidos para {strategy}")
        return ContractSelection(
            strategy_type=strategy, legs=[], warnings=warnings, is_valid=False,
        )

    # 5. Calcular debit/credit neto
    net = sum(
        (leg["mid"] if leg["side"] == "short" else -leg["mid"])
        for leg in legs
    )
    debit  = max(0.0, -net)
    credit = max(0.0, net)

    # 6. Strike anchor (primer leg long o primera pata)
    anchor = legs[0].get("strike", spot)

    # 7. Score de calidad de la selección
    avg_iv = sum(leg.get("iv", 0) for leg in legs) / max(len(legs), 1)
    score  = min(1.0, (iv_rank / 100) * 0.4 + (1 - max(0, avg_iv - 0.3)) * 0.3 + 0.3)

    # 8. option_symbol para single-leg
    option_symbol = legs[0].get("symbol", "") if len(legs) == 1 else ""

    from datetime import date, datetime
    dte = (datetime.strptime(exp, "%Y-%m-%d").date() - date.today()).days

    return ContractSelection(
        strategy_type        = strategy,
        legs                 = legs,
        option_symbol        = option_symbol,
        estimated_debit      = round(debit, 4),
        estimated_credit     = round(credit, 4),
        selected_dte         = dte,
        anchor_strike        = anchor,
        width_pct            = width_pct,
        iv_rank_at_selection = iv_rank,
        selection_score      = round(score, 3),
        warnings             = warnings,
        is_valid             = True,
    )
