"""Plantillas de estrategias estándar para ATLAS-Quant OptionStrat.

Cada función construye una Strategy lista para usar con strategy_engine.py.

Convenciones
------------
- Los strikes se expresan como precio absoluto O como offset relativo al spot.
- `quantity` = número de "unidades" de la estructura (1 iron condor = 1 unidad).
- Las primas se pueden dejar en 0.0 y actualizar con datos de mercado reales.
- `iv` se aplica uniformemente salvo que se pase un dict {strike: iv}.

Plantillas disponibles
-----------------------
Opciones simples:
  long_call, short_call, long_put, short_put

Spreads:
  bull_call_spread, bear_put_spread, bear_call_spread, bull_put_spread

Volatilidad:
  long_straddle, short_straddle, long_strangle, short_strangle

Multi-leg:
  iron_condor, iron_butterfly, long_butterfly, broken_wing_butterfly

Con subyacente:
  covered_call, protective_put, collar

Calendarios:
  calendar_spread, diagonal_spread

Especiales:
  ratio_spread, jade_lizard, synthetic_long, synthetic_short
"""
from __future__ import annotations

from datetime import date, timedelta
from typing import Dict, Optional, Tuple

from .strategy_engine import (
    LinearLeg,
    OptionLeg,
    Strategy,
)


# ---------------------------------------------------------------------------
# Helper interno
# ---------------------------------------------------------------------------

def _expiry_from_dte(dte: int) -> date:
    return date.today() + timedelta(days=dte)


def _ol(
    symbol: str,
    expiry: date,
    strike: float,
    option_type: str,
    side: str,
    quantity: int = 1,
    premium: float = 0.0,
    iv: float = 0.25,
    multiplier: int = 100,
    underlying_price: float = 0.0,
) -> OptionLeg:
    return OptionLeg(
        underlying_symbol=symbol,
        expiry=expiry,
        strike=strike,
        option_type=option_type,
        side=side,
        quantity=quantity,
        premium=premium,
        multiplier=multiplier,
        iv=iv,
        underlying_price=underlying_price,
    )


# ---------------------------------------------------------------------------
# Simples
# ---------------------------------------------------------------------------

def long_call(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1, premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Long Call {symbol} {strike}",
        underlying=symbol,
        legs=[_ol(symbol, exp, strike, "call", "long", quantity, premium, iv, underlying_price=spot)],
        metadata={"bias": "bullish", "type": "single_leg", "max_loss": premium * quantity * 100},
    )


def short_call(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1, premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Short Call {symbol} {strike}",
        underlying=symbol,
        legs=[_ol(symbol, exp, strike, "call", "short", quantity, premium, iv, underlying_price=spot)],
        metadata={"bias": "neutral/bearish", "type": "single_leg", "unlimited_risk": True},
    )


def long_put(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1, premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Long Put {symbol} {strike}",
        underlying=symbol,
        legs=[_ol(symbol, exp, strike, "put", "long", quantity, premium, iv, underlying_price=spot)],
        metadata={"bias": "bearish", "type": "single_leg"},
    )


def short_put(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1, premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Short Put {symbol} {strike}",
        underlying=symbol,
        legs=[_ol(symbol, exp, strike, "put", "short", quantity, premium, iv, underlying_price=spot)],
        metadata={"bias": "neutral/bullish", "type": "single_leg"},
    )


# ---------------------------------------------------------------------------
# Vertical Spreads
# ---------------------------------------------------------------------------

def bull_call_spread(
    symbol: str, spot: float,
    long_strike: float, short_strike: float,
    dte: int, quantity: int = 1,
    long_premium: float = 0.0, short_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Bull call spread: compra call inferior, vende call superior."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Bull Call Spread {symbol} {long_strike}/{short_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, long_strike,  "call", "long",  quantity, long_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, short_strike, "call", "short", quantity, short_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "bullish", "type": "vertical_spread", "max_profit": (short_strike - long_strike) * quantity * 100},
    )


def bear_put_spread(
    symbol: str, spot: float,
    long_strike: float, short_strike: float,
    dte: int, quantity: int = 1,
    long_premium: float = 0.0, short_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Bear put spread: compra put superior, vende put inferior."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Bear Put Spread {symbol} {short_strike}/{long_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, long_strike,  "put", "long",  quantity, long_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, short_strike, "put", "short", quantity, short_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "bearish", "type": "vertical_spread"},
    )


def bear_call_spread(
    symbol: str, spot: float,
    short_strike: float, long_strike: float,
    dte: int, quantity: int = 1,
    short_premium: float = 0.0, long_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Bear call credit spread: vende call inferior, compra call superior."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Bear Call Spread {symbol} {short_strike}/{long_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, short_strike, "call", "short", quantity, short_premium, iv, underlying_price=spot),
            _ol(symbol, exp, long_strike,  "call", "long",  quantity, long_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral/bearish", "type": "credit_spread"},
    )


def bull_put_spread(
    symbol: str, spot: float,
    short_strike: float, long_strike: float,
    dte: int, quantity: int = 1,
    short_premium: float = 0.0, long_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Bull put credit spread: vende put superior, compra put inferior."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Bull Put Spread {symbol} {short_strike}/{long_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, short_strike, "put", "short", quantity, short_premium, iv, underlying_price=spot),
            _ol(symbol, exp, long_strike,  "put", "long",  quantity, long_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral/bullish", "type": "credit_spread"},
    )


# ---------------------------------------------------------------------------
# Volatilidad
# ---------------------------------------------------------------------------

def long_straddle(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Long Straddle {symbol} {strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, strike, "call", "long", quantity, call_premium, iv, underlying_price=spot),
            _ol(symbol, exp, strike, "put",  "long", quantity, put_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral_vol", "type": "straddle", "max_loss": (call_premium + put_premium) * quantity * 100},
    )


def short_straddle(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Short Straddle {symbol} {strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, strike, "call", "short", quantity, call_premium, iv, underlying_price=spot),
            _ol(symbol, exp, strike, "put",  "short", quantity, put_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral_vol_short", "type": "straddle", "unlimited_risk": True},
    )


def long_strangle(
    symbol: str, spot: float,
    put_strike: float, call_strike: float,
    dte: int, quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Long Strangle {symbol} {put_strike}/{call_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, put_strike,  "put",  "long", quantity, put_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, call_strike, "call", "long", quantity, call_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral_vol", "type": "strangle"},
    )


def short_strangle(
    symbol: str, spot: float,
    put_strike: float, call_strike: float,
    dte: int, quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Short Strangle {symbol} {put_strike}/{call_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, put_strike,  "put",  "short", quantity, put_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, call_strike, "call", "short", quantity, call_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral_vol_short", "type": "strangle", "unlimited_risk": True},
    )


# ---------------------------------------------------------------------------
# Iron Condor & Iron Butterfly
# ---------------------------------------------------------------------------

def iron_condor(
    symbol: str, spot: float,
    long_put_strike: float,
    short_put_strike: float,
    short_call_strike: float,
    long_call_strike: float,
    dte: int, quantity: int = 1,
    lp_prem: float = 0.0, sp_prem: float = 0.0,
    sc_prem: float = 0.0, lc_prem: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Iron Condor: [long put] [short put] ATM [short call] [long call]"""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Iron Condor {symbol} {short_put_strike}/{short_call_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, long_put_strike,   "put",  "long",  quantity, lp_prem, iv, underlying_price=spot),
            _ol(symbol, exp, short_put_strike,  "put",  "short", quantity, sp_prem, iv, underlying_price=spot),
            _ol(symbol, exp, short_call_strike, "call", "short", quantity, sc_prem, iv, underlying_price=spot),
            _ol(symbol, exp, long_call_strike,  "call", "long",  quantity, lc_prem, iv, underlying_price=spot),
        ],
        metadata={
            "bias": "neutral",
            "type": "iron_condor",
            "put_width": short_put_strike - long_put_strike,
            "call_width": long_call_strike - short_call_strike,
        },
    )


def iron_butterfly(
    symbol: str, spot: float,
    wing_width: float,      # ancho de cada ala (e.g. 10 puntos)
    atm_strike: float,      # strike central (normalmente ATM)
    dte: int, quantity: int = 1,
    lp_prem: float = 0.0, sp_prem: float = 0.0,
    sc_prem: float = 0.0, lc_prem: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Iron Butterfly: short straddle + long wings."""
    exp = _expiry_from_dte(dte)
    lp_k = atm_strike - wing_width
    lc_k = atm_strike + wing_width
    return Strategy(
        name=f"Iron Butterfly {symbol} {lp_k}/{atm_strike}/{lc_k}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, lp_k,      "put",  "long",  quantity, lp_prem, iv, underlying_price=spot),
            _ol(symbol, exp, atm_strike, "put",  "short", quantity, sp_prem, iv, underlying_price=spot),
            _ol(symbol, exp, atm_strike, "call", "short", quantity, sc_prem, iv, underlying_price=spot),
            _ol(symbol, exp, lc_k,      "call", "long",  quantity, lc_prem, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral", "type": "iron_butterfly", "max_profit_at": atm_strike},
    )


# ---------------------------------------------------------------------------
# Butterfly
# ---------------------------------------------------------------------------

def long_butterfly(
    symbol: str, spot: float,
    low_strike: float, mid_strike: float, high_strike: float,
    option_type: str,  # "call" o "put"
    dte: int, quantity: int = 1,
    iv: float = 0.25,
) -> Strategy:
    """Long butterfly: compra low+high, vende 2× mid."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Long {option_type.capitalize()} Butterfly {symbol} {low_strike}/{mid_strike}/{high_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, low_strike,  option_type, "long",  quantity,     0.0, iv, underlying_price=spot),
            _ol(symbol, exp, mid_strike,  option_type, "short", quantity * 2, 0.0, iv, underlying_price=spot),
            _ol(symbol, exp, high_strike, option_type, "long",  quantity,     0.0, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral_low_vol", "type": "butterfly"},
    )


def broken_wing_butterfly(
    symbol: str, spot: float,
    low_strike: float, mid_strike: float, high_strike: float,
    dte: int, quantity: int = 1,
    iv: float = 0.25,
) -> Strategy:
    """Broken Wing Butterfly (put BWB): put spread asimétrico para crédito."""
    # Ala corta: mid−low; ala larga: high−mid (más ancha → crédito)
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Broken Wing Butterfly {symbol} {low_strike}/{mid_strike}/{high_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, high_strike, "put", "long",  quantity,     0.0, iv, underlying_price=spot),
            _ol(symbol, exp, mid_strike,  "put", "short", quantity * 2, 0.0, iv, underlying_price=spot),
            _ol(symbol, exp, low_strike,  "put", "long",  quantity,     0.0, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral/bullish_skew", "type": "broken_wing_butterfly"},
    )


# ---------------------------------------------------------------------------
# Estrategias con subyacente
# ---------------------------------------------------------------------------

def covered_call(
    symbol: str, spot: float, strike: float, dte: int,
    shares: int = 100,
    call_premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    """Covered call: larga en acciones + short call."""
    exp = _expiry_from_dte(dte)
    contracts = shares // 100
    return Strategy(
        name=f"Covered Call {symbol} {strike}",
        underlying=symbol,
        legs=[
            LinearLeg(symbol=symbol, asset_class="stock", side="long",
                      quantity=shares, entry_price=spot, multiplier=1.0),
            _ol(symbol, exp, strike, "call", "short", contracts, call_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral/bullish", "type": "covered_call"},
    )


def protective_put(
    symbol: str, spot: float, strike: float, dte: int,
    shares: int = 100,
    put_premium: float = 0.0, iv: float = 0.25,
) -> Strategy:
    """Protective put: larga en acciones + long put (seguro)."""
    exp = _expiry_from_dte(dte)
    contracts = shares // 100
    return Strategy(
        name=f"Protective Put {symbol} {strike}",
        underlying=symbol,
        legs=[
            LinearLeg(symbol=symbol, asset_class="stock", side="long",
                      quantity=shares, entry_price=spot, multiplier=1.0),
            _ol(symbol, exp, strike, "put", "long", contracts, put_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "long_with_hedge", "type": "protective_put"},
    )


def collar(
    symbol: str, spot: float,
    put_strike: float, call_strike: float,
    dte: int, shares: int = 100,
    put_premium: float = 0.0, call_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Collar: long stock + long put + short call."""
    exp = _expiry_from_dte(dte)
    contracts = shares // 100
    return Strategy(
        name=f"Collar {symbol} {put_strike}/{call_strike}",
        underlying=symbol,
        legs=[
            LinearLeg(symbol=symbol, asset_class="stock", side="long",
                      quantity=shares, entry_price=spot, multiplier=1.0),
            _ol(symbol, exp, put_strike,  "put",  "long",  contracts, put_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, call_strike, "call", "short", contracts, call_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "hedge_with_cap", "type": "collar"},
    )


# ---------------------------------------------------------------------------
# Calendar / Diagonal
# ---------------------------------------------------------------------------

def calendar_spread(
    symbol: str, spot: float, strike: float,
    dte_near: int, dte_far: int,
    option_type: str = "call",
    quantity: int = 1,
    near_premium: float = 0.0, far_premium: float = 0.0,
    near_iv: float = 0.25, far_iv: float = 0.22,
) -> Strategy:
    """Calendar spread: vende vencimiento cercano, compra vencimiento lejano."""
    exp_near = _expiry_from_dte(dte_near)
    exp_far  = _expiry_from_dte(dte_far)
    return Strategy(
        name=f"Calendar Spread {symbol} {strike} {dte_near}d/{dte_far}d",
        underlying=symbol,
        legs=[
            _ol(symbol, exp_near, strike, option_type, "short", quantity, near_premium, near_iv, underlying_price=spot),
            _ol(symbol, exp_far,  strike, option_type, "long",  quantity, far_premium,  far_iv,  underlying_price=spot),
        ],
        metadata={"bias": "neutral_low_vol", "type": "calendar"},
    )


def diagonal_spread(
    symbol: str, spot: float,
    near_strike: float, far_strike: float,
    dte_near: int, dte_far: int,
    option_type: str = "call",
    side_near: str = "short",
    quantity: int = 1,
    near_premium: float = 0.0, far_premium: float = 0.0,
    near_iv: float = 0.25, far_iv: float = 0.22,
) -> Strategy:
    """Diagonal: diferentes strikes + diferentes vencimientos."""
    exp_near = _expiry_from_dte(dte_near)
    exp_far  = _expiry_from_dte(dte_far)
    side_far = "long" if side_near == "short" else "short"
    return Strategy(
        name=f"Diagonal {symbol} {near_strike}/{far_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp_near, near_strike, option_type, side_near, quantity, near_premium, near_iv, underlying_price=spot),
            _ol(symbol, exp_far,  far_strike,  option_type, side_far,  quantity, far_premium,  far_iv,  underlying_price=spot),
        ],
        metadata={"bias": "directional_time", "type": "diagonal"},
    )


# ---------------------------------------------------------------------------
# Especiales
# ---------------------------------------------------------------------------

def ratio_spread(
    symbol: str, spot: float,
    long_strike: float, short_strike: float,
    ratio: int = 2,  # 1 long : ratio short
    dte: int = 30,
    option_type: str = "call",
    quantity: int = 1,
    long_premium: float = 0.0, short_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """1×2 (o 1×ratio) ratio spread."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Ratio Spread {symbol} {long_strike}/{short_strike} 1×{ratio}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, long_strike,  option_type, "long",  quantity,       long_premium,  iv, underlying_price=spot),
            _ol(symbol, exp, short_strike, option_type, "short", quantity * ratio, short_premium, iv, underlying_price=spot),
        ],
        metadata={"bias": "directional_low_cost", "type": "ratio_spread"},
    )


def jade_lizard(
    symbol: str, spot: float,
    short_put_strike: float,
    short_call_strike: float,
    long_call_strike: float,
    dte: int, quantity: int = 1,
    sp_prem: float = 0.0, sc_prem: float = 0.0, lc_prem: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Jade Lizard: short put + bear call spread (sin riesgo alcista)."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Jade Lizard {symbol} {short_put_strike}/{short_call_strike}/{long_call_strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, short_put_strike,  "put",  "short", quantity, sp_prem, iv, underlying_price=spot),
            _ol(symbol, exp, short_call_strike, "call", "short", quantity, sc_prem, iv, underlying_price=spot),
            _ol(symbol, exp, long_call_strike,  "call", "long",  quantity, lc_prem, iv, underlying_price=spot),
        ],
        metadata={"bias": "neutral/bullish", "type": "jade_lizard"},
    )


def synthetic_long(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Synthetic long: long call + short put al mismo strike."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Synthetic Long {symbol} {strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, strike, "call", "long",  quantity, call_premium, iv, underlying_price=spot),
            _ol(symbol, exp, strike, "put",  "short", quantity, put_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "bullish", "type": "synthetic"},
    )


def synthetic_short(
    symbol: str, spot: float, strike: float, dte: int,
    quantity: int = 1,
    call_premium: float = 0.0, put_premium: float = 0.0,
    iv: float = 0.25,
) -> Strategy:
    """Synthetic short: short call + long put al mismo strike."""
    exp = _expiry_from_dte(dte)
    return Strategy(
        name=f"Synthetic Short {symbol} {strike}",
        underlying=symbol,
        legs=[
            _ol(symbol, exp, strike, "call", "short", quantity, call_premium, iv, underlying_price=spot),
            _ol(symbol, exp, strike, "put",  "long",  quantity, put_premium,  iv, underlying_price=spot),
        ],
        metadata={"bias": "bearish", "type": "synthetic"},
    )


# ---------------------------------------------------------------------------
# Factory — construir estrategia por nombre
# ---------------------------------------------------------------------------

TEMPLATE_REGISTRY = {
    "long_call":           long_call,
    "short_call":          short_call,
    "long_put":            long_put,
    "short_put":           short_put,
    "bull_call_spread":    bull_call_spread,
    "bear_put_spread":     bear_put_spread,
    "bear_call_spread":    bear_call_spread,
    "bull_put_spread":     bull_put_spread,
    "long_straddle":       long_straddle,
    "short_straddle":      short_straddle,
    "long_strangle":       long_strangle,
    "short_strangle":      short_strangle,
    "iron_condor":         iron_condor,
    "iron_butterfly":      iron_butterfly,
    "long_butterfly":      long_butterfly,
    "broken_wing_butterfly": broken_wing_butterfly,
    "covered_call":        covered_call,
    "protective_put":      protective_put,
    "collar":              collar,
    "calendar_spread":     calendar_spread,
    "diagonal_spread":     diagonal_spread,
    "ratio_spread":        ratio_spread,
    "jade_lizard":         jade_lizard,
    "synthetic_long":      synthetic_long,
    "synthetic_short":     synthetic_short,
}


def list_templates() -> list:
    """Lista de plantillas disponibles con metadatos."""
    meta = {
        "long_call":           {"category": "single", "bias": "bullish",          "legs": 1},
        "short_call":          {"category": "single", "bias": "neutral/bearish",   "legs": 1},
        "long_put":            {"category": "single", "bias": "bearish",           "legs": 1},
        "short_put":           {"category": "single", "bias": "neutral/bullish",   "legs": 1},
        "bull_call_spread":    {"category": "spread",  "bias": "bullish",          "legs": 2},
        "bear_put_spread":     {"category": "spread",  "bias": "bearish",          "legs": 2},
        "bear_call_spread":    {"category": "spread",  "bias": "neutral/bearish",  "legs": 2},
        "bull_put_spread":     {"category": "spread",  "bias": "neutral/bullish",  "legs": 2},
        "long_straddle":       {"category": "vol",     "bias": "vol_long",         "legs": 2},
        "short_straddle":      {"category": "vol",     "bias": "vol_short",        "legs": 2},
        "long_strangle":       {"category": "vol",     "bias": "vol_long",         "legs": 2},
        "short_strangle":      {"category": "vol",     "bias": "vol_short",        "legs": 2},
        "iron_condor":         {"category": "multileg","bias": "neutral",          "legs": 4},
        "iron_butterfly":      {"category": "multileg","bias": "neutral",          "legs": 4},
        "long_butterfly":      {"category": "multileg","bias": "neutral_low_vol",  "legs": 3},
        "broken_wing_butterfly":{"category":"multileg","bias": "neutral/bullish",  "legs": 3},
        "covered_call":        {"category": "stock",   "bias": "neutral/bullish",  "legs": 2},
        "protective_put":      {"category": "stock",   "bias": "long_hedge",       "legs": 2},
        "collar":              {"category": "stock",   "bias": "hedge_with_cap",   "legs": 3},
        "calendar_spread":     {"category": "time",    "bias": "neutral_vol",      "legs": 2},
        "diagonal_spread":     {"category": "time",    "bias": "directional",      "legs": 2},
        "ratio_spread":        {"category": "special", "bias": "directional",      "legs": 2},
        "jade_lizard":         {"category": "special", "bias": "neutral/bullish",  "legs": 3},
        "synthetic_long":      {"category": "special", "bias": "bullish",          "legs": 2},
        "synthetic_short":     {"category": "special", "bias": "bearish",          "legs": 2},
    }
    return [{"name": k, **v} for k, v in meta.items()]
