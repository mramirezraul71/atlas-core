"""Universo de índices con opciones — Fase 3.

SPX, NDX, RUT, VIX: European style, cash-settled, exentos de PDT (FINRA Rule 4210).
El nocional es grande — límite conservador de contratos por defecto.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import date


@dataclass(frozen=True)
class IndexProfile:
    root:                 str     # "SPX"
    tradier_root:         str     # símbolo en Tradier para cadena de opciones
    yfinance_ticker:      str     # "^GSPC"
    display_name:         str
    multiplier:           int     # $100 por punto = $100
    is_european:          bool    # True → no early assignment
    is_cash_settled:      bool    # True → no entrega de acciones
    no_pdt:               bool    # True → exento de PDT (broad-based)
    weekly_expirations:   bool
    has_0dte:             bool
    min_dte:              int
    typical_spread_pts:   float   # spread bid-ask en puntos del índice
    max_contracts:        int     # límite conservador por ATLAS
    approx_point_value:   float   # valor aproximado de 1 punto (USD)
    notes:                str


INDEX_PROFILES: dict[str, IndexProfile] = {
    "SPX": IndexProfile(
        root="SPX", tradier_root="SPX", yfinance_ticker="^GSPC",
        display_name="S&P 500 Index",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=True,
        min_dte=0, typical_spread_pts=0.50, max_contracts=2,
        approx_point_value=100.0,
        notes="CBOE listed. Weeklies Mon/Wed/Fri. 0DTE disponible. Portfolio margin.",
    ),
    "SPXW": IndexProfile(
        root="SPXW", tradier_root="SPXW", yfinance_ticker="^GSPC",
        display_name="S&P 500 Weekly",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=True,
        min_dte=0, typical_spread_pts=0.50, max_contracts=2,
        approx_point_value=100.0,
        notes="Weekly SPX (SPXW). Usa ^GSPC como subyacente para precios.",
    ),
    "NDX": IndexProfile(
        root="NDX", tradier_root="NDX", yfinance_ticker="^NDX",
        display_name="Nasdaq 100 Index",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=False,
        min_dte=7, typical_spread_pts=2.0, max_contracts=1,
        approx_point_value=100.0,
        notes="Alto nocional (~$200k por contrato). Spread mayor que SPX.",
    ),
    "RUT": IndexProfile(
        root="RUT", tradier_root="RUT", yfinance_ticker="^RUT",
        display_name="Russell 2000 Index",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=False,
        min_dte=7, typical_spread_pts=1.0, max_contracts=2,
        approx_point_value=100.0,
        notes="Small-cap index. Bueno para iron condors con alta IV.",
    ),
    "VIX": IndexProfile(
        root="VIX", tradier_root="VIX", yfinance_ticker="^VIX",
        display_name="CBOE Volatility Index",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=False,
        min_dte=14, typical_spread_pts=0.30, max_contracts=2,
        approx_point_value=100.0,
        notes="VIX options usan forward VIX como subyacente, no el spot. Solo calls.",
    ),
    "XSP": IndexProfile(
        root="XSP", tradier_root="XSP", yfinance_ticker="^GSPC",
        display_name="Mini S&P 500 (1/10 SPX)",
        multiplier=100, is_european=True, is_cash_settled=True,
        no_pdt=True, weekly_expirations=True, has_0dte=True,
        min_dte=0, typical_spread_pts=0.10, max_contracts=10,
        approx_point_value=10.0,
        notes="Mini SPX — nocional 10x menor, ideal para cuentas pequeñas.",
    ),
}

# Mapa de ticker yfinance → símbolo de índice ATLAS
YFINANCE_TO_INDEX: dict[str, str] = {
    "^GSPC": "SPX",
    "^NDX":  "NDX",
    "^RUT":  "RUT",
    "^VIX":  "VIX",
}


def get_index_profile(root: str) -> IndexProfile | None:
    return INDEX_PROFILES.get(root.upper())


def get_all_index_symbols() -> list[str]:
    return list(INDEX_PROFILES.keys())


def resolve_yfinance_ticker(symbol: str) -> str:
    """Traduce símbolo ATLAS → ticker yfinance (ej: SPX → ^GSPC)."""
    reverse = {v: k for k, v in YFINANCE_TO_INDEX.items()}
    return reverse.get(symbol.upper(), symbol)


def build_index_option_symbol(
    root: str,
    expiration: date,
    option_type: str,   # "C" o "P"
    strike: float,
) -> str:
    """
    Construye el símbolo OCC estándar para una opción de índice.
    Formato: ROOT + YYMMDD + C/P + 00000000 (strike × 1000, 8 dígitos)
    Ejemplo: SPX240119C04500000
    """
    exp_str = expiration.strftime("%y%m%d")
    strike_str = f"{int(strike * 1000):08d}"
    return f"{root.upper()}{exp_str}{option_type.upper()}{strike_str}"


def estimate_bpr(
    width_points: float,
    multiplier: int = 100,
    is_credit_spread: bool = True,
) -> float:
    """Buying Power Reduction estimado para un spread de índice."""
    if is_credit_spread:
        return width_points * multiplier    # max loss en credit spread
    return width_points * multiplier * 0.5  # aprox para débito
