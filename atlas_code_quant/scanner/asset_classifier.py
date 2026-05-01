"""Clasificador de activos — Fase 2 base de arquitectura multi-asset.

Clasifica cualquier símbolo en su AssetClass correspondiente y genera
un AssetProfile con metadatos de opciones, multiplicador y margen.

Pure functions — sin I/O, sin estado global, 100% testeable.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum


class AssetClass(str, Enum):
    EQUITY_STOCK  = "equity_stock"
    EQUITY_ETF    = "equity_etf"
    INDEX_OPTION  = "index_option"    # SPX, NDX, RUT — cash-settled, European
    CRYPTO        = "crypto"
    FUTURE        = "future"
    FOREX         = "forex"
    UNKNOWN       = "unknown"


# ── Universos estáticos ────────────────────────────────────────────────────────

_BROAD_MARKET_ETFS: frozenset[str] = frozenset({
    "SPY", "QQQ", "IWM", "DIA", "MDY", "VTI", "VOO", "SCHB",
    "VXX", "UVXY", "SVXY", "SQQQ", "TQQQ", "SPXU", "SPXL",
})
_SECTOR_ETFS: frozenset[str] = frozenset({
    "XLF", "XLE", "XLK", "XLV", "XLI", "XLB", "XLP", "XLU", "XLRE", "XLC",
    "XBI", "IBB", "SMH", "SOXX", "ARKK", "ARKW", "ARKG",
    "GLD", "SLV", "GDX", "GDXJ", "USO", "UNG",
    "TLT", "IEF", "HYG", "JNK", "LQD", "TBT",
    "EEM", "EFA", "FXI", "EWJ", "VWO",
})
_ALL_ETFS: frozenset[str] = _BROAD_MARKET_ETFS | _SECTOR_ETFS

_INDEX_ROOTS: frozenset[str] = frozenset({
    "SPX", "SPXW", "NDX", "RUT", "VIX", "OEX", "DJX", "XSP",
})

_CRYPTO_RE = re.compile(
    r"^(BTC|ETH|SOL|ADA|DOT|MATIC|AVAX|LINK|UNI|DOGE|XRP|BNB)"
    r"[-/](USDT|USD|USDC|BTC|ETH)$",
    re.IGNORECASE,
)
_FUTURE_RE = re.compile(r"^(ES|NQ|RTY|YM|GC|SI|CL|NG|ZB|ZN|MES|MNQ|MRTY|MYM)([A-Z]\d{2})?$")
_FOREX_RE  = re.compile(
    r"^(EUR|GBP|USD|JPY|AUD|CAD|CHF|NZD)[-/](EUR|GBP|USD|JPY|AUD|CAD|CHF|NZD)$",
    re.IGNORECASE,
)

# ETFs con opciones semanales (0DTE disponible)
_WEEKLIES_ETFS: frozenset[str] = frozenset({
    "SPY", "QQQ", "IWM", "GLD", "TLT", "XLF", "EEM",
})

# Índices con opciones semanals
_WEEKLIES_INDICES: frozenset[str] = frozenset({
    "SPX", "SPXW", "NDX", "RUT",
})


# ── Perfil de activo ───────────────────────────────────────────────────────────

@dataclass
class AssetProfile:
    symbol:              str
    asset_class:         AssetClass
    has_options:         bool          = False
    has_weekly_options:  bool          = False
    is_european_style:   bool          = False   # True → no early assignment
    is_cash_settled:     bool          = False   # True → índices
    no_pdt:              bool          = False   # True → exento de PDT (FINRA)
    multiplier:          int           = 100     # contratos std = 100 acc
    min_dte:             int           = 14
    max_dte:             int           = 45
    typical_spread_pct:  float         = 0.05    # bid-ask / mid estimado
    max_contracts:       int           = 10
    margin_type:         str           = "reg_t" # "reg_t" | "portfolio" | "cash"
    notes:               str           = ""


def classify_asset(symbol: str, is_etf: bool = False) -> AssetProfile:
    """Clasifica un símbolo y retorna su perfil completo."""
    s = symbol.strip().upper()

    # ── Índices (máxima prioridad — SPX no es equity) ──────────────────────
    if s in _INDEX_ROOTS:
        return AssetProfile(
            symbol             = s,
            asset_class        = AssetClass.INDEX_OPTION,
            has_options        = True,
            has_weekly_options = s in _WEEKLIES_INDICES,
            is_european_style  = True,
            is_cash_settled    = True,
            no_pdt             = True,   # broad-based index → FINRA Rule 4210
            multiplier         = 100,
            min_dte            = 7,
            max_dte            = 45,
            typical_spread_pct = 0.02,   # muy líquido
            max_contracts      = 2,      # conservador — nocional alto
            margin_type        = "portfolio",
            notes              = "European style, cash-settled, PDT-exempt",
        )

    # ── Crypto ──────────────────────────────────────────────────────────────
    if _CRYPTO_RE.match(s):
        return AssetProfile(
            symbol             = s,
            asset_class        = AssetClass.CRYPTO,
            has_options        = False,   # Deribit futuro — fase 4
            has_weekly_options = False,
            no_pdt             = True,    # crypto markets never subject to PDT rule
            multiplier         = 1,
            min_dte            = 0,
            max_dte            = 0,
            typical_spread_pct = 0.02,
            margin_type        = "cash",
            notes              = "Spot/perp crypto — broker ccxt",
        )

    # ── Futuros ─────────────────────────────────────────────────────────────
    if _FUTURE_RE.match(s):
        return AssetProfile(
            symbol             = s,
            asset_class        = AssetClass.FUTURE,
            has_options        = False,
            no_pdt             = True,    # CME futures not subject to FINRA PDT rule
            multiplier         = _FUTURES_MULTIPLIER.get(s, _FUTURES_MULTIPLIER.get(s[:2], 50)),
            min_dte            = 0,
            max_dte            = 90,
            typical_spread_pct = 0.01,
            margin_type        = "portfolio",
            notes              = "CME futures — broker extensible (Fase 5)",
        )

    # ── Forex ────────────────────────────────────────────────────────────────
    if _FOREX_RE.match(s):
        return AssetProfile(
            symbol             = s,
            asset_class        = AssetClass.FOREX,
            has_options        = False,
            no_pdt             = True,    # forex not subject to equity PDT rule
            multiplier         = 1,
            typical_spread_pct = 0.01,
            margin_type        = "cash",
            notes              = "Forex — broker extensible (Fase 5)",
        )

    # ── ETF ─────────────────────────────────────────────────────────────────
    if s in _ALL_ETFS or is_etf:
        is_broad = s in _BROAD_MARKET_ETFS
        return AssetProfile(
            symbol             = s,
            asset_class        = AssetClass.EQUITY_ETF,
            has_options        = True,
            has_weekly_options = s in _WEEKLIES_ETFS,
            is_european_style  = False,
            is_cash_settled    = False,
            no_pdt             = False,
            multiplier         = 100,
            min_dte            = 1 if s in _WEEKLIES_ETFS else 7,
            max_dte            = 45,
            typical_spread_pct = 0.02 if is_broad else 0.05,
            max_contracts      = 20 if is_broad else 10,
            margin_type        = "reg_t",
            notes              = "ETF broad" if is_broad else "ETF sectorial",
        )

    # ── Equity stock (default) ───────────────────────────────────────────────
    # La mayor parte de los símbolos de 1-5 letras son acciones US
    has_opts = len(s) <= 5 and s.isalpha()
    return AssetProfile(
        symbol             = s,
        asset_class        = AssetClass.EQUITY_STOCK,
        has_options        = has_opts,
        has_weekly_options = s in {"AAPL", "TSLA", "MSFT", "AMZN", "NVDA",
                                   "META", "GOOGL", "NFLX", "AMD", "COIN"},
        is_european_style  = False,
        multiplier         = 100,
        min_dte            = 14,
        max_dte            = 45,
        typical_spread_pct = 0.06,
        max_contracts      = 5,
        margin_type        = "reg_t",
        notes              = "US equity",
    )


def classify_many(symbols: list[str], etf_set: frozenset[str] | None = None) -> dict[str, AssetProfile]:
    """Clasifica una lista de símbolos en paralelo (dict)."""
    etfs = etf_set or _ALL_ETFS
    return {s: classify_asset(s, is_etf=(s.upper() in etfs)) for s in symbols}


def filter_by_class(
    symbols: list[str],
    allowed: set[AssetClass],
    etf_set: frozenset[str] | None = None,
) -> list[str]:
    """Retorna solo los símbolos que pertenecen a las clases permitidas."""
    profiles = classify_many(symbols, etf_set)
    return [s for s, p in profiles.items() if p.asset_class in allowed]


def get_options_eligible(profiles: dict[str, AssetProfile]) -> list[str]:
    """Símbolos que tienen opciones disponibles."""
    return [s for s, p in profiles.items() if p.has_options]


# ── Tablas auxiliares ──────────────────────────────────────────────────────────

_FUTURES_MULTIPLIER: dict[str, int] = {
    "ES": 50, "NQ": 20, "RTY": 50, "YM": 5,
    "MES": 5, "MNQ": 2, "MRTY": 5, "MYM": 1,
    "GC": 100, "SI": 5000, "CL": 1000, "NG": 10000,
    "ZB": 1000, "ZN": 1000,
}
