"""Universo curado de ETFs con opciones líquidas — Fase 2.

Incluye ETFs de mercado amplio, sectoriales, commodities, bonos y volatilidad.
Datos estáticos actualizados 2026-Q1.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ETFProfile:
    symbol:         str
    name:           str
    sector:         str
    has_0dte:       bool
    min_dte:        int
    typical_oi:     int    # open interest típico en strike ATM
    avg_daily_vol:  int    # volumen diario promedio en opciones


ETF_OPTIONS_UNIVERSE: dict[str, ETFProfile] = {
    # ── Mercado Amplio ────────────────────────────────────────────────────────
    "SPY":  ETFProfile("SPY",  "SPDR S&P 500",           "broad_market",  True,  1, 800_000, 2_000_000),
    "QQQ":  ETFProfile("QQQ",  "Invesco Nasdaq 100",      "tech_broad",    True,  1, 400_000, 1_500_000),
    "IWM":  ETFProfile("IWM",  "iShares Russell 2000",    "small_cap",     True,  1, 200_000,   500_000),
    "DIA":  ETFProfile("DIA",  "SPDR Dow Jones",          "broad_market",  False, 7,  50_000,    80_000),
    "VTI":  ETFProfile("VTI",  "Vanguard Total Market",   "broad_market",  False, 7,  30_000,    40_000),
    "VOO":  ETFProfile("VOO",  "Vanguard S&P 500",        "broad_market",  False, 7,  40_000,    60_000),
    # ── Leveraged / Inverse ────────────────────────────────────────────────────
    "TQQQ": ETFProfile("TQQQ", "ProShares UltraPro QQQ",  "leveraged",     False, 7, 100_000,   300_000),
    "SQQQ": ETFProfile("SQQQ", "ProShares UltraPro Short","leveraged",     False, 7,  80_000,   200_000),
    "SPXL": ETFProfile("SPXL", "Direxion S&P500 3x Bull", "leveraged",     False, 7,  30_000,    60_000),
    "SPXU": ETFProfile("SPXU", "Direxion S&P500 3x Bear", "leveraged",     False, 7,  25_000,    50_000),
    # ── Volatilidad ────────────────────────────────────────────────────────────
    "VXX":  ETFProfile("VXX",  "iPath VIX Short-Term",    "volatility",    False, 7, 150_000,   400_000),
    "UVXY": ETFProfile("UVXY", "ProShares Ultra VIX",     "volatility",    False, 7,  80_000,   200_000),
    "SVXY": ETFProfile("SVXY", "ProShares Short VIX",     "volatility",    False, 7,  30_000,    80_000),
    # ── Sectoriales ────────────────────────────────────────────────────────────
    "XLF":  ETFProfile("XLF",  "Financial Select Sector", "financials",    False, 7,  80_000,   200_000),
    "XLE":  ETFProfile("XLE",  "Energy Select Sector",    "energy",        False, 7,  60_000,   150_000),
    "XLK":  ETFProfile("XLK",  "Technology Select Sector","technology",    False, 7,  70_000,   180_000),
    "XLV":  ETFProfile("XLV",  "Health Care Select Sector","healthcare",   False, 7,  50_000,   100_000),
    "XLI":  ETFProfile("XLI",  "Industrial Select Sector","industrials",   False, 7,  30_000,    60_000),
    "XLP":  ETFProfile("XLP",  "Consumer Staples Sector", "staples",       False, 7,  25_000,    50_000),
    "XLU":  ETFProfile("XLU",  "Utilities Select Sector", "utilities",     False, 7,  20_000,    40_000),
    "XLRE": ETFProfile("XLRE", "Real Estate Sector",      "real_estate",   False, 7,  15_000,    30_000),
    "XLC":  ETFProfile("XLC",  "Communication Services",  "communication", False, 7,  20_000,    40_000),
    # ── Biotech / Tech ─────────────────────────────────────────────────────────
    "XBI":  ETFProfile("XBI",  "SPDR Biotech",            "biotech",       False, 7,  80_000,   200_000),
    "IBB":  ETFProfile("IBB",  "iShares Biotech",         "biotech",       False, 7,  40_000,    80_000),
    "SMH":  ETFProfile("SMH",  "VanEck Semiconductor",    "semiconductors",False, 7,  60_000,   150_000),
    "SOXX": ETFProfile("SOXX", "iShares Semiconductor",   "semiconductors",False, 7,  40_000,    80_000),
    "ARKK": ETFProfile("ARKK", "ARK Innovation",          "disruptive",    False, 7,  50_000,   120_000),
    # ── Commodities ────────────────────────────────────────────────────────────
    "GLD":  ETFProfile("GLD",  "SPDR Gold Shares",        "gold",          False, 7, 120_000,   300_000),
    "SLV":  ETFProfile("SLV",  "iShares Silver",          "silver",        False, 7,  40_000,    80_000),
    "GDX":  ETFProfile("GDX",  "VanEck Gold Miners",      "gold_miners",   False, 7,  60_000,   150_000),
    "GDXJ": ETFProfile("GDXJ", "VanEck Jr Gold Miners",   "gold_miners",   False, 7,  30_000,    60_000),
    "USO":  ETFProfile("USO",  "United States Oil Fund",  "oil",           False, 7,  30_000,    80_000),
    # ── Bonos / Rates ─────────────────────────────────────────────────────────
    "TLT":  ETFProfile("TLT",  "iShares 20+ Year Treasury","bonds_long",   False, 7, 100_000,   250_000),
    "IEF":  ETFProfile("IEF",  "iShares 7-10 Year Treasury","bonds_mid",   False, 7,  30_000,    60_000),
    "HYG":  ETFProfile("HYG",  "iShares High Yield Corp", "bonds_hy",     False, 7,  50_000,   100_000),
    "LQD":  ETFProfile("LQD",  "iShares Investment Grade","bonds_ig",      False, 7,  25_000,    50_000),
    "TBT":  ETFProfile("TBT",  "ProShares Short 20+ Yr",  "bonds_short",   False, 7,  20_000,    40_000),
    # ── Internacional ─────────────────────────────────────────────────────────
    "EEM":  ETFProfile("EEM",  "iShares Emerging Markets","emerging",      False, 7,  80_000,   200_000),
    "EFA":  ETFProfile("EFA",  "iShares MSCI EAFE",       "developed_intl",False,7,  30_000,    60_000),
    "FXI":  ETFProfile("FXI",  "iShares China Large-Cap", "china",         False, 7,  50_000,   120_000),
}


def get_all_etf_symbols() -> list[str]:
    return list(ETF_OPTIONS_UNIVERSE.keys())


def get_etfs_with_0dte() -> list[str]:
    return [s for s, p in ETF_OPTIONS_UNIVERSE.items() if p.has_0dte]


def get_etfs_by_sector(sector: str) -> list[str]:
    return [s for s, p in ETF_OPTIONS_UNIVERSE.items() if p.sector == sector]


def get_etf_profile(symbol: str) -> ETFProfile | None:
    return ETF_OPTIONS_UNIVERSE.get(symbol.upper())


def get_liquid_etfs(min_oi: int = 50_000) -> list[str]:
    """ETFs con suficiente liquidez de opciones para operar con spread razonable."""
    return [s for s, p in ETF_OPTIONS_UNIVERSE.items() if p.typical_oi >= min_oi]
