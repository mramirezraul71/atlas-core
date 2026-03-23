"""Universo de futuros — Fase 5 (esqueleto extensible).

CME equity index futures (ES, NQ, RTY, YM) + commodities.
Sin ejecución real — la arquitectura está preparada para cuando
se conecte un broker compatible (Interactive Brokers TWS / Tradier Futures).
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import date


@dataclass(frozen=True)
class FuturesProfile:
    root:              str     # "ES"
    exchange:          str     # "CME"
    display_name:      str
    tick_size:         float   # 0.25 puntos
    tick_value:        float   # $12.50 por tick
    multiplier:        int     # $50 por punto
    margin_initial:    float   # margen inicial aprox (USD)
    margin_maint:      float   # margen de mantenimiento aprox (USD)
    trading_hours:     str     # "23h/5d" | "RTH only"
    yfinance_ticker:   str     # "ES=F"
    has_options:       bool
    continuous_symbol: str     # "@ES" para datos continuos


FUTURES_UNIVERSE: dict[str, FuturesProfile] = {
    # ── Equity Index ──────────────────────────────────────────────────────────
    "ES": FuturesProfile(
        root="ES", exchange="CME",
        display_name="E-mini S&P 500",
        tick_size=0.25, tick_value=12.50, multiplier=50,
        margin_initial=14_000, margin_maint=12_750,
        trading_hours="23h/5d", yfinance_ticker="ES=F",
        has_options=True, continuous_symbol="@ES",
    ),
    "NQ": FuturesProfile(
        root="NQ", exchange="CME",
        display_name="E-mini Nasdaq 100",
        tick_size=0.25, tick_value=5.00, multiplier=20,
        margin_initial=21_000, margin_maint=19_000,
        trading_hours="23h/5d", yfinance_ticker="NQ=F",
        has_options=True, continuous_symbol="@NQ",
    ),
    "RTY": FuturesProfile(
        root="RTY", exchange="CME",
        display_name="E-mini Russell 2000",
        tick_size=0.10, tick_value=5.00, multiplier=50,
        margin_initial=8_000, margin_maint=7_250,
        trading_hours="23h/5d", yfinance_ticker="RTY=F",
        has_options=True, continuous_symbol="@RTY",
    ),
    "YM": FuturesProfile(
        root="YM", exchange="CBOT",
        display_name="E-mini Dow Jones",
        tick_size=1.0, tick_value=5.00, multiplier=5,
        margin_initial=10_000, margin_maint=9_000,
        trading_hours="23h/5d", yfinance_ticker="YM=F",
        has_options=True, continuous_symbol="@YM",
    ),
    # ── Micro Futures (menor capital requerido) ───────────────────────────────
    "MES": FuturesProfile(
        root="MES", exchange="CME",
        display_name="Micro E-mini S&P 500",
        tick_size=0.25, tick_value=1.25, multiplier=5,
        margin_initial=1_400, margin_maint=1_275,
        trading_hours="23h/5d", yfinance_ticker="ES=F",
        has_options=False, continuous_symbol="@MES",
    ),
    "MNQ": FuturesProfile(
        root="MNQ", exchange="CME",
        display_name="Micro E-mini Nasdaq 100",
        tick_size=0.25, tick_value=0.50, multiplier=2,
        margin_initial=2_100, margin_maint=1_900,
        trading_hours="23h/5d", yfinance_ticker="NQ=F",
        has_options=False, continuous_symbol="@MNQ",
    ),
    # ── Commodities ───────────────────────────────────────────────────────────
    "GC": FuturesProfile(
        root="GC", exchange="COMEX",
        display_name="Gold",
        tick_size=0.10, tick_value=10.00, multiplier=100,
        margin_initial=12_000, margin_maint=10_900,
        trading_hours="23h/5d", yfinance_ticker="GC=F",
        has_options=True, continuous_symbol="@GC",
    ),
    "CL": FuturesProfile(
        root="CL", exchange="NYMEX",
        display_name="Crude Oil WTI",
        tick_size=0.01, tick_value=10.00, multiplier=1000,
        margin_initial=7_000, margin_maint=6_350,
        trading_hours="23h/5d", yfinance_ticker="CL=F",
        has_options=True, continuous_symbol="@CL",
    ),
}


def get_all_futures_symbols() -> list[str]:
    return list(FUTURES_UNIVERSE.keys())


def get_futures_profile(root: str) -> FuturesProfile | None:
    return FUTURES_UNIVERSE.get(root.upper())


def get_micro_futures() -> list[str]:
    """Futuros micro — menor capital requerido."""
    return [s for s, p in FUTURES_UNIVERSE.items()
            if p.display_name.lower().startswith("micro")]


def point_value(root: str) -> float:
    """Valor de 1 punto en USD para el futuro dado."""
    p = FUTURES_UNIVERSE.get(root.upper())
    return float(p.multiplier) if p else 0.0


def notional_value(root: str, price: float) -> float:
    """Valor nocional de 1 contrato al precio dado."""
    return point_value(root) * price
