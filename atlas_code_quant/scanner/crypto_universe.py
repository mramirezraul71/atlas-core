"""Universo crypto — Fase 4.

Spot, perpetuals (Binance/ccxt) y opciones (Deribit — futuro).
Sin I/O — datos estáticos + helpers de normalización de símbolos.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class CryptoProfile:
    symbol:         str    # "BTC/USDT"
    base:           str    # "BTC"
    quote:          str    # "USDT"
    exchange:       str    # "binance" | "coinbase" | "kraken"
    min_size:       float  # mínimo de contrato en base asset
    tick_size:      float  # paso mínimo de precio
    leverage_max:   int    # apalancamiento máximo permitido
    has_options:    bool   # opciones disponibles (Deribit)
    yfinance_ticker: str   # equivalente para datos históricos


CRYPTO_SPOT_UNIVERSE: dict[str, CryptoProfile] = {
    "BTC/USDT":  CryptoProfile("BTC/USDT",  "BTC",  "USDT", "binance",   0.001,  0.01,   10, True,  "BTC-USD"),
    "ETH/USDT":  CryptoProfile("ETH/USDT",  "ETH",  "USDT", "binance",   0.01,   0.01,   10, True,  "ETH-USD"),
    "SOL/USDT":  CryptoProfile("SOL/USDT",  "SOL",  "USDT", "binance",   0.1,    0.001,   5, False, "SOL-USD"),
    "BNB/USDT":  CryptoProfile("BNB/USDT",  "BNB",  "USDT", "binance",   0.01,   0.01,    5, False, "BNB-USD"),
    "ADA/USDT":  CryptoProfile("ADA/USDT",  "ADA",  "USDT", "binance",   1.0,    0.0001,  3, False, "ADA-USD"),
    "AVAX/USDT": CryptoProfile("AVAX/USDT", "AVAX", "USDT", "binance",   0.1,    0.01,    5, False, "AVAX-USD"),
    "MATIC/USDT":CryptoProfile("MATIC/USDT","MATIC","USDT", "binance",   1.0,    0.0001,  3, False, "MATIC-USD"),
    "LINK/USDT": CryptoProfile("LINK/USDT", "LINK", "USDT", "binance",   0.1,    0.001,   5, False, "LINK-USD"),
    "DOT/USDT":  CryptoProfile("DOT/USDT",  "DOT",  "USDT", "binance",   0.1,    0.001,   3, False, "DOT-USD"),
    "DOGE/USDT": CryptoProfile("DOGE/USDT", "DOGE", "USDT", "binance",  10.0,    0.00001, 3, False, "DOGE-USD"),
    # ── Coinbase ────────────────────────────────────────────────────────────
    "BTC/USD":   CryptoProfile("BTC/USD",   "BTC",  "USD",  "coinbase",  0.001,  0.01,    5, True,  "BTC-USD"),
    "ETH/USD":   CryptoProfile("ETH/USD",   "ETH",  "USD",  "coinbase",  0.01,   0.01,    5, True,  "ETH-USD"),
}


def normalize_to_yfinance(symbol: str) -> str:
    """BTC/USDT → BTC-USD para yfinance."""
    p = CRYPTO_SPOT_UNIVERSE.get(symbol.upper())
    if p:
        return p.yfinance_ticker
    # Fallback genérico: BTC/USDT → BTC-USD
    return symbol.replace("/USDT", "-USD").replace("/USD", "-USD")


def normalize_to_ccxt(symbol: str) -> str:
    """BTC-USD → BTC/USDT (formato ccxt Binance)."""
    reverse = {p.yfinance_ticker: sym for sym, p in CRYPTO_SPOT_UNIVERSE.items()
               if p.exchange == "binance"}
    return reverse.get(symbol.upper(), symbol)


def get_all_crypto_symbols() -> list[str]:
    return list(CRYPTO_SPOT_UNIVERSE.keys())


def get_crypto_by_exchange(exchange: str) -> list[str]:
    return [s for s, p in CRYPTO_SPOT_UNIVERSE.items() if p.exchange == exchange]


def get_crypto_profile(symbol: str) -> CryptoProfile | None:
    return CRYPTO_SPOT_UNIVERSE.get(symbol.upper())
