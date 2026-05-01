"""Tests para ``implied_vol`` (bisección sobre ``bs_price``)."""
from __future__ import annotations

import math

import pytest

from atlas_code_quant.options.strategy_engine import bs_price, implied_vol


def test_implied_vol_call_round_trip() -> None:
    S, K, T, r = 100.0, 100.0, 30.0 / 365.0, 0.05
    sigma_true = 0.25
    market = bs_price(S, K, T, r, sigma_true, "call")
    sigma_iv = implied_vol(S, K, T, r, market, "call", tol=1e-7, max_iter=80)
    assert sigma_iv is not None
    assert abs(sigma_iv - sigma_true) < 1e-4


def test_implied_vol_put_round_trip() -> None:
    S, K, T, r = 120.0, 115.0, 45.0 / 365.0, 0.03
    sigma_true = 0.42
    market = bs_price(S, K, T, r, sigma_true, "put")
    sigma_iv = implied_vol(S, K, T, r, market, "put", tol=1e-7, max_iter=80)
    assert sigma_iv is not None
    assert abs(sigma_iv - sigma_true) < 1e-4


def test_implied_vol_call_uppercase_type() -> None:
    S, K, T, r = 50.0, 52.0, 60 / 365.0, 0.01
    sig = 0.18
    px = bs_price(S, K, T, r, sig, "call")
    out = implied_vol(S, K, T, r, px, "CALL")
    assert out is not None
    assert abs(out - sig) < 1e-4


def test_implied_vol_time_non_positive_returns_none() -> None:
    assert implied_vol(100, 100, 0.0, 0.05, 5.0, "call") is None
    assert implied_vol(100, 100, -0.1, 0.05, 5.0, "call") is None


def test_implied_vol_invalid_spot_strike() -> None:
    assert implied_vol(0, 100, 0.25, 0.05, 1.0, "call") is None
    assert implied_vol(100, 0, 0.25, 0.05, 1.0, "call") is None


def test_implied_vol_invalid_option_type() -> None:
    assert implied_vol(100, 100, 0.25, 0.05, 5.0, "straddle") is None


def test_implied_vol_price_above_arbitrage_ceiling() -> None:
    S, K, T, r = 100.0, 100.0, 30 / 365.0, 0.05
    assert implied_vol(S, K, T, r, S + 1.0, "call") is None


def test_implied_vol_price_below_arbitrage_floor() -> None:
    S, K, T, r = 100.0, 100.0, 30 / 365.0, 0.05
    disc = math.exp(-r * T)
    floor = max(0.0, S - K * disc)
    assert implied_vol(S, K, T, r, floor - 0.05, "call") is None


def test_implied_vol_price_outside_sigma_bracket_returns_none() -> None:
    """Precio que requeriría sigma > 5 (fuera del rango de bisección)."""
    S, K, T, r = 100.0, 100.0, 2.0, 0.05  # T grande en años
    p_cap = bs_price(S, K, T, r, 5.0, "call")
    assert implied_vol(S, K, T, r, p_cap + 0.5, "call", tol=1e-9) is None


def test_implied_vol_at_sigma_low_boundary() -> None:
    S, K, T, r = 100.0, 100.0, 30 / 365.0, 0.05
    px = bs_price(S, K, T, r, 1e-4, "call")
    out = implied_vol(S, K, T, r, px, "call", tol=1e-9)
    assert out is not None
    assert abs(out - 1e-4) < 1e-3

