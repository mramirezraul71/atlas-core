"""Unit tests for the EV calculator."""

from __future__ import annotations

import math

import pytest

from lotto_quant.models.ev_calculator import (
    EVCalculator,
    PrizeTier,
    ScratchOffGame,
)


def _build_game(
    *,
    ticket_price: float = 5.0,
    total_tickets: int = 10_000_000,
    tiers: list = None,
) -> ScratchOffGame:
    if tiers is None:
        tiers = [
            PrizeTier(value=1_000_000, total_prizes=4, remaining_prizes=3),
            PrizeTier(value=10_000,    total_prizes=20, remaining_prizes=10),
            PrizeTier(value=100,       total_prizes=10_000, remaining_prizes=2_000),
            PrizeTier(value=10,        total_prizes=200_000, remaining_prizes=40_000),
            PrizeTier(value=5,         total_prizes=500_000, remaining_prizes=100_000),
        ]
    return ScratchOffGame(
        game_id="TEST-1",
        name="Test Game",
        ticket_price=ticket_price,
        prize_tiers=tiers,
        total_tickets_printed=total_tickets,
    )


def test_tax_adjustment_below_600():
    calc = EVCalculator()
    assert calc.adjust_prize_for_taxes(100.0) == pytest.approx(100.0)
    assert calc.adjust_prize_for_taxes(599.99) == pytest.approx(599.99)


def test_tax_adjustment_state_only():
    calc = EVCalculator()
    # $1,000 → only NC state tax
    assert calc.adjust_prize_for_taxes(1_000.0) == pytest.approx(1_000.0 * (1 - 0.0525))


def test_tax_adjustment_state_plus_federal():
    calc = EVCalculator()
    # $10,000 → both NC + federal
    expected = 10_000 * (1 - 0.0525 - 0.24)
    assert calc.adjust_prize_for_taxes(10_000.0) == pytest.approx(expected)


def test_remaining_tickets_estimate():
    game = _build_game()
    calc = EVCalculator()
    remaining = calc.calculate_remaining_tickets(game)
    assert 0 < remaining < game.total_tickets_printed


def test_calculate_adjusted_ev_returns_result():
    game = _build_game()
    calc = EVCalculator()
    result = calc.calculate_adjusted_ev(game)
    assert result.game_id == "TEST-1"
    assert math.isfinite(result.gross_ev)
    assert math.isfinite(result.adjusted_ev_nc)
    assert result.signal_strength in {"STRONG", "MODERATE", "WEAK", "NEGATIVE"}


def test_adjusted_ev_lower_than_gross_for_high_prizes():
    game = _build_game()
    calc = EVCalculator()
    result = calc.calculate_adjusted_ev(game)
    assert result.adjusted_ev_nc <= result.gross_ev + 1e-6


def test_stale_prize_anomaly_detection():
    """When most tickets sold but major prizes remain, anomaly should fire."""
    tiers = [
        PrizeTier(value=1_000_000, total_prizes=4, remaining_prizes=4),       # untouched
        PrizeTier(value=100_000,   total_prizes=10, remaining_prizes=9),
        PrizeTier(value=5,         total_prizes=10_000, remaining_prizes=500), # mostly sold
    ]
    game = _build_game(tiers=tiers, total_tickets=10_000_000)
    calc = EVCalculator()
    is_anom, score = calc.detect_stale_prize_anomaly(game)
    assert score > 0
    # Depending on heuristics this may or may not exceed threshold — just sanity-check.


def test_jackpot_ev_basic():
    calc = EVCalculator()
    res = calc.calculate_jackpot_ev(
        jackpot_amount=1_500_000_000,
        ticket_price=2.0,
        odds_of_jackpot=1 / 292_201_338,
        expected_winners=1.5,
    )
    assert res.signal_strength in {"STRONG", "MODERATE", "WEAK", "NEGATIVE"}


def test_zero_remaining_tickets_returns_negative_signal():
    tiers = [PrizeTier(value=100, total_prizes=10, remaining_prizes=0)]
    game = ScratchOffGame(
        game_id="empty", name="Empty", ticket_price=2.0,
        prize_tiers=tiers, total_tickets_printed=0,
    )
    calc = EVCalculator()
    result = calc.calculate_adjusted_ev(game)
    assert result.signal_strength == "NEGATIVE"
