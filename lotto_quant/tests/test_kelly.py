"""Unit tests for the Kelly allocator."""

from __future__ import annotations

import pytest

from lotto_quant.models.ev_calculator import (
    EVCalculator,
    EVResult,
    PrizeTier,
    ScratchOffGame,
)
from lotto_quant.models.kelly_allocator import KellyAllocator


def _ev_result(adjusted: float = 0.10, ticket_price: float = 5.0) -> EVResult:
    return EVResult(
        game_id="K-1",
        game_name="Kelly Test",
        ticket_price=ticket_price,
        gross_ev=adjusted + 0.02,
        adjusted_ev_nc=adjusted,
        ev_per_dollar=adjusted / ticket_price,
        signal_strength="MODERATE" if adjusted > 0 else "NEGATIVE",
        depletion_ratio=0.8,
        major_prize_retention=0.85,
        anomaly_score=4.0 if adjusted > 0 else 0.0,
        is_anomaly=adjusted > 0,
    )


def test_negative_ev_returns_zero_position():
    kelly = KellyAllocator()
    rec = kelly.calculate_lottery_kelly(
        ev_result=_ev_result(adjusted=-0.5),
        bankroll=10_000,
    )
    assert rec.recommended_position == 0
    assert rec.capped_by == "negative_ev"


def test_positive_ev_returns_capped_position():
    kelly = KellyAllocator()
    game = ScratchOffGame(
        game_id="K-1",
        name="Kelly Test",
        ticket_price=5.0,
        prize_tiers=[
            PrizeTier(value=1_000_000, total_prizes=4, remaining_prizes=4),
            PrizeTier(value=100, total_prizes=10_000, remaining_prizes=2_000),
        ],
        total_tickets_printed=10_000_000,
    )
    rec = kelly.calculate_lottery_kelly(
        ev_result=_ev_result(adjusted=0.5),
        bankroll=10_000,
        game=game,
    )
    assert rec.recommended_position >= 0
    # MAX_POSITION_PCT = 2% of $10,000 = $200
    assert rec.recommended_position <= 200.01


def test_exposure_cap_applied():
    kelly = KellyAllocator()
    game = ScratchOffGame(
        game_id="K-1",
        name="Kelly Test",
        ticket_price=5.0,
        prize_tiers=[PrizeTier(value=1_000_000, total_prizes=4, remaining_prizes=4)],
        total_tickets_printed=10_000_000,
    )
    rec = kelly.calculate_lottery_kelly(
        ev_result=_ev_result(adjusted=0.5),
        bankroll=10_000,
        current_total_exposure=950,  # 9.5% already in
        game=game,
    )
    # Only 0.5% of bankroll = $50 left within exposure cap
    assert rec.recommended_position <= 50.01
