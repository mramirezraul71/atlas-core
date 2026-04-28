"""Unit tests for the Markov scratch-off model."""

from __future__ import annotations

import numpy as np
import pytest

from lotto_quant.models.ev_calculator import PrizeTier, ScratchOffGame
from lotto_quant.models.markov_scratchoff import (
    GameState,
    ScratchOffMarkovModel,
)


@pytest.fixture
def small_game() -> ScratchOffGame:
    tiers = [
        PrizeTier(value=1_000, total_prizes=10,  remaining_prizes=8),
        PrizeTier(value=100,   total_prizes=100, remaining_prizes=60),
        PrizeTier(value=10,    total_prizes=500, remaining_prizes=200),
    ]
    return ScratchOffGame(
        game_id="MK-1",
        name="Markov Test",
        ticket_price=2.0,
        prize_tiers=tiers,
        total_tickets_printed=5_000,
    )


def test_transition_probabilities_sum_to_one(small_game):
    model = ScratchOffMarkovModel(small_game, n_simulations=10, rng_seed=0)
    p = model.transition_probabilities()
    assert p.shape[0] == len(small_game.prize_tiers) + 1
    assert pytest.approx(p.sum(), rel=1e-6) == 1.0


def test_simulate_trajectories_shape(small_game):
    model = ScratchOffMarkovModel(small_game, n_simulations=20, rng_seed=42)
    horizon = 50
    prize_remaining, ev_track = model.simulate_trajectories(horizon)
    n_tiers = len(small_game.prize_tiers)
    assert prize_remaining.shape == (20, horizon + 1, n_tiers)
    assert ev_track.shape == (20, horizon + 1)
    # Remaining prizes should monotonically decrease over time per simulation
    diffs = np.diff(prize_remaining, axis=1)
    assert (diffs <= 0).all()


def test_predict_ev_turning_point_returns_tuple(small_game):
    model = ScratchOffMarkovModel(small_game, n_simulations=200, rng_seed=1)
    tickets, conf = model.predict_ev_turning_point(max_horizon=200)
    assert (tickets is None) or isinstance(tickets, int)
    assert 0.0 <= conf <= 1.0


def test_state_anomaly_returns_diagnostic_dict(small_game):
    model = ScratchOffMarkovModel(small_game, n_simulations=10, rng_seed=0)
    observed = GameState(
        remaining_prizes={"a": 8, "b": 60, "c": 200},
        remaining_blanks=1_000,
        remaining_tickets=1_268,
        cumulative_tickets_sold=3_732,
    )
    out = model.detect_state_anomaly(observed)
    assert "anomaly_score" in out
    assert "interpretation" in out
    assert out["interpretation"] in {"STRONG", "MODERATE", "NEUTRAL"}
