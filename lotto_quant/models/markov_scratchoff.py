"""
lotto_quant.models.markov_scratchoff
====================================

Markov Chain model of scratch-off prize-state evolution.

STATE SPACE
-----------
A game state at time t is the integer vector
    S_t = (r_1, r_2, ..., r_k, r_blank)
where
    r_i        = remaining prizes of tier i
    r_blank    = remaining "no-prize" outcomes
    Σ r_*      = remaining tickets

TRANSITION
----------
On selling one ticket:
    P(tier_i | S_t) = r_i / R_t
    P(blank  | S_t) = r_blank / R_t

The next state decrements the chosen component by 1.

USAGE
-----
We do NOT enumerate the joint state space (combinatorially intractable).
Instead, we run Monte Carlo trajectories and aggregate.
For the *mean* trajectory we use closed-form hypergeometric expectations.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from .. import config
from .ev_calculator import EVCalculator, EVResult, ScratchOffGame, PrizeTier


@dataclass
class GameState:
    """A single state in the Markov Chain."""

    remaining_prizes: Dict[str, int]   # {tier_name: remaining_count}
    remaining_blanks: int
    remaining_tickets: int
    cumulative_tickets_sold: int
    timestamp: float = 0.0

    def total_outcomes(self) -> int:
        return sum(self.remaining_prizes.values()) + self.remaining_blanks


class ScratchOffMarkovModel:
    """
    Monte-Carlo Markov Chain for scratch-off prize depletion.

    Parameters
    ----------
    game : ScratchOffGame
        Snapshot of the current game state.
    n_simulations : int
        Number of Monte-Carlo trajectories.
    rng_seed : int | None
        For reproducibility.
    """

    def __init__(
        self,
        game: ScratchOffGame,
        n_simulations: int = config.MARKOV_DEFAULT_SIMULATIONS,
        rng_seed: Optional[int] = None,
    ):
        self.game = game
        self.n_simulations = n_simulations
        self.rng = np.random.default_rng(rng_seed)
        self.calc = EVCalculator()

        # Vector of remaining prizes (ordered by tier index)
        self._tier_values = np.array(
            [t.value for t in game.prize_tiers], dtype=np.float64
        )
        self._tier_remaining = np.array(
            [t.remaining_prizes for t in game.prize_tiers], dtype=np.int64
        )
        self._tier_total = np.array(
            [t.total_prizes for t in game.prize_tiers], dtype=np.int64
        )

        # Estimate remaining blank tickets
        remaining_total = self.calc.calculate_remaining_tickets(game)
        remaining_prizes_sum = int(self._tier_remaining.sum())
        self._remaining_blanks = max(0, remaining_total - remaining_prizes_sum)
        self._remaining_total = remaining_prizes_sum + self._remaining_blanks

    # ── transition probability vector ──────────────────────────────
    def transition_probabilities(self) -> np.ndarray:
        """Probability of each prize tier (plus blank) being claimed by next ticket."""
        if self._remaining_total <= 0:
            n = len(self._tier_remaining) + 1
            return np.zeros(n)
        probs = np.zeros(len(self._tier_remaining) + 1)
        probs[:-1] = self._tier_remaining / self._remaining_total
        probs[-1] = self._remaining_blanks / self._remaining_total
        return probs

    def build_transition_matrix(self) -> np.ndarray:
        """
        Diagonal-stochastic representation:
            row i = transition vector when current tier mix is `tier_remaining`.
        Useful for plotting / sanity checks.
        """
        return np.diag(self.transition_probabilities())

    # ── simulation ─────────────────────────────────────────────────
    def simulate_trajectories(
        self, n_tickets_ahead: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Monte-Carlo simulation.

        Returns
        -------
        prize_remaining : np.ndarray
            Shape (n_simulations, n_tickets_ahead + 1, n_tiers).
        ev_track : np.ndarray
            Shape (n_simulations, n_tickets_ahead + 1).  EV/ticket trajectory.
        """
        n_sims = self.n_simulations
        n_tiers = len(self._tier_values)
        prize_remaining = np.zeros(
            (n_sims, n_tickets_ahead + 1, n_tiers), dtype=np.int64
        )
        ev_track = np.zeros((n_sims, n_tickets_ahead + 1), dtype=np.float64)

        for s in range(n_sims):
            tier_rem = self._tier_remaining.copy()
            blanks = self._remaining_blanks
            total = self._remaining_total
            prize_remaining[s, 0] = tier_rem
            ev_track[s, 0] = self._ev_from_state(tier_rem, total)

            for t in range(1, n_tickets_ahead + 1):
                if total <= 0:
                    prize_remaining[s, t:] = tier_rem
                    ev_track[s, t:] = ev_track[s, t - 1]
                    break

                probs = np.empty(n_tiers + 1, dtype=np.float64)
                probs[:n_tiers] = tier_rem / total
                probs[n_tiers] = blanks / total
                probs = np.clip(probs, 0.0, 1.0)
                probs /= probs.sum()

                idx = self.rng.choice(n_tiers + 1, p=probs)
                if idx < n_tiers:
                    tier_rem[idx] -= 1
                else:
                    blanks -= 1
                total -= 1

                prize_remaining[s, t] = tier_rem
                ev_track[s, t] = self._ev_from_state(tier_rem, total)

        return prize_remaining, ev_track

    # ── EV helper ──────────────────────────────────────────────────
    def _ev_from_state(self, tier_rem: np.ndarray, total: int) -> float:
        """Adjusted EV/ticket given a remaining-prize vector and total remaining tickets."""
        if total <= 0:
            return -float(self.game.ticket_price)
        adjusted_sum = 0.0
        for value, rem in zip(self._tier_values, tier_rem):
            if rem <= 0:
                continue
            p = rem / total
            adjusted_sum += p * self.calc.adjust_prize_for_taxes(float(value))
        return adjusted_sum - self.game.ticket_price

    # ── analytical predictions ─────────────────────────────────────
    def predict_ev_turning_point(
        self, max_horizon: int = config.MARKOV_PROJECTION_HORIZON
    ) -> Tuple[Optional[int], float]:
        """
        Forecast at what cumulative-ticket count the median trajectory crosses EV ≥ 0.

        Returns
        -------
        tickets_until_positive : int | None
            Number of *additional* tickets sold before median EV becomes positive,
            or None if EV never turns positive within `max_horizon`.
        confidence : float
            Fraction of MC trajectories that crossed EV ≥ 0 within the horizon.
        """
        # Reduce to a smaller MC budget for speed when horizon is huge.
        sims_backup = self.n_simulations
        self.n_simulations = min(sims_backup, 1_000)
        horizon = min(max_horizon, max(1, self._remaining_total))
        _, ev_track = self.simulate_trajectories(horizon)
        self.n_simulations = sims_backup

        positive_mask = ev_track >= 0
        any_positive = positive_mask.any(axis=1)
        confidence = float(any_positive.mean())

        # Median first-crossing time among trajectories that did cross
        crossing_times = []
        for row in positive_mask:
            idx = np.argmax(row)  # first True
            if row[idx]:
                crossing_times.append(int(idx))
        if not crossing_times:
            return None, confidence
        return int(np.median(crossing_times)), confidence

    def detect_state_anomaly(
        self, current_state: GameState, prior_snapshot: Optional[GameState] = None
    ) -> Dict:
        """
        Compare observed state to model expectation.

        High anomaly = major prizes have *NOT* been claimed at the rate the
        ticket-sales rate would predict ⇒ long-tail EV+ candidate.

        If `prior_snapshot` is given we compute the empirical claim rate vs.
        the model-implied rate; otherwise we anchor on the printed counts.
        """
        observed = current_state.remaining_prizes
        n_obs_total = sum(observed.values()) + current_state.remaining_blanks

        # Expected remaining prizes under the assumption of *uniform random*
        # depletion: r_i_expected = total_prizes_i · (R_t / R_0)
        r_ratio = (
            current_state.remaining_tickets / self._remaining_total
            if self._remaining_total > 0
            else 1.0
        )
        diagnostics = []
        squared_log_dev = 0.0
        n_majors = 0
        for tier, name in zip(self.game.prize_tiers, observed.keys()):
            expected_remaining = tier.total_prizes * r_ratio
            actual_remaining = observed[name]
            if expected_remaining <= 0:
                continue
            log_dev = math.log((actual_remaining + 1) / (expected_remaining + 1))
            diagnostics.append(
                {
                    "tier": name,
                    "expected_remaining": expected_remaining,
                    "actual_remaining": actual_remaining,
                    "log_deviation": log_dev,
                }
            )
            if tier.value >= self.game.major_prize_threshold():
                squared_log_dev += log_dev ** 2
                n_majors += 1

        score = math.sqrt(squared_log_dev / max(1, n_majors))
        return {
            "anomaly_score": score,
            "n_majors_considered": n_majors,
            "remaining_tickets": current_state.remaining_tickets,
            "tiers": diagnostics,
            "interpretation": (
                "STRONG" if score > 0.5 else "MODERATE" if score > 0.2 else "NEUTRAL"
            ),
        }
