"""
lotto_quant.models.kelly_allocator
==================================

Kelly Criterion bankroll sizing with **lottery-specific** safeguards:

    f*           = (p · b − q) / b              (binary Kelly)
    f_atlas      = KELLY_FRACTION · f*          (Quarter-Kelly default)
    f_lot        = f_atlas / (1 + variance_factor)
    f_final      = min(f_lot, MAX_POSITION_PCT) AND
                   sum(positions) ≤ MAX_TOTAL_EXPOSURE_PCT

`variance_factor` is the coefficient of variation of prize values
(σ/μ over remaining prize tiers). Lotteries have CoVar > 100 typically,
so the variance penalty drives the allocation toward zero unless the
EV signal is exceptionally strong.

For multi-outcome scratch-offs we use a generalized Kelly:

    f* = sum_i  ( p_i / b_i )  with  b_i = prize_i / ticket_price − 1
    bounded by 0 ≤ f* ≤ 1.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Optional

import numpy as np

from .. import config

if TYPE_CHECKING:  # avoid circular imports at runtime
    from .ev_calculator import EVResult, ScratchOffGame


@dataclass
class KellyRecommendation:
    game_id: str
    game_name: str
    bankroll: float
    full_kelly: float
    fractional_kelly: float
    variance_penalty: float
    final_fraction: float
    recommended_position: float
    n_tickets: int
    capped_by: str        # 'kelly' | 'max_position' | 'exposure_cap' | 'negative_ev'
    diagnostics: Dict = field(default_factory=dict)


class KellyAllocator:
    """Kelly sizing for lottery-style EV signals."""

    def __init__(
        self,
        kelly_fraction: float = config.KELLY_FRACTION,
        max_position_pct: float = config.MAX_POSITION_PCT,
        max_exposure_pct: float = config.MAX_TOTAL_EXPOSURE_PCT,
    ):
        self.kelly_fraction = kelly_fraction
        self.max_position_pct = max_position_pct
        self.max_exposure_pct = max_exposure_pct

    # ── core Kelly maths ───────────────────────────────────────────
    @staticmethod
    def binary_kelly(p_win: float, b_net_odds: float) -> float:
        if b_net_odds <= 0 or p_win <= 0 or p_win >= 1:
            return 0.0
        q = 1.0 - p_win
        return (p_win * b_net_odds - q) / b_net_odds

    @staticmethod
    def multi_outcome_kelly(
        win_probs: np.ndarray, prize_values: np.ndarray, ticket_price: float
    ) -> float:
        """
        Approximate multi-outcome Kelly for lottery:
            f* = ev_per_dollar / variance_per_dollar
        which collapses to binary Kelly for two-outcome bets.
        """
        if ticket_price <= 0:
            return 0.0
        net_returns = prize_values / ticket_price - 1.0
        loss_prob = 1.0 - float(win_probs.sum())
        ev_per_dollar = float((win_probs * net_returns).sum()) - loss_prob
        var_per_dollar = float((win_probs * (net_returns ** 2)).sum()) + loss_prob
        if var_per_dollar <= 0:
            return 0.0
        return max(0.0, ev_per_dollar / var_per_dollar)

    # ── variance penalty ───────────────────────────────────────────
    @staticmethod
    def variance_factor(prize_values: np.ndarray, weights: np.ndarray) -> float:
        if prize_values.size == 0 or weights.sum() <= 0:
            return 0.0
        w = weights / weights.sum()
        mean = float((w * prize_values).sum())
        if mean <= 0:
            return 0.0
        var = float((w * (prize_values - mean) ** 2).sum())
        std = math.sqrt(var)
        return std / mean

    # ── main entry point ───────────────────────────────────────────
    def calculate_lottery_kelly(
        self,
        ev_result: "EVResult",
        bankroll: float,
        current_total_exposure: float = 0.0,
        game: Optional["ScratchOffGame"] = None,
    ) -> KellyRecommendation:
        """
        Return a KellyRecommendation given an EVResult and a game snapshot.
        """
        if bankroll <= 0:
            raise ValueError("bankroll must be > 0")

        # ── Negative EV → no allocation ──────────────────────────
        if ev_result.adjusted_ev_nc <= 0:
            return KellyRecommendation(
                game_id=ev_result.game_id,
                game_name=ev_result.game_name,
                bankroll=bankroll,
                full_kelly=0.0,
                fractional_kelly=0.0,
                variance_penalty=0.0,
                final_fraction=0.0,
                recommended_position=0.0,
                n_tickets=0,
                capped_by="negative_ev",
                diagnostics={"ev_per_dollar": ev_result.ev_per_dollar},
            )

        # ── Build prize / probability vectors from game snapshot ──
        if game is not None and game.prize_tiers:
            remaining_total = max(
                1, sum(t.remaining_prizes for t in game.prize_tiers) + 1
            )
            prize_values = np.array(
                [t.value for t in game.prize_tiers], dtype=np.float64
            )
            win_probs = np.array(
                [t.remaining_prizes / remaining_total for t in game.prize_tiers],
                dtype=np.float64,
            )
            full_kelly = self.multi_outcome_kelly(
                win_probs, prize_values, ev_result.ticket_price
            )
            var_factor = self.variance_factor(
                prize_values, win_probs * remaining_total
            )
        else:
            # Approximate using EV-per-dollar + small fixed variance assumption
            ev_pd = ev_result.ev_per_dollar
            var_per_dollar = max(0.5, ev_pd ** 2 + 1.0)
            full_kelly = max(0.0, ev_pd / var_per_dollar)
            var_factor = 1.0  # unknown — apply moderate penalty

        fractional = self.kelly_fraction * full_kelly
        penalized = fractional / (1.0 + var_factor)

        # ── Apply caps ───────────────────────────────────────────
        cap_reason = "kelly"
        final = penalized
        if final > self.max_position_pct:
            final = self.max_position_pct
            cap_reason = "max_position"

        remaining_exposure = max(0.0, self.max_exposure_pct - current_total_exposure / bankroll)
        if final > remaining_exposure:
            final = remaining_exposure
            cap_reason = "exposure_cap"

        position = round(final * bankroll, 2)
        n_tickets = int(position // ev_result.ticket_price) if ev_result.ticket_price > 0 else 0
        # Don't return fractional ticket allocations
        position = round(n_tickets * ev_result.ticket_price, 2)
        if n_tickets == 0:
            cap_reason = "below_one_ticket"
            final = 0.0

        return KellyRecommendation(
            game_id=ev_result.game_id,
            game_name=ev_result.game_name,
            bankroll=bankroll,
            full_kelly=full_kelly,
            fractional_kelly=fractional,
            variance_penalty=var_factor,
            final_fraction=final,
            recommended_position=position,
            n_tickets=n_tickets,
            capped_by=cap_reason,
            diagnostics={
                "ev_per_dollar": ev_result.ev_per_dollar,
                "signal_strength": ev_result.signal_strength,
                "current_total_exposure": current_total_exposure,
            },
        )
