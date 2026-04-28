"""
lotto_quant.models.ev_calculator
================================

Core Expected-Value engine for NCEL scratch-offs and draw games.

MATHEMATICAL FOUNDATION
-----------------------

    Gross EV = Σ ( P(win_i) · prize_i )  -  ticket_price

    P(win_i) = remaining_prizes_i / remaining_tickets

    remaining_tickets ≈ total_tickets · ( 1 - depletion_ratio )

Adjusted prize after withholding:
    prize >= $5,000           → prize · (1 − NC_TAX − FED_TAX)
    $600 ≤ prize < $5,000     → prize · (1 − NC_TAX)
    prize < $600              → prize  (no withholding)

Stale-Prize anomaly condition:
    depletion_ratio > 0.75  AND  major_prize_retention > 0.80
    → real_odds  ≫  stated_odds  →  EV+ candidate.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .. import config


# ─────────────────────────────────────────────────────────────────────
# Data classes
# ─────────────────────────────────────────────────────────────────────
@dataclass
class PrizeTier:
    """A single prize level inside a scratch-off game."""

    value: float
    total_prizes: int
    remaining_prizes: int
    odds_initial: Optional[float] = None  # 1 / odds  (probability)

    def claimed(self) -> int:
        return max(0, self.total_prizes - self.remaining_prizes)

    def claim_ratio(self) -> float:
        if self.total_prizes <= 0:
            return 0.0
        return self.claimed() / self.total_prizes

    def remaining_ratio(self) -> float:
        if self.total_prizes <= 0:
            return 0.0
        return self.remaining_prizes / self.total_prizes


@dataclass
class ScratchOffGame:
    """Represents a NCEL scratch-off ticket family."""

    game_id: str
    name: str
    ticket_price: float
    prize_tiers: List[PrizeTier]
    total_tickets_printed: int = 0
    tickets_sold_estimate: int = 0
    snapshot_ts: Optional[str] = None  # ISO timestamp, set by scraper

    def major_prize_threshold(self) -> float:
        """Return the dollar value used to classify 'major' prizes."""
        # Heuristic: top-30% of prize values, or >=$1,000, whichever is smaller.
        values = sorted([t.value for t in self.prize_tiers], reverse=True)
        if not values:
            return float("inf")
        cutoff_idx = max(0, int(len(values) * 0.3))
        cutoff = values[cutoff_idx] if cutoff_idx < len(values) else values[-1]
        return min(cutoff, 1_000.0)

    def major_prizes(self) -> List[PrizeTier]:
        threshold = self.major_prize_threshold()
        return [t for t in self.prize_tiers if t.value >= threshold]


@dataclass
class EVResult:
    """Output of an EV calculation for a single game."""

    game_id: str
    game_name: str
    ticket_price: float
    gross_ev: float                  # Raw EV before taxes (per ticket, $)
    adjusted_ev_nc: float            # After NC + federal taxes (per ticket, $)
    ev_per_dollar: float             # adjusted_ev_nc / ticket_price
    signal_strength: str             # "STRONG" | "MODERATE" | "WEAK" | "NEGATIVE"
    depletion_ratio: float
    major_prize_retention: float
    anomaly_score: float = 0.0
    is_anomaly: bool = False
    diagnostics: dict = field(default_factory=dict)


# ─────────────────────────────────────────────────────────────────────
# Calculator
# ─────────────────────────────────────────────────────────────────────
class EVCalculator:
    """Core EV engine for NCEL scratch-off + draw game analysis."""

    def __init__(
        self,
        nc_tax: float = config.NC_STATE_TAX_RATE,
        federal_tax: float = config.FEDERAL_TAX_RATE,
        federal_threshold: float = config.FEDERAL_WITHHOLDING_THRESHOLD,
        small_prize_threshold: float = config.SMALL_PRIZE_THRESHOLD,
    ):
        self.nc_tax = nc_tax
        self.federal_tax = federal_tax
        self.federal_threshold = federal_threshold
        self.small_prize_threshold = small_prize_threshold

    # ── tax adjustment ─────────────────────────────────────────────
    def adjust_prize_for_taxes(self, prize: float) -> float:
        """Apply NC + Federal withholding to a prize amount."""
        if prize < self.small_prize_threshold:
            return prize
        if prize < self.federal_threshold:
            return prize * (1.0 - self.nc_tax)
        return prize * (1.0 - self.nc_tax - self.federal_tax)

    # ── ticket-sold estimation ─────────────────────────────────────
    def calculate_remaining_tickets(self, game: ScratchOffGame) -> int:
        """
        Estimate remaining tickets using prize-tier depletion.

        Method
        ------
        For each tier, the implied tickets-sold count is
            sold_i = total_tickets_printed · (claimed_i / total_prizes_i)

        We average across tiers weighted by total_prizes_i (more granular tiers
        carry more statistical weight).
        """
        if game.total_tickets_printed <= 0 or not game.prize_tiers:
            return 0

        weighted_sold = 0.0
        weight_sum = 0.0
        for tier in game.prize_tiers:
            if tier.total_prizes <= 0:
                continue
            implied_sold = game.total_tickets_printed * tier.claim_ratio()
            weight = tier.total_prizes
            weighted_sold += implied_sold * weight
            weight_sum += weight

        if weight_sum == 0:
            return game.total_tickets_printed  # nothing to estimate from

        sold = int(weighted_sold / weight_sum)
        sold = max(0, min(sold, game.total_tickets_printed))
        return game.total_tickets_printed - sold

    def depletion_ratio(self, game: ScratchOffGame) -> float:
        """Fraction of printed tickets that have been sold (estimate)."""
        if game.total_tickets_printed <= 0:
            return 0.0
        remaining = self.calculate_remaining_tickets(game)
        return 1.0 - (remaining / game.total_tickets_printed)

    def major_prize_retention(self, game: ScratchOffGame) -> float:
        """Fraction of *major* prizes still unclaimed."""
        majors = game.major_prizes()
        if not majors:
            return 0.0
        total = sum(t.total_prizes for t in majors)
        remaining = sum(t.remaining_prizes for t in majors)
        if total <= 0:
            return 0.0
        return remaining / total

    # ── core EV ────────────────────────────────────────────────────
    def calculate_adjusted_ev(self, game: ScratchOffGame) -> EVResult:
        """
        Compute gross + tax-adjusted EV plus anomaly classification.
        Returns an EVResult suitable for ranking, persistence, and alerting.
        """
        if game.ticket_price <= 0:
            raise ValueError(f"Ticket price must be > 0 (game={game.game_id})")

        remaining_tickets = self.calculate_remaining_tickets(game)
        if remaining_tickets <= 0:
            # Game is over or no data — return a NEGATIVE / unusable signal.
            return EVResult(
                game_id=game.game_id,
                game_name=game.name,
                ticket_price=game.ticket_price,
                gross_ev=-game.ticket_price,
                adjusted_ev_nc=-game.ticket_price,
                ev_per_dollar=-1.0,
                signal_strength="NEGATIVE",
                depletion_ratio=1.0,
                major_prize_retention=0.0,
                diagnostics={"reason": "no_remaining_tickets"},
            )

        gross_sum = 0.0
        adjusted_sum = 0.0
        for tier in game.prize_tiers:
            if tier.remaining_prizes <= 0:
                continue
            p_win = tier.remaining_prizes / remaining_tickets
            gross_sum += p_win * tier.value
            adjusted_sum += p_win * self.adjust_prize_for_taxes(tier.value)

        gross_ev = gross_sum - game.ticket_price
        adjusted_ev = adjusted_sum - game.ticket_price
        ev_per_dollar = adjusted_ev / game.ticket_price

        depletion = self.depletion_ratio(game)
        major_retention = self.major_prize_retention(game)
        is_anomaly, anomaly_strength = self.detect_stale_prize_anomaly(game)
        signal = self._classify_signal(ev_per_dollar, is_anomaly)

        return EVResult(
            game_id=game.game_id,
            game_name=game.name,
            ticket_price=game.ticket_price,
            gross_ev=gross_ev,
            adjusted_ev_nc=adjusted_ev,
            ev_per_dollar=ev_per_dollar,
            signal_strength=signal,
            depletion_ratio=depletion,
            major_prize_retention=major_retention,
            anomaly_score=anomaly_strength,
            is_anomaly=is_anomaly,
            diagnostics={
                "remaining_tickets_est": remaining_tickets,
                "n_prize_tiers": len(game.prize_tiers),
                "gross_sum": gross_sum,
                "adjusted_sum": adjusted_sum,
            },
        )

    # ── anomaly detection ──────────────────────────────────────────
    def detect_stale_prize_anomaly(
        self, game: ScratchOffGame
    ) -> Tuple[bool, float]:
        """
        Returns (is_anomaly, anomaly_score).

        anomaly_score = depletion_ratio / max(epsilon, 1 - major_retention)

        anomaly is True when
            depletion_ratio > MIN_PRIZE_DEPLETION_RATIO
            AND major_retention > MIN_MAJOR_PRIZE_REMAINING_RATIO
        """
        depletion = self.depletion_ratio(game)
        retention = self.major_prize_retention(game)
        denom = max(1e-6, 1.0 - retention)
        score = depletion / denom

        is_anom = (
            depletion > config.MIN_PRIZE_DEPLETION_RATIO
            and retention > config.MIN_MAJOR_PRIZE_REMAINING_RATIO
        )
        return is_anom, score

    # ── jackpot (draw games) ───────────────────────────────────────
    def calculate_jackpot_ev(
        self,
        jackpot_amount: float,
        ticket_price: float,
        odds_of_jackpot: float,
        expected_winners: float = 1.0,
        secondary_prizes_ev: float = 0.0,
    ) -> EVResult:
        """
        Draw-game EV with splitting risk.

        EV_jackpot = (jackpot / max(1, expected_winners)) · p_jackpot · (1 − tax)
                   + secondary_prizes_ev
                   − ticket_price

        where p_jackpot = odds_of_jackpot (assumed = 1/N as decimal probability).
        """
        if ticket_price <= 0:
            raise ValueError("ticket_price must be > 0")

        share = jackpot_amount / max(1.0, expected_winners)
        # Big prizes always cross both NC and federal thresholds.
        net_jackpot = share * (1.0 - self.nc_tax - self.federal_tax)
        gross_jackpot_ev = jackpot_amount * odds_of_jackpot - ticket_price
        adjusted_ev = (
            net_jackpot * odds_of_jackpot + secondary_prizes_ev - ticket_price
        )
        ev_per_dollar = adjusted_ev / ticket_price
        signal = self._classify_signal(ev_per_dollar, False)

        return EVResult(
            game_id="DRAW",
            game_name="Draw Game",
            ticket_price=ticket_price,
            gross_ev=gross_jackpot_ev,
            adjusted_ev_nc=adjusted_ev,
            ev_per_dollar=ev_per_dollar,
            signal_strength=signal,
            depletion_ratio=0.0,
            major_prize_retention=1.0,
            diagnostics={
                "jackpot": jackpot_amount,
                "expected_winners": expected_winners,
                "share_per_winner": share,
                "secondary_ev": secondary_prizes_ev,
            },
        )

    # ── helpers ────────────────────────────────────────────────────
    @staticmethod
    def _classify_signal(ev_per_dollar: float, is_anomaly: bool) -> str:
        if math.isnan(ev_per_dollar):
            return "NEGATIVE"
        if ev_per_dollar >= config.MIN_EV_NET_STRONG and is_anomaly:
            return "STRONG"
        if ev_per_dollar >= config.MIN_EV_NET_STRONG:
            return "MODERATE"
        if ev_per_dollar >= config.MIN_EV_NET_POSITIVE:
            return "WEAK"
        return "NEGATIVE"
