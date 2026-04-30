"""
lotto_quant.models.jackpot_simulator
====================================

Monte-Carlo simulator for draw-game (Powerball / Mega Millions) jackpot EV
including jackpot-splitting risk.

KEY EQUATIONS
-------------

    p_jackpot          = 1 / N_combinations
    expected_winners   = n_tickets_sold * p_jackpot
    share_per_winner   = jackpot / max(1, expected_winners)
    EV_jackpot_net     = share_per_winner * (1 - tax) * p_jackpot
    EV_total           = EV_jackpot_net + Σ EV_secondary - ticket_price
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from .. import config
from .ev_calculator import EVCalculator, EVResult


@dataclass
class JackpotEVResult:
    game_name: str
    jackpot: float
    expected_winners: float
    share_per_winner: float
    ev_per_ticket_gross: float
    ev_per_ticket_adjusted: float
    ev_per_dollar: float
    signal_strength: str
    monte_carlo_breakeven_prob: float
    diagnostics: Dict = field(default_factory=dict)


# Standard Powerball secondary prize structure (post-2015 rules)
POWERBALL_SECONDARY = [
    # (probability_per_ticket, prize_value)
    (1 / 11_688_053.52, 1_000_000),    # 5 white
    (1 / 913_129.18,    50_000),       # 4 white + PB
    (1 / 36_525.17,     100),          # 4 white
    (1 / 14_494.11,     100),          # 3 white + PB
    (1 / 579.76,        7),            # 3 white
    (1 / 701.33,        7),            # 2 white + PB
    (1 / 91.98,         4),            # 1 white + PB
    (1 / 38.32,         4),            # PB only
]

MEGA_MILLIONS_SECONDARY = [
    (1 / 12_607_306, 1_000_000),
    (1 / 931_001,    10_000),
    (1 / 38_792,     500),
    (1 / 14_547,     200),
    (1 / 606,        10),
    (1 / 693,        10),
    (1 / 89,         4),
    (1 / 37,         2),
]


class JackpotSimulator:
    """Compute EV for draw games with splitting risk and tax adjustments."""

    def __init__(self, ev_calc: Optional[EVCalculator] = None):
        self.ev_calc = ev_calc or EVCalculator()

    # ── analytical secondary EV ────────────────────────────────────
    def secondary_ev(
        self, secondary_table: List[tuple]
    ) -> float:
        ev = 0.0
        for prob, prize in secondary_table:
            ev += prob * self.ev_calc.adjust_prize_for_taxes(prize)
        return ev

    # ── single-shot analytical EV ──────────────────────────────────
    def calculate(
        self,
        game_name: str,
        jackpot: float,
        n_tickets_sold: float,
        odds_jackpot: float,
        ticket_price: float = 2.0,
        secondary_table: Optional[List[tuple]] = None,
    ) -> JackpotEVResult:
        if ticket_price <= 0:
            raise ValueError("ticket_price must be > 0")

        expected_winners = max(1.0, n_tickets_sold * odds_jackpot)
        share = jackpot / expected_winners
        net_share = share * (1.0 - self.ev_calc.nc_tax - self.ev_calc.federal_tax)

        sec_ev = (
            self.secondary_ev(secondary_table)
            if secondary_table
            else 0.0
        )

        gross_ev = jackpot * odds_jackpot - ticket_price
        adjusted_ev = net_share * odds_jackpot + sec_ev - ticket_price
        ev_per_dollar = adjusted_ev / ticket_price

        signal = EVCalculator._classify_signal(ev_per_dollar, False)

        return JackpotEVResult(
            game_name=game_name,
            jackpot=jackpot,
            expected_winners=expected_winners,
            share_per_winner=share,
            ev_per_ticket_gross=gross_ev,
            ev_per_ticket_adjusted=adjusted_ev,
            ev_per_dollar=ev_per_dollar,
            signal_strength=signal,
            monte_carlo_breakeven_prob=0.0,  # filled by simulate()
            diagnostics={
                "n_tickets_sold": n_tickets_sold,
                "odds_jackpot": odds_jackpot,
                "secondary_ev": sec_ev,
            },
        )

    # ── Monte Carlo splitting simulator ────────────────────────────
    def simulate(
        self,
        game_name: str,
        jackpot: float,
        n_tickets_sold: float,
        odds_jackpot: float,
        ticket_price: float = 2.0,
        secondary_table: Optional[List[tuple]] = None,
        n_simulations: int = 50_000,
        rng_seed: Optional[int] = None,
    ) -> JackpotEVResult:
        """
        Run a Monte Carlo simulation of one ticket purchase across N draws,
        sampling actual #winners ~ Poisson(λ = n_sold · p).
        """
        rng = np.random.default_rng(rng_seed)
        lam = n_tickets_sold * odds_jackpot

        n_winners = rng.poisson(lam, size=n_simulations) + 1  # +1 includes "us"
        share_samples = jackpot / n_winners
        net_shares = share_samples * (
            1.0 - self.ev_calc.nc_tax - self.ev_calc.federal_tax
        )
        # Probability our single ticket actually wins jackpot: p = odds_jackpot
        # EV contribution from jackpot per simulation:
        ev_jackpot_per_sim = net_shares * odds_jackpot

        sec_ev = (
            self.secondary_ev(secondary_table) if secondary_table else 0.0
        )
        ev_total_per_sim = ev_jackpot_per_sim + sec_ev - ticket_price
        ev_mean = float(ev_total_per_sim.mean())
        breakeven = float((ev_total_per_sim >= 0).mean())

        analytical = self.calculate(
            game_name=game_name,
            jackpot=jackpot,
            n_tickets_sold=n_tickets_sold,
            odds_jackpot=odds_jackpot,
            ticket_price=ticket_price,
            secondary_table=secondary_table,
        )
        analytical.ev_per_ticket_adjusted = ev_mean
        analytical.ev_per_dollar = ev_mean / ticket_price
        analytical.monte_carlo_breakeven_prob = breakeven
        analytical.signal_strength = EVCalculator._classify_signal(
            analytical.ev_per_dollar, False
        )
        analytical.diagnostics["n_simulations"] = n_simulations
        analytical.diagnostics["expected_winners_poisson_lambda"] = lam
        return analytical

    # ── convenience presets ────────────────────────────────────────
    def powerball(
        self,
        jackpot: float,
        n_tickets_sold: float,
        n_simulations: int = 50_000,
    ) -> JackpotEVResult:
        return self.simulate(
            "Powerball",
            jackpot,
            n_tickets_sold,
            config.POWERBALL_ODDS_JACKPOT,
            config.POWERBALL_TICKET_PRICE,
            POWERBALL_SECONDARY,
            n_simulations,
        )

    def mega_millions(
        self,
        jackpot: float,
        n_tickets_sold: float,
        n_simulations: int = 50_000,
    ) -> JackpotEVResult:
        return self.simulate(
            "Mega Millions",
            jackpot,
            n_tickets_sold,
            config.MEGA_MILLIONS_ODDS_JACKPOT,
            config.MEGA_MILLIONS_TICKET_PRICE,
            MEGA_MILLIONS_SECONDARY,
            n_simulations,
        )
