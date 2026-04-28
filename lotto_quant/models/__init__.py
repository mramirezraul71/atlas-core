"""Quantitative models for Atlas Lotto-Quant."""

from .ev_calculator import (
    EVCalculator,
    EVResult,
    PrizeTier,
    ScratchOffGame,
)
from .markov_scratchoff import GameState, ScratchOffMarkovModel
from .jackpot_simulator import JackpotSimulator, JackpotEVResult
from .kelly_allocator import KellyAllocator, KellyRecommendation

__all__ = [
    "EVCalculator",
    "EVResult",
    "PrizeTier",
    "ScratchOffGame",
    "GameState",
    "ScratchOffMarkovModel",
    "JackpotSimulator",
    "JackpotEVResult",
    "KellyAllocator",
    "KellyRecommendation",
]
