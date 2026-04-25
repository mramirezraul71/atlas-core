from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Literal

from atlas_scanner.models import SymbolSnapshot


def compute_gamma(symbol: SymbolSnapshot) -> float:
    _ = symbol
    return 0.0


@dataclass(frozen=True)
class StrikeGamma:
    strike: float
    call_gamma: float
    put_gamma: float

    @property
    def net_gamma(self) -> float:
        return self.call_gamma + self.put_gamma


class GammaRegime(str, Enum):
    POSITIVE = "positive"
    NEGATIVE = "negative"
    NEUTRAL = "neutral"


def _select_wall_strike(
    strikes: tuple[StrikeGamma, ...],
    side: Literal["call", "put"],
) -> float | None:
    if not strikes:
        return None

    if side == "call":
        scored = [(abs(item.call_gamma), item.strike) for item in strikes]
    else:
        scored = [(abs(item.put_gamma), item.strike) for item in strikes]

    best_score, best_strike = max(scored, key=lambda pair: pair[0])
    if best_score == 0.0:
        return None
    return best_strike


def find_call_wall(strikes: tuple[StrikeGamma, ...]) -> float | None:
    """
    Return strike with highest absolute call gamma.

    Returns None when sequence is empty or all call gammas are zero.
    """
    return _select_wall_strike(strikes=strikes, side="call")


def find_put_wall(strikes: tuple[StrikeGamma, ...]) -> float | None:
    """
    Return strike with highest absolute put gamma.

    Returns None when sequence is empty or all put gammas are zero.
    """
    return _select_wall_strike(strikes=strikes, side="put")


def find_gamma_flip(strikes: tuple[StrikeGamma, ...]) -> float | None:
    """
    Find a simple zero-gamma level by scanning ordered strikes.

    - Returns strike directly when a net gamma value is exactly zero.
    - If sign changes between adjacent points, returns linearly interpolated strike.
    - Returns None if no sign change exists or fewer than two points are provided.
    """
    if len(strikes) < 2:
        return None

    ordered = sorted(strikes, key=lambda item: item.strike)
    for index in range(len(ordered) - 1):
        left = ordered[index]
        right = ordered[index + 1]

        left_net = left.net_gamma
        right_net = right.net_gamma

        if left_net == 0.0:
            return left.strike
        if right_net == 0.0:
            return right.strike

        if (left_net > 0.0 and right_net < 0.0) or (left_net < 0.0 and right_net > 0.0):
            slope = (right_net - left_net) / (right.strike - left.strike)
            if slope == 0.0:
                return right.strike
            flip_strike = left.strike - (left_net / slope)
            return flip_strike

    return None


def classify_gamma_regime(
    net_gamma: float,
    neutral_threshold: float = 0.0,
) -> GammaRegime:
    if abs(net_gamma) <= neutral_threshold:
        return GammaRegime.NEUTRAL
    if net_gamma > neutral_threshold:
        return GammaRegime.POSITIVE
    return GammaRegime.NEGATIVE

