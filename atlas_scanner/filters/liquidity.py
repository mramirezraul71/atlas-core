from __future__ import annotations

from typing import Tuple

from atlas_scanner.config import LiquidityThresholds, SCORING_CONFIG, ScoringConfig
from atlas_scanner.models import SymbolSnapshot


def apply_liquidity_filter(
    symbols: Tuple[SymbolSnapshot, ...],
    thresholds: LiquidityThresholds | None = None,
) -> Tuple[Tuple[SymbolSnapshot, ...], Tuple[str, ...]]:
    """
    S0 stub: comportamiento neutro.
    Devuelve todos los simbolos como kept y ningun simbolo rechazado.
    """
    config: ScoringConfig = SCORING_CONFIG
    _effective_thresholds = thresholds or config.liquidity
    return (symbols, ())

