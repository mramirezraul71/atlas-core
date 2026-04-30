"""Fase 2 (slice): simulador paper mínimo — apertura, mark-to-market, cierre."""
from .paper import (
    LegMatchInfo,
    MatchKind,
    PaperSimulator,
    Position,
    PositionSnapshot,
    rebuild_contracts_from_chain,
)

__all__ = [
    "LegMatchInfo",
    "MatchKind",
    "PaperSimulator",
    "Position",
    "PositionSnapshot",
    "rebuild_contracts_from_chain",
]
