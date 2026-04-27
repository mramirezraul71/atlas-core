"""Imbalance helpers — F7.

Clasifica un score continuo de imbalance en lados ``buy_pressure``,
``sell_pressure`` o ``neutral`` con umbrales configurables.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


ImbalanceSide = Literal["buy_pressure", "sell_pressure", "neutral"]


@dataclass(slots=True, frozen=True)
class ImbalanceSnapshot:
    score: float           # rango aproximado [-1, +1]
    confidence: float = 0.5  # [0, 1]
    source: str = "stub"


def classify_imbalance(
    score: float,
    *,
    buy_threshold: float = 0.20,
    sell_threshold: float = -0.20,
) -> ImbalanceSide:
    if score >= buy_threshold:
        return "buy_pressure"
    if score <= sell_threshold:
        return "sell_pressure"
    return "neutral"


def from_score(score: float, confidence: float = 0.5) -> ImbalanceSnapshot:
    return ImbalanceSnapshot(score=float(score), confidence=float(confidence))
