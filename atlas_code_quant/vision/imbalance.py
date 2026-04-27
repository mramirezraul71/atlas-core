"""Helpers de imbalance visual (placeholder F1)."""
from __future__ import annotations


def classify_imbalance(score: float) -> str:
    if score > 0.2:
        return "buy_pressure"
    if score < -0.2:
        return "sell_pressure"
    return "neutral"
