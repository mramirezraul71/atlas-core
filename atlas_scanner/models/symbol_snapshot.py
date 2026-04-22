from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class SymbolSnapshot:
    symbol: str
    asset_type: str
    base_currency: str
    ref_price: float
    volatility_lookback: float
    liquidity_score: float
    meta: dict[str, object] = field(default_factory=dict)

