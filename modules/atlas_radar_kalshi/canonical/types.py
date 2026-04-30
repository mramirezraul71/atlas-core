"""Modelos canónicos multi-venue (desacoplan lógica por venue de los DTOs legacy)."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Literal, Optional

from pydantic import BaseModel, Field


class CanonicalMarket(BaseModel):
    key: str
    title_norm: str
    close_day: str = "na"
    source_tickers: dict[str, str] = Field(
        default_factory=dict,
        description="venue -> ticker nativo (kalshi, polymarket, ...).",
    )


class CanonicalQuote(BaseModel):
    venue: Literal["kalshi", "polymarket"]
    ticker: str
    title: str
    close_time: Optional[str] = None
    yes_mid: float
    yes_ask: Optional[float] = None
    no_ask: Optional[float] = None
    ts: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))


class ExecutionIntent(BaseModel):
    """Intento de ejecución neutral respecto al venue (el router añade el adapter)."""
    market_ticker: str
    side: str
    price_cents: int
    contracts: int
    client_order_id: str
    reason: str = "entry"
    order_type: str = "limit"


class FillEvent(BaseModel):
    ok: bool
    order_id: Optional[str] = None
    venue: str = "unknown"
    filled_contracts: int = 0
    avg_price_cents: float = 0.0
    status: str = ""
    latency_ms: int = 0
    error: Optional[str] = None
    raw: dict[str, Any] = Field(default_factory=dict)
    ts: datetime = Field(default_factory=lambda: datetime.now(timezone.utc))
