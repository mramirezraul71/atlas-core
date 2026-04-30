"""
lotto_quant.data.schemas
========================

Pydantic v2 models for typed validation of scraped data and DB rows.
"""

from __future__ import annotations

from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field, field_validator


class PrizeTierSchema(BaseModel):
    value: float = Field(..., gt=0, description="Prize amount in USD")
    total_prizes: int = Field(..., ge=0)
    remaining_prizes: int = Field(..., ge=0)
    odds_initial: Optional[float] = Field(default=None, ge=0)

    @field_validator("remaining_prizes")
    @classmethod
    def remaining_le_total(cls, v: int, info) -> int:
        total = info.data.get("total_prizes")
        if total is not None and v > total:
            raise ValueError(
                f"remaining_prizes ({v}) > total_prizes ({total})"
            )
        return v


class ScratchOffGameSchema(BaseModel):
    game_id: str
    name: str
    ticket_price: float = Field(..., gt=0)
    prize_tiers: List[PrizeTierSchema]
    total_tickets_printed: int = Field(default=0, ge=0)
    tickets_sold_estimate: int = Field(default=0, ge=0)
    snapshot_ts: datetime = Field(default_factory=datetime.utcnow)
    source: str = Field(default="ncel")


class DrawGameJackpotSchema(BaseModel):
    game_name: str   # 'powerball' | 'mega_millions'
    jackpot_amount: float = Field(..., ge=0)
    estimated_winners: float = Field(default=1.0, ge=0)
    next_draw: Optional[datetime] = None
    snapshot_ts: datetime = Field(default_factory=datetime.utcnow)
