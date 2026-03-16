"""Atlas Code-Quant — Contratos Pydantic para la API REST."""
from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Any

from pydantic import BaseModel, Field


class SignalEnum(str, Enum):
    BUY  = "BUY"
    SELL = "SELL"
    HOLD = "HOLD"


class StatusEnum(str, Enum):
    OK      = "ok"
    ERROR   = "error"
    PAUSED  = "paused"


# ── Request bodies ───────────────────────────────────────────────────────────

class EvalSignalRequest(BaseModel):
    """POST /signal — Solicita evaluación de señal para un ticker."""
    symbol: str = Field(..., example="BTC/USDT")
    strategy: str | None = Field(None, example="ma_cross")
    timeframe: str = Field("1h", example="1h")


class ActivateStrategyRequest(BaseModel):
    """POST /strategy/activate — Activa una estrategia."""
    strategy: str = Field(..., example="ma_cross")
    symbols: list[str] = Field(..., example=["BTC/USDT"])


class OrderRequest(BaseModel):
    """POST /order — Orden de trading desde Atlas/ROS2."""
    symbol: str
    side: str = Field(..., pattern="^(buy|sell)$")
    size: float = Field(..., gt=0)
    order_type: str = Field("market", pattern="^(market|limit)$")
    price: float | None = None
    stop_loss: float | None = None
    take_profit: float | None = None


# ── Response bodies ──────────────────────────────────────────────────────────

class SignalResponse(BaseModel):
    symbol: str
    signal: SignalEnum
    confidence: float
    price: float
    stop_loss: float | None = None
    take_profit: float | None = None
    timestamp: datetime
    strategy: str
    metadata: dict[str, Any] = {}


class PositionResponse(BaseModel):
    symbol: str
    side: str
    size: float
    entry_price: float
    current_price: float
    pnl: float
    pnl_pct: float
    stop_loss: float | None = None
    take_profit: float | None = None
    opened_at: datetime


class PortfolioResponse(BaseModel):
    initial_capital: float
    current_equity: float
    free_capital: float
    open_positions: int
    total_pnl: float
    drawdown_pct: float
    positions: list[PositionResponse] = []


class HealthResponse(BaseModel):
    status: StatusEnum
    version: str = "0.1.0"
    uptime_sec: float
    active_strategies: list[str] = []
    open_positions: int = 0
    timestamp: datetime


class StdResponse(BaseModel):
    ok: bool
    data: Any = None
    error: str | None = None
    ms: float = 0.0
