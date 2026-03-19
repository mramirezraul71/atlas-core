"""Atlas Code-Quant — Contratos Pydantic para la API REST."""
from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, Field

from backtesting.winning_probability import StrategyType


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
    options_probability: "WinningProbabilityRequest | None" = None


class ActivateStrategyRequest(BaseModel):
    """POST /strategy/activate — Activa una estrategia."""
    strategy: str = Field(..., example="ma_cross")
    symbols: list[str] = Field(..., example=["BTC/USDT"])


class TradierOrderLeg(BaseModel):
    option_symbol: str | None = None
    side: str = Field(
        ...,
        pattern="^(buy|sell|sell_short|buy_to_cover|buy_to_open|sell_to_open|buy_to_close|sell_to_close)$",
    )
    quantity: float = Field(..., gt=0)
    instrument_type: Literal["option", "equity"] = "option"


class OrderRequest(BaseModel):
    """POST /order — Orden de trading desde Atlas/ROS2."""
    symbol: str
    side: str = Field(
        ...,
        pattern="^(buy|sell|sell_short|buy_to_cover|buy_to_open|sell_to_open|buy_to_close|sell_to_close)$",
    )
    size: float = Field(..., gt=0)
    order_type: str = Field("market", pattern="^(market|limit|stop|stop_limit|debit|credit|even)$")
    price: float | None = None
    stop_price: float | None = None
    stop_loss: float | None = None
    take_profit: float | None = None
    asset_class: Literal["auto", "equity", "option", "multileg", "combo"] = "auto"
    option_symbol: str | None = None
    duration: str = Field("day", pattern="^(day|gtc|pre|post)$")
    preview: bool = True
    extended_hours: bool = False
    tag: str | None = None
    tradier_class: Literal["equity", "option", "multileg", "combo"] | None = None
    legs: list[TradierOrderLeg] = Field(default_factory=list)
    probability_gate: "ProbabilityGateRequest | None" = None
    account_scope: Literal["live", "paper"] | None = None
    account_id: str | None = None
    position_effect: Literal["auto", "open", "close"] = "auto"
    strategy_type: StrategyType | None = None
    live_confirmed: bool = False


class WinningProbabilityRequest(BaseModel):
    """POST /probability/options â€” Probabilidad de victoria para opciones."""
    symbol: str = Field(..., example="AAPL")
    strategy_type: StrategyType = Field(..., example="iron_condor")
    account_scope: Literal["live", "paper"] | None = None
    account_id: str | None = None
    tradier_token: str | None = Field(None, description="Opcional. Si no se envÃ­a, usa TRADIER_API_TOKEN del entorno.")
    tradier_base_url: str | None = Field(None, example="https://api.tradier.com/v1")
    history_days: int = Field(252, ge=30, le=1260)
    min_dte: int = Field(14, ge=1, le=180)
    max_dte: int = Field(45, ge=1, le=365)
    n_paths: int = Field(10000, ge=1000, le=100000)
    random_seed: int = Field(42, ge=0, le=2_147_483_647)


class ProbabilityGateRequest(WinningProbabilityRequest):
    """Compuerta opcional de probabilidad antes de abrir una posiciÃ³n."""
    enabled: bool = True
    min_win_rate_pct: float = Field(50.0, ge=0, le=100)


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


class QuantStatusPayload(BaseModel):
    generated_at: str
    service_status: str
    uptime_sec: float
    account_session: dict[str, Any] | None = None
    pdt_status: dict[str, Any] | None = None
    days_trades_used: int = 0
    active_strategies: list[str] = Field(default_factory=list)
    open_positions: int = 0


class PayoffPoint(BaseModel):
    underlying_price: float
    pnl_at_expiration: float
    pnl_theoretical_today: float


class QuantPayoffPayload(BaseModel):
    generated_at: str
    strategy_id: str
    strategy_type: str
    underlying: str
    spot: float
    points: list[PayoffPoint] = Field(default_factory=list)
    break_even_points: list[float] = Field(default_factory=list)


class JournalHeatmapCell(BaseModel):
    weekday: int
    hour: int
    trades: int
    wins: int
    success_rate_pct: float


class JournalEquityPoint(BaseModel):
    timestamp: str
    equity: float
    pnl: float


class JournalAccountStats(BaseModel):
    trades_closed: int = 0
    trades_open: int = 0
    win_rate_pct: float = 0.0
    profit_factor: float | None = None
    sharpe_ratio: float | None = None
    expectancy: float = 0.0
    gross_wins: float = 0.0
    gross_losses: float = 0.0
    realized_pnl: float = 0.0
    unrealized_pnl: float = 0.0
    equity_curve: list[JournalEquityPoint] = Field(default_factory=list)
    heatmap: list[JournalHeatmapCell] = Field(default_factory=list)


class JournalStatsPayload(BaseModel):
    generated_at: str
    accounts: dict[str, JournalAccountStats] = Field(default_factory=dict)
    comparison: dict[str, float] = Field(default_factory=dict)


EvalSignalRequest.model_rebuild()
OrderRequest.model_rebuild()
