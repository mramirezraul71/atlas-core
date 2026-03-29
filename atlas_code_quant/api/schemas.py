"""Atlas Code-Quant â€” Contratos Pydantic para la API REST."""
from __future__ import annotations

from datetime import datetime
from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, Field

from atlas_code_quant.backtesting.winning_probability import StrategyType

OrderStrategyType = StrategyType | Literal["equity_long", "equity_short"]


class SignalEnum(str, Enum):
    BUY  = "BUY"
    SELL = "SELL"
    HOLD = "HOLD"


class StatusEnum(str, Enum):
    OK      = "ok"
    ERROR   = "error"
    PAUSED  = "paused"


# â”€â”€ Request bodies â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class EvalSignalRequest(BaseModel):
    """POST /signal â€” Solicita evaluaciÃ³n de seÃ±al para un ticker."""
    symbol: str = Field(..., json_schema_extra={"example": "BTC/USDT"})
    strategy: str | None = Field(None, json_schema_extra={"example": "ma_cross"})
    timeframe: str = Field("1h", json_schema_extra={"example": "1h"})
    options_probability: "WinningProbabilityRequest | None" = None


class ActivateStrategyRequest(BaseModel):
    """POST /strategy/activate â€” Activa una estrategia."""
    strategy: str = Field(..., json_schema_extra={"example": "ma_cross"})
    symbols: list[str] = Field(..., json_schema_extra={"example": ["BTC/USDT"]})


class TradierOrderLeg(BaseModel):
    option_symbol: str | None = None
    side: str = Field(
        ...,
        pattern="^(buy|sell|sell_short|buy_to_cover|buy_to_open|sell_to_open|buy_to_close|sell_to_close)$",
    )
    quantity: float = Field(..., gt=0)
    instrument_type: Literal["option", "equity"] = "option"


class OrderRequest(BaseModel):
    """POST /order â€” Orden de trading desde Atlas/ROS2."""
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
    strategy_type: OrderStrategyType | None = None
    entry_reference_price: float | None = None
    entry_expected_move_pct: float | None = None
    entry_confidence_reference_pct: float | None = None
    max_entry_drift_pct: float | None = None
    max_entry_spread_pct: float | None = None
    chart_plan: dict[str, Any] = Field(default_factory=dict)
    camera_plan: dict[str, Any] = Field(default_factory=dict)
    live_confirmed: bool = False


class WinningProbabilityRequest(BaseModel):
    """POST /probability/options Ã¢â‚¬â€ Probabilidad de victoria para opciones."""
    symbol: str = Field(..., json_schema_extra={"example": "AAPL"})
    strategy_type: StrategyType = Field(..., json_schema_extra={"example": "iron_condor"})
    account_scope: Literal["live", "paper"] | None = None
    account_id: str | None = None
    tradier_token: str | None = Field(None, description="Opcional. Si no se envÃƒÂ­a, usa TRADIER_API_TOKEN del entorno.")
    tradier_base_url: str | None = Field(None, json_schema_extra={"example": "https://api.tradier.com/v1"})
    history_days: int = Field(252, ge=30, le=1260)
    min_dte: int = Field(14, ge=1, le=180)
    max_dte: int = Field(45, ge=1, le=365)
    n_paths: int = Field(10000, ge=1000, le=100000)
    random_seed: int = Field(42, ge=0, le=2_147_483_647)


class ProbabilityGateRequest(WinningProbabilityRequest):
    """Compuerta opcional de probabilidad antes de abrir una posiciÃƒÂ³n."""
    enabled: bool = True
    min_win_rate_pct: float = Field(50.0, ge=0, le=100)


# â”€â”€ Response bodies â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

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
    source: str = "tradier"
    source_label: str = "Tradier canonical"
    canonical_scope: str | None = None
    canonical_account_id: str | None = None
    canonical_updated_at: str | None = None
    account_session: dict[str, Any] | None = None
    balances: dict[str, Any] = Field(default_factory=dict)
    pdt_status: dict[str, Any] | None = None
    days_trades_used: int = 0
    active_strategies: list[str] = Field(default_factory=list)
    open_positions: int = 0
    reconciliation: dict[str, Any] = Field(default_factory=dict)
    simulators: dict[str, Any] = Field(default_factory=dict)


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


class JournalEntryPayload(BaseModel):
    id: int
    journal_key: str
    account_type: str
    account_id: str
    strategy_id: str
    tracker_strategy_id: str | None = None
    strategy_type: str
    symbol: str
    status: str
    is_level4: bool = False
    entry_time: str
    exit_time: str | None = None
    entry_price: float | None = None
    exit_price: float | None = None
    fees: float = 0.0
    win_rate_at_entry: float | None = None
    current_win_rate_pct: float | None = None
    iv_rank: float | None = None
    realized_pnl: float = 0.0
    unrealized_pnl: float = 0.0
    mark_price: float | None = None
    spot_price: float | None = None
    entry_notional: float | None = None
    risk_at_entry: float | None = None
    thesis_rich_text: str = ""
    legs: list[dict[str, Any]] = Field(default_factory=list)
    greeks: dict[str, Any] = Field(default_factory=dict)
    attribution: dict[str, Any] = Field(default_factory=dict)
    post_mortem: dict[str, Any] = Field(default_factory=dict)
    post_mortem_text: str = ""
    broker_order_ids: list[Any] | dict[str, Any] = Field(default_factory=list)
    raw_entry_payload: dict[str, Any] = Field(default_factory=dict)
    raw_exit_payload: list[dict[str, Any]] | dict[str, Any] = Field(default_factory=list)
    updated_at: str | None = None
    last_synced_at: str | None = None


class JournalEntriesPayload(BaseModel):
    generated_at: str
    count: int = 0
    items: list[JournalEntryPayload] = Field(default_factory=list)


class OperationConfigPayload(BaseModel):
    account_scope: Literal["paper", "live"] = "paper"
    paper_only: bool = True
    auton_mode: Literal["off", "paper_supervised", "paper_autonomous"] = "paper_supervised"
    executor_mode: Literal["disabled", "paper_api", "desktop_dry_run"] = "paper_api"
    vision_mode: Literal["off", "manual", "desktop_capture", "direct_nexus", "atlas_push_bridge", "insta360_pending"] = "direct_nexus"
    require_operator_present: bool = False
    operator_present: bool = True
    screen_integrity_ok: bool = True
    sentiment_score: float = Field(0.0, ge=-1.0, le=1.0)
    sentiment_source: str = "manual"
    min_auton_win_rate_pct: float = Field(65.0, ge=0.0, le=100.0)
    max_level4_bpr_pct: float = Field(20.0, ge=0.0, le=100.0)
    auto_pause_on_operational_errors: bool = True
    operational_error_limit: int = Field(3, ge=1, le=10)
    notes: str = ""
    kill_switch_active: bool = False


class OperationCycleRequest(BaseModel):
    order: OrderRequest
    action: Literal["evaluate", "preview", "submit"] = "evaluate"
    capture_context: bool = True


class EmergencyStopRequest(BaseModel):
    reason: str = "manual_stop"


class OperationStatusPayload(BaseModel):
    generated_at: str
    config: dict[str, Any] = Field(default_factory=dict)
    pdt_status: dict[str, Any] = Field(default_factory=dict)
    win_rate_positions: list[dict[str, Any]] = Field(default_factory=list)
    ai_sentiment: dict[str, Any] = Field(default_factory=dict)
    vision: dict[str, Any] = Field(default_factory=dict)
    executor: dict[str, Any] = Field(default_factory=dict)
    brain: dict[str, Any] = Field(default_factory=dict)
    chart_execution: dict[str, Any] = Field(default_factory=dict)
    learning: dict[str, Any] = Field(default_factory=dict)
    journal: dict[str, Any] = Field(default_factory=dict)
    attribution_integrity: dict[str, Any] = Field(default_factory=dict)
    position_management: dict[str, Any] = Field(default_factory=dict)
    exit_governance: dict[str, Any] = Field(default_factory=dict)
    post_trade_learning: dict[str, Any] = Field(default_factory=dict)
    visual_benchmark: dict[str, Any] = Field(default_factory=dict)
    failsafe: dict[str, Any] = Field(default_factory=dict)
    monitor_summary: dict[str, Any] = Field(default_factory=dict)
    scorecard: dict[str, Any] = Field(default_factory=dict)
    auton_mode_active: bool = False
    last_decision: dict[str, Any] | None = None
    last_candidate: dict[str, Any] | None = None


class OperationCyclePayload(BaseModel):
    generated_at: str
    decision: str
    allowed: bool
    blocked: bool = False
    action: Literal["evaluate", "preview", "submit"]
    reasons: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)
    gates: dict[str, Any] = Field(default_factory=dict)
    execution: dict[str, Any] | None = None
    probability: dict[str, Any] | None = None
    vision_snapshot: dict[str, Any] | None = None
    monitor_summary: dict[str, Any] = Field(default_factory=dict)
    sentiment: dict[str, Any] = Field(default_factory=dict)
    what_if: dict[str, Any] = Field(default_factory=dict)
    entry_validation: dict[str, Any] = Field(default_factory=dict)
    visual_entry_gate: dict[str, Any] = Field(default_factory=dict)
    execution_quality: dict[str, Any] = Field(default_factory=dict)
    operation_status: dict[str, Any] = Field(default_factory=dict)


class VisionCalibrationStartPayload(BaseModel):
    width: int = Field(1920, ge=1, le=16384)
    height: int = Field(1080, ge=1, le=16384)
    rows: int = Field(3, ge=1, le=9)
    cols: int = Field(3, ge=1, le=9)
    zoom_center: float = Field(1.0, ge=1.0, le=4.0)
    label: str = "calibracion_principal"
    save_path: str | None = None


class VisionCalibrationSamplePayload(BaseModel):
    point_id: str | None = None
    use_current_pose: bool = True
    yaw: float | None = None
    pitch: float | None = None
    zoom: float | None = None
    note: str = ""


class VisionCalibrationMovePosePayload(BaseModel):
    yaw: float = Field(..., ge=-1.0, le=1.0)
    pitch: float = Field(..., ge=-1.0, le=1.0)
    zoom: float = Field(1.0, ge=1.0, le=4.0)
    source: str = "camera"


class VisionCalibrationResetPayload(BaseModel):
    clear_active: bool = False


class VisionCalibrationStatusPayload(BaseModel):
    generated_at: str
    active_calibration_path: str | None = None
    calibration_exists: bool = False
    calibration: dict[str, Any] | None = None
    session_active: bool = False
    label: str = ""
    monitor: dict[str, Any] = Field(default_factory=dict)
    points: list[dict[str, Any]] = Field(default_factory=list)
    samples: list[dict[str, Any]] = Field(default_factory=list)
    sample_count: int = 0
    last_pose: dict[str, Any] | None = None
    last_move: dict[str, Any] | None = None
    last_fit: dict[str, Any] | None = None
    notes: str = ""


class StrategySelectorCandidatePayload(BaseModel):
    symbol: str
    timeframe: str
    direction: str
    price: float | None = None
    strategy_key: str | None = None
    strategy_label: str | None = None
    selection_score: float = 0.0
    local_win_rate_pct: float = 0.0
    predicted_move_pct: float = 0.0
    relative_strength_pct: float = 0.0
    order_flow: dict[str, Any] = Field(default_factory=dict)
    confirmation: dict[str, Any] = Field(default_factory=dict)
    why_selected: list[str] = Field(default_factory=list)


class StrategySelectorRequest(BaseModel):
    candidate: StrategySelectorCandidatePayload
    account_scope: Literal["paper", "live"] = "paper"
    account_id: str | None = None
    chart_provider: Literal["tradingview", "yahoo"] = "tradingview"
    prefer_defined_risk: bool = True
    allow_equity: bool = True
    allow_credit: bool = True
    risk_budget_pct: float = Field(0.75, ge=0.25, le=2.0)


class StrategySelectorPayload(BaseModel):
    generated_at: str
    account_scope: str
    account_session: dict[str, Any] = Field(default_factory=dict)
    balances: dict[str, Any] = Field(default_factory=dict)
    candidate: dict[str, Any] = Field(default_factory=dict)
    selected: dict[str, Any] = Field(default_factory=dict)
    alternatives: list[dict[str, Any]] = Field(default_factory=list)
    probability: dict[str, Any] | None = None
    size_plan: dict[str, Any] = Field(default_factory=dict)
    chart_plan: dict[str, Any] = Field(default_factory=dict)
    camera_plan: dict[str, Any] = Field(default_factory=dict)
    entry_plan: dict[str, Any] = Field(default_factory=dict)
    confidence_breakdown: dict[str, Any] = Field(default_factory=dict)
    adaptive_context: dict[str, Any] = Field(default_factory=dict)
    risk_profile: dict[str, Any] = Field(default_factory=dict)
    automation_ready: dict[str, Any] = Field(default_factory=dict)
    exit_plan: dict[str, Any] = Field(default_factory=dict)
    playbook: dict[str, Any] = Field(default_factory=dict)
    order_seed: dict[str, Any] = Field(default_factory=dict)
    warnings: list[str] = Field(default_factory=list)


# â”€â”€ Auto-cycle loop control â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class LoopStartRequest(BaseModel):
    """POST /operation/loop/start â€” Inicia el ciclo autÃ³nomo scannerâ†’operaciÃ³n."""
    interval_sec: int = Field(120, ge=15, le=3600,
        description="Segundos entre ciclos de evaluaciÃ³n.")
    max_per_cycle: int = Field(1, ge=1, le=5,
        description="MÃ¡ximo de candidatos a evaluar por ciclo.")


class VisionProviderRequest(BaseModel):
    """POST /operation/vision/provider â€” Cambia el proveedor de visiÃ³n activo."""
    provider: str = Field(...,
        description="Uno de: off, manual, desktop_capture, direct_nexus, atlas_push_bridge")
    notes: str | None = None


class ScannerConfigPayload(BaseModel):
    enabled: bool = True
    source: Literal["yfinance", "ccxt"] = "yfinance"
    scan_interval_sec: int = Field(180, ge=15, le=3600)
    min_signal_strength: float = Field(0.55, ge=0.0, le=1.0)
    min_local_win_rate_pct: float = Field(53.0, ge=0.0, le=100.0)
    min_selection_score: float = Field(75.0, ge=0.0, le=100.0)
    max_candidates: int = Field(8, ge=1, le=50)
    require_higher_tf_confirmation: bool = True
    universe_mode: Literal["manual", "us_equities_rotating"] = "us_equities_rotating"
    universe_batch_size: int = Field(80, ge=20, le=500)
    prefilter_count: int = Field(20, ge=8, le=80)
    universe: list[str] = Field(default_factory=list)
    timeframes: list[str] = Field(default_factory=list)
    notes: str = ""


class ScannerControlRequest(BaseModel):
    action: Literal["start", "stop", "run_once"] = "run_once"


class ScannerStatusPayload(BaseModel):
    generated_at: str
    running: bool
    cycle_count: int = 0
    last_cycle_at: str | None = None
    last_cycle_ms: float | None = None
    current_symbol: str | None = None
    current_timeframe: str | None = None
    current_step: str | None = None
    last_error: str | None = None
    config: dict[str, Any] = Field(default_factory=dict)
    summary: dict[str, Any] = Field(default_factory=dict)
    learning: dict[str, Any] = Field(default_factory=dict)


class ScannerReportPayload(BaseModel):
    generated_at: str
    status: dict[str, Any] = Field(default_factory=dict)
    summary: dict[str, Any] = Field(default_factory=dict)
    criteria: list[dict[str, Any]] = Field(default_factory=list)
    universe: dict[str, Any] = Field(default_factory=dict)
    candidates: list[dict[str, Any]] = Field(default_factory=list)
    rejections: list[dict[str, Any]] = Field(default_factory=list)
    activity: list[dict[str, Any]] = Field(default_factory=list)
    current_work: dict[str, Any] = Field(default_factory=dict)
    learning: dict[str, Any] = Field(default_factory=dict)
    error: str | None = None


EvalSignalRequest.model_rebuild()
OrderRequest.model_rebuild()


