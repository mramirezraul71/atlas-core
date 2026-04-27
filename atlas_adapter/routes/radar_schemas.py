"""Contratos Pydantic v2 (incrementales) para la API JSON del Radar institucional.

Objetivo: documentación OpenAPI + validación de salida sin cambiar rutas ni shapes
consumidos por el dashboard. Campos dinámicos usan ``extra="allow"`` donde el
motor puede añadir claves en el futuro.

**Parcial por diseño**
- ``/api/radar/stream``: respuesta ``StreamingResponse`` (text/event-stream); no
  hay ``response_model`` HTTP único. Ver ``RadarSseEnvelope`` como modelo
  reutilizable (tests / docs).
- HTML/archivos bajo ``/radar/dashboard``: fuera de este módulo JSON.
"""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


# --- Búsqueda de símbolos -----------------------------------------------------


class RadarSearchResult(BaseModel):
    """Un candidato de búsqueda (Quant puede añadir campos)."""

    model_config = ConfigDict(extra="allow")

    symbol: str | None = None
    score: float | None = None


class RadarSearchResponse(BaseModel):
    """Cuerpo de ``GET /api/radar/symbols/search``."""

    model_config = ConfigDict(extra="allow")

    ok: bool
    source: str
    message: str | None = None
    query: str | None = None
    matches: list[Any] = Field(default_factory=list)
    truncated: bool = False


# --- Diagnósticos / proveedores -----------------------------------------------


class RadarProviderRow(BaseModel):
    """Fila de proveedor en ``/diagnostics/providers``."""

    model_config = ConfigDict(extra="allow")

    provider: str = ""
    name: str = ""
    is_ready: bool = False
    ready: bool = False
    stale_indicator: bool = False
    latency_ms: int = 0
    p95_latency_ms: int = 0
    active_fallback_indicator: bool = False
    circuit_open_indicator: bool = False
    consecutive_errors: int = 0
    availability_ratio: float = 0.0
    last_error_type: str = ""
    ui_status_hint: str = ""


class RadarDiagnosticsPayload(BaseModel):
    """``GET /api/radar/diagnostics/providers``."""

    providers: list[RadarProviderRow]


# Alias semántico (un subconjunto del diagnóstico; misma forma en la práctica).
RadarProviderHealthPayload = RadarDiagnosticsPayload


# --- Resumen dashboard --------------------------------------------------------


class RadarDegradationEntry(BaseModel):
    """Entrada en ``degradations_active`` (M2)."""

    model_config = ConfigDict(extra="allow")

    code: str
    label: str
    severity: Literal["info", "warning", "critical"] = "warning"
    source: str


class RadarTransportBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    sse: bool = True
    stub: bool = False
    quant: bool | None = None
    quant_scanner_ms: float | None = None
    sse_heartbeat_sec: float | int | None = None
    sse_snapshot_sec: float | int | None = None


class RadarQuantBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    connected: bool = False
    scanner_running: bool = False
    cycle_count: int = 0
    current_symbol: str | None = None
    current_step: str | None = None


class RadarSignalMeta(BaseModel):
    model_config = ConfigDict(extra="allow")

    snapshot_classification: str
    fast_pressure_score: float = 0.0
    structural_confidence_score: float = 0.0
    fast_structural_alignment: float = 0.0
    fast_structural_divergence_score: float = 0.0
    horizon_conflict: bool = False
    cross_horizon_alignment: str = "-"


class RadarSignalBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    timestamp: str
    bias: str
    meta: RadarSignalMeta


class RadarRadarBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    signal: RadarSignalBlock


class RadarDecisionGate(BaseModel):
    model_config = ConfigDict(extra="allow")

    recent: list[Any] = Field(default_factory=list)
    latest: dict[str, Any] | None = None


class RadarProviderHealthSummary(BaseModel):
    model_config = ConfigDict(extra="allow")

    providers_checked: int = 0
    degraded_count: int = 0


class RadarSummaryPayload(BaseModel):
    """``GET /api/radar/dashboard/summary`` y alias ``/summary``."""

    model_config = ConfigDict(extra="allow")

    symbol: str
    last_update: str
    stream_available: bool = True
    transport: RadarTransportBlock
    radar: RadarRadarBlock
    decision_gate: RadarDecisionGate
    camera_context: dict[str, Any]
    degradations_active: list[RadarDegradationEntry] = Field(default_factory=list)
    provider_health_summary: RadarProviderHealthSummary
    quant: RadarQuantBlock | None = None


# --- Decisiones ---------------------------------------------------------------


class RadarDecisionsRecentPayload(BaseModel):
    recent: list[dict[str, Any]]


class RadarDecisionStatsBlock(BaseModel):
    model_config = ConfigDict(extra="allow")

    by_decision: dict[str, int] = Field(default_factory=dict)
    avg_fast_pressure_score: float | None = None
    avg_structural_confidence_score: float | None = None


class RadarDecisionsStatsPayload(BaseModel):
    stats: RadarDecisionStatsBlock


# --- Por símbolo: fast / structural / dealer / political ----------------------


class RadarFastPayload(BaseModel):
    model_config = ConfigDict(extra="allow")

    symbol: str
    timestamp: str
    fast_pressure_score: float | None = None
    fast_risk_score: float | None = None
    fast_directional_bias_score: float | None = None


class RadarStructuralPayload(BaseModel):
    model_config = ConfigDict(extra="allow")

    symbol: str
    timestamp: str
    structural_confidence_score: float | None = None
    structural_bullish_score: float | None = None
    structural_bearish_score: float | None = None


class RadarDealerPayload(BaseModel):
    model_config = ConfigDict(extra="allow")

    symbol: str
    timestamp: str
    gamma_flip_level: str = "-"
    dealer_skew_score: float | None = None
    call_wall: str = "-"
    put_wall: str = "-"
    acceleration_zone_score: float | None = None


class RadarPoliticalPayload(BaseModel):
    model_config = ConfigDict(extra="allow")

    symbol: str
    timestamp: str
    aggregate_signal_score: float | None = None


# --- Cámara -------------------------------------------------------------------


class RadarCameraHealthPayload(BaseModel):
    """``GET /api/radar/sensors/camera/health`` — solo panel bajo ``camera``."""

    camera: dict[str, Any]


class RadarCameraStatusPayload(BaseModel):
    """``GET /api/radar/camera/status`` — sobre con ``ok`` / ``source``."""

    model_config = ConfigDict(extra="allow")

    ok: bool
    source: str
    timestamp: str
    camera: dict[str, Any]


# --- SSE (referencia; no es response_model del stream) -------------------------


class RadarSseEnvelope(BaseModel):
    """Envelope canónico de eventos SSE (heartbeat, snapshot_update, …)."""

    model_config = ConfigDict(extra="allow")

    type: str
    timestamp: str
    symbol: str
    source: str
    sequence: int
    data: dict[str, Any] = Field(default_factory=dict)


# --- Oportunidades multi-símbolo (F2) -----------------------------------------


class RadarOpportunity(BaseModel):
    """Una fila del ranking multi-símbolo (REST / SSE ``data``)."""

    model_config = ConfigDict(extra="allow")

    symbol: str
    asset_class: str
    score: float
    classification: str
    timestamp: str = ""
    horizon_min: int = 0
    direction: str = "neutral"
    snapshot: dict[str, Any] = Field(default_factory=dict)
    degradations_active: list[dict[str, Any]] = Field(default_factory=list)
    source: Literal["quant", "stub"] = "stub"
    trace_id: str = ""


class RadarOpportunitiesResponse(BaseModel):
    """``GET /api/radar/opportunities``."""

    model_config = ConfigDict(extra="allow")

    ok: bool = True
    opportunities: list[RadarOpportunity] = Field(default_factory=list)
    truncated: bool = False
    universe_evaluated: int = 0
    min_score: float = 0.0
    trace_id: str = ""
    degraded_globally: bool = False
    global_degradations: list[dict[str, Any]] = Field(default_factory=list)
    message: str | None = None


class RadarOpportunityDetailResponse(BaseModel):
    """``GET /api/radar/opportunities/{symbol}``."""

    model_config = ConfigDict(extra="allow")

    ok: bool
    opportunity: RadarOpportunity | None = None
    message: str | None = None


class RadarSseUniverseSnapshotData(BaseModel):
    """Cuerpo típico de ``data`` para ``type=universe_snapshot`` (F2)."""

    model_config = ConfigDict(extra="allow")

    symbols: list[str] = Field(default_factory=list)
    count: int = 0
    truncated: bool = False
    trace_id: str = ""


class RadarSseOpportunityEventData(BaseModel):
    """Cuerpo típico de ``data`` para ``opportunity_*`` (F2)."""

    model_config = ConfigDict(extra="allow")

    opportunity: RadarOpportunity | None = None


class RadarSseHeartbeatData(BaseModel):
    """Heartbeat F2 (``data`` puede ir vacío o con contadores)."""

    model_config = ConfigDict(extra="allow")

    batch_trace_id: str | None = None
