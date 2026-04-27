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


# --- F5 — Radar multi-símbolo: oportunidades ---------------------------------
#
# Contratos introducidos en F5 para el barrido multi-símbolo del Radar. Son
# **aditivos**: no modifican los modelos previos ni los endpoints existentes.
# Mantienen ``extra="allow"`` para permitir que el motor real (Quant) añada
# campos sin romper consumidores. Ver:
#     docs/ATLAS_RADAR_F5_MULTI_SYMBOL_OPPORTUNITIES.md


RadarOpportunityClassification = Literal["high_conviction", "watchlist", "reject"]
RadarOpportunityDirection = Literal["long", "short", "neutral"]
RadarOpportunitySource = Literal["quant", "stub"]


class RadarOpportunitySnapshot(BaseModel):
    """Subconjunto canónico del snapshot por símbolo embebido en una oportunidad.

    Es una vista **mínima** y estable derivada del summary del Radar. El
    motor real puede añadir campos adicionales gracias a ``extra="allow"``.
    """

    model_config = ConfigDict(extra="allow")

    timestamp: str
    snapshot_classification: str
    fast_pressure_score: float = 0.0
    structural_confidence_score: float = 0.0
    fast_structural_alignment: float = 0.0
    fast_structural_divergence_score: float = 0.0
    horizon_conflict: bool = False
    cross_horizon_alignment: str = "-"
    bias: str = "neutral"


class RadarOpportunity(BaseModel):
    """Oportunidad multi-símbolo emitida por el batch engine del Radar (F5).

    El score es 0..100 (compuesto), la clasificación se deriva por umbrales
    documentados en el batch engine. ``source`` distingue motor vivo (quant)
    de stub honesto.
    """

    model_config = ConfigDict(extra="allow")

    symbol: str
    asset_class: str
    sector: str | None = None
    optionable: bool = True
    score: float = 0.0
    classification: RadarOpportunityClassification = "reject"
    direction: RadarOpportunityDirection = "neutral"
    timestamp: str
    horizon_min: int | None = None
    snapshot: RadarOpportunitySnapshot
    degradations_active: list[RadarDegradationEntry] = Field(default_factory=list)
    source: RadarOpportunitySource = "stub"
    trace_id: str


class RadarOpportunitiesResponse(BaseModel):
    """Cuerpo de ``GET /api/radar/opportunities``.

    Mantiene contadores de evaluación, recortes aplicados y degradaciones
    globales del batch (independientes de las degradaciones por símbolo).
    """

    model_config = ConfigDict(extra="allow")

    ok: bool = True
    source: RadarOpportunitySource = "stub"
    timestamp: str
    universe_size: int
    evaluated: int
    succeeded: int
    failed: int
    returned: int
    limit: int
    min_score: float
    filters: dict[str, Any] = Field(default_factory=dict)
    items: list[RadarOpportunity] = Field(default_factory=list)
    degradations_active: list[RadarDegradationEntry] = Field(default_factory=list)
    trace_id: str


class RadarOpportunityStreamEvent(BaseModel):
    """Envelope canónico de eventos SSE para el stream multi-símbolo.

    Reusa los 6 campos canónicos del envelope SSE existente
    (``type, timestamp, symbol, source, sequence, data``). Los tipos válidos
    para F5 son: ``opportunity_added``, ``opportunity_updated``,
    ``opportunity_removed``, ``universe_snapshot``, ``heartbeat``.

    Cuando ``type == "universe_snapshot"`` o ``"heartbeat"``, ``symbol`` es
    ``"*"`` (multi-símbolo), respetando la forma del envelope.
    """

    model_config = ConfigDict(extra="allow")

    type: Literal[
        "opportunity_added",
        "opportunity_updated",
        "opportunity_removed",
        "universe_snapshot",
        "heartbeat",
    ]
    timestamp: str
    symbol: str
    source: RadarOpportunitySource
    sequence: int
    data: dict[str, Any] = Field(default_factory=dict)
