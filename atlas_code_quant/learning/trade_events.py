"""Dataclasses centrales para el sistema AtlasLearningBrain.

Define las estructuras de datos que fluyen por todas las capas del
subsistema de aprendizaje: TradeEvent (trade cerrado), SignalContext
(señal en vivo antes de ejecutar), métricas, reportes y políticas.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, date
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# TradeEvent — trade cerrado con resultado conocido
# ---------------------------------------------------------------------------

@dataclass
class TradeEvent:
    """Captura completa de un trade cerrado para aprendizaje.

    Todos los campos que describen *qué pasó* y *bajo qué condiciones*:
    indicadores en entrada, contexto de mercado, resultado en R-múltiplos.
    """

    # Identidad
    trade_id: str
    symbol: str
    asset_class: str          # equity_stock, equity_etf, index_option, crypto, future, forex
    side: str                 # buy | sell

    # Timing
    entry_time: datetime
    exit_time: datetime
    timeframe: str            # 1m, 5m, 15m, 1h, …

    # Precios
    entry_price: float
    exit_price: float
    stop_loss_price: float

    # R-múltiplos (normalizados por riesgo inicial)
    r_initial: float          # distancia entry→stop en puntos
    r_realized: float         # (exit - entry) / r_initial; negativo = pérdida
    mae_r: float              # Maximum Adverse Excursion en R (siempre ≤ 0)
    mfe_r: float              # Maximum Favorable Excursion en R (siempre ≥ 0)

    # Contexto estratégico
    setup_type: str           # breakout, pullback, inside_bar, iron_condor, …
    regime: str               # BULL, BEAR, SIDEWAYS, VOLATILE
    exit_type: str            # target, stop, time_exit, manual, eod_close

    # Indicadores técnicos en entrada
    rsi: float = 0.0
    macd_hist: float = 0.0
    atr: float = 0.0
    bb_pct: float = 0.0       # posición dentro de Bollinger Bands (0-1)
    volume_ratio: float = 1.0  # volumen / media 20 barras
    cvd: float = 0.0          # Cumulative Volume Delta
    iv_rank: float = 0.0      # IV Rank 0-100
    iv_hv_ratio: float = 1.0  # IV / HV ratio

    # Contexto de capital
    capital_at_entry: float = 100_000.0
    position_size: float = 0.0
    commission: float = 0.0

    # Señal de aprendizaje (si ya existía)
    signal_score_at_entry: float = 0.5   # 0.0-1.0; 0.5 = sin puntuación
    ml_score_at_entry: float = 0.5
    stats_score_at_entry: float = 0.5

    # Indicadores de error / calidad operativa
    error_flags: List[str] = field(default_factory=list)
    # Ejemplos: "late_entry", "missed_stop", "oversize", "chased_entry",
    #           "wrong_regime", "early_exit", "held_too_long"

    # Metadatos opcionales
    notes: str = ""
    tags: List[str] = field(default_factory=list)
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def is_winner(self) -> bool:
        return self.r_realized > 0

    @property
    def duration_minutes(self) -> float:
        delta = self.exit_time - self.entry_time
        return delta.total_seconds() / 60.0

    @property
    def risk_reward_ratio(self) -> float:
        """MFE / |MAE| — ratio riesgo/recompensa realizado."""
        if self.mae_r == 0:
            return float("inf")
        return abs(self.mfe_r / self.mae_r)


# ---------------------------------------------------------------------------
# SignalContext — espejo de TradeEvent *sin* resultado (para scoring en vivo)
# ---------------------------------------------------------------------------

@dataclass
class SignalContext:
    """Contexto de una señal *antes* de ejecutar.

    Contiene los mismos campos descriptivos que TradeEvent pero sin
    precios de salida ni R realizado.  Es la entrada al método
    ``score_signal()`` del AtlasLearningBrain.
    """

    symbol: str
    asset_class: str
    side: str
    setup_type: str
    regime: str
    timeframe: str

    entry_price: float
    stop_loss_price: float
    r_initial: float

    rsi: float = 0.0
    macd_hist: float = 0.0
    atr: float = 0.0
    bb_pct: float = 0.0
    volume_ratio: float = 1.0
    cvd: float = 0.0
    iv_rank: float = 0.0
    iv_hv_ratio: float = 1.0

    capital: float = 100_000.0
    position_size: float = 0.0

    signal_time: Optional[datetime] = None
    extra: Dict[str, Any] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# MetricsSummary — resumen estadístico de un grupo de trades
# ---------------------------------------------------------------------------

@dataclass
class MetricsSummary:
    """Métricas estadísticas para un grupo de trades (estrategia, símbolo, etc.)."""

    n_trades: int = 0
    n_winners: int = 0
    n_losers: int = 0

    winrate: float = 0.0            # n_winners / n_trades
    profit_factor: float = 0.0      # sum(ganadores R) / |sum(perdedores R)|
    expectancy_r: float = 0.0       # media(R_realizados)
    avg_winner_r: float = 0.0
    avg_loser_r: float = 0.0

    total_r: float = 0.0            # suma acumulada de R
    max_drawdown_r: float = 0.0     # peak-to-trough en R (valor negativo)
    max_consecutive_losses: int = 0
    max_consecutive_wins: int = 0

    avg_mae_r: float = 0.0
    avg_mfe_r: float = 0.0
    avg_duration_minutes: float = 0.0

    calmar_ratio: float = 0.0       # annualized_return_r / |max_drawdown_r|
    sharpe_r: float = 0.0           # media(R) / std(R) * sqrt(252)

    @property
    def is_profitable(self) -> bool:
        return self.profit_factor > 1.0 and self.expectancy_r > 0

    @property
    def is_statistically_significant(self) -> bool:
        """≥30 trades = mínimo estadístico básico."""
        return self.n_trades >= 30


# ---------------------------------------------------------------------------
# LearningReport — resultado del análisis diario/semanal
# ---------------------------------------------------------------------------

@dataclass
class LearningReport:
    """Resultado completo de ``run_daily_analysis()``.

    Contiene métricas globales + desglose por dimensiones + patrones
    detectados + políticas propuestas.
    """

    analysis_date: date
    period_start: datetime
    period_end: datetime
    n_trades_analyzed: int

    # Métricas globales
    global_metrics: MetricsSummary = field(default_factory=MetricsSummary)

    # Desglose por dimensión
    by_setup: Dict[str, MetricsSummary] = field(default_factory=dict)
    by_symbol: Dict[str, MetricsSummary] = field(default_factory=dict)
    by_timeframe: Dict[str, MetricsSummary] = field(default_factory=dict)
    by_regime: Dict[str, MetricsSummary] = field(default_factory=dict)
    by_asset_class: Dict[str, MetricsSummary] = field(default_factory=dict)
    by_hour: Dict[int, MetricsSummary] = field(default_factory=dict)  # hora UTC

    # Patrones detectados
    error_patterns: List[str] = field(default_factory=list)
    # e.g. ["late_entry aparece en 40% de pérdidas",
    #        "chased_entry correlaciona con mae_r > -2"]

    success_patterns: List[str] = field(default_factory=list)
    # e.g. ["breakout+BULL_regime: PF=2.4 (n=45)",
    #        "iv_rank>50 + iv_hv>1.2: winrate 68%"]

    # Puntuación de estabilidad temporal (first_half vs second_half)
    stability_score: float = 0.0   # 0.0 – 1.0; >0.70 = sistema estable

    # Políticas propuestas por MetricsEngine
    proposed_policies: List["PolicyAction"] = field(default_factory=list)

    # Metadatos
    generated_at: datetime = field(default_factory=datetime.utcnow)
    warnings: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# PolicyAction — acción de política individual
# ---------------------------------------------------------------------------

@dataclass
class PolicyAction:
    """Una acción de ajuste de política propuesta por el sistema de aprendizaje."""

    setup_type: str                # setup afectado; "*" = global
    action: str                    # "disable" | "reduce_size" | "increase_priority"
                                   # | "enable" | "set_threshold"
    reason: str                    # descripción legible

    size_multiplier: float = 1.0   # factor de ajuste de tamaño (0.5 = mitad)
    score_boost: float = 0.0       # ajuste al score (puede ser negativo)
    min_score_threshold: float = 0.0  # umbral mínimo de score para ejecutar

    confidence: float = 1.0        # confianza de la propuesta (0-1)
    n_trades_basis: int = 0        # trades en los que se basa


# ---------------------------------------------------------------------------
# PolicySnapshot — estado activo de políticas (caché en memoria)
# ---------------------------------------------------------------------------

@dataclass
class PolicySnapshot:
    """Estado completo de políticas de trading activas.

    Es el resultado de ``get_policy_snapshot()`` y la referencia
    que usan ``score_signal()`` y ``live_loop`` para decidir si
    ejecutar una señal.
    """

    enabled_setups: List[str] = field(default_factory=list)
    disabled_setups: List[str] = field(default_factory=list)

    # Multiplicadores de tamaño por setup (default = 1.0)
    size_multipliers: Dict[str, float] = field(default_factory=dict)

    # Umbrales mínimos de score por setup (default usa global)
    score_thresholds: Dict[str, float] = field(default_factory=dict)

    # Umbral global de score mínimo para ejecutar cualquier señal
    global_score_threshold: float = 0.40

    # Ajustes de score por setup (boost/penalty)
    score_boosts: Dict[str, float] = field(default_factory=dict)

    # Setups que requieren confirmación manual antes de ejecutar
    manual_review_setups: List[str] = field(default_factory=list)

    # Timestamp de la última actualización
    last_updated: Optional[datetime] = None

    # Fuente de la última actualización
    last_update_source: str = "init"   # "daily_analysis" | "manual" | "init"

    def is_setup_enabled(self, setup_type: str) -> bool:
        if setup_type in self.disabled_setups:
            return False
        return True  # default: habilitado si no está explícitamente deshabilitado

    def get_size_multiplier(self, setup_type: str) -> float:
        return self.size_multipliers.get(setup_type, 1.0)

    def get_score_threshold(self, setup_type: str) -> float:
        return self.score_thresholds.get(setup_type, self.global_score_threshold)

    def get_score_boost(self, setup_type: str) -> float:
        return self.score_boosts.get(setup_type, 0.0)


# ---------------------------------------------------------------------------
# ReadinessCriterion — criterio individual de readiness para live
# ---------------------------------------------------------------------------

@dataclass
class ReadinessCriterion:
    """Un criterio individual del sistema de readiness."""

    name: str               # identificador del criterio
    description: str        # descripción legible
    value: float            # valor actual medido
    threshold: float        # valor mínimo requerido
    passed: bool            # value >= threshold (o lógica propia)
    weight: float = 1.0     # peso en la decisión final (informativo)
    unit: str = ""          # "R", "%", "months", "trades", etc.

    @property
    def gap(self) -> float:
        """Cuánto falta para pasar (negativo = ya pasó)."""
        return self.threshold - self.value


# ---------------------------------------------------------------------------
# SystemReadinessReport — evaluación completa del sistema para live
# ---------------------------------------------------------------------------

@dataclass
class SystemReadinessReport:
    """Resultado de ``is_system_ready_for_live()``.

    Evalúa 7 criterios cuantitativos y emite un veredicto binario.
    """

    ready: bool                   # True solo si TODOS los criterios pasan
    criteria: List[ReadinessCriterion] = field(default_factory=list)
    passed: List[str] = field(default_factory=list)   # nombres de criterios OK
    failed: List[str] = field(default_factory=list)   # nombres de criterios NOK

    evaluated_at: datetime = field(default_factory=datetime.utcnow)
    n_trades_evaluated: int = 0

    # Mensaje de estado para UI/log
    summary: str = ""

    # Consejo accionable cuando no está listo
    next_step: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "ready": self.ready,
            "passed": self.passed,
            "failed": self.failed,
            "n_trades_evaluated": self.n_trades_evaluated,
            "summary": self.summary,
            "next_step": self.next_step,
            "evaluated_at": self.evaluated_at.isoformat(),
            "criteria": [
                {
                    "name": c.name,
                    "value": c.value,
                    "threshold": c.threshold,
                    "passed": c.passed,
                    "unit": c.unit,
                }
                for c in self.criteria
            ],
        }


# ---------------------------------------------------------------------------
# ScoreResult — resultado del scoring híbrido de una señal
# ---------------------------------------------------------------------------

@dataclass
class ScoreResult:
    """Resultado de ``score_signal()``."""

    total_score: float          # 0.0 – 1.0; combinación ponderada
    ml_score: float             # score de la capa ML
    stats_score: float          # score de la capa estadística

    approved: bool              # total_score >= threshold activo
    threshold_used: float       # umbral que se comparó

    setup_type: str = ""
    symbol: str = ""
    regime: str = ""

    # Detalle explicativo (para logs)
    ml_available: bool = True
    stats_basis_n: int = 0      # cuántos trades históricos informaron stats_score
    reasoning: str = ""

    # Ajustes aplicados
    size_multiplier: float = 1.0
    score_boost_applied: float = 0.0
