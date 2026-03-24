"""AtlasLearningBrain — cerebro de aprendizaje híbrido del sistema ATLAS-Quant.

Integra las capas estadística y ML en una única clase con 6 métodos públicos:

    1. record_trade(trade_event)
       Registra un trade cerrado en el historial interno.

    2. run_daily_analysis(date) → LearningReport
       Genera métricas completas, detecta patrones y propone políticas.

    3. update_policies(report) → PolicySnapshot
       Aplica las políticas propuestas en el reporte al PolicyManager.

    4. get_policy_snapshot() → PolicySnapshot
       Devuelve el estado activo de políticas (caché 4h).

    5. score_signal(signal_context) → ScoreResult
       Scoring híbrido: 0.4 × ML + 0.6 × stats.

    6. is_system_ready_for_live() → SystemReadinessReport
       Evalúa 7 criterios cuantitativos para habilitar cuenta real.

Arquitectura
-----------
    AtlasLearningBrain
        ├── _trade_store: List[TradeEvent]      # historial en memoria
        ├── _metrics_engine: módulo puro        # sin estado
        ├── MLSignalRanker                      # capa B (ML)
        └── PolicyManager                       # snapshot + persistencia

El historial se persiste opcionalmente en SQLite via LearningTradeRecord
(ver journal/models.py) — si no está disponible usa sólo memoria.
"""
from __future__ import annotations

import logging
import threading
from datetime import date, datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional

from atlas_code_quant.learning.metrics_engine import (
    build_learning_report,
    check_readiness,
    compute_metrics,
)
from atlas_code_quant.learning.ml_signal_ranker import MLSignalRanker
from atlas_code_quant.learning.policy_manager import PolicyManager
from atlas_code_quant.learning.trade_events import (
    LearningReport,
    PolicySnapshot,
    ScoreResult,
    SignalContext,
    SystemReadinessReport,
    TradeEvent,
)

logger = logging.getLogger(__name__)


class AtlasLearningBrain:
    """Cerebro de aprendizaje híbrido del sistema ATLAS-Quant.

    Args:
        policy_path:     ruta al JSON de persistencia de políticas
        ml_model_path:   ruta al pickle del modelo ML
        readiness_cfg:   dict con umbrales de readiness (override)
        ml_weight:       peso de la capa ML en el score (default 0.4)
        stats_weight:    peso de la capa stats en el score (default 0.6)
        retrain_every_n: reentrenar ML cada N trades nuevos (default 50)
    """

    _DEFAULT_READINESS = {
        "min_n_trades": 300,
        "min_months": 3.0,
        "min_profit_factor": 1.5,
        "min_calmar": 1.5,
        "max_dd_pct": 15.0,
        "min_expectancy_r": 0.20,
        "min_stability": 0.70,
    }

    def __init__(
        self,
        policy_path: Optional[Path] = None,
        ml_model_path: Optional[Path] = None,
        readiness_cfg: Optional[Dict[str, Any]] = None,
        ml_weight: float = 0.4,
        stats_weight: float = 0.6,
        retrain_every_n: int = 50,
    ):
        self._lock = threading.RLock()
        self._trade_store: List[TradeEvent] = []
        self._trades_since_retrain: int = 0
        self._retrain_every_n = retrain_every_n

        self.ml_weight = ml_weight
        self.stats_weight = stats_weight

        # Normalizar pesos
        total_w = ml_weight + stats_weight
        if total_w > 0:
            self.ml_weight = ml_weight / total_w
            self.stats_weight = stats_weight / total_w

        self._readiness_cfg = {**self._DEFAULT_READINESS, **(readiness_cfg or {})}

        self._policy_manager = PolicyManager(storage_path=policy_path)
        self._ml_ranker = MLSignalRanker(model_path=ml_model_path)

        # Caché del último reporte diario
        self._last_report: Optional[LearningReport] = None
        self._last_report_date: Optional[date] = None

        logger.info(
            "AtlasLearningBrain iniciado: ml_weight=%.2f, stats_weight=%.2f, "
            "retrain_every_n=%d",
            self.ml_weight, self.stats_weight, retrain_every_n,
        )

    # ------------------------------------------------------------------
    # 1. record_trade
    # ------------------------------------------------------------------

    def record_trade(self, trade_event: TradeEvent) -> None:
        """Registra un trade cerrado en el historial interno.

        También persiste en SQLite si `LearningTradeRecord` está disponible.
        Activa reentrenamiento del modelo ML cada `retrain_every_n` trades.
        """
        with self._lock:
            self._trade_store.append(trade_event)
            self._trades_since_retrain += 1

            # Persistir en SQLite (no-bloquea si el módulo no está disponible)
            self._persist_to_db(trade_event)

            # Reentrenamiento automático
            if self._trades_since_retrain >= self._retrain_every_n:
                self._retrain_ml()
                self._trades_since_retrain = 0

        logger.debug(
            "record_trade: %s %s %.2fR (total=%d)",
            trade_event.symbol, trade_event.setup_type,
            trade_event.r_realized, len(self._trade_store),
        )

    # ------------------------------------------------------------------
    # 2. run_daily_analysis
    # ------------------------------------------------------------------

    def run_daily_analysis(
        self,
        analysis_date: Optional[date] = None,
        lookback_days: int = 90,
    ) -> LearningReport:
        """Genera análisis completo de performance para `analysis_date`.

        Args:
            analysis_date: fecha del análisis (default hoy UTC)
            lookback_days: cuántos días hacia atrás considerar (default 90)

        Returns:
            LearningReport con métricas, patrones y políticas propuestas.
        """
        if analysis_date is None:
            analysis_date = datetime.utcnow().date()

        period_end = datetime.combine(analysis_date, datetime.max.time())
        period_start = period_end - timedelta(days=lookback_days)

        with self._lock:
            trades = [
                t for t in self._trade_store
                if period_start <= t.entry_time <= period_end
            ]

        report = build_learning_report(
            trades=trades,
            analysis_date=analysis_date,
            period_start=period_start,
            period_end=period_end,
        )

        with self._lock:
            self._last_report = report
            self._last_report_date = analysis_date

        logger.info(
            "run_daily_analysis(%s): n=%d, PF=%.2f, stability=%.2f, "
            "policies_proposed=%d",
            analysis_date, report.n_trades_analyzed,
            report.global_metrics.profit_factor,
            report.stability_score,
            len(report.proposed_policies),
        )
        return report

    # ------------------------------------------------------------------
    # 3. update_policies
    # ------------------------------------------------------------------

    def update_policies(self, report: LearningReport) -> PolicySnapshot:
        """Aplica las políticas propuestas en el reporte al PolicyManager.

        Las políticas son siempre el output de ``run_daily_analysis()``.
        Persiste el snapshot actualizado en disco.

        Returns:
            nuevo PolicySnapshot activo.
        """
        snapshot = self._policy_manager.apply_policies(
            actions=report.proposed_policies,
            source=f"daily_analysis_{report.analysis_date.isoformat()}",
        )

        if report.proposed_policies:
            actions_log = [
                f"{a.setup_type}:{a.action}(×{a.size_multiplier:.2f})"
                for a in report.proposed_policies
            ]
            logger.info(
                "update_policies: %d acciones aplicadas — %s",
                len(report.proposed_policies), ", ".join(actions_log),
            )
        else:
            logger.info("update_policies: sin cambios de política")

        return snapshot

    # ------------------------------------------------------------------
    # 4. get_policy_snapshot
    # ------------------------------------------------------------------

    def get_policy_snapshot(self) -> PolicySnapshot:
        """Devuelve el PolicySnapshot activo (caché 4h)."""
        return self._policy_manager.get_snapshot()

    # ------------------------------------------------------------------
    # 5. score_signal
    # ------------------------------------------------------------------

    def score_signal(self, ctx: SignalContext) -> ScoreResult:
        """Scoring híbrido de una señal en vivo.

        Fórmula:
            total_score = w_ml × ml_score + w_stats × stats_score

        La capa stats usa el historial filtrado por setup_type del contexto.
        La capa ML delega a MLSignalRanker.

        Returns:
            ScoreResult con score total, aprobación y detalles.
        """
        snapshot = self._policy_manager.get_snapshot()

        # Verificar si el setup está habilitado
        if not snapshot.is_setup_enabled(ctx.setup_type):
            return ScoreResult(
                total_score=0.0,
                ml_score=0.0,
                stats_score=0.0,
                approved=False,
                threshold_used=1.0,  # umbral inalcanzable
                setup_type=ctx.setup_type,
                symbol=ctx.symbol,
                regime=ctx.regime,
                ml_available=self._ml_ranker.is_trained,
                stats_basis_n=0,
                reasoning=f"Setup '{ctx.setup_type}' está deshabilitado por política",
                size_multiplier=0.0,
            )

        # --- Capa A: stats ---
        stats_score, stats_n = self._compute_stats_score(ctx)

        # --- Capa B: ML ---
        ml_score = self._ml_ranker.score(ctx)
        ml_available = self._ml_ranker.is_trained

        # Si ML no disponible → usar solo stats con weight=1.0
        if not ml_available:
            total_score = stats_score
        else:
            total_score = self.ml_weight * ml_score + self.stats_weight * stats_score

        # Aplicar boost de política para este setup
        boost = snapshot.get_score_boost(ctx.setup_type)
        total_score = max(0.0, min(1.0, total_score + boost))

        # Determinar umbral y aprobación
        threshold = snapshot.get_score_threshold(ctx.setup_type)
        approved = total_score >= threshold

        # Multiplicador de tamaño de política
        size_mult = snapshot.get_size_multiplier(ctx.setup_type)

        reasoning = (
            f"ml={ml_score:.3f}(w={self.ml_weight:.2f}) + "
            f"stats={stats_score:.3f}(w={self.stats_weight:.2f})"
            f"{f' +boost={boost:.2f}' if boost else ''} "
            f"→ total={total_score:.3f} vs threshold={threshold:.2f}"
        )

        if not ml_available:
            reasoning = f"ML no disponible — stats_only={stats_score:.3f} vs threshold={threshold:.2f}"

        return ScoreResult(
            total_score=round(total_score, 4),
            ml_score=round(ml_score, 4),
            stats_score=round(stats_score, 4),
            approved=approved,
            threshold_used=threshold,
            setup_type=ctx.setup_type,
            symbol=ctx.symbol,
            regime=ctx.regime,
            ml_available=ml_available,
            stats_basis_n=stats_n,
            reasoning=reasoning,
            size_multiplier=size_mult,
            score_boost_applied=boost,
        )

    # ------------------------------------------------------------------
    # 6. is_system_ready_for_live
    # ------------------------------------------------------------------

    def is_system_ready_for_live(self) -> SystemReadinessReport:
        """Evalúa si el sistema está listo para cuenta real (7 criterios).

        Usa todos los trades registrados en el historial interno.
        Los umbrales se pueden configurar en el constructor vía readiness_cfg.

        Returns:
            SystemReadinessReport con veredicto y detalles por criterio.
        """
        with self._lock:
            trades = list(self._trade_store)

        report = check_readiness(trades, **self._readiness_cfg)

        logger.info(
            "is_system_ready_for_live: ready=%s, n=%d, failed=%s",
            report.ready, report.n_trades_evaluated, report.failed,
        )
        return report

    # ------------------------------------------------------------------
    # Helpers internos
    # ------------------------------------------------------------------

    def _compute_stats_score(self, ctx: SignalContext) -> tuple[float, int]:
        """Calcula score estadístico basado en historial del mismo setup.

        Retorna (score 0-1, n_trades_usados).

        Lógica:
        - Filtra trades con mismo setup_type (y opcionalmente regime)
        - Calcula profit_factor, winrate, expectancy_r
        - Combina en score normalizado
        """
        with self._lock:
            # Filtrar por setup
            setup_trades = [
                t for t in self._trade_store
                if t.setup_type == ctx.setup_type
            ]

        n = len(setup_trades)
        if n < 10:
            # Sin datos suficientes → score neutral
            return 0.5, n

        metrics = compute_metrics(setup_trades)

        # Normalizar profit_factor → 0-1
        # PF=1.0 → 0.4, PF=2.0 → 0.7, PF=3.0+ → 1.0
        pf_score = min(1.0, max(0.0, (metrics.profit_factor - 0.5) / 2.5))

        # Normalizar winrate → 0-1
        wr_score = min(1.0, max(0.0, metrics.winrate))

        # Normalizar expectancy_r: E[R]=0 → 0.4, E[R]=0.5 → 0.7, E[R]=1.0+ → 1.0
        exp_score = min(1.0, max(0.0, 0.4 + metrics.expectancy_r * 0.6))

        # Bonus por tamaño de muestra (más trades = más confianza)
        sample_weight = min(1.0, n / 50)  # máximo peso con ≥50 trades

        raw = (pf_score * 0.45 + wr_score * 0.30 + exp_score * 0.25)
        # Ajustar hacia neutral (0.5) si muestra pequeña
        score = 0.5 + (raw - 0.5) * sample_weight

        return round(score, 4), n

    def _persist_to_db(self, trade: TradeEvent) -> None:
        """Persiste en LearningTradeRecord (SQLAlchemy). Falla silenciosamente."""
        try:
            from atlas_code_quant.journal.models import LearningTradeRecord
            from atlas_code_quant.journal.db import session_scope
            record = LearningTradeRecord.from_trade_event(trade)
            with session_scope() as session:
                session.add(record)
        except Exception:
            # No disponible o error de DB — continuar solo con memoria
            pass

    def _retrain_ml(self) -> None:
        """Reentrenar modelo ML con el historial actual."""
        with self._lock:
            trades = list(self._trade_store)

        result = self._ml_ranker.train(trades)
        if result.get("trained"):
            logger.info(
                "ML reentrenado: backend=%s, n=%d",
                result.get("backend"), result.get("n_samples"),
            )

    # ------------------------------------------------------------------
    # Estado y utilidades
    # ------------------------------------------------------------------

    def status(self) -> Dict[str, Any]:
        """Resumen del estado del cerebro."""
        with self._lock:
            n_trades = len(self._trade_store)

        snapshot = self._policy_manager.get_snapshot()
        return {
            "n_trades_in_memory": n_trades,
            "ml_ranker": self._ml_ranker.status(),
            "policy": self._policy_manager.status(),
            "last_report_date": (
                self._last_report_date.isoformat() if self._last_report_date else None
            ),
            "readiness_cfg": self._readiness_cfg,
        }

    def load_trades_from_db(self) -> int:
        """Carga historial desde LearningTradeRecord en SQLite al arrancar.

        Retorna el número de trades cargados.
        """
        try:
            from atlas_code_quant.journal.models import LearningTradeRecord
            from atlas_code_quant.journal.db import session_scope
            with session_scope() as session:
                records = session.query(LearningTradeRecord).all()
            trades = [r.to_trade_event() for r in records]
            with self._lock:
                self._trade_store = trades
            logger.info("load_trades_from_db: %d trades cargados", len(trades))
            return len(trades)
        except Exception as exc:
            logger.warning("load_trades_from_db: %s", exc)
            return 0

    def force_retrain(self) -> Dict[str, Any]:
        """Fuerza reentrenamiento ML inmediato (útil post-carga de DB)."""
        with self._lock:
            trades = list(self._trade_store)
        result = self._ml_ranker.train(trades)
        self._trades_since_retrain = 0
        return result

    def revert_policies(self) -> Optional[PolicySnapshot]:
        """Revierte las últimas políticas aplicadas."""
        return self._policy_manager.revert()
