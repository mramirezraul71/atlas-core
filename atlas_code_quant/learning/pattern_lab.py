"""Pattern Lab — Orquestador del Módulo de Aprendizaje de Patrones.

PatternLabService orquesta los cuatro sub-módulos:
    1. MotifDiscoveryService   → motif_edge_score
    2. IndicatorSelector       → selección dinámica de features
    3. TINTrainer / TINPredictor → tin_prob_positive
    4. TimeSeriesBaselines     → arima_deviation, ets_deviation

Y produce PatternLabContext con el campo `signal_score` [0-1]:

    signal_score = w_motif × motif_edge_score
                 + w_tin   × tin_prob_positive
                 + w_regime × regime_conf

Flujo offline (entrenamiento):
    lab = PatternLabService()
    lab.fit(price_df, ohlcv_df, config)

Flujo en tiempo real (inferencia):
    ctx = lab.evaluate(recent_window_df, regime_output, price_series)
    # ctx.signal_score, ctx.motif_edge_score, ctx.tin_prob_positive, ...
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from atlas_code_quant.learning.motif_lab import (
    MotifConfig,
    MotifDiscoveryService,
    ReturnDistribution,
)
from atlas_code_quant.learning.indicator_selector import (
    IndicatorSelector,
    IndicatorSelectorConfig,
    IndicatorRanking,
)
from atlas_code_quant.learning.tin_models import (
    TINConfig,
    TINPredictor,
    TINTrainer,
)
from atlas_code_quant.learning.ts_baselines import (
    TSBaselineConfig,
    TSForecast,
    TimeSeriesBaselines,
)

logger = logging.getLogger("quant.pattern_lab")


# ── Configuración global ───────────────────────────────────────────────────────

@dataclass
class PatternLabConfig:
    """Configuración maestra del Pattern Lab."""

    # Pesos del signal_score compuesto
    w_motif:  float = 0.35
    w_tin:    float = 0.40
    w_regime: float = 0.25

    # Sub-configs
    motif:     MotifConfig            = field(default_factory=MotifConfig)
    indicator: IndicatorSelectorConfig = field(default_factory=IndicatorSelectorConfig)
    tin:       TINConfig              = field(default_factory=TINConfig)
    ts:        TSBaselineConfig       = field(default_factory=TSBaselineConfig)

    # Comportamiento
    top_n_indicators: int = 20
    """Número de indicadores top para alimentar el motif discovery y el TIN."""
    min_signal_score_to_pass: float = 0.55
    """Umbral mínimo de signal_score para pasar como señal válida."""
    neutral_regime_conf: float = 0.60
    """Confianza de régimen que se usa cuando no se provee RegimeOutput."""
    enable_ts_baselines: bool = True
    """Deshabilitar si statsmodels no está instalado."""
    enable_tin: bool = True
    """Deshabilitar para usar solo motifs + régimen."""

    def __post_init__(self):
        # Normalizar pesos
        total = self.w_motif + self.w_tin + self.w_regime
        if abs(total - 1.0) > 0.01:
            self.w_motif  /= total
            self.w_tin    /= total
            self.w_regime /= total


# ── Contexto de salida ─────────────────────────────────────────────────────────

@dataclass
class PatternLabContext:
    """
    Resultado completo del Pattern Lab para una barra / símbolo.

    El campo `signal_score` [0-1] es el resumen utilizable por
    SignalGenerator y KellyRiskEngine.
    """
    symbol: str = ""
    timestamp: float = field(default_factory=time.time)

    # Sub-scores componentes
    motif_edge_score:  float = 0.5
    """Edge del motif: 0.5=neutral, >0.5=ventaja long."""
    tin_prob_positive: float = 0.5
    """Probabilidad TIN de retorno positivo [0-1]."""
    regime_conf:       float = 0.5
    """Confianza del régimen actual [0-1]."""

    # Score compuesto
    signal_score: float = 0.5
    """Score final [0-1]. Umbral recomendado: >0.55 para long, <0.45 para short."""

    # Info adicional TS
    arima_deviation:   float = 0.0
    ets_deviation:     float = 0.0
    arima_zscore:      float = 0.0
    trend_slope:       float = 0.0

    # Motif stats
    n_motifs_found:    int = 0
    motif_n_samples:   int = 0
    motif_horizon:     int = 5

    # TIN info
    tin_backend:       str = ""

    # Flags de calidad
    is_valid:          bool = True
    """False si no hay suficientes datos para producir scores fiables."""
    warnings:          list[str] = field(default_factory=list)

    # Meta
    metadata:          dict = field(default_factory=dict)

    def passes(self, config: PatternLabConfig | None = None) -> bool:
        """True si el signal_score supera el umbral mínimo."""
        threshold = config.min_signal_score_to_pass if config else 0.55
        return self.is_valid and self.signal_score >= threshold

    def to_dict(self) -> dict[str, Any]:
        return {
            "symbol": self.symbol,
            "signal_score": round(self.signal_score, 4),
            "motif_edge_score": round(self.motif_edge_score, 4),
            "tin_prob_positive": round(self.tin_prob_positive, 4),
            "regime_conf": round(self.regime_conf, 4),
            "arima_deviation": round(self.arima_deviation, 6),
            "ets_deviation": round(self.ets_deviation, 6),
            "arima_zscore": round(self.arima_zscore, 4),
            "trend_slope": round(self.trend_slope, 6),
            "n_motifs_found": self.n_motifs_found,
            "motif_n_samples": self.motif_n_samples,
            "tin_backend": self.tin_backend,
            "is_valid": self.is_valid,
            "warnings": self.warnings,
            **self.metadata,
        }


# ── Servicio principal ─────────────────────────────────────────────────────────

class PatternLabService:
    """
    Orquesta el ciclo completo del Pattern Lab:
    fit offline → evaluate en tiempo real.

    Diseñado para mantener estado entre llamadas (modelo ajustado)
    y operar con latencia <100ms por llamada a evaluate().
    """

    def __init__(self, config: PatternLabConfig | None = None) -> None:
        self.config = config or PatternLabConfig()

        self._indicator_selector = IndicatorSelector()
        self._motif_service      = MotifDiscoveryService()
        self._tin_predictor: TINPredictor | None = None
        self._ts_baselines: dict[str, TimeSeriesBaselines] = {}
        # Per-symbol ts baselines
        self._ts_global: TimeSeriesBaselines | None = None

        self._top_indicators: list[str] = []
        self._indicator_ranking: IndicatorRanking | None = None
        self._is_fitted: bool = False
        self._fit_metadata: dict = {}

    # ── Entrenamiento offline ──────────────────────────────────────────────────

    def fit(
        self,
        price_series: pd.Series,
        ohlcv_df: pd.DataFrame | None = None,
        config: PatternLabConfig | None = None,
    ) -> "PatternLabService":
        """
        Ajusta todos los sub-módulos con datos históricos.

        Args:
            price_series: Serie de precios de cierre con índice temporal.
            ohlcv_df:     DataFrame OHLCV completo. Si None, se usa solo close.
            config:       Configuración (sobreescribe la del constructor si se provee).

        Returns:
            self (para encadenamiento).
        """
        if config is not None:
            self.config = config
        cfg = self.config

        t0 = time.time()
        logger.info("PatternLabService.fit: iniciando con %d barras...", len(price_series))

        # ── 1. Construir pool de indicadores ──────────────────────────────────
        base_df = ohlcv_df if ohlcv_df is not None else price_series.rename("close").to_frame()
        try:
            all_features = self._indicator_selector.build_indicator_pool(base_df)
        except Exception as e:
            logger.warning("PatternLabService.fit: build_indicator_pool falló: %s", e)
            all_features = price_series.rename("close").to_frame()

        # ── 2. Rankear indicadores ────────────────────────────────────────────
        try:
            self._indicator_ranking = self._indicator_selector.evaluate_indicators(
                features=all_features,
                config=cfg.indicator,
                price_series=price_series,
            )
            self._top_indicators = self._indicator_selector.get_top_indicators(
                self._indicator_ranking, top_n=cfg.top_n_indicators
            )
        except Exception as e:
            logger.warning("PatternLabService.fit: evaluate_indicators falló: %s", e)
            self._top_indicators = [c for c in all_features.columns[:cfg.top_n_indicators]]

        logger.info(
            "PatternLabService: top %d indicadores: %s",
            len(self._top_indicators),
            ", ".join(self._top_indicators[:5]) + ("..." if len(self._top_indicators) > 5 else ""),
        )

        # ── 3. Filtrar feature matrix a top indicadores ───────────────────────
        avail_top = [c for c in self._top_indicators if c in all_features.columns]
        feature_df = all_features[avail_top] if avail_top else all_features

        # ── 4. Ajustar MotifDiscoveryService ─────────────────────────────────
        try:
            self._motif_service.fit(price_series, feature_df, cfg.motif)
        except Exception as e:
            logger.warning("PatternLabService.fit: motif fit falló: %s", e)

        # ── 5. Entrenar TIN ───────────────────────────────────────────────────
        if cfg.enable_tin:
            try:
                trainer = TINTrainer(cfg.tin)
                self._tin_predictor = trainer.fit(
                    features=feature_df,
                    price_series=price_series,
                )
                logger.info(
                    "PatternLabService: TIN entrenado (%s)", self._tin_predictor.backend
                )
            except Exception as e:
                logger.warning("PatternLabService.fit: TIN fit falló: %s", e)
                self._tin_predictor = None

        # ── 6. Ajustar baselines TS ───────────────────────────────────────────
        if cfg.enable_ts_baselines:
            try:
                self._ts_global = TimeSeriesBaselines(cfg.ts)
                self._ts_global.fit(price_series)
            except Exception as e:
                logger.warning("PatternLabService.fit: TS baselines fit falló: %s", e)
                self._ts_global = None

        elapsed = time.time() - t0
        self._is_fitted = True
        self._fit_metadata = {
            "n_bars": len(price_series),
            "n_indicators": len(self._top_indicators),
            "tin_backend": self._tin_predictor.backend if self._tin_predictor else "none",
            "ts_fitted": self._ts_global is not None and self._ts_global.is_fitted,
            "elapsed_s": round(elapsed, 2),
        }
        logger.info(
            "PatternLabService.fit completado en %.1fs — %s",
            elapsed, self._fit_metadata,
        )
        return self

    # ── Evaluación en tiempo real ──────────────────────────────────────────────

    def evaluate(
        self,
        recent_window: pd.DataFrame,
        regime_output=None,
        price_series: pd.Series | None = None,
        symbol: str = "",
        horizon_bars: int | None = None,
    ) -> PatternLabContext:
        """
        Produce PatternLabContext para la ventana de features actual.

        Args:
            recent_window:  DataFrame de features de las últimas N barras.
                            Debe tener las mismas columnas que se usaron en fit().
            regime_output:  RegimeOutput del RegimeClassifier (opcional).
                            Si None, se usa neutral_regime_conf.
            price_series:   Serie de precios recientes (para TS baselines).
            symbol:         Identificador del símbolo.
            horizon_bars:   Horizonte de forecast. Usa config.motif.window_size si None.

        Returns:
            PatternLabContext con signal_score y sub-scores.
        """
        cfg = self.config
        ctx = PatternLabContext(symbol=symbol, timestamp=time.time())

        if not self._is_fitted:
            ctx.is_valid = False
            ctx.warnings.append("Pattern Lab no ajustado — llame fit() primero.")
            return ctx

        h = horizon_bars or cfg.motif.target_horizon if hasattr(cfg.motif, "target_horizon") else 5

        # ── 1. Motif edge score ───────────────────────────────────────────────
        try:
            dist: ReturnDistribution = self._motif_service.forecast_distribution(
                recent_window, horizon_bars=h
            )
            ctx.motif_edge_score = dist.edge_score
            ctx.n_motifs_found   = dist.metadata.get("n_motifs_found", 0)
            ctx.motif_n_samples  = dist.n_samples
            ctx.motif_horizon    = h
        except Exception as e:
            logger.debug("PatternLabService.evaluate: motif error: %s", e)
            ctx.warnings.append(f"motif_error: {e}")

        # ── 2. TIN prob positive ──────────────────────────────────────────────
        if cfg.enable_tin and self._tin_predictor is not None:
            try:
                ctx.tin_prob_positive = self._tin_predictor.predict_proba(recent_window)
                ctx.tin_backend       = self._tin_predictor.backend
            except Exception as e:
                logger.debug("PatternLabService.evaluate: TIN error: %s", e)
                ctx.warnings.append(f"tin_error: {e}")
        else:
            ctx.tin_backend = "disabled"

        # ── 3. Regime conf ────────────────────────────────────────────────────
        if regime_output is not None:
            try:
                ctx.regime_conf = float(getattr(regime_output, "confidence", cfg.neutral_regime_conf))
            except Exception:
                ctx.regime_conf = cfg.neutral_regime_conf
        else:
            ctx.regime_conf = cfg.neutral_regime_conf

        # ── 4. TS baselines ───────────────────────────────────────────────────
        if cfg.enable_ts_baselines and self._ts_global is not None and price_series is not None:
            try:
                arima_fc = self._ts_global.forecast_now(price_series, model="arima")
                ets_fc   = self._ts_global.forecast_now(price_series, model="ets")
                ctx.arima_deviation = arima_fc.deviation_from_forecast
                ctx.ets_deviation   = ets_fc.deviation_from_forecast
                ctx.arima_zscore    = arima_fc.residual_zscore
                ctx.trend_slope     = arima_fc.trend_slope
            except Exception as e:
                logger.debug("PatternLabService.evaluate: TS error: %s", e)
                ctx.warnings.append(f"ts_error: {e}")

        # ── 5. Signal score compuesto ─────────────────────────────────────────
        ctx.signal_score = _compute_signal_score(
            motif_edge=ctx.motif_edge_score,
            tin_prob=ctx.tin_prob_positive,
            regime_conf=ctx.regime_conf,
            w_motif=cfg.w_motif,
            w_tin=cfg.w_tin,
            w_regime=cfg.w_regime,
        )

        # ── 6. Validez ────────────────────────────────────────────────────────
        ctx.is_valid = (
            ctx.motif_n_samples >= 1 or self._tin_predictor is not None
        )

        return ctx

    # ── Fit por símbolo ────────────────────────────────────────────────────────

    def fit_symbol(
        self,
        symbol: str,
        price_series: pd.Series,
        ohlcv_df: pd.DataFrame | None = None,
    ) -> None:
        """Ajusta TS baselines para un símbolo específico."""
        cfg = self.config
        if not cfg.enable_ts_baselines:
            return
        try:
            ts = TimeSeriesBaselines(cfg.ts)
            ts.fit(price_series)
            self._ts_baselines[symbol] = ts
            logger.debug("PatternLabService: TS ajustado para %s", symbol)
        except Exception as e:
            logger.warning("PatternLabService.fit_symbol(%s): %s", symbol, e)

    def evaluate_symbol(
        self,
        symbol: str,
        recent_window: pd.DataFrame,
        regime_output=None,
        price_series: pd.Series | None = None,
        horizon_bars: int | None = None,
    ) -> PatternLabContext:
        """
        Evaluación con TS baselines por símbolo.
        Si no hay TS por símbolo, usa el global.
        """
        # Redirigir al TS del símbolo si existe
        orig_global = self._ts_global
        if symbol in self._ts_baselines:
            self._ts_global = self._ts_baselines[symbol]
        ctx = self.evaluate(recent_window, regime_output, price_series, symbol, horizon_bars)
        self._ts_global = orig_global
        return ctx

    # ── Estado / info ──────────────────────────────────────────────────────────

    @property
    def is_fitted(self) -> bool:
        return self._is_fitted

    @property
    def top_indicators(self) -> list[str]:
        return list(self._top_indicators)

    def status(self) -> dict[str, Any]:
        return {
            "is_fitted": self._is_fitted,
            "top_indicators": self._top_indicators[:10],
            "tin_backend": self._tin_predictor.backend if self._tin_predictor else "none",
            "ts_fitted": self._ts_global is not None and self._ts_global.is_fitted,
            "motif_fitted": self._motif_service._is_fitted,
            "config_weights": {
                "w_motif": self.config.w_motif,
                "w_tin": self.config.w_tin,
                "w_regime": self.config.w_regime,
            },
            **self._fit_metadata,
        }

    # ── Hook LEAN Simulator ────────────────────────────────────────────────────

    def retrain_from_lean(
        self,
        years: int = 5,
        symbols: list[str] | None = None,
        use_full_universe: bool = True,
        output_dir: str | None = None,
    ) -> dict:
        """Alias principal — firma del prompt original."""
        return self.retrain_from_lean_simulator(
            symbols=symbols,
            years=years,
            out_dir=output_dir,
            use_full_universe=use_full_universe,
        )

    def retrain_from_lean_simulator(
        self,
        symbols: list[str] | None = None,
        years: int = 5,
        out_dir: str | None = None,
        use_full_universe: bool = False,
    ) -> dict:
        """Reentrena motifs y TIN usando datos sintéticos del LeanSimulator.

        Genera un dataset de entrenamiento (GBM + Markov), luego ajusta
        motif_service y tin_predictor sobre los trades producidos.

        Args:
            symbols: Lista de tickers a simular. Default: ["SPY", "QQQ", "IWM"].
            years:   Años de historia sintética a generar por símbolo.
            out_dir: Carpeta de salida para los CSV. Default: logs/lean_sim/.

        Returns:
            dict con paths generados y métricas de reentrenamiento.
        """
        from atlas_code_quant.backtest.lean_simulator import LeanSimulator  # lazy import

        _out = out_dir or str(Path(__file__).resolve().parents[2] / "logs" / "lean_sim")

        logger.info(
            "PatternLabService.retrain_from_lean_simulator: use_full_universe=%s years=%d",
            use_full_universe, years,
        )

        simulator = LeanSimulator(
            symbols=symbols,
            use_full_universe=use_full_universe,
            out_dir=_out,
        )
        paths = simulator.generate_training_dataset(years=years, out_dir=_out)

        print(f"PatternLab reentrenado con {paths.get('total_trades', 0):,} trades")

        trades_path   = paths.get("trades_csv")
        features_path = paths.get("features_csv")

        if not trades_path or not Path(trades_path).exists():
            logger.warning("LeanSim: trades_csv no generado — abortando retrain")
            return {"ok": False, "error": "trades_csv missing", "paths": paths}

        # ── Cargar datos y reentrenar ─────────────────────────────────────────
        try:
            trades_df   = pd.read_csv(trades_path)
            features_df = pd.read_csv(features_path) if features_path and Path(features_path).exists() else None
        except Exception as exc:
            logger.error("LeanSim: error cargando CSV: %s", exc)
            return {"ok": False, "error": str(exc), "paths": paths}

        results: dict = {"ok": True, "paths": paths, "motif": {}, "tin": {}}

        # ── Reentrenar MotifDiscoveryService ─────────────────────────────────
        if "close" in trades_df.columns:
            close_series = trades_df.set_index(
                trades_df.columns[0] if trades_df.columns[0] != "close" else "timestamp"
            )["close"].dropna()
            try:
                self._motif_service.fit(close_series, n_motifs=self.config.n_motifs)
                results["motif"] = {"fitted": True, "n_bars": len(close_series)}
                logger.info("LeanSim: motif_service reentrenado con %d barras", len(close_series))
            except Exception as exc:
                logger.error("LeanSim: motif_service.fit falló: %s", exc)
                results["motif"] = {"fitted": False, "error": str(exc)}

        # ── Reentrenar TINPredictor desde features ────────────────────────────
        if features_df is not None and self._tin_predictor is not None:
            feature_cols = [c for c in features_df.columns
                            if c not in ("timestamp", "symbol", "signal_score", "outcome")]
            if feature_cols and "outcome" in features_df.columns:
                X = features_df[feature_cols].fillna(0.0).values
                y = (features_df["outcome"] > 0).astype(float).values
                try:
                    self._tin_predictor.fit(X, y)
                    results["tin"] = {"fitted": True, "n_samples": len(y)}
                    logger.info("LeanSim: tin_predictor reentrenado con %d muestras", len(y))
                except Exception as exc:
                    logger.error("LeanSim: tin_predictor.fit falló: %s", exc)
                    results["tin"] = {"fitted": False, "error": str(exc)}
            else:
                results["tin"] = {"fitted": False, "reason": "missing feature_cols or outcome column"}
        else:
            results["tin"] = {"fitted": False, "reason": "no features_df or tin_predictor not initialized"}

        self._fit_metadata["lean_sim_retrain"] = {
            "symbols": _symbols,
            "years": years,
            "trades_rows": len(trades_df),
        }

        logger.info(
            "PatternLabService.retrain_from_lean_simulator: completado | motif=%s tin=%s",
            results["motif"].get("fitted"),
            results["tin"].get("fitted"),
        )
        return results


# ── Función auxiliar ───────────────────────────────────────────────────────────

def _compute_signal_score(
    motif_edge: float,
    tin_prob: float,
    regime_conf: float,
    w_motif: float = 0.35,
    w_tin: float = 0.40,
    w_regime: float = 0.25,
) -> float:
    """
    Score compuesto [0, 1]:
        0.5 = neutral
        > 0.55 = ventaja long
        < 0.45 = desventaja long / ventaja short

    regime_conf ya es [0,1] representando confianza del régimen
    (sin dirección). Se usa como multiplicador de la certeza, no
    como dirección en sí — la dirección la captura motif_edge y tin_prob.
    """
    # regime_conf actúa como "confianza general del mercado":
    # un régimen con alta confianza amplifica la señal de motif+TIN,
    # mientras que baja confianza la acerca al neutral (0.5).
    base = w_motif * motif_edge + w_tin * tin_prob
    # Interpolar entre neutral (0.5) y base según confianza del régimen
    score = base * (1 - w_regime) + (0.5 * w_regime * (1 - regime_conf) + base * w_regime * regime_conf)
    return float(np.clip(score, 0.0, 1.0))
