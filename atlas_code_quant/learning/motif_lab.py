"""Pattern Lab — Módulo 1: Descubrimiento de motifs y distribución de retornos futuros.

MotifDiscoveryService descubre subsecuencias históricas similares ("motifs") en el espacio
de features técnicas normalizado, y construye una distribución empírica de retornos futuros
para enriquecer el criterio de entrada de ATLAS-Quant.

Flujo:
    1. fit(price_series, feature_df, config) → indexa ventanas históricas
    2. find_similar_motifs(recent_window) → lista de coincidencias históricas
    3. forecast_distribution(recent_window, horizon) → ReturnDistribution [edge_score 0-1]

El campo `edge_score` de ReturnDistribution es el input clave para pattern_lab.py.
"""
from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.pattern_lab.motif")

# Optional: stumpy para matrix profile (más eficiente en series largas)
try:
    import stumpy  # noqa: F401
    _HAS_STUMPY = True
except ImportError:
    _HAS_STUMPY = False


# ── Configuración ─────────────────────────────────────────────────────────────

@dataclass
class MotifConfig:
    """Configuración del motor de búsqueda de motifs."""
    window_size: int = 20
    """Número de barras de cada subsecuencia."""
    distance: str = "pearson"
    """Métrica: 'pearson' (similitud 0-1) o 'euclidean' (convertida a similitud)."""
    normalize: bool = True
    """Z-score por columna antes de comparar."""
    similarity_threshold: float = 0.80
    """Umbral mínimo de similitud para incluir un match (Pearson: 0-1)."""
    max_stored_motifs: int = 4000
    """Máximo de subsecuencias indexadas."""
    exclude_trivial: bool = True
    """Excluir matches que solapan con la posición actual (evita autocorrelación trivial)."""
    trivial_radius: int = 10
    """Radio de exclusión trivial en barras."""
    min_future_samples: int = 3
    """Mínimo de matches con retorno futuro observable para producir distribución."""


# ── Data classes de salida ────────────────────────────────────────────────────

@dataclass
class MotifMatch:
    """Una coincidencia histórica de motif."""
    start_idx: int
    end_idx: int
    similarity: float
    start_date: Any = None
    end_date: Any = None
    next_return: float | None = None
    """Retorno observado a horizon_bars barras después del match."""
    future_returns: list[float] = field(default_factory=list)
    """Retornos barra a barra hasta horizon_bars."""


@dataclass
class ReturnDistribution:
    """Distribución empírica de retornos futuros construida desde motifs similares."""
    horizon_bars: int = 5
    n_samples: int = 0
    prob_positive: float = 0.5
    prob_negative: float = 0.5
    # Percentiles del retorno futuro
    p05: float = 0.0
    p25: float = 0.0
    p50: float = 0.0
    p75: float = 0.0
    p95: float = 0.0
    expected_return: float = 0.0
    volatility: float = 0.0
    edge_score: float = 0.5
    """Score de edge [0-1]: 0.5=neutral, >0.5=ventaja long, <0.5=desventaja."""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "horizon_bars": self.horizon_bars,
            "n_samples": self.n_samples,
            "prob_positive": round(self.prob_positive, 4),
            "prob_negative": round(self.prob_negative, 4),
            "p05": round(self.p05, 6),
            "p50": round(self.p50, 6),
            "p95": round(self.p95, 6),
            "expected_return": round(self.expected_return, 6),
            "volatility": round(self.volatility, 6),
            "edge_score": round(self.edge_score, 4),
            **self.metadata,
        }


# ── Servicio principal ────────────────────────────────────────────────────────

class MotifDiscoveryService:
    """
    Descubre motifs (patrones repetitivos) en el espacio de features técnicas
    y construye distribuciones empíricas de retornos futuros.

    Diseñado para ser pre-entrenado offline y consultado en tiempo real
    con latencia <50ms por llamada a find_similar_motifs / forecast_distribution.
    """

    def __init__(self) -> None:
        self._config: MotifConfig | None = None
        self._price_array: np.ndarray | None = None
        self._feature_matrix: np.ndarray | None = None
        self._feature_index: pd.Index | None = None
        self._feature_cols: list[str] = []
        self._is_fitted: bool = False

    # ── Entrenamiento offline ──────────────────────────────────────────────────

    def fit(
        self,
        price_series: pd.Series,
        feature_df: pd.DataFrame,
        config: MotifConfig | None = None,
    ) -> None:
        """
        Indexa el histórico para búsqueda rápida de motifs.

        Args:
            price_series: Precios de cierre (índice temporal recomendado).
            feature_df:   DataFrame de features normalizado con el mismo índice.
            config:       Configuración del motor.
        """
        self._config = config or MotifConfig()
        cfg = self._config

        # Alinear índices
        common = price_series.index.intersection(feature_df.index)
        if len(common) < cfg.window_size * 3:
            logger.warning(
                "MotifDiscoveryService.fit: datos insuficientes (%d barras < %d requeridas)",
                len(common), cfg.window_size * 3,
            )
            return

        price_series = price_series.loc[common]
        feature_df = feature_df.loc[common]

        self._price_array = price_series.values.astype(np.float64)
        self._feature_index = feature_df.index
        self._feature_cols = list(feature_df.columns)

        mat = feature_df.values.astype(np.float64)
        if cfg.normalize:
            mat = _zscore_cols(mat)

        # Limitar a max_stored_motifs ventanas para eficiencia
        n_windows = len(mat) - cfg.window_size
        if n_windows > cfg.max_stored_motifs:
            step = max(1, n_windows // cfg.max_stored_motifs)
            keep = list(range(0, n_windows, step))[:cfg.max_stored_motifs]
            self._feature_matrix = mat
            self._window_starts: list[int] = keep
        else:
            self._feature_matrix = mat
            self._window_starts = list(range(n_windows))

        self._is_fitted = True
        logger.info(
            "MotifDiscoveryService ajustado: %d barras, %d features, %d ventanas, ventana_size=%d",
            len(price_series), mat.shape[1], len(self._window_starts), cfg.window_size,
        )

    # ── Inferencia en tiempo real ─────────────────────────────────────────────

    def find_similar_motifs(
        self,
        recent_window: pd.DataFrame,
        top_k: int = 20,
    ) -> list[MotifMatch]:
        """
        Busca en el histórico las ventanas más similares a recent_window.

        Args:
            recent_window: DataFrame de features con las últimas window_size barras.
            top_k:         Máximo de resultados a devolver.

        Returns:
            Lista de MotifMatch ordenada por similitud descendente.
        """
        if not self._is_fitted or self._feature_matrix is None:
            return []
        cfg = self._config
        assert cfg is not None

        # Construir query vector
        avail = [c for c in self._feature_cols if c in recent_window.columns]
        if not avail:
            return []

        query_raw = recent_window[avail].values.astype(np.float64)
        if cfg.normalize:
            query_raw = _zscore_cols(query_raw)

        mat = self._feature_matrix
        w = cfg.window_size
        # Usar solo los features disponibles en el índice de feature_matrix
        feat_idx = [self._feature_cols.index(c) for c in avail if c in self._feature_cols]
        if not feat_idx:
            return []
        mat_sub = mat[:, feat_idx]

        scores: list[tuple[float, int]] = []
        last_accepted = -(cfg.trivial_radius + 1)

        for start_i in self._window_starts:
            subseq = mat_sub[start_i: start_i + w]
            if len(subseq) < w:
                continue
            sim = _similarity(query_raw, subseq, cfg.distance)
            scores.append((sim, start_i))

        # Ordenar por similitud
        scores.sort(key=lambda x: x[0], reverse=True)

        results: list[MotifMatch] = []
        for sim, start_i in scores:
            if sim < cfg.similarity_threshold:
                break
            if cfg.exclude_trivial and abs(start_i - last_accepted) < cfg.trivial_radius:
                continue
            last_accepted = start_i
            end_i = start_i + w
            m = MotifMatch(start_idx=start_i, end_idx=end_i, similarity=sim)
            if self._feature_index is not None:
                idx = self._feature_index
                if start_i < len(idx):
                    m.start_date = idx[start_i]
                if end_i - 1 < len(idx):
                    m.end_date = idx[end_i - 1]
            results.append(m)
            if len(results) >= top_k:
                break

        return results

    def forecast_distribution(
        self,
        recent_window: pd.DataFrame,
        horizon_bars: int = 5,
        top_k: int = 30,
    ) -> ReturnDistribution:
        """
        Construye distribución empírica de retornos futuros a horizon_bars pasos.

        Args:
            recent_window: DataFrame de features recientes (ventana actual).
            horizon_bars:  Número de barras hacia adelante para el retorno objetivo.
            top_k:         Motifs a considerar para la distribución.

        Returns:
            ReturnDistribution con edge_score y percentiles de retorno.
        """
        if self._price_array is None or not self._is_fitted:
            return ReturnDistribution(horizon_bars=horizon_bars)

        matches = self.find_similar_motifs(recent_window, top_k=top_k)
        if not matches:
            return ReturnDistribution(horizon_bars=horizon_bars, n_samples=0)

        prices = self._price_array
        n = len(prices)
        future_rets: list[float] = []

        for m in matches:
            future_end = m.end_idx + horizon_bars
            if future_end >= n:
                continue
            p_entry = prices[m.end_idx]
            p_exit = prices[future_end]
            if p_entry <= 0:
                continue
            ret = (p_exit - p_entry) / p_entry
            m.next_return = ret
            m.future_returns = [
                (prices[m.end_idx + k] - p_entry) / p_entry
                for k in range(1, horizon_bars + 1)
                if m.end_idx + k < n
            ]
            future_rets.append(ret)

        cfg = self._config
        assert cfg is not None
        if len(future_rets) < cfg.min_future_samples:
            return ReturnDistribution(
                horizon_bars=horizon_bars,
                n_samples=len(future_rets),
                metadata={"n_motifs_found": len(matches)},
            )

        arr = np.array(future_rets)
        prob_pos = float(np.mean(arr > 0))
        edge = _compute_edge_score(arr)

        return ReturnDistribution(
            horizon_bars=horizon_bars,
            n_samples=len(arr),
            prob_positive=prob_pos,
            prob_negative=1.0 - prob_pos,
            p05=float(np.percentile(arr, 5)),
            p25=float(np.percentile(arr, 25)),
            p50=float(np.median(arr)),
            p75=float(np.percentile(arr, 75)),
            p95=float(np.percentile(arr, 95)),
            expected_return=float(arr.mean()),
            volatility=float(arr.std()),
            edge_score=edge,
            metadata={
                "n_motifs_found": len(matches),
                "n_usable": len(future_rets),
            },
        )


# ── Funciones auxiliares ──────────────────────────────────────────────────────

def _zscore_cols(mat: np.ndarray) -> np.ndarray:
    """Z-score por columna; robusto a varianza cero y NaN."""
    result = np.zeros_like(mat, dtype=np.float64)
    for j in range(mat.shape[1]):
        col = mat[:, j]
        mu = np.nanmean(col)
        sigma = np.nanstd(col)
        result[:, j] = (col - mu) / (sigma + 1e-8)
    return result


def _similarity(a: np.ndarray, b: np.ndarray, method: str = "pearson") -> float:
    """Similitud entre dos matrices de features [W x D] aplanadas."""
    a_f = a.flatten()
    b_f = b.flatten()
    if len(a_f) != len(b_f):
        min_len = min(len(a_f), len(b_f))
        a_f, b_f = a_f[:min_len], b_f[:min_len]
    if method == "pearson":
        std_a = a_f.std()
        std_b = b_f.std()
        if std_a < 1e-8 or std_b < 1e-8:
            return 0.0
        corr = float(np.corrcoef(a_f, b_f)[0, 1])
        return float(np.clip((corr + 1.0) / 2.0, 0.0, 1.0))
    # euclidean → similitud
    dist = float(np.linalg.norm(a_f - b_f))
    return float(1.0 / (1.0 + dist))


def _compute_edge_score(rets: np.ndarray) -> float:
    """
    Score de edge [0, 1]:
        0.5 = neutral (profit factor = 1)
        > 0.5 = ventaja long
        < 0.5 = desventaja long / ventaja short
    Basado en profit factor con suavizado sigmoid.
    """
    if len(rets) < 3:
        return 0.5
    wins = rets[rets > 0]
    losses = rets[rets < 0]
    prob_pos = len(wins) / len(rets)
    mean_win = float(wins.mean()) if len(wins) > 0 else 0.0
    mean_loss = float(abs(losses.mean())) if len(losses) > 0 else 1e-8
    profit_factor = (prob_pos * mean_win) / ((1 - prob_pos) * mean_loss + 1e-8)
    # Sigmoid centrada en PF=1: f(PF) = 1/(1 + exp(-k*(PF-1)))
    edge = 1.0 / (1.0 + np.exp(-2.5 * (profit_factor - 1.0)))
    return float(np.clip(edge, 0.0, 1.0))
