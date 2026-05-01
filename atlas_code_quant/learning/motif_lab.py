"""Motif Discovery — Descubrimiento de subsecuencias repetitivas en series temporales.

MotifLabService detecta "motifs" (ventanas históricas similares) en el espacio de
features técnicas normalizado, persiste el índice en SQLite y construye distribuciones
empíricas de retornos futuros para enriquecer el criterio de entrada de ATLAS-Quant.

API principal:
    lab = MotifLabService(db_path="data/motifs.db")
    lab.fit(symbol="SPY", timeframe="1m", window_size=20, historical_df=df)
    matches = lab.find_motifs(recent_data=recent_df, top_k=20)
    dist    = lab.get_forecast_dist(matches, horizon=5)
    # dist["edge_score"], dist["prob_positive"], dist["mean_return"]

Integración LiveLoop:
    LiveLoop acepta `motif_lab_service=lab` y llama find_motifs() cada
    `motif_cycle_interval` ciclos (default 15 × 5s = 75s).
    El resultado queda en `loop._motif_forecast_state[symbol]` y se inyecta
    en SignalGenerator como metadata de la señal.

Columnas requeridas en recent_data / historical_df:
    close, rsi, macd, volume_rel, cvd

Dependencias: pandas, numpy, scipy, sqlite3 (stdlib).
"""
from __future__ import annotations

import json
import logging
import sqlite3
import threading
import warnings
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.motif_lab")

# ── Columnas de features ──────────────────────────────────────────────────────
REQUIRED_COLS: list[str] = ["close", "rsi", "macd", "volume_rel", "cvd"]
_DEFAULT_COLS: list[str] = REQUIRED_COLS  # alias legible

# Número máximo de barras futuras que se almacenan por ventana
_MAX_FUTURE_BARS: int = 20


# ── Data classes ───────────────────────────────────────────────────────────────

@dataclass
class MotifMatch:
    """Una coincidencia histórica de motif con su ventana y similitud."""
    historical_start: int
    """Índice de barra de inicio en el histórico (base 0)."""
    historical_end: int
    """Índice de barra de fin (exclusive) = historical_start + window_size."""
    similarity: float
    """Similitud normalizada [0, 1]. 1 = idéntico, 0 = opuesto."""
    db_id: int = -1
    """ID en la tabla SQLite (para recuperar retornos futuros)."""
    symbol: str = ""
    timeframe: str = ""
    close_at_end: float = 0.0
    """Precio de cierre al final de la ventana histórica."""
    future_returns: list[float] = field(default_factory=list)
    """Retornos log barra-a-barra desde close_at_end hasta horizon."""


@dataclass
class ForecastDistribution:
    """Distribución empírica de retornos futuros construida desde motifs similares."""
    horizon_bars: int = 5
    n_samples: int = 0
    mean_return: float = 0.0
    std: float = 0.0
    prob_positive: float = 0.5
    p05: float = 0.0
    p25: float = 0.0
    p50: float = 0.0
    p75: float = 0.0
    p95: float = 0.0
    edge_score: float = 0.5
    """Score de edge [0-1]: 0.5=neutral, >0.5=ventaja long, <0.5=ventaja short."""
    returns: list[float] = field(default_factory=list)
    """Retornos individuales usados para construir la distribución."""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "horizon_bars":  self.horizon_bars,
            "n_samples":     self.n_samples,
            "mean_return":   round(self.mean_return,   6),
            "std":           round(self.std,           6),
            "prob_positive": round(self.prob_positive, 4),
            "p05":           round(self.p05,           6),
            "p25":           round(self.p25,           6),
            "p50":           round(self.p50,           6),
            "p75":           round(self.p75,           6),
            "p95":           round(self.p95,           6),
            "edge_score":    round(self.edge_score,    4),
            **self.metadata,
        }


# ── SQLite persistence ─────────────────────────────────────────────────────────

_SCHEMA = """
CREATE TABLE IF NOT EXISTS motif_windows (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    symbol          TEXT    NOT NULL,
    timeframe       TEXT    NOT NULL,
    window_size     INTEGER NOT NULL,
    bar_idx         INTEGER NOT NULL,
    features_json   TEXT    NOT NULL,
    close_at_end    REAL    NOT NULL DEFAULT 0,
    returns_json    TEXT    NOT NULL DEFAULT '[]',
    ts_start        TEXT,
    ts_end          TEXT,
    UNIQUE(symbol, timeframe, window_size, bar_idx)
);
CREATE INDEX IF NOT EXISTS idx_motif_key
    ON motif_windows(symbol, timeframe, window_size);
"""


class MotifDB:
    """
    Capa de persistencia SQLite para ventanas de motifs.

    Thread-safe mediante lock por instancia. Cada conexión se
    abre y cierra dentro de un contextmanager para evitar
    conflictos entre threads.
    """

    def __init__(self, db_path: str = "data/motifs.db") -> None:
        self.db_path = str(Path(db_path))
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._init_db()

    def _init_db(self) -> None:
        with self._connect() as conn:
            conn.executescript(_SCHEMA)

    @contextmanager
    def _connect(self):
        conn = sqlite3.connect(self.db_path, timeout=10)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            conn.close()

    # ── Escritura ──────────────────────────────────────────────────────────────

    def store_window(
        self,
        symbol: str,
        timeframe: str,
        window_size: int,
        bar_idx: int,
        features: np.ndarray,
        close_at_end: float,
        future_returns: list[float],
        ts_start: str | None = None,
        ts_end: str | None = None,
    ) -> None:
        """Inserta o reemplaza una ventana de motif."""
        feat_json = json.dumps(features.flatten().tolist())
        ret_json  = json.dumps([round(r, 8) for r in future_returns[:_MAX_FUTURE_BARS]])
        with self._lock, self._connect() as conn:
            conn.execute("""
                INSERT OR REPLACE INTO motif_windows
                    (symbol, timeframe, window_size, bar_idx,
                     features_json, close_at_end, returns_json, ts_start, ts_end)
                VALUES (?,?,?,?,?,?,?,?,?)
            """, (symbol, timeframe, window_size, bar_idx,
                  feat_json, close_at_end, ret_json, ts_start, ts_end))

    def bulk_store(self, rows: list[tuple]) -> None:
        """Inserta muchas ventanas de una sola vez (más rápido que store_window en bucle)."""
        with self._lock, self._connect() as conn:
            conn.executemany("""
                INSERT OR REPLACE INTO motif_windows
                    (symbol, timeframe, window_size, bar_idx,
                     features_json, close_at_end, returns_json, ts_start, ts_end)
                VALUES (?,?,?,?,?,?,?,?,?)
            """, rows)

    # ── Lectura ────────────────────────────────────────────────────────────────

    def load_windows(
        self,
        symbol: str,
        timeframe: str,
        window_size: int,
    ) -> list[dict]:
        """
        Carga todas las ventanas indexadas para (symbol, timeframe, window_size).

        Returns:
            Lista de dicts con keys: id, bar_idx, features (np.ndarray),
            close_at_end, future_returns (list[float]), ts_start, ts_end.
        """
        with self._connect() as conn:
            rows = conn.execute("""
                SELECT id, bar_idx, features_json, close_at_end,
                       returns_json, ts_start, ts_end
                FROM   motif_windows
                WHERE  symbol=? AND timeframe=? AND window_size=?
                ORDER  BY bar_idx
            """, (symbol, timeframe, window_size)).fetchall()

        result = []
        for r in rows:
            result.append({
                "id":             r["id"],
                "bar_idx":        r["bar_idx"],
                "features":       np.array(json.loads(r["features_json"]), dtype=np.float64),
                "close_at_end":   float(r["close_at_end"]),
                "future_returns": json.loads(r["returns_json"]),
                "ts_start":       r["ts_start"],
                "ts_end":         r["ts_end"],
            })
        return result

    def load_returns(self, db_ids: list[int]) -> dict[int, list[float]]:
        """Carga solo los retornos futuros para una lista de IDs (más rápido en búsquedas)."""
        if not db_ids:
            return {}
        placeholders = ",".join("?" * len(db_ids))
        with self._connect() as conn:
            rows = conn.execute(
                f"SELECT id, returns_json FROM motif_windows WHERE id IN ({placeholders})",
                db_ids,
            ).fetchall()
        return {r["id"]: json.loads(r["returns_json"]) for r in rows}

    def count(self, symbol: str, timeframe: str, window_size: int | None = None) -> int:
        """Número de ventanas almacenadas para el símbolo/timeframe."""
        with self._connect() as conn:
            if window_size is not None:
                n = conn.execute(
                    "SELECT COUNT(*) FROM motif_windows WHERE symbol=? AND timeframe=? AND window_size=?",
                    (symbol, timeframe, window_size),
                ).fetchone()[0]
            else:
                n = conn.execute(
                    "SELECT COUNT(*) FROM motif_windows WHERE symbol=? AND timeframe=?",
                    (symbol, timeframe),
                ).fetchone()[0]
        return int(n)

    def clear(self, symbol: str, timeframe: str, window_size: int | None = None) -> int:
        """Elimina ventanas almacenadas. Retorna número de filas eliminadas."""
        with self._lock, self._connect() as conn:
            if window_size is not None:
                cur = conn.execute(
                    "DELETE FROM motif_windows WHERE symbol=? AND timeframe=? AND window_size=?",
                    (symbol, timeframe, window_size),
                )
            else:
                cur = conn.execute(
                    "DELETE FROM motif_windows WHERE symbol=? AND timeframe=?",
                    (symbol, timeframe),
                )
        return cur.rowcount


# ── Servicio principal ─────────────────────────────────────────────────────────

class MotifLabService:
    """
    Descubre motifs en series de precio/indicadores y produce distribuciones
    empíricas de retornos futuros.

    Uso::

        lab = MotifLabService(db_path="data/operation/motifs.db")

        # --- Offline (fit una sola vez o periódicamente) ---
        lab.fit("SPY", "1m", window_size=20, historical_df=hist_df)

        # --- En tiempo real ---
        matches  = lab.find_motifs(recent_df, top_k=20)
        dist     = lab.get_forecast_dist(matches, horizon=5)
        print(dist["edge_score"], dist["prob_positive"])
    """

    # Parámetros por defecto
    DEFAULT_SIMILARITY_THRESHOLD: float = 0.75
    """Similitud mínima para incluir un match."""
    DEFAULT_TRIVIAL_RADIUS: int = 10
    """Mínimo de barras entre matches para evitar autocorrelación."""
    DEFAULT_MAX_WINDOWS: int = 5_000
    """Número máximo de ventanas a indexar."""
    DEFAULT_MIN_FUTURE_SAMPLES: int = 3
    """Mínimo de matches con retorno futuro para producir distribución."""

    def __init__(
        self,
        db_path: str = "data/operation/motifs.db",
        similarity_threshold: float = DEFAULT_SIMILARITY_THRESHOLD,
        trivial_radius: int = DEFAULT_TRIVIAL_RADIUS,
        max_windows: int = DEFAULT_MAX_WINDOWS,
        min_future_samples: int = DEFAULT_MIN_FUTURE_SAMPLES,
        normalize: bool = True,
        feature_cols: list[str] | None = None,
    ) -> None:
        self.db = MotifDB(db_path)
        self.similarity_threshold = similarity_threshold
        self.trivial_radius       = trivial_radius
        self.max_windows          = max_windows
        self.min_future_samples   = min_future_samples
        self.normalize            = normalize
        self.feature_cols         = feature_cols or _DEFAULT_COLS

        # Estado del último fit
        self._symbol:      str | None = None
        self._timeframe:   str | None = None
        self._window_size: int        = 20
        self._is_fitted:   bool       = False

        # Cache en memoria de la sesión (se recarga en fit())
        self._cached_windows: list[dict] | None = None
        self._cache_matrix:   np.ndarray | None = None   # [N × (w×d)] normalizado
        self._cache_lock      = threading.Lock()

    # ── Fit ───────────────────────────────────────────────────────────────────

    def fit(
        self,
        symbol: str,
        timeframe: str,
        window_size: int,
        historical_df: pd.DataFrame | None = None,
        force_refit: bool = False,
        max_future_bars: int = 20,
    ) -> "MotifLabService":
        """
        Indexa el histórico del símbolo y lo persiste en SQLite.

        Args:
            symbol:        Ticker del activo (ej: "SPY", "ES").
            timeframe:     Granularidad temporal (ej: "1m", "5m", "1d").
            window_size:   Número de barras por ventana de motif.
            historical_df: DataFrame con columnas mínimas [close, rsi, macd,
                           volume_rel, cvd] ordenado ascendentemente por tiempo.
                           Si None y ya hay datos en SQLite, reutiliza los existentes.
            force_refit:   Si True, borra el índice existente y reindexar desde cero.
            max_future_bars: Máximo de barras futuras a almacenar por ventana.

        Returns:
            self (para encadenamiento).
        """
        self._symbol      = symbol
        self._timeframe   = timeframe
        self._window_size = window_size

        # Si ya hay datos y no se pide re-fit, cargar cache y salir
        if (not force_refit
                and historical_df is None
                and self.db.count(symbol, timeframe, window_size) > 0):
            logger.info(
                "MotifLabService.fit: usando índice existente (%d ventanas) para %s/%s w=%d",
                self.db.count(symbol, timeframe, window_size), symbol, timeframe, window_size,
            )
            self._reload_cache()
            self._is_fitted = True
            return self

        if historical_df is None:
            logger.warning(
                "MotifLabService.fit: no hay historical_df ni índice previo para %s/%s.",
                symbol, timeframe,
            )
            return self

        # ── Preparar features ──────────────────────────────────────────────
        df = self._prepare_df(historical_df)
        if len(df) < window_size * 3:
            logger.warning(
                "MotifLabService.fit: datos insuficientes (%d barras < %d requeridas)",
                len(df), window_size * 3,
            )
            return self

        if force_refit:
            self.db.clear(symbol, timeframe, window_size)

        prices = df["close"].values.astype(np.float64)
        avail_cols = [c for c in self.feature_cols if c in df.columns]
        feat_mat   = df[avail_cols].values.astype(np.float64)

        n          = len(df)
        n_windows  = n - window_size - max_future_bars
        if n_windows <= 0:
            logger.warning("MotifLabService.fit: no hay suficientes barras futuras.")
            return self

        # Submuestrear si hay demasiadas ventanas
        step = max(1, n_windows // self.max_windows)
        indices = list(range(0, n_windows, step))[: self.max_windows]

        # Construir timestamps si el índice es temporal
        has_ts = isinstance(df.index, pd.DatetimeIndex)

        rows: list[tuple] = []
        for start_i in indices:
            end_i   = start_i + window_size
            window  = feat_mat[start_i:end_i]
            if len(window) < window_size:
                continue

            p_entry = prices[end_i - 1]
            if p_entry <= 0:
                continue

            # Retornos futuros (log)
            future_rets = []
            for k in range(1, max_future_bars + 1):
                idx_k = end_i - 1 + k
                if idx_k >= n:
                    break
                p_k = prices[idx_k]
                if p_k > 0:
                    future_rets.append(float(np.log(p_k / p_entry)))

            feat_json  = json.dumps(window.flatten().tolist())
            ret_json   = json.dumps([round(r, 8) for r in future_rets])
            ts_start   = str(df.index[start_i]) if has_ts else None
            ts_end     = str(df.index[end_i - 1]) if has_ts else None

            rows.append((
                symbol, timeframe, window_size, start_i,
                feat_json, float(p_entry), ret_json, ts_start, ts_end,
            ))

        self.db.bulk_store(rows)
        logger.info(
            "MotifLabService.fit: %d ventanas indexadas para %s/%s w=%d",
            len(rows), symbol, timeframe, window_size,
        )
        self._reload_cache()
        self._is_fitted = True
        return self

    # ── Búsqueda de motifs ────────────────────────────────────────────────────

    def find_motifs(
        self,
        recent_data: pd.DataFrame,
        top_k: int = 20,
    ) -> list[MotifMatch]:
        """
        Busca las ventanas históricas más similares a recent_data.

        Args:
            recent_data: DataFrame de features con las últimas window_size barras.
                         Columnas requeridas: [close, rsi, macd, volume_rel, cvd]
                         (cualquier subconjunto disponible se usa; extras se ignoran).
            top_k:       Máximo de coincidencias a retornar.

        Returns:
            Lista de MotifMatch ordenada por similitud descendente.
        """
        if not self._is_fitted or self._cache_matrix is None:
            logger.debug("MotifLabService.find_motifs: service no ajustado.")
            return []

        # Construir vector de query
        query = self._build_query_vector(recent_data)
        if query is None:
            return []

        # Similitud vectorizada contra toda la matriz cached
        with self._cache_lock:
            mat = self._cache_matrix          # [N × feat_dim]
            wins = self._cached_windows       # lista de dicts

        if mat is None or len(mat) == 0:
            return []

        # Pearson similarity por fila: clip a [0, 1]
        sims = _batch_pearson_similarity(query, mat)

        # Ordenar y filtrar
        order = np.argsort(sims)[::-1]
        results: list[MotifMatch] = []
        last_accepted = -(self.trivial_radius + 1)

        for idx in order:
            sim = float(sims[idx])
            if sim < self.similarity_threshold:
                break
            w = wins[idx]
            start_i = w["bar_idx"]
            if abs(start_i - last_accepted) < self.trivial_radius:
                continue
            last_accepted = start_i
            results.append(MotifMatch(
                historical_start = start_i,
                historical_end   = start_i + self._window_size,
                similarity       = sim,
                db_id            = w["id"],
                symbol           = self._symbol or "",
                timeframe        = self._timeframe or "",
                close_at_end     = w["close_at_end"],
                future_returns   = [],   # se cargan en get_forecast_dist si se pide
            ))
            if len(results) >= top_k:
                break

        logger.debug(
            "MotifLabService.find_motifs: %d matches (threshold=%.2f)",
            len(results), self.similarity_threshold,
        )
        return results

    # ── Distribución de retornos ───────────────────────────────────────────────

    def get_forecast_dist(
        self,
        motif_matches: list[MotifMatch],
        horizon: int = 5,
    ) -> dict[str, Any]:
        """
        Construye distribución empírica de retornos futuros a `horizon` barras.

        Args:
            motif_matches: Resultado de find_motifs().
            horizon:       Barras hacia adelante para el retorno objetivo.

        Returns:
            Dict con: n_samples, mean_return, std, prob_positive,
                      p05/p25/p50/p75/p95, edge_score, returns, horizon_bars.
        """
        if not motif_matches:
            return ForecastDistribution(horizon_bars=horizon).to_dict()

        # Cargar retornos desde DB en un solo query
        db_ids = [m.db_id for m in motif_matches if m.db_id >= 0]
        returns_map = self.db.load_returns(db_ids) if db_ids else {}

        future_rets: list[float] = []
        for m in motif_matches:
            rets = (returns_map.get(m.db_id, [])
                    if m.db_id >= 0 else m.future_returns)
            if len(rets) >= horizon:
                future_rets.append(float(rets[horizon - 1]))
            elif rets:
                future_rets.append(float(rets[-1]))

        if len(future_rets) < self.min_future_samples:
            return ForecastDistribution(
                horizon_bars = horizon,
                n_samples    = len(future_rets),
                metadata     = {"n_motifs": len(motif_matches), "n_usable": len(future_rets)},
            ).to_dict()

        arr  = np.array(future_rets, dtype=np.float64)
        edge = _compute_edge_score(arr)

        dist = ForecastDistribution(
            horizon_bars  = horizon,
            n_samples     = len(arr),
            mean_return   = float(arr.mean()),
            std           = float(arr.std()),
            prob_positive = float(np.mean(arr > 0)),
            p05           = float(np.percentile(arr, 5)),
            p25           = float(np.percentile(arr, 25)),
            p50           = float(np.median(arr)),
            p75           = float(np.percentile(arr, 75)),
            p95           = float(np.percentile(arr, 95)),
            edge_score    = edge,
            returns       = arr.tolist(),
            metadata      = {
                "n_motifs":  len(motif_matches),
                "n_usable":  len(future_rets),
                "avg_sim":   round(float(np.mean([m.similarity for m in motif_matches])), 4),
            },
        )
        return dist.to_dict()

    # ── Estado / utilidades ───────────────────────────────────────────────────

    @property
    def is_fitted(self) -> bool:
        return self._is_fitted

    def status(self) -> dict[str, Any]:
        n = (self.db.count(self._symbol, self._timeframe, self._window_size)
             if self._symbol else 0)
        return {
            "symbol":       self._symbol,
            "timeframe":    self._timeframe,
            "window_size":  self._window_size,
            "is_fitted":    self._is_fitted,
            "n_indexed":    n,
            "cache_loaded": self._cache_matrix is not None,
            "cache_size":   len(self._cached_windows) if self._cached_windows else 0,
            "db_path":      self.db.db_path,
        }

    def _reload_cache(self) -> None:
        """Recarga el índice de la DB a memoria para búsquedas rápidas."""
        if self._symbol is None:
            return
        wins = self.db.load_windows(self._symbol, self._timeframe, self._window_size)
        if not wins:
            return

        w = self._window_size
        n_cols = len(self.feature_cols)
        expected_dim = w * n_cols

        rows = []
        valid_wins = []
        for win in wins:
            f = win["features"]
            if f is None or len(f) == 0:
                continue
            # Adaptar si el número de columnas almacenadas difiere
            if len(f) != expected_dim:
                # Truncar o rellenar para mantener compatibilidad
                if len(f) > expected_dim:
                    f = f[:expected_dim]
                else:
                    f = np.pad(f, (0, expected_dim - len(f)))
            rows.append(f)
            valid_wins.append(win)

        if not rows:
            return

        mat = np.array(rows, dtype=np.float64)
        if self.normalize:
            mat = _zscore_rows(mat)

        with self._cache_lock:
            self._cached_windows = valid_wins
            self._cache_matrix   = mat

        logger.debug(
            "MotifLabService: cache recargado — %d ventanas [%d×%d]",
            len(valid_wins), mat.shape[0], mat.shape[1],
        )

    def _prepare_df(self, df: pd.DataFrame) -> pd.DataFrame:
        """Normaliza columnas y rellena NaN."""
        df = df.copy()
        df.columns = [c.lower().strip() for c in df.columns]
        # Renombres comunes
        rename_map = {
            "vol_rel": "volume_rel", "volume_ratio": "volume_rel",
            "cvd_delta": "cvd", "delta": "cvd",
        }
        df.rename(columns=rename_map, inplace=True)
        avail = [c for c in self.feature_cols if c in df.columns]
        if not avail:
            raise ValueError(
                f"MotifLabService: ninguna columna de features encontrada. "
                f"Esperadas: {self.feature_cols}. Disponibles: {list(df.columns)}"
            )
        return df.ffill().fillna(0)

    def _build_query_vector(self, recent_data: pd.DataFrame) -> np.ndarray | None:
        """Construye el vector de query normalizado desde recent_data."""
        try:
            df = self._prepare_df(recent_data)
        except ValueError as e:
            logger.debug("_build_query_vector: %s", e)
            return None

        avail = [c for c in self.feature_cols if c in df.columns]
        # Tomar las últimas window_size filas
        w = self._window_size
        arr = df[avail].values[-w:].astype(np.float64)
        if len(arr) < w:
            # Pad con ceros si hay menos barras
            arr = np.pad(arr, ((w - len(arr), 0), (0, 0)))

        # Rellenar columnas faltantes con ceros
        if len(avail) < len(self.feature_cols):
            n_missing = len(self.feature_cols) - len(avail)
            arr = np.concatenate([arr, np.zeros((w, n_missing))], axis=1)

        query = arr.flatten()
        if self.normalize:
            mu  = query.mean()
            std = query.std() + 1e-8
            query = (query - mu) / std
        return query


# ── Funciones auxiliares ──────────────────────────────────────────────────────

def _zscore_rows(mat: np.ndarray) -> np.ndarray:
    """Z-score por fila; robusto a std≈0."""
    mu  = mat.mean(axis=1, keepdims=True)
    std = mat.std(axis=1, keepdims=True) + 1e-8
    return (mat - mu) / std


def _batch_pearson_similarity(query: np.ndarray, mat: np.ndarray) -> np.ndarray:
    """
    Calcula similitud Pearson entre `query` (1D) y cada fila de `mat` (2D).
    Retorna array de similitudes en [0, 1].
    """
    # Centrar query
    q = query - query.mean()
    q_std = q.std() + 1e-8
    q = q / q_std

    # Centrar filas de mat
    m_mu  = mat.mean(axis=1, keepdims=True)
    m_std = mat.std(axis=1, keepdims=True) + 1e-8
    m_norm = (mat - m_mu) / m_std

    n = mat.shape[1]
    # Correlación: dot(q, row) / n
    corr = m_norm @ q / n
    # Convertir [-1, 1] → [0, 1]
    return np.clip((corr + 1.0) / 2.0, 0.0, 1.0)


def _compute_edge_score(rets: np.ndarray) -> float:
    """
    Score de edge [0, 1] basado en profit factor con normalización sigmoid.
        0.5 = neutral (PF = 1)
        > 0.5 = ventaja long
        < 0.5 = desventaja long
    """
    if len(rets) < 3:
        return 0.5
    wins   = rets[rets > 0]
    losses = rets[rets < 0]
    n      = len(rets)
    p_pos  = len(wins) / n
    mean_w = float(wins.mean())      if len(wins)   > 0 else 0.0
    mean_l = float(abs(losses.mean())) if len(losses) > 0 else 1e-8
    pf     = (p_pos * mean_w) / ((1 - p_pos) * mean_l + 1e-8)
    edge   = 1.0 / (1.0 + np.exp(-2.5 * (pf - 1.0)))
    return float(np.clip(edge, 0.0, 1.0))


# ── Backward-compatibility: API anterior ──────────────────────────────────────
# Las clases MotifConfig, MotifMatch (old), ReturnDistribution y
# MotifDiscoveryService siguen disponibles para el patrón de llamada
# usado en pattern_lab.py (fit(price_series, feature_df, config)).

try:
    from dataclasses import dataclass as _dc, field as _f
    from typing import Any as _Any

    @_dc
    class MotifConfig:
        window_size: int = 20
        distance: str = "pearson"
        normalize: bool = True
        similarity_threshold: float = 0.80
        max_stored_motifs: int = 4000
        exclude_trivial: bool = True
        trivial_radius: int = 10
        min_future_samples: int = 3
        target_horizon: int = 5

    @_dc
    class ReturnDistribution:
        horizon_bars: int = 5
        n_samples: int = 0
        prob_positive: float = 0.5
        prob_negative: float = 0.5
        p05: float = 0.0
        p25: float = 0.0
        p50: float = 0.0
        p75: float = 0.0
        p95: float = 0.0
        expected_return: float = 0.0
        volatility: float = 0.0
        edge_score: float = 0.5
        metadata: dict = _f(default_factory=dict)

        def to_dict(self) -> dict[str, _Any]:
            return {
                "horizon_bars":    self.horizon_bars,
                "n_samples":       self.n_samples,
                "prob_positive":   round(self.prob_positive, 4),
                "prob_negative":   round(self.prob_negative, 4),
                "p05":             round(self.p05, 6),
                "p50":             round(self.p50, 6),
                "p95":             round(self.p95, 6),
                "expected_return": round(self.expected_return, 6),
                "volatility":      round(self.volatility, 6),
                "edge_score":      round(self.edge_score, 4),
                **self.metadata,
            }

    class MotifDiscoveryService:
        """API legacy — wrappea MotifLabService para compatibilidad con pattern_lab.py."""

        def __init__(self) -> None:
            self._lab: MotifLabService | None = None
            self._price_array: np.ndarray | None = None
            self._is_fitted: bool = False
            self._config: MotifConfig | None = None

        def fit(self, price_series, feature_df, config=None) -> None:
            cfg = config or MotifConfig()
            self._config = cfg
            self._price_array = price_series.values.astype(np.float64)
            cols = list(feature_df.columns)
            df = feature_df.copy()
            df["close"] = price_series.values
            # Renombrar para cubrir el mínimo requerido
            if "volume_ratio" in cols:
                df["volume_rel"] = df["volume_ratio"]
            for dummy in [c for c in _DEFAULT_COLS if c not in df.columns]:
                df[dummy] = 0.0
            self._lab = MotifLabService(
                db_path="data/operation/motifs_legacy.db",
                similarity_threshold=cfg.similarity_threshold,
                trivial_radius=cfg.trivial_radius,
                max_windows=cfg.max_stored_motifs,
                min_future_samples=cfg.min_future_samples,
                feature_cols=_DEFAULT_COLS,
            )
            self._lab.fit(
                symbol="__legacy__",
                timeframe="__any__",
                window_size=cfg.window_size,
                historical_df=df,
                force_refit=True,
                max_future_bars=cfg.target_horizon + 5,
            )
            self._is_fitted = self._lab.is_fitted

        def find_similar_motifs(self, recent_window, top_k=20):
            if not self._is_fitted or self._lab is None:
                return []
            w = self._config.window_size if self._config else 20
            df = recent_window.copy()
            df["close"] = df.get("close", pd.Series(np.zeros(len(df))))
            for dummy in [c for c in _DEFAULT_COLS if c not in df.columns]:
                df[dummy] = 0.0
            return self._lab.find_motifs(df.tail(w), top_k=top_k)

        def forecast_distribution(self, recent_window, horizon_bars=5, top_k=30):
            matches = self.find_similar_motifs(recent_window, top_k=top_k)
            if not matches or self._lab is None:
                return ReturnDistribution(horizon_bars=horizon_bars)
            d = self._lab.get_forecast_dist(matches, horizon=horizon_bars)
            return ReturnDistribution(
                horizon_bars   = d.get("horizon_bars", horizon_bars),
                n_samples      = d.get("n_samples", 0),
                prob_positive  = d.get("prob_positive", 0.5),
                prob_negative  = 1.0 - d.get("prob_positive", 0.5),
                p05            = d.get("p05", 0.0),
                p25            = d.get("p25", 0.0),
                p50            = d.get("p50", 0.0),
                p75            = d.get("p75", 0.0),
                p95            = d.get("p95", 0.0),
                expected_return= d.get("mean_return", 0.0),
                volatility     = d.get("std", 0.0),
                edge_score     = d.get("edge_score", 0.5),
                metadata       = d.get("metadata", {}),
            )

except Exception as _compat_err:
    logger.warning("motif_lab: backward-compat block error: %s", _compat_err)


# ── Ejemplo de uso con datos simulados ────────────────────────────────────────

def _demo() -> None:
    """
    Ejemplo completo con datos simulados.
    Ejecutar directamente: python -m atlas_code_quant.learning.motif_lab
    """
    import numpy as np
    import pandas as pd
    from pathlib import Path

    print("=" * 60)
    print("MotifLabService — Demo con datos simulados")
    print("=" * 60)

    # ── 1. Generar datos históricos simulados ──────────────────────────────
    np.random.seed(42)
    N = 1000
    idx = pd.date_range("2024-01-01", periods=N, freq="1min")

    # Precio con drift + ruido
    log_ret  = np.random.normal(0.0001, 0.002, N)
    close    = 450.0 * np.exp(np.cumsum(log_ret))
    rsi      = 50 + 20 * np.sin(np.linspace(0, 20, N)) + np.random.normal(0, 5, N)
    macd     = np.gradient(close) / close * 100 + np.random.normal(0, 0.01, N)
    vol_rel  = 1.0 + np.abs(np.random.normal(0, 0.5, N))
    cvd      = np.cumsum(np.random.normal(0, 1, N)) / 100

    hist_df = pd.DataFrame({
        "close":      close,
        "rsi":        np.clip(rsi, 0, 100),
        "macd":       macd,
        "volume_rel": vol_rel,
        "cvd":        cvd,
    }, index=idx)

    print(f"\n[1] Histórico: {len(hist_df)} barras | {hist_df.index[0]} → {hist_df.index[-1]}")

    # ── 2. Fit ─────────────────────────────────────────────────────────────
    db_path = "data/operation/demo_motifs.db"
    Path(db_path).parent.mkdir(parents=True, exist_ok=True)

    lab = MotifLabService(
        db_path=db_path,
        similarity_threshold=0.70,
        max_windows=2000,
    )
    lab.fit("SPY", "1m", window_size=20, historical_df=hist_df, force_refit=True)
    print(f"[2] Fit completado: {lab.status()}")

    # ── 3. find_motifs con ventana reciente ────────────────────────────────
    recent_df = hist_df.iloc[-20:]   # últimas 20 barras
    matches   = lab.find_motifs(recent_df, top_k=15)

    print(f"\n[3] Motifs encontrados: {len(matches)}")
    for i, m in enumerate(matches[:5]):
        print(f"     #{i+1}: start={m.historical_start:4d} end={m.historical_end:4d} "
              f"sim={m.similarity:.4f} close_hist={m.close_at_end:.2f}")

    # ── 4. get_forecast_dist ───────────────────────────────────────────────
    dist = lab.get_forecast_dist(matches, horizon=5)

    print(f"\n[4] Distribución forecast (horizon=5 barras):")
    print(f"     n_samples    = {dist['n_samples']}")
    print(f"     mean_return  = {dist['mean_return']:.4%}")
    print(f"     std          = {dist['std']:.4%}")
    print(f"     prob_positive= {dist['prob_positive']:.1%}")
    print(f"     p05/p50/p95  = {dist['p05']:.4%} / {dist['p50']:.4%} / {dist['p95']:.4%}")
    print(f"     edge_score   = {dist['edge_score']:.4f}")

    # ── 5. DB status ───────────────────────────────────────────────────────
    print(f"\n[5] DB status: {lab.status()}")
    print(f"\n✓ Demo completado. DB en: {db_path}")


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    _demo()
