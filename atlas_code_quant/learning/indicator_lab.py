"""Indicator Lab — Selección de indicadores + TechnicalIndicatorNet (TIN).

Pipeline completo en un solo módulo:

    IndicatorPool    → calcula 35+ indicadores desde OHLCV
    IndicatorSelector → XGBoost/RF ranking → persiste top-8 en SQLite
    TechnicalIndicatorNet (PyTorch) → bloques RSI/MACD/VOL/MISC interpretables
    IndicatorLabService → fit() offline + get_top_features() + predict() en <5ms

Integración LiveLoop / SignalGenerator:

    # LiveLoop (hook periódico)
    features   = lab.get_top_features(symbol, timeframe, snap)
    tin_score  = lab.predict(features, motif_edge=0.63)

    # SignalGenerator
    final_score = 0.30 * motif_edge + 0.40 * tin_score + 0.30 * regime_conf

Dependencias: torch (con fallback sklearn GBM), xgboost (con fallback RF), sqlite3.
"""
from __future__ import annotations

import io
import json
import logging
import pickle
import sqlite3
import threading
import time
import warnings
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.indicator_lab")

# ── Dependencias opcionales ────────────────────────────────────────────────────
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import DataLoader, TensorDataset
    _HAS_TORCH = True
except ImportError:
    _HAS_TORCH = False

try:
    import xgboost as xgb
    _HAS_XGB = True
except ImportError:
    _HAS_XGB = False

try:
    from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
    from sklearn.preprocessing import StandardScaler as _SKScaler
    _HAS_SKLEARN = True
except ImportError:
    _HAS_SKLEARN = False


# ══════════════════════════════════════════════════════════════════════════════
# 1. INDICATOR POOL
# ══════════════════════════════════════════════════════════════════════════════

# Grupos semánticos → bloques TIN
INDICATOR_GROUPS: dict[str, list[str]] = {
    "rsi": [
        "rsi_7", "rsi_14", "rsi_21",
        "stoch_k_14", "stoch_d_14",
        "williams_r_14", "cci_14",
    ],
    "macd": [
        "macd_hist_8_17_4",   # fast MACD (scalping)
        "macd_hist_12_26_9",  # standard MACD
        "macd_line_12_26_9",
        "ema_diff_9_21",
        "ema_diff_21_50",
        "mom_5", "mom_10", "roc_10",
    ],
    "vol": [
        "bb_pct_20",
        "bb_width_20",
        "atr_14_norm",
        "atr_20_norm",
        "hv_20",
        "natr_14",
    ],
    "volume": [
        "volume_ratio_20",
        "volume_ratio_5",
        "volume_zscore",
        "obv_norm",
        "cmf_20",
    ],
}

ALL_INDICATORS: list[str] = [i for grp in INDICATOR_GROUPS.values() for i in grp]


class IndicatorPool:
    """
    Calcula el pool completo de indicadores desde un DataFrame OHLCV.

    Columnas de entrada esperadas (case-insensitive):
        open, high, low, close, volume

    Retorna DataFrame con el mismo índice y una columna por indicador.
    """

    def compute(self, df: pd.DataFrame) -> pd.DataFrame:
        """Construye todas las features. Columnas faltantes (e.g. sin volume) → 0."""
        d = df.copy()
        d.columns = [c.lower().strip() for c in d.columns]
        close  = d["close"].astype(float)
        has_hl = "high" in d.columns and "low" in d.columns
        has_v  = "volume" in d.columns

        feats: dict[str, pd.Series] = {}

        # ── RSI group ────────────────────────────────────────────────────────
        for p in [7, 14, 21]:
            feats[f"rsi_{p}"] = _rsi(close, p)

        if has_hl:
            h, l = d["high"].astype(float), d["low"].astype(float)
            stoch = _stochastic(h, l, close, 14, 3)
            feats["stoch_k_14"]     = stoch["k"]
            feats["stoch_d_14"]     = stoch["d"]
            feats["williams_r_14"]  = _williams_r(h, l, close, 14)
            feats["cci_14"]         = _cci(h, l, close, 14)
        else:
            for k in ["stoch_k_14", "stoch_d_14", "williams_r_14", "cci_14"]:
                feats[k] = pd.Series(np.zeros(len(close)), index=close.index)

        # ── MACD group ────────────────────────────────────────────────────────
        # Fast MACD (8,17,4) — scalping
        ema8  = close.ewm(span=8,  adjust=False).mean()
        ema17 = close.ewm(span=17, adjust=False).mean()
        ml84  = ema8 - ema17
        feats["macd_hist_8_17_4"] = (ml84 - ml84.ewm(span=4, adjust=False).mean()) / (close.abs() + 1e-8) * 100

        # Standard MACD (12,26,9)
        ema12 = close.ewm(span=12, adjust=False).mean()
        ema26 = close.ewm(span=26, adjust=False).mean()
        ml    = ema12 - ema26
        ms    = ml.ewm(span=9, adjust=False).mean()
        feats["macd_hist_12_26_9"] = (ml - ms) / (close.abs() + 1e-8) * 100
        feats["macd_line_12_26_9"] = ml / (close.abs() + 1e-8) * 100

        # EMA crosses
        ema9  = close.ewm(span=9,  adjust=False).mean()
        ema21 = close.ewm(span=21, adjust=False).mean()
        ema50 = close.ewm(span=50, adjust=False).mean()
        feats["ema_diff_9_21"]  = (ema9  - ema21) / (close.abs() + 1e-8)
        feats["ema_diff_21_50"] = (ema21 - ema50) / (close.abs() + 1e-8)

        # Momentum
        for p in [5, 10]:
            feats[f"mom_{p}"] = close.diff(p) / (close.abs() + 1e-8)
        feats["roc_10"] = close.pct_change(10)

        # ── VOL group ─────────────────────────────────────────────────────────
        sma20 = close.rolling(20).mean()
        std20 = close.rolling(20).std()
        bb_u  = sma20 + 2 * std20
        bb_l  = sma20 - 2 * std20
        feats["bb_pct_20"]   = (close - bb_l) / (bb_u - bb_l + 1e-8)
        feats["bb_width_20"] = (bb_u - bb_l) / (sma20 + 1e-8)

        if has_hl:
            tr    = _true_range(h, l, close)
            atr14 = tr.rolling(14).mean()
            atr20 = tr.rolling(20).mean()
            feats["atr_14_norm"] = atr14 / (close.abs() + 1e-8)
            feats["atr_20_norm"] = atr20 / (close.abs() + 1e-8)
            feats["natr_14"]     = atr14 / close * 100
        else:
            feats["atr_14_norm"] = pd.Series(np.zeros(len(close)), index=close.index)
            feats["atr_20_norm"] = feats["atr_14_norm"]
            feats["natr_14"]     = feats["atr_14_norm"]

        log_ret = np.log(close / close.shift(1))
        feats["hv_20"] = log_ret.rolling(20).std() * np.sqrt(252)

        # ── Volume group ──────────────────────────────────────────────────────
        if has_v:
            vol = d["volume"].astype(float)
            vm20 = vol.rolling(20).mean()
            feats["volume_ratio_20"] = vol / (vm20 + 1e-8)
            feats["volume_ratio_5"]  = vol / (vol.rolling(5).mean() + 1e-8)
            feats["volume_zscore"]   = (vol - vm20) / (vol.rolling(20).std() + 1e-8)
            obv = (np.sign(close.diff().fillna(0)) * vol).cumsum()
            feats["obv_norm"] = (obv - obv.rolling(50).mean()) / (obv.rolling(50).std() + 1e-8)
            if has_hl:
                feats["cmf_20"] = _cmf(h, l, close, vol, 20)
            else:
                feats["cmf_20"] = pd.Series(np.zeros(len(close)), index=close.index)
        else:
            for k in ["volume_ratio_20", "volume_ratio_5", "volume_zscore", "obv_norm", "cmf_20"]:
                feats[k] = pd.Series(np.zeros(len(close)), index=close.index)

        result = pd.DataFrame(feats, index=close.index)
        result = result.replace([np.inf, -np.inf], np.nan).ffill().fillna(0)
        return result

    def compute_single_bar(self, snap: Any, closes: list[float],
                           highs: list[float] | None = None,
                           lows: list[float] | None = None,
                           volumes: list[float] | None = None,
                           feature_names: list[str] | None = None) -> dict[str, float]:
        """
        Calcula un subconjunto de indicadores para la barra actual.
        Mucho más rápido que compute() porque usa solo el buffer de closes.

        Args:
            snap:         TechnicalSnapshot con rsi_14, macd_hist, atr_20, etc.
            closes:       Lista de closes recientes (al menos 50).
            highs/lows:   Opcionales para ATR/Stoch/BB exactos.
            volumes:      Opcionales para OBV/Volume ratio.
            feature_names: Si se provee, calcula solo esas features.

        Returns:
            Dict {indicator_name: float}.
        """
        result: dict[str, float] = {}
        c = np.array(closes, dtype=np.float64)
        n = len(c)
        names = set(feature_names) if feature_names else set(ALL_INDICATORS)

        # RSI
        for p in [7, 14, 21]:
            k = f"rsi_{p}"
            if k in names:
                result[k] = (float(getattr(snap, f"rsi_{p}", 0) or 0)
                             if hasattr(snap, f"rsi_{p}") else _rsi_fast(c, p))

        # Stoch / Williams / CCI — from snap or approx
        for k in ["stoch_k_14", "stoch_d_14", "williams_r_14", "cci_14"]:
            if k in names:
                result[k] = float(getattr(snap, k, 50) or 50)

        # MACD variants
        if "macd_hist_12_26_9" in names:
            result["macd_hist_12_26_9"] = float(getattr(snap, "macd_hist", 0) or 0)
        if "macd_hist_8_17_4" in names and n >= 17:
            s = pd.Series(c)
            ml = s.ewm(span=8, adjust=False).mean() - s.ewm(span=17, adjust=False).mean()
            hist = (ml - ml.ewm(span=4, adjust=False).mean()).iloc[-1]
            result["macd_hist_8_17_4"] = float(hist / (abs(c[-1]) + 1e-8) * 100)
        if "macd_line_12_26_9" in names and n >= 26:
            s = pd.Series(c)
            ml = s.ewm(span=12, adjust=False).mean() - s.ewm(span=26, adjust=False).mean()
            result["macd_line_12_26_9"] = float(ml.iloc[-1] / (abs(c[-1]) + 1e-8) * 100)
        if "ema_diff_9_21" in names and n >= 21:
            s = pd.Series(c)
            result["ema_diff_9_21"] = float(
                (s.ewm(span=9, adjust=False).mean() - s.ewm(span=21, adjust=False).mean()).iloc[-1]
                / (abs(c[-1]) + 1e-8))
        if "ema_diff_21_50" in names and n >= 50:
            s = pd.Series(c)
            result["ema_diff_21_50"] = float(
                (s.ewm(span=21, adjust=False).mean() - s.ewm(span=50, adjust=False).mean()).iloc[-1]
                / (abs(c[-1]) + 1e-8))
        if "mom_5" in names and n >= 6:
            result["mom_5"] = float((c[-1] - c[-6]) / (abs(c[-6]) + 1e-8))
        if "mom_10" in names and n >= 11:
            result["mom_10"] = float((c[-1] - c[-11]) / (abs(c[-11]) + 1e-8))
        if "roc_10" in names and n >= 11:
            result["roc_10"] = float((c[-1] - c[-11]) / (c[-11] + 1e-8))

        # BB / VOL
        if n >= 20:
            s = pd.Series(c)
            mu  = float(s.rolling(20).mean().iloc[-1])
            std = float(s.rolling(20).std().iloc[-1])
            bbu = mu + 2 * std
            bbl = mu - 2 * std
            if "bb_pct_20" in names:
                result["bb_pct_20"]   = float((c[-1] - bbl) / (bbu - bbl + 1e-8))
            if "bb_width_20" in names:
                result["bb_width_20"] = float((bbu - bbl) / (mu + 1e-8))
        if "atr_14_norm" in names:
            result["atr_14_norm"] = float(getattr(snap, "atr_20", 0) or 0) / (abs(c[-1]) + 1e-8)
        if "atr_20_norm" in names:
            result["atr_20_norm"] = result.get("atr_14_norm", 0)
        if "natr_14" in names:
            result["natr_14"] = float(getattr(snap, "atr_20", 0) or 0) / (abs(c[-1]) + 1e-8) * 100
        if "hv_20" in names and n >= 21:
            lr = np.diff(np.log(c[-21:]))
            result["hv_20"] = float(lr.std() * np.sqrt(252))

        # Volume
        if "volume_ratio_20" in names:
            result["volume_ratio_20"] = float(getattr(snap, "volume_ratio", 1.0) or 1.0)
        for k in ["volume_ratio_5", "volume_zscore", "obv_norm", "cmf_20"]:
            if k in names:
                result[k] = 0.0

        return result


# ══════════════════════════════════════════════════════════════════════════════
# 2. INDICATOR SELECTOR  (XGBoost-first ranking)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class SelectorConfig:
    top_n:         int   = 8
    n_estimators:  int   = 200
    max_depth:     int   = 5
    target_horizon:int   = 5
    random_state:  int   = 42
    nan_fill:      str   = "ffill"


@dataclass
class RankedIndicator:
    name:       str
    importance: float
    group:      str
    rank:       int


class IndicatorSelector:
    """
    Rankea indicadores por importancia XGBoost vs retornos futuros.
    Persiste top-N por symbol/timeframe en SQLite.
    """

    _SCHEMA = """
    CREATE TABLE IF NOT EXISTS indicator_rankings (
        symbol      TEXT NOT NULL,
        timeframe   TEXT NOT NULL,
        rank        INTEGER NOT NULL,
        name        TEXT NOT NULL,
        importance  REAL NOT NULL,
        grp         TEXT NOT NULL,
        updated_at  TEXT NOT NULL,
        PRIMARY KEY (symbol, timeframe, rank)
    );
    """

    def __init__(self, db_path: str, config: SelectorConfig | None = None):
        self.db_path = db_path
        self.config  = config or SelectorConfig()
        self._lock   = threading.Lock()
        self._init_db()

    @contextmanager
    def _conn(self):
        c = sqlite3.connect(self.db_path, timeout=10)
        c.row_factory = sqlite3.Row
        try:
            yield c; c.commit()
        except Exception:
            c.rollback(); raise
        finally:
            c.close()

    def _init_db(self):
        Path(self.db_path).parent.mkdir(parents=True, exist_ok=True)
        with self._conn() as c:
            c.executescript(self._SCHEMA)

    # ── Ranking ───────────────────────────────────────────────────────────────

    def rank(
        self,
        feature_df: pd.DataFrame,
        price_series: pd.Series,
        symbol: str,
        timeframe: str,
    ) -> list[RankedIndicator]:
        """XGBoost (fallback RF) importance → top-N → guarda en SQLite."""
        cfg = self.config
        fwd = price_series.pct_change(cfg.target_horizon).shift(-cfg.target_horizon)
        tgt = (fwd > 0).astype(int).reindex(feature_df.index).dropna()
        common = feature_df.index.intersection(tgt.index)
        X = feature_df.loc[common].copy()
        y = tgt.loc[common]

        # Fill NaN
        if cfg.nan_fill == "ffill":
            X = X.ffill().fillna(0)
        else:
            X = X.fillna(0)

        valid = y.notna() & X.notna().all(axis=1)
        X, y = X.loc[valid], y.loc[valid]

        if len(X) < 50:
            logger.warning("IndicatorSelector.rank: datos insuficientes (%d filas)", len(X))
            return self._fallback_ranking(list(feature_df.columns))

        cols = list(X.columns)
        Xa, ya = X.values, y.values.astype(int)
        importances = self._fit_model(Xa, ya, cols, cfg)

        # Normalizar + rankear
        total = sum(importances.values()) + 1e-12
        norm  = {k: v / total for k, v in importances.items()}
        ranked = sorted(norm.items(), key=lambda x: x[1], reverse=True)

        result: list[RankedIndicator] = []
        for r, (name, imp) in enumerate(ranked[:cfg.top_n], start=1):
            grp = self._group_of(name)
            result.append(RankedIndicator(name=name, importance=imp, group=grp, rank=r))

        # Persistir
        self._save(symbol, timeframe, result)
        logger.info(
            "IndicatorSelector: top-%d para %s/%s → %s",
            cfg.top_n, symbol, timeframe,
            [r.name for r in result[:4]],
        )
        return result

    def load(self, symbol: str, timeframe: str) -> list[RankedIndicator]:
        """Carga ranking guardado de SQLite. Retorna [] si no existe."""
        with self._conn() as c:
            rows = c.execute(
                "SELECT rank, name, importance, grp FROM indicator_rankings "
                "WHERE symbol=? AND timeframe=? ORDER BY rank",
                (symbol, timeframe),
            ).fetchall()
        return [
            RankedIndicator(name=r["name"], importance=r["importance"],
                            group=r["grp"], rank=r["rank"])
            for r in rows
        ]

    def top_names(self, symbol: str, timeframe: str) -> list[str]:
        return [r.name for r in self.load(symbol, timeframe)]

    # ── Internals ─────────────────────────────────────────────────────────────

    def _fit_model(self, X, y, cols, cfg):
        if _HAS_XGB:
            try:
                model = xgb.XGBClassifier(
                    n_estimators=cfg.n_estimators, max_depth=cfg.max_depth,
                    use_label_encoder=False, eval_metric="logloss",
                    random_state=cfg.random_state, verbosity=0, n_jobs=-1,
                )
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    model.fit(X, y)
                return dict(zip(cols, model.feature_importances_.astype(float)))
            except Exception as e:
                logger.debug("XGB falló: %s — usando RF", e)

        if _HAS_SKLEARN:
            try:
                from sklearn.ensemble import RandomForestClassifier as RFC
                model = RFC(n_estimators=cfg.n_estimators, max_depth=cfg.max_depth,
                            random_state=cfg.random_state, n_jobs=-1)
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    model.fit(X, y)
                return dict(zip(cols, model.feature_importances_.astype(float)))
            except Exception as e:
                logger.debug("RF falló: %s — usando correlación", e)

        # Fallback: correlación absoluta
        feat_df = pd.DataFrame(X, columns=cols)
        tgt_s   = pd.Series(y.astype(float))
        return {c: abs(float(feat_df[c].corr(tgt_s) or 0)) for c in cols}

    def _save(self, symbol: str, timeframe: str, ranked: list[RankedIndicator]):
        ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        rows = [(symbol, timeframe, r.rank, r.name, r.importance, r.group, ts)
                for r in ranked]
        with self._lock, self._conn() as c:
            c.execute("DELETE FROM indicator_rankings WHERE symbol=? AND timeframe=?",
                      (symbol, timeframe))
            c.executemany(
                "INSERT INTO indicator_rankings VALUES (?,?,?,?,?,?,?)", rows
            )

    @staticmethod
    def _group_of(name: str) -> str:
        for grp, members in INDICATOR_GROUPS.items():
            if name in members:
                return grp
        return "misc"

    @staticmethod
    def _fallback_ranking(cols: list[str]) -> list[RankedIndicator]:
        return [
            RankedIndicator(name=c, importance=1.0 / len(cols),
                            group=IndicatorSelector._group_of(c), rank=i + 1)
            for i, c in enumerate(cols[:8])
        ]


# ══════════════════════════════════════════════════════════════════════════════
# 3. TechnicalIndicatorNet  (PyTorch con bloques interpretables)
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class TINConfig:
    hidden_per_group: int   = 16
    blend_dim:        int   = 32
    dropout:          float = 0.25
    n_epochs:         int   = 60
    batch_size:       int   = 64
    lr:               float = 1e-3
    weight_decay:     float = 1e-4
    patience:         int   = 10
    target_horizon:   int   = 5
    random_state:     int   = 42
    device:           str   = "cpu"


if _HAS_TORCH:
    class _Block(nn.Module):
        """Bloque interpretable para un grupo de indicadores."""
        def __init__(self, in_dim: int, hidden: int, dropout: float):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(in_dim, hidden),
                nn.LayerNorm(hidden),
                nn.ReLU(),
                nn.Dropout(dropout),
                nn.Linear(hidden, hidden),
                nn.ReLU(),
            )
        def forward(self, x):
            return self.net(x)

    class TechnicalIndicatorNet(nn.Module):
        """
        Red neuronal con bloques separados por grupo de indicadores.

        Arquitectura:
            [RSI_block | MACD_block | VOL_block | VOLUME_block | MISC_block]
            → concat → BatchNorm → FC(blend_dim) → FC(16) → sigmoid

        El bloque MISC siempre incluye motif_edge como último feature.
        """

        def __init__(
            self,
            group_sizes: dict[str, int],   # {group_name: n_features}
            config: TINConfig,
        ):
            super().__init__()
            self.group_sizes = group_sizes
            self.group_names = list(group_sizes.keys())
            cfg = config

            self.blocks = nn.ModuleDict({
                name: _Block(sz, cfg.hidden_per_group, cfg.dropout)
                for name, sz in group_sizes.items()
                if sz > 0
            })

            n_groups = sum(1 for s in group_sizes.values() if s > 0)
            concat_dim = n_groups * cfg.hidden_per_group

            self.blend = nn.Sequential(
                nn.Linear(concat_dim, cfg.blend_dim),
                nn.BatchNorm1d(cfg.blend_dim),
                nn.ReLU(),
                nn.Dropout(cfg.dropout),
                nn.Linear(cfg.blend_dim, 16),
                nn.ReLU(),
                nn.Linear(16, 1),
                nn.Sigmoid(),
            )

        def forward(self, x: "torch.Tensor") -> "torch.Tensor":
            parts = []
            offset = 0
            for name in self.group_names:
                sz = self.group_sizes[name]
                if sz == 0:
                    continue
                chunk = x[:, offset: offset + sz]
                parts.append(self.blocks[name](chunk))
                offset += sz
            return self.blend(torch.cat(parts, dim=1)).squeeze(1)


class TINTrainer:
    """
    Entrena TechnicalIndicatorNet sobre features + motif_edge.
    Persiste el modelo serializado en SQLite (TINStore).
    """

    _SCHEMA = """
    CREATE TABLE IF NOT EXISTS tin_models (
        symbol      TEXT NOT NULL,
        timeframe   TEXT NOT NULL,
        feature_cols TEXT NOT NULL,
        group_sizes  TEXT NOT NULL,
        scaler_blob  BLOB,
        model_blob   BLOB NOT NULL,
        backend      TEXT NOT NULL,
        updated_at   TEXT NOT NULL,
        PRIMARY KEY (symbol, timeframe)
    );
    """

    def __init__(self, db_path: str, config: TINConfig | None = None):
        self.db_path = db_path
        self.config  = config or TINConfig()
        self._lock   = threading.Lock()
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(db_path) as c:
            c.executescript(self._SCHEMA)

    # ── Fit ───────────────────────────────────────────────────────────────────

    def fit(
        self,
        feature_df: pd.DataFrame,
        price_series: pd.Series,
        symbol: str,
        timeframe: str,
        ranked_names: list[str],          # top-8 from IndicatorSelector
        motif_edge_series: pd.Series | None = None,  # optional per-bar edge
    ) -> "TINPredictor":
        """
        Entrena el TIN y guarda el modelo en SQLite.

        Args:
            feature_df:         DataFrame con todos los indicadores calculados.
            price_series:       Close prices para construir el target.
            symbol, timeframe:  Clave de persistencia.
            ranked_names:       Top-N indicadores seleccionados por XGBoost.
            motif_edge_series:  Si se provee, se añade como feature extra.

        Returns:
            TINPredictor listo para inferencia.
        """
        cfg = self.config

        # Seleccionar columnas disponibles
        avail = [c for c in ranked_names if c in feature_df.columns]
        if not avail:
            logger.warning("TINTrainer.fit: ningún indicador ranked en feature_df")
            avail = [c for c in ranked_names[:8] if c in feature_df.columns]
        if not avail:
            avail = list(feature_df.columns[:8])

        feat = feature_df[avail].copy()

        # Añadir motif_edge como feature
        if motif_edge_series is not None:
            feat = feat.join(motif_edge_series.rename("motif_edge"), how="left")
            feat["motif_edge"] = feat["motif_edge"].fillna(0.5)
            if "motif_edge" not in avail:
                avail = avail + ["motif_edge"]

        # Target binario
        fwd = price_series.pct_change(cfg.target_horizon).shift(-cfg.target_horizon)
        tgt = (fwd > 0).astype(int).reindex(feat.index).dropna()
        common = feat.index.intersection(tgt.index)
        X_df = feat.loc[common].ffill().fillna(0)
        y    = tgt.loc[common].values.astype(int)

        if len(X_df) < 40:
            logger.warning("TINTrainer.fit: datos insuficientes (%d filas)", len(X_df))
            return self._fallback(X_df, y, avail, symbol, timeframe)

        # Scaler
        if _HAS_SKLEARN:
            from sklearn.preprocessing import StandardScaler as SS
            scaler = SS()
            X = scaler.fit_transform(X_df.values).astype(np.float32)
        else:
            scaler = _NumpyScaler()
            X = scaler.fit_transform(X_df.values).astype(np.float32)

        # Determinar group_sizes para el TIN
        group_sizes = self._compute_group_sizes(avail)

        if _HAS_TORCH and len(X) >= 60:
            model, backend = self._fit_torch(X, y.astype(np.float32), group_sizes, cfg)
        elif _HAS_SKLEARN:
            model, backend = self._fit_sklearn(X, y), "sklearn/gbm"
        else:
            model, backend = _ConstantModel(float(y.mean())), "constant"

        predictor = TINPredictor(
            model=model, feature_cols=avail, scaler=scaler,
            group_sizes=group_sizes, backend=backend, config=cfg,
        )
        self._save(symbol, timeframe, predictor)
        logger.info(
            "TINTrainer.fit: %s/%s — %d features, %d filas, backend=%s",
            symbol, timeframe, len(avail), len(X), backend,
        )
        return predictor

    def load(self, symbol: str, timeframe: str) -> "TINPredictor | None":
        """Carga el modelo persistido. Retorna None si no existe."""
        with sqlite3.connect(self.db_path) as c:
            row = c.execute(
                "SELECT feature_cols, group_sizes, scaler_blob, model_blob, backend "
                "FROM tin_models WHERE symbol=? AND timeframe=?",
                (symbol, timeframe),
            ).fetchone()
        if row is None:
            return None
        try:
            cols     = json.loads(row[0])
            gs       = json.loads(row[1])
            scaler   = pickle.loads(row[2]) if row[2] else None
            model    = pickle.loads(row[3])
            backend  = row[4]
            return TINPredictor(
                model=model, feature_cols=cols, scaler=scaler,
                group_sizes=gs, backend=backend, config=self.config,
            )
        except Exception as e:
            logger.warning("TINTrainer.load: error deserializando modelo: %s", e)
            return None

    # ── Internals ─────────────────────────────────────────────────────────────

    def _fit_torch(self, X, y, group_sizes, cfg):
        device = torch.device(cfg.device if cfg.device != "auto"
                              else ("cuda" if torch.cuda.is_available() else "cpu"))
        model = TechnicalIndicatorNet(group_sizes, cfg).to(device)
        opt   = optim.Adam(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
        sch   = optim.lr_scheduler.ReduceLROnPlateau(opt, patience=3, factor=0.5)
        crit  = nn.BCELoss()

        n_val = max(int(len(X) * 0.15), 4)
        np.random.seed(cfg.random_state)
        idx = np.random.permutation(len(X))
        tr_i, va_i = idx[n_val:], idx[:n_val]

        Xt = torch.tensor(X[tr_i], dtype=torch.float32).to(device)
        yt = torch.tensor(y[tr_i], dtype=torch.float32).to(device)
        Xv = torch.tensor(X[va_i], dtype=torch.float32).to(device)
        yv = torch.tensor(y[va_i], dtype=torch.float32).to(device)

        loader = DataLoader(TensorDataset(Xt, yt), batch_size=cfg.batch_size, shuffle=True)
        best_loss, best_state, no_imp = float("inf"), None, 0

        for epoch in range(cfg.n_epochs):
            model.train()
            for xb, yb in loader:
                opt.zero_grad()
                loss = crit(model(xb), yb)
                loss.backward()
                opt.step()
            model.eval()
            with torch.no_grad():
                vl = crit(model(Xv), yv).item()
            sch.step(vl)
            if vl < best_loss - 1e-5:
                best_loss = vl
                best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
                no_imp = 0
            else:
                no_imp += 1
                if no_imp >= cfg.patience:
                    break

        if best_state:
            model.load_state_dict({k: v.to(device) for k, v in best_state.items()})
        model.eval()
        model.to("cpu")
        return model, "torch"

    def _fit_sklearn(self, X, y):
        from sklearn.ensemble import GradientBoostingClassifier as GBC
        m = GBC(n_estimators=100, max_depth=4, random_state=self.config.random_state,
                n_iter_no_change=self.config.patience, validation_fraction=0.15)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            m.fit(X, y)
        return m

    def _fallback(self, X_df, y, cols, symbol, timeframe):
        p = float(y.mean()) if len(y) > 0 else 0.5
        model = _ConstantModel(p)
        pred  = TINPredictor(model=model, feature_cols=cols, scaler=None,
                             group_sizes={}, backend="constant", config=self.config)
        self._save(symbol, timeframe, pred)
        return pred

    def _save(self, symbol: str, timeframe: str, predictor: "TINPredictor"):
        ts = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        model_blob  = pickle.dumps(predictor.model, protocol=4)
        scaler_blob = pickle.dumps(predictor.scaler, protocol=4) if predictor.scaler else None
        with self._lock, sqlite3.connect(self.db_path) as c:
            c.execute("""
                INSERT OR REPLACE INTO tin_models
                    (symbol, timeframe, feature_cols, group_sizes,
                     scaler_blob, model_blob, backend, updated_at)
                VALUES (?,?,?,?,?,?,?,?)
            """, (symbol, timeframe,
                  json.dumps(predictor.feature_cols),
                  json.dumps(predictor.group_sizes),
                  scaler_blob, model_blob, predictor.backend, ts))

    @staticmethod
    def _compute_group_sizes(feature_cols: list[str]) -> dict[str, int]:
        sizes: dict[str, int] = {g: 0 for g in list(INDICATOR_GROUPS.keys()) + ["misc"]}
        for c in feature_cols:
            grp = IndicatorSelector._group_of(c)
            sizes[grp] = sizes.get(grp, 0) + 1
        return {k: v for k, v in sizes.items() if v > 0}


# ── TINPredictor — interface de inferencia ────────────────────────────────────

class TINPredictor:
    """Interface unificada para inferencia. Oculta backend (torch/sklearn/constant)."""

    def __init__(self, model, feature_cols, scaler, group_sizes, backend, config):
        self.model        = model
        self.feature_cols = feature_cols
        self.scaler       = scaler
        self.group_sizes  = group_sizes
        self.backend      = backend
        self.config       = config

    def predict(self, features: dict[str, float], motif_edge: float = 0.5) -> float:
        """
        Predice probabilidad de retorno positivo.

        Args:
            features:   Dict {indicator_name: value} (puede tener extras; se filtra).
            motif_edge: Edge score de MotifLabService [0-1].

        Returns:
            Probabilidad [0, 1].
        """
        cols = self.feature_cols
        vec  = np.array(
            [features.get(c, 0.5 if c == "motif_edge" else 0.0) for c in cols],
            dtype=np.float32,
        )
        # Inyectar motif_edge si el modelo lo espera
        if "motif_edge" in cols:
            vec[cols.index("motif_edge")] = float(motif_edge)

        if self.scaler is not None:
            try:
                vec = self.scaler.transform(vec.reshape(1, -1))[0].astype(np.float32)
            except Exception:
                pass

        if self.backend == "torch" and _HAS_TORCH:
            t = torch.tensor(vec, dtype=torch.float32).unsqueeze(0)
            with torch.no_grad():
                prob = float(self.model(t).item())
        elif hasattr(self.model, "predict_proba"):
            p = self.model.predict_proba(vec.reshape(1, -1))[0]
            prob = float(p[1]) if len(p) > 1 else float(p[0])
        else:
            prob = float(self.model.predict(vec.reshape(1, -1))[0])

        return float(np.clip(prob, 0.0, 1.0))


# ── Helpers pequeños ──────────────────────────────────────────────────────────

class _ConstantModel:
    def __init__(self, p): self.p = float(p)
    def predict_proba(self, X): return np.array([[1 - self.p, self.p]] * len(X))
    def predict(self, X): return np.array([self.p] * len(X))

class _NumpyScaler:
    def fit_transform(self, X):
        self.mu_  = X.mean(axis=0)
        self.std_ = X.std(axis=0) + 1e-8
        return (X - self.mu_) / self.std_
    def transform(self, X):
        return (X - self.mu_) / self.std_


# ══════════════════════════════════════════════════════════════════════════════
# 4. IndicatorLabService — orquestador principal
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class IndicatorLabConfig:
    selector: SelectorConfig = field(default_factory=SelectorConfig)
    tin:      TINConfig      = field(default_factory=TINConfig)
    db_path:  str            = "data/operation/indicator_lab.db"


class IndicatorLabService:
    """
    Orquesta fit() offline + inferencia en tiempo real.

    Uso::

        lab = IndicatorLabService()

        # --- Offline ---
        lab.fit("SPY", "5m", ohlcv_df)

        # --- Tiempo real ---
        features = lab.get_top_features("SPY", "5m", snap, closes)
        tin_score = lab.predict(features, motif_edge=0.63)
        # tin_score → 0.71

        # --- Fórmula final en SignalGenerator ---
        final_score = 0.30 * motif_edge + 0.40 * tin_score + 0.30 * regime_conf
    """

    def __init__(self, config: IndicatorLabConfig | None = None):
        cfg = config or IndicatorLabConfig()
        self.config    = cfg
        self._pool     = IndicatorPool()
        self._selector = IndicatorSelector(cfg.db_path, cfg.selector)
        self._trainer  = TINTrainer(cfg.db_path, cfg.tin)
        # Cache en memoria: symbol+tf → TINPredictor
        self._predictors: dict[str, TINPredictor] = {}
        # Cache top-names: symbol+tf → list[str]
        self._top_names: dict[str, list[str]] = {}

    # ── Fit offline ───────────────────────────────────────────────────────────

    def fit(
        self,
        symbol: str,
        timeframe: str,
        ohlcv_df: pd.DataFrame,
        motif_edge_series: pd.Series | None = None,
        force_refit: bool = False,
    ) -> "IndicatorLabService":
        """
        Fit completo: calcula pool → XGBoost ranking → entrena TIN → SQLite.

        Args:
            symbol, timeframe: Clave del activo.
            ohlcv_df:          DataFrame OHLCV (columnas: open/high/low/close/volume).
            motif_edge_series: Edge score histórico per-bar (opcional).
            force_refit:       Si False y ya hay modelo en DB, lo carga sin re-entrenar.

        Returns:
            self.
        """
        key = f"{symbol}:{timeframe}"

        # Si ya hay modelo y no se pide refit, cargar desde DB
        if not force_refit and key not in self._predictors:
            names = self._selector.top_names(symbol, timeframe)
            pred  = self._trainer.load(symbol, timeframe)
            if names and pred is not None:
                self._top_names[key]   = names
                self._predictors[key]  = pred
                logger.info("IndicatorLabService: modelo cargado desde DB para %s/%s", symbol, timeframe)
                return self

        t0 = time.time()
        logger.info("IndicatorLabService.fit: calculando pool para %s/%s...", symbol, timeframe)

        # 1. Calcular pool completo
        all_feats = self._pool.compute(ohlcv_df)

        # 2. XGBoost ranking
        close_col = "close" if "close" in ohlcv_df.columns else ohlcv_df.columns[0]
        price     = ohlcv_df[close_col].astype(float).reindex(all_feats.index)
        ranked    = self._selector.rank(all_feats, price, symbol, timeframe)
        names     = [r.name for r in ranked]
        self._top_names[key] = names

        # 3. Entrenar TIN
        pred = self._trainer.fit(
            all_feats, price, symbol, timeframe, names, motif_edge_series
        )
        self._predictors[key] = pred

        logger.info(
            "IndicatorLabService.fit completado en %.1fs | top-8: %s | backend=%s",
            time.time() - t0, names[:4], pred.backend,
        )
        return self

    # ── Inferencia en tiempo real ─────────────────────────────────────────────

    def get_top_features(
        self,
        symbol: str,
        timeframe: str,
        snap: Any,
        closes: list[float],
        highs: list[float] | None = None,
        lows: list[float] | None = None,
        volumes: list[float] | None = None,
    ) -> dict[str, float]:
        """
        Calcula solo los top-8 indicadores para la barra actual.
        Mucho más rápido que recalcular el pool completo.

        Args:
            symbol, timeframe: Para recuperar top-8 del selector.
            snap:    TechnicalSnapshot con rsi_14, macd_hist, atr_20, volume_ratio, etc.
            closes:  Buffer de closes recientes (mínimo 50).

        Returns:
            Dict {indicator_name: float} con solo los top-8 features.
        """
        key   = f"{symbol}:{timeframe}"
        names = self._top_names.get(key, ALL_INDICATORS[:8])
        return self._pool.compute_single_bar(
            snap, closes, highs, lows, volumes, feature_names=names
        )

    def predict(self, features: dict[str, float], motif_edge: float = 0.5) -> float:
        """
        Inferencia TIN → tin_score [0-1].

        Args:
            features:   Salida de get_top_features().
            motif_edge: Edge score de MotifLabService [0-1].

        Returns:
            Probabilidad de retorno positivo [0-1]. 0.5 = neutral.
        """
        # Usar el primer predictor disponible si no se sabe el key
        if not self._predictors:
            return 0.5
        pred = next(iter(self._predictors.values()))
        return pred.predict(features, motif_edge=motif_edge)

    def predict_for(
        self,
        symbol: str,
        timeframe: str,
        features: dict[str, float],
        motif_edge: float = 0.5,
    ) -> float:
        """Versión con clave explícita symbol/timeframe."""
        key  = f"{symbol}:{timeframe}"
        pred = self._predictors.get(key)
        if pred is None:
            return 0.5
        return pred.predict(features, motif_edge=motif_edge)

    @property
    def is_fitted(self) -> bool:
        return bool(self._predictors)

    def status(self) -> dict[str, Any]:
        return {
            "fitted_keys": list(self._predictors.keys()),
            "db_path":     self.config.db_path,
            "backends":    {k: v.backend for k, v in self._predictors.items()},
        }


# ══════════════════════════════════════════════════════════════════════════════
# 5. Funciones auxiliares de indicadores
# ══════════════════════════════════════════════════════════════════════════════

def _rsi(close: pd.Series, p: int) -> pd.Series:
    d = close.diff()
    g = d.clip(lower=0).rolling(p).mean()
    l = (-d.clip(upper=0)).rolling(p).mean()
    return 100 - 100 / (1 + g / (l + 1e-8))

def _rsi_fast(c: np.ndarray, p: int = 14) -> float:
    if len(c) < p + 1: return 50.0
    d = np.diff(c[-(p+1):])
    g = d[d > 0].mean() if (d > 0).any() else 0.0
    l = (-d[d < 0]).mean() if (d < 0).any() else 0.0
    return float(100 - 100 / (1 + g / (l + 1e-8)))

def _true_range(h, l, c):
    pc = c.shift(1)
    return pd.concat([(h-l), (h-pc).abs(), (l-pc).abs()], axis=1).max(axis=1)

def _stochastic(h, l, c, k=14, d=3):
    lo = l.rolling(k).min(); hi = h.rolling(k).max()
    K  = 100 * (c - lo) / (hi - lo + 1e-8)
    return {"k": K, "d": K.rolling(d).mean()}

def _williams_r(h, l, c, p=14):
    return -100 * (h.rolling(p).max() - c) / (h.rolling(p).max() - l.rolling(p).min() + 1e-8)

def _cci(h, l, c, p=14):
    tp  = (h + l + c) / 3
    mad = tp.rolling(p).apply(lambda x: np.abs(x - x.mean()).mean(), raw=True)
    return (tp - tp.rolling(p).mean()) / (0.015 * mad + 1e-8)

def _cmf(h, l, c, v, p=20):
    mfm = ((c - l) - (h - c)) / (h - l + 1e-8)
    return (mfm * v).rolling(p).sum() / (v.rolling(p).sum() + 1e-8)
