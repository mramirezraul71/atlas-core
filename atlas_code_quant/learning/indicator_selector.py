"""Indicator Selector — Módulo 2: Selección y ranking de indicadores técnicos.

IndicatorSelector construye un pool completo de indicadores técnicos a partir de
una serie de precios/OHLCV y aplica ML (RF/LightGBM/XGBoost) para rankearlos
por importancia predictiva respecto al retorno futuro.

Flujo:
    1. build_indicator_pool(price_df) → pd.DataFrame  [N barras × M features]
    2. evaluate_indicators(features, targets, config) → IndicatorRanking
    3. get_top_indicators(ranking, top_n) → list[str]

El resultado alimenta MotifDiscoveryService.fit(feature_df=...) y TINTrainer.
"""
from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import pandas as pd

logger = logging.getLogger("quant.pattern_lab.indicator_selector")

# ── Dependencias opcionales ────────────────────────────────────────────────────
try:
    from sklearn.ensemble import RandomForestClassifier
    from sklearn.model_selection import cross_val_score
    _HAS_SKLEARN = True
except ImportError:
    _HAS_SKLEARN = False

try:
    import lightgbm as lgb
    _HAS_LGB = True
except ImportError:
    _HAS_LGB = False

try:
    import xgboost as xgb
    _HAS_XGB = True
except ImportError:
    _HAS_XGB = False


# ── Configuración ──────────────────────────────────────────────────────────────

@dataclass
class IndicatorSelectorConfig:
    """Configuración del selector de indicadores."""
    method: str = "auto"
    """'auto' elige el mejor disponible: lgb → xgb → rf → correlation"""
    n_estimators: int = 200
    max_depth: int = 6
    cv_folds: int = 3
    """Folds de cross-validation para estimar AUC de cada indicador."""
    target_horizon: int = 5
    """Barras hacia adelante para construir el target binario."""
    target_threshold: float = 0.0
    """Retorno mínimo para considerar target=1 (default: cualquier retorno positivo)."""
    min_importance: float = 0.001
    """Importancia mínima para incluir un indicador en el ranking."""
    nan_fill: str = "ffill"
    """Estrategia de llenado de NaN antes de evaluar: 'ffill', 'zero', 'mean'."""
    random_state: int = 42


# ── Data classes de salida ─────────────────────────────────────────────────────

@dataclass
class IndicatorScore:
    """Score de un indicador individual."""
    name: str
    importance: float
    """Importancia normalizada [0, 1] — suma del ranking = 1."""
    rank: int
    corr_with_target: float = 0.0
    """Correlación de Pearson con el target futuro."""
    auc_univariate: float = 0.5
    """AUC estimado con solo este indicador (si cv_folds > 0)."""
    group: str = ""
    """Grupo semántico: momentum, volatility, volume, trend, other."""


@dataclass
class IndicatorRanking:
    """Resultado completo del proceso de evaluación."""
    scores: list[IndicatorScore] = field(default_factory=list)
    method_used: str = "correlation"
    n_features_evaluated: int = 0
    n_features_selected: int = 0
    model_oob_score: float | None = None
    """OOB accuracy del modelo ensemble (si disponible)."""
    metadata: dict = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "method_used": self.method_used,
            "n_evaluated": self.n_features_evaluated,
            "n_selected": self.n_features_selected,
            "model_oob_score": self.model_oob_score,
            "top_10": [
                {"name": s.name, "importance": round(s.importance, 5), "group": s.group}
                for s in self.scores[:10]
            ],
        }


# ── Servicio principal ─────────────────────────────────────────────────────────

class IndicatorSelector:
    """
    Construye un pool de indicadores técnicos y los rankea por importancia
    predictiva usando modelos ensemble.

    Diseñado para correrse offline (fit) y luego extraer la lista de
    top indicadores para alimentar MotifDiscoveryService y TINTrainer.
    """

    # Grupos semánticos — para interpretabilidad
    _GROUPS: dict[str, str] = {
        "rsi_14": "momentum", "rsi_7": "momentum", "rsi_21": "momentum",
        "rsi_28": "momentum",
        "macd": "momentum", "macd_signal": "momentum", "macd_hist": "momentum",
        "mom_5": "momentum", "mom_10": "momentum", "mom_20": "momentum",
        "roc_5": "momentum", "roc_10": "momentum", "roc_20": "momentum",
        "ema_diff_9_21": "trend", "ema_diff_21_50": "trend",
        "ema_9": "trend", "ema_21": "trend", "ema_50": "trend",
        "sma_20": "trend", "sma_50": "trend",
        "adx_14": "trend", "plus_di": "trend", "minus_di": "trend",
        "bb_upper": "volatility", "bb_lower": "volatility",
        "bb_pct": "volatility", "bb_width": "volatility",
        "atr_14": "volatility", "atr_20": "volatility",
        "natr_14": "volatility",
        "hv_10": "volatility", "hv_20": "volatility", "hv_60": "volatility",
        "hurst": "volatility",
        "stoch_k": "momentum", "stoch_d": "momentum",
        "cci_14": "momentum", "cci_20": "momentum",
        "williams_r": "momentum",
        "obv": "volume", "obv_ema": "volume",
        "vwap_dev": "volume",
        "cmf_20": "volume",
        "volume_ratio": "volume", "volume_ratio_5": "volume",
        "volume_zscore": "volume",
    }

    def build_indicator_pool(self, price_df: pd.DataFrame) -> pd.DataFrame:
        """
        Construye el pool completo de indicadores técnicos.

        Args:
            price_df: DataFrame con columnas [open, high, low, close, volume].
                      Columna 'volume' opcional. Índice temporal recomendado.

        Returns:
            DataFrame de features con el mismo índice que price_df.
        """
        df = price_df.copy()
        # Normalizar nombres de columna
        df.columns = [c.lower() for c in df.columns]
        required = ["close"]
        missing = [c for c in required if c not in df.columns]
        if missing:
            raise ValueError(f"IndicatorSelector: columnas requeridas ausentes: {missing}")

        close = df["close"].astype(np.float64)
        has_high  = "high" in df.columns
        has_low   = "low" in df.columns
        has_vol   = "volume" in df.columns
        has_ohlcv = has_high and has_low and has_vol

        features: dict[str, pd.Series] = {}

        # ── Momentum / RSI ─────────────────────────────────────────────────────
        for period in [7, 14, 21, 28]:
            features[f"rsi_{period}"] = _rsi(close, period)

        # ── Momentum bruto ────────────────────────────────────────────────────
        for period in [5, 10, 20]:
            features[f"mom_{period}"] = close.diff(period)
            features[f"roc_{period}"] = close.pct_change(period)

        # ── EMA / Trend ───────────────────────────────────────────────────────
        ema9  = close.ewm(span=9,  adjust=False).mean()
        ema21 = close.ewm(span=21, adjust=False).mean()
        ema50 = close.ewm(span=50, adjust=False).mean()
        sma20 = close.rolling(20).mean()
        sma50 = close.rolling(50).mean()
        features["ema_9"]  = ema9 / close - 1
        features["ema_21"] = ema21 / close - 1
        features["ema_50"] = ema50 / close - 1
        features["sma_20"] = sma20 / close - 1
        features["sma_50"] = sma50 / close - 1
        features["ema_diff_9_21"]  = (ema9  - ema21) / (close + 1e-8)
        features["ema_diff_21_50"] = (ema21 - ema50) / (close + 1e-8)

        # ── MACD ──────────────────────────────────────────────────────────────
        ema12 = close.ewm(span=12, adjust=False).mean()
        ema26 = close.ewm(span=26, adjust=False).mean()
        macd_line   = ema12 - ema26
        macd_signal = macd_line.ewm(span=9, adjust=False).mean()
        features["macd"]        = macd_line / (close + 1e-8)
        features["macd_signal"] = macd_signal / (close + 1e-8)
        features["macd_hist"]   = (macd_line - macd_signal) / (close + 1e-8)

        # ── Bollinger Bands ───────────────────────────────────────────────────
        sma20_bb = close.rolling(20).mean()
        std20    = close.rolling(20).std()
        bb_upper = sma20_bb + 2 * std20
        bb_lower = sma20_bb - 2 * std20
        bb_width = (bb_upper - bb_lower) / (sma20_bb + 1e-8)
        bb_pct   = (close - bb_lower) / (bb_upper - bb_lower + 1e-8)
        features["bb_upper"] = (bb_upper - close) / (close + 1e-8)
        features["bb_lower"] = (close - bb_lower) / (close + 1e-8)
        features["bb_width"] = bb_width
        features["bb_pct"]   = bb_pct.clip(0, 1)

        # ── ATR / Volatility ──────────────────────────────────────────────────
        if has_high and has_low:
            high = df["high"].astype(np.float64)
            low  = df["low"].astype(np.float64)
            tr   = _true_range(high, low, close)
            atr14 = tr.rolling(14).mean()
            atr20 = tr.rolling(20).mean()
            features["atr_14"]  = atr14 / (close + 1e-8)
            features["atr_20"]  = atr20 / (close + 1e-8)
            features["natr_14"] = atr14 / close * 100

            # ADX
            adx_vals = _adx(high, low, close, 14)
            features["adx_14"]   = adx_vals["adx"]
            features["plus_di"]  = adx_vals["plus_di"]
            features["minus_di"] = adx_vals["minus_di"]

            # Stochastic
            stoch = _stochastic(high, low, close, 14, 3)
            features["stoch_k"] = stoch["k"]
            features["stoch_d"] = stoch["d"]

            # CCI
            features["cci_14"] = _cci(high, low, close, 14)
            features["cci_20"] = _cci(high, low, close, 20)

            # Williams %R
            features["williams_r"] = _williams_r(high, low, close, 14)

        # ── Historical Volatility ─────────────────────────────────────────────
        log_ret = np.log(close / close.shift(1))
        for period in [10, 20, 60]:
            features[f"hv_{period}"] = log_ret.rolling(period).std() * np.sqrt(252)

        # ── Hurst exponent (approx) ───────────────────────────────────────────
        features["hurst"] = _hurst_series(close, window=40)

        # ── Volume-based ──────────────────────────────────────────────────────
        if has_vol:
            vol = df["volume"].astype(np.float64)
            vol_ma20 = vol.rolling(20).mean()
            features["volume_ratio"]   = vol / (vol_ma20 + 1e-8)
            features["volume_ratio_5"] = vol / (vol.rolling(5).mean() + 1e-8)
            features["volume_zscore"]  = (vol - vol_ma20) / (vol.rolling(20).std() + 1e-8)

            # OBV
            obv = _obv(close, vol)
            features["obv"]     = obv / (obv.rolling(20).std() + 1e-8)
            features["obv_ema"] = obv.ewm(span=10, adjust=False).mean() / (obv.rolling(20).std() + 1e-8)

            if has_high and has_low:
                # CMF
                features["cmf_20"] = _cmf(df["high"].astype(np.float64),
                                          df["low"].astype(np.float64),
                                          close, vol, 20)
                # VWAP deviation
                features["vwap_dev"] = _vwap_deviation(df["high"].astype(np.float64),
                                                        df["low"].astype(np.float64),
                                                        close, vol, 20)

        result = pd.DataFrame(features, index=df.index)
        result = result.replace([np.inf, -np.inf], np.nan)
        logger.info(
            "IndicatorSelector.build_indicator_pool: %d barras × %d features",
            len(result), len(result.columns),
        )
        return result

    def evaluate_indicators(
        self,
        features: pd.DataFrame,
        targets: pd.Series | None = None,
        config: IndicatorSelectorConfig | None = None,
        price_series: pd.Series | None = None,
    ) -> IndicatorRanking:
        """
        Rankea los indicadores por importancia predictiva.

        Args:
            features:      DataFrame de features (salida de build_indicator_pool).
            targets:       Serie binaria de targets (1=positivo). Si None y se
                           provee price_series, se construye automáticamente.
            config:        Configuración del selector.
            price_series:  Serie de precios para construir targets automáticamente.

        Returns:
            IndicatorRanking ordenado por importancia descendente.
        """
        cfg = config or IndicatorSelectorConfig()

        # Construir target si no se provee
        if targets is None:
            if price_series is None:
                raise ValueError("Se requiere 'targets' o 'price_series' para evaluar.")
            fwd_ret = price_series.pct_change(cfg.target_horizon).shift(-cfg.target_horizon)
            targets = (fwd_ret > cfg.target_threshold).astype(int)

        # Alinear índices
        common = features.index.intersection(targets.index)
        feat = features.loc[common].copy()
        tgt  = targets.loc[common].copy()

        # Llenar NaN
        feat = _fill_nan(feat, cfg.nan_fill)

        # Eliminar filas con NaN en target o features constantes
        valid_mask = tgt.notna() & feat.notna().all(axis=1)
        feat = feat.loc[valid_mask]
        tgt  = tgt.loc[valid_mask]

        if len(feat) < 50 or feat.shape[1] == 0:
            logger.warning("IndicatorSelector: datos insuficientes para evaluar (%d filas)", len(feat))
            return IndicatorRanking(
                method_used="insufficient_data",
                n_features_evaluated=feat.shape[1],
            )

        # Elegir método
        method = _pick_method(cfg.method)

        importances: dict[str, float] = {}
        oob_score: float | None = None

        X = feat.values
        y = tgt.values.astype(int)

        if method == "lgb" and _HAS_LGB:
            importances, oob_score = _lgb_importances(X, y, feat.columns.tolist(), cfg)
        elif method == "xgb" and _HAS_XGB:
            importances, oob_score = _xgb_importances(X, y, feat.columns.tolist(), cfg)
        elif method == "rf" and _HAS_SKLEARN:
            importances, oob_score = _rf_importances(X, y, feat.columns.tolist(), cfg)
        else:
            importances = _correlation_importances(feat, tgt)
            method = "correlation"

        # Normalizar importancias
        total = sum(importances.values()) + 1e-12
        importances = {k: v / total for k, v in importances.items()}

        # Correlaciones con el target
        corrs = {col: float(feat[col].corr(tgt.astype(float))) for col in feat.columns}

        # Construir ranking
        scores = sorted(importances.items(), key=lambda x: x[1], reverse=True)
        result_scores: list[IndicatorScore] = []
        for rank, (name, imp) in enumerate(scores, start=1):
            if imp < cfg.min_importance:
                continue
            result_scores.append(IndicatorScore(
                name=name,
                importance=imp,
                rank=rank,
                corr_with_target=corrs.get(name, 0.0),
                group=self._GROUPS.get(name, "other"),
            ))

        ranking = IndicatorRanking(
            scores=result_scores,
            method_used=method,
            n_features_evaluated=feat.shape[1],
            n_features_selected=len(result_scores),
            model_oob_score=oob_score,
            metadata={
                "n_rows": len(feat),
                "n_classes": int(tgt.sum()),
                "positive_rate": float(tgt.mean()),
            },
        )
        logger.info(
            "IndicatorSelector.evaluate_indicators: método=%s features_evaluadas=%d seleccionadas=%d oob=%.4f",
            method, feat.shape[1], len(result_scores),
            oob_score if oob_score else 0.0,
        )
        return ranking

    def get_top_indicators(
        self,
        ranking: IndicatorRanking,
        top_n: int = 15,
    ) -> list[str]:
        """
        Extrae los nombres de los top_n indicadores del ranking.

        Args:
            ranking: Resultado de evaluate_indicators().
            top_n:   Número máximo de indicadores a devolver.

        Returns:
            Lista de nombres de columna, ordenada por importancia descendente.
        """
        return [s.name for s in ranking.scores[:top_n]]

    def build_and_rank(
        self,
        price_df: pd.DataFrame,
        config: IndicatorSelectorConfig | None = None,
        top_n: int = 15,
    ) -> tuple[pd.DataFrame, IndicatorRanking, list[str]]:
        """
        Atajo: construye el pool, evalúa y devuelve (features, ranking, top_names).

        Usa el precio de cierre como fuente del target.
        """
        close_col = "close" if "close" in price_df.columns else price_df.columns[0]
        price_series = price_df[close_col].astype(np.float64)
        features = self.build_indicator_pool(price_df)
        ranking = self.evaluate_indicators(features, config=config, price_series=price_series)
        top = self.get_top_indicators(ranking, top_n=top_n)
        return features, ranking, top


# ── Funciones auxiliares — indicadores técnicos ───────────────────────────────

def _rsi(close: pd.Series, period: int) -> pd.Series:
    delta = close.diff()
    gain  = delta.clip(lower=0).rolling(period).mean()
    loss  = (-delta.clip(upper=0)).rolling(period).mean()
    rs    = gain / (loss + 1e-8)
    return 100 - (100 / (1 + rs))


def _true_range(high: pd.Series, low: pd.Series, close: pd.Series) -> pd.Series:
    prev_close = close.shift(1)
    return pd.concat([
        high - low,
        (high - prev_close).abs(),
        (low  - prev_close).abs(),
    ], axis=1).max(axis=1)


def _adx(high: pd.Series, low: pd.Series, close: pd.Series, period: int = 14) -> pd.DataFrame:
    tr      = _true_range(high, low, close)
    plus_dm = (high.diff()).clip(lower=0)
    minus_dm = (-low.diff()).clip(lower=0)
    # Cuando +DM > -DM, -DM = 0 y vice-versa
    mask = plus_dm >= minus_dm
    plus_dm  = plus_dm.where(mask, 0)
    minus_dm = minus_dm.where(~mask, 0)

    atr      = tr.rolling(period).mean()
    plus_di  = 100 * plus_dm.rolling(period).mean() / (atr + 1e-8)
    minus_di = 100 * minus_dm.rolling(period).mean() / (atr + 1e-8)
    dx       = 100 * (plus_di - minus_di).abs() / (plus_di + minus_di + 1e-8)
    adx      = dx.rolling(period).mean()
    return pd.DataFrame({"adx": adx, "plus_di": plus_di, "minus_di": minus_di})


def _stochastic(
    high: pd.Series, low: pd.Series, close: pd.Series,
    k_period: int = 14, d_period: int = 3
) -> pd.DataFrame:
    lowest  = low.rolling(k_period).min()
    highest = high.rolling(k_period).max()
    k = 100 * (close - lowest) / (highest - lowest + 1e-8)
    d = k.rolling(d_period).mean()
    return pd.DataFrame({"k": k, "d": d})


def _cci(high: pd.Series, low: pd.Series, close: pd.Series, period: int) -> pd.Series:
    typical = (high + low + close) / 3
    sma     = typical.rolling(period).mean()
    mad     = typical.rolling(period).apply(lambda x: np.abs(x - x.mean()).mean(), raw=True)
    return (typical - sma) / (0.015 * mad + 1e-8)


def _williams_r(high: pd.Series, low: pd.Series, close: pd.Series, period: int = 14) -> pd.Series:
    highest = high.rolling(period).max()
    lowest  = low.rolling(period).min()
    return -100 * (highest - close) / (highest - lowest + 1e-8)


def _obv(close: pd.Series, volume: pd.Series) -> pd.Series:
    direction = np.sign(close.diff().fillna(0))
    return (direction * volume).cumsum()


def _cmf(high: pd.Series, low: pd.Series, close: pd.Series, volume: pd.Series, period: int) -> pd.Series:
    mfm = ((close - low) - (high - close)) / (high - low + 1e-8)
    mfv = mfm * volume
    return mfv.rolling(period).sum() / (volume.rolling(period).sum() + 1e-8)


def _vwap_deviation(
    high: pd.Series, low: pd.Series, close: pd.Series, volume: pd.Series, period: int
) -> pd.Series:
    typical = (high + low + close) / 3
    vwap = (typical * volume).rolling(period).sum() / (volume.rolling(period).sum() + 1e-8)
    return (close - vwap) / (vwap + 1e-8)


def _hurst_series(close: pd.Series, window: int = 40) -> pd.Series:
    """Exponente de Hurst aproximado via R/S analysis en ventana rodante."""
    def _hurst_window(arr: np.ndarray) -> float:
        n = len(arr)
        if n < 8:
            return 0.5
        lags = [2, 4, 8, min(16, n // 2)]
        rs_vals = []
        lag_vals = []
        for lag in lags:
            if lag >= n:
                continue
            chunks = [arr[i:i + lag] for i in range(0, n - lag + 1, lag)]
            rs_chunk = []
            for c in chunks:
                if len(c) < 2:
                    continue
                mean_c = np.mean(c)
                dev    = np.cumsum(c - mean_c)
                r_val  = dev.max() - dev.min()
                s_val  = np.std(c) + 1e-8
                rs_chunk.append(r_val / s_val)
            if rs_chunk:
                rs_vals.append(np.mean(rs_chunk))
                lag_vals.append(lag)
        if len(rs_vals) < 2:
            return 0.5
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            poly = np.polyfit(np.log(lag_vals), np.log(rs_vals), 1)
        return float(np.clip(poly[0], 0.0, 1.0))

    log_ret = np.log(close / close.shift(1)).fillna(0).values
    result = np.full(len(close), 0.5)
    for i in range(window, len(log_ret)):
        result[i] = _hurst_window(log_ret[i - window:i])
    return pd.Series(result, index=close.index)


# ── Funciones auxiliares — ML ─────────────────────────────────────────────────

def _pick_method(method: str) -> str:
    if method != "auto":
        return method
    if _HAS_LGB:
        return "lgb"
    if _HAS_XGB:
        return "xgb"
    if _HAS_SKLEARN:
        return "rf"
    return "correlation"


def _fill_nan(df: pd.DataFrame, strategy: str) -> pd.DataFrame:
    if strategy == "ffill":
        return df.ffill().fillna(0)
    if strategy == "mean":
        return df.fillna(df.mean())
    return df.fillna(0)


def _lgb_importances(
    X: np.ndarray, y: np.ndarray, cols: list[str], cfg: IndicatorSelectorConfig
) -> tuple[dict[str, float], float | None]:
    import lightgbm as lgb  # noqa: F811
    params = {
        "objective": "binary", "verbosity": -1, "n_estimators": cfg.n_estimators,
        "max_depth": cfg.max_depth, "random_state": cfg.random_state,
        "n_jobs": -1,
    }
    model = lgb.LGBMClassifier(**params)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        model.fit(X, y)
    imp = dict(zip(cols, model.feature_importances_.astype(float)))
    return imp, None


def _xgb_importances(
    X: np.ndarray, y: np.ndarray, cols: list[str], cfg: IndicatorSelectorConfig
) -> tuple[dict[str, float], float | None]:
    import xgboost as xgb  # noqa: F811
    model = xgb.XGBClassifier(
        n_estimators=cfg.n_estimators, max_depth=cfg.max_depth,
        use_label_encoder=False, eval_metric="logloss",
        random_state=cfg.random_state, n_jobs=-1, verbosity=0,
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        model.fit(X, y)
    imp = dict(zip(cols, model.feature_importances_.astype(float)))
    return imp, None


def _rf_importances(
    X: np.ndarray, y: np.ndarray, cols: list[str], cfg: IndicatorSelectorConfig
) -> tuple[dict[str, float], float | None]:
    from sklearn.ensemble import RandomForestClassifier  # noqa: F811
    model = RandomForestClassifier(
        n_estimators=cfg.n_estimators, max_depth=cfg.max_depth,
        oob_score=True, random_state=cfg.random_state, n_jobs=-1,
    )
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        model.fit(X, y)
    imp = dict(zip(cols, model.feature_importances_.astype(float)))
    oob = float(model.oob_score_) if hasattr(model, "oob_score_") else None
    return imp, oob


def _correlation_importances(
    features: pd.DataFrame, targets: pd.Series
) -> dict[str, float]:
    """Fallback: usa correlación absoluta como proxy de importancia."""
    tgt_float = targets.astype(float)
    corrs = {col: abs(float(features[col].corr(tgt_float))) for col in features.columns}
    # Reemplazar NaN por 0
    return {k: (v if not np.isnan(v) else 0.0) for k, v in corrs.items()}
