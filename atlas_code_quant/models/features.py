"""Atlas Code-Quant — Feature engineering para modelos ML.

Transforma OHLCV en un DataFrame de features técnicas listo para
entrenar clasificadores de señales (BUY/HOLD/SELL).

v2 (logarítmico): Retornos logarítmicos ln(P_t/P_{t-1}) como features
principales — más precisos y aditivos en series temporales financieras.
Grok/xAI recomendación: calcular retornos log en lugar de aritméticos
para backtesting preciso y como features para IA.
"""
from __future__ import annotations

import numpy as np
import pandas as pd


def build_features(df: pd.DataFrame, target_bars: int = 5) -> pd.DataFrame:
    """Construye features técnicas a partir de OHLCV.

    Args:
        df: OHLCV DataFrame con columnas open/high/low/close/volume.
        target_bars: Ventana futura para calcular el target (retorno log).

    Returns:
        DataFrame con features + columna 'target' (1=BUY, 0=HOLD, -1=SELL).
        Filas con NaN eliminadas.
    """
    out = df.copy()
    c = out["close"]
    h = out["high"]
    l = out["low"]
    v = out["volume"]

    # ── Retornos logarítmicos (v2) ───────────────────────────────────────────
    # ln(P_t / P_{t-1}) — aditivos, más precisos que aritméticos para ML
    out["log_ret_1"]  = np.log(c / c.shift(1))
    out["log_ret_3"]  = np.log(c / c.shift(3))
    out["log_ret_5"]  = np.log(c / c.shift(5))
    out["log_ret_10"] = np.log(c / c.shift(10))
    out["log_ret_20"] = np.log(c / c.shift(20))

    # Retornos aritméticos mantenidos para compatibilidad con modelos legacy
    out["ret_1"]  = c.pct_change(1)
    out["ret_5"]  = c.pct_change(5)
    out["ret_10"] = c.pct_change(10)

    # ── Medias móviles ───────────────────────────────────────────────────────
    for p in [5, 10, 20, 50, 200]:
        out[f"sma_{p}"] = c.rolling(p).mean()
    for p in [5, 10, 20, 50]:
        out[f"ema_{p}"] = c.ewm(span=p, adjust=False).mean()

    # ── Distancia logarítmica a MAs ──────────────────────────────────────────
    # Usar log-ratio en lugar de ratio aritmético — más estable para ML
    out["ldist_sma20"]  = np.log(c / out["sma_20"])
    out["ldist_sma50"]  = np.log(c / out["sma_50"])
    out["ldist_sma200"] = np.log(c / out["sma_200"])

    # ── RSI ──────────────────────────────────────────────────────────────────
    out["rsi_14"] = _rsi(c, 14)
    out["rsi_7"]  = _rsi(c, 7)

    # ── MACD ─────────────────────────────────────────────────────────────────
    ema12 = c.ewm(span=12, adjust=False).mean()
    ema26 = c.ewm(span=26, adjust=False).mean()
    macd  = ema12 - ema26
    sig   = macd.ewm(span=9, adjust=False).mean()
    out["macd"]      = macd
    out["macd_sig"]  = sig
    out["macd_hist"] = macd - sig
    # MACD normalizado por precio (escala independiente)
    out["macd_norm"] = macd / (c + 1e-10)

    # ── Bollinger Bands ──────────────────────────────────────────────────────
    bb_mid   = c.rolling(20).mean()
    bb_std   = c.rolling(20).std()
    out["bb_upper"]  = bb_mid + 2 * bb_std
    out["bb_lower"]  = bb_mid - 2 * bb_std
    out["bb_pct"]    = (c - out["bb_lower"]) / (out["bb_upper"] - out["bb_lower"] + 1e-10)
    out["bb_width"]  = (out["bb_upper"] - out["bb_lower"]) / (bb_mid + 1e-10)

    # ── ATR — volatilidad realizada ──────────────────────────────────────────
    tr = pd.concat([
        h - l,
        (h - c.shift(1)).abs(),
        (l - c.shift(1)).abs(),
    ], axis=1).max(axis=1)
    out["atr_14"] = tr.rolling(14).mean()
    out["atr_pct"] = out["atr_14"] / (c + 1e-10)

    # ── Régimen de volatilidad (proxy VIX) ───────────────────────────────────
    # Desviación estándar de retornos log en ventana rodante = volatilidad realizada
    out["vol_realized_20"] = out["log_ret_1"].rolling(20).std() * np.sqrt(252)
    out["vol_realized_5"]  = out["log_ret_1"].rolling(5).std()  * np.sqrt(252)
    # Ratio volatilidad corta/larga — detecta expansión de vol (breakouts)
    out["vol_regime"] = out["vol_realized_5"] / (out["vol_realized_20"] + 1e-10)

    # ── Volumen ──────────────────────────────────────────────────────────────
    out["vol_sma20"]   = v.rolling(20).mean()
    out["vol_ratio"]   = v / (out["vol_sma20"] + 1e-10)
    out["vol_log_ret"] = np.log(v / v.shift(1) + 1e-10)

    # ── Order Flow Proxy ─────────────────────────────────────────────────────
    # Aproximación de presión compradora/vendedora con datos OHLCV
    # (sustituye Order Book imbalance cuando no hay datos L2)
    out["close_pct_range"] = (c - l) / (h - l + 1e-10)  # 0=mínimo, 1=máximo
    out["up_vol_ratio"]    = np.where(c > c.shift(1), v, 0) / (out["vol_sma20"] + 1e-10)
    out["down_vol_ratio"]  = np.where(c < c.shift(1), v, 0) / (out["vol_sma20"] + 1e-10)
    # On-Balance Volume normalizado
    obv = (np.sign(c.diff()) * v).fillna(0).cumsum()
    out["obv_norm"] = (obv - obv.rolling(20).mean()) / (obv.rolling(20).std() + 1e-10)

    # ── Momentum ─────────────────────────────────────────────────────────────
    out["mom_10"]  = c - c.shift(10)
    out["roc_10"]  = np.log(c / c.shift(10))   # Log ROC — más estable
    out["roc_20"]  = np.log(c / c.shift(20))

    # ── Stochastic ───────────────────────────────────────────────────────────
    low14  = l.rolling(14).min()
    high14 = h.rolling(14).max()
    out["stoch_k"] = (c - low14) / (high14 - low14 + 1e-10) * 100
    out["stoch_d"] = out["stoch_k"].rolling(3).mean()

    # ── Candlestick body / shadow ────────────────────────────────────────────
    out["body_pct"]      = (out["close"] - out["open"]).abs() / (h - l + 1e-10)
    out["upper_shadow"]  = (h - out[["open", "close"]].max(axis=1)) / (h - l + 1e-10)
    out["lower_shadow"]  = (out[["open", "close"]].min(axis=1) - l) / (h - l + 1e-10)

    # ── Target logarítmico: retorno futuro log → label ───────────────────────
    # Usar retorno logarítmico para el target — más preciso (Grok criterio v2)
    future_log_ret = np.log(c.shift(-target_bars) / c)
    threshold = 0.003   # ±0.3% en log-space ≈ ±0.3% aritmético para pequeños valores
    out["target"] = np.where(future_log_ret > threshold, 1,
                    np.where(future_log_ret < -threshold, -1, 0))
    # Guardar también el retorno log futuro como referencia
    out["future_log_ret"] = future_log_ret

    # Eliminar columnas intermedias no deseadas como features
    drop_cols = [
        "open", "high", "low", "close", "volume",
        "sma_5", "sma_10", "sma_20", "sma_50", "sma_200",
        "ema_5", "ema_10", "ema_20", "ema_50",
        "bb_upper", "bb_lower", "vol_sma20",
        "macd_sig", "atr_14",
        "future_log_ret",   # solo usada para target, no como feature
        "mom_10",           # colineal con roc_10
    ]
    out.drop(columns=[col for col in drop_cols if col in out.columns], inplace=True)

    return out.dropna()


def get_feature_names(df_features: pd.DataFrame) -> list[str]:
    """Retorna la lista de nombres de features (excluye 'target')."""
    return [c for c in df_features.columns if c != "target"]


def compute_atr(df: pd.DataFrame, period: int = 14) -> pd.Series:
    """Calcula ATR sobre un DataFrame OHLCV. Útil desde execution/strategy."""
    h = df["high"]
    l = df["low"]
    c = df["close"]
    tr = pd.concat([
        h - l,
        (h - c.shift(1)).abs(),
        (l - c.shift(1)).abs(),
    ], axis=1).max(axis=1)
    return tr.rolling(period).mean()


# ── Helpers ──────────────────────────────────────────────────────────────────

def _rsi(series: pd.Series, period: int = 14) -> pd.Series:
    delta  = series.diff()
    gain   = delta.clip(lower=0).rolling(period).mean()
    loss   = (-delta.clip(upper=0)).rolling(period).mean()
    rs     = gain / (loss + 1e-10)
    return 100 - (100 / (1 + rs))
