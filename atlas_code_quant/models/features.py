"""Atlas Code-Quant — Feature engineering para modelos ML.

Transforma OHLCV en un DataFrame de features técnicas listo para
entrenar clasificadores de señales (BUY/HOLD/SELL).
"""
from __future__ import annotations

import numpy as np
import pandas as pd


def build_features(df: pd.DataFrame, target_bars: int = 5) -> pd.DataFrame:
    """Construye features técnicas a partir de OHLCV.

    Args:
        df: OHLCV DataFrame con columnas open/high/low/close/volume.
        target_bars: Ventana futura para calcular el target (retorno).

    Returns:
        DataFrame con features + columna 'target' (1=BUY, 0=HOLD, -1=SELL).
        Filas con NaN eliminadas.
    """
    out = df.copy()
    c = out["close"]
    h = out["high"]
    l = out["low"]
    v = out["volume"]

    # ── Retornos ─────────────────────────────────────────────────────────────
    out["ret_1"]  = c.pct_change(1)
    out["ret_3"]  = c.pct_change(3)
    out["ret_5"]  = c.pct_change(5)
    out["ret_10"] = c.pct_change(10)

    # ── Medias móviles ───────────────────────────────────────────────────────
    for p in [5, 10, 20, 50, 200]:
        out[f"sma_{p}"] = c.rolling(p).mean()
    for p in [5, 10, 20, 50]:
        out[f"ema_{p}"] = c.ewm(span=p, adjust=False).mean()

    # ── Distancia a MAs ──────────────────────────────────────────────────────
    out["dist_sma20"]  = (c - out["sma_20"])  / out["sma_20"]
    out["dist_sma50"]  = (c - out["sma_50"])  / out["sma_50"]
    out["dist_sma200"] = (c - out["sma_200"]) / out["sma_200"]

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

    # ── Bollinger Bands ──────────────────────────────────────────────────────
    bb_mid   = c.rolling(20).mean()
    bb_std   = c.rolling(20).std()
    out["bb_upper"]  = bb_mid + 2 * bb_std
    out["bb_lower"]  = bb_mid - 2 * bb_std
    out["bb_pct"]    = (c - out["bb_lower"]) / (out["bb_upper"] - out["bb_lower"] + 1e-10)
    out["bb_width"]  = (out["bb_upper"] - out["bb_lower"]) / bb_mid

    # ── ATR ──────────────────────────────────────────────────────────────────
    tr = pd.concat([
        h - l,
        (h - c.shift(1)).abs(),
        (l - c.shift(1)).abs(),
    ], axis=1).max(axis=1)
    out["atr_14"] = tr.rolling(14).mean()
    out["atr_pct"] = out["atr_14"] / c

    # ── Volumen ──────────────────────────────────────────────────────────────
    out["vol_sma20"]   = v.rolling(20).mean()
    out["vol_ratio"]   = v / (out["vol_sma20"] + 1e-10)
    out["vol_ret"]     = v.pct_change(1)

    # ── Momentum ─────────────────────────────────────────────────────────────
    out["mom_10"]  = c - c.shift(10)
    out["mom_20"]  = c - c.shift(20)
    out["roc_10"]  = (c - c.shift(10)) / (c.shift(10) + 1e-10) * 100

    # ── Stochastic ───────────────────────────────────────────────────────────
    low14  = l.rolling(14).min()
    high14 = h.rolling(14).max()
    out["stoch_k"] = (c - low14) / (high14 - low14 + 1e-10) * 100
    out["stoch_d"] = out["stoch_k"].rolling(3).mean()

    # ── Candlestick body / shadow ────────────────────────────────────────────
    out["body_pct"]      = (out["close"] - out["open"]).abs() / (out["high"] - out["low"] + 1e-10)
    out["upper_shadow"]  = (out["high"] - out[["open", "close"]].max(axis=1)) / (out["high"] - out["low"] + 1e-10)
    out["lower_shadow"]  = (out[["open", "close"]].min(axis=1) - out["low"]) / (out["high"] - out["low"] + 1e-10)

    # ── Target: retorno futuro → label ───────────────────────────────────────
    future_ret = c.shift(-target_bars) / c - 1
    threshold  = 0.005   # ±0.5%
    out["target"] = np.where(future_ret > threshold, 1,
                    np.where(future_ret < -threshold, -1, 0))

    # Eliminar columnas intermedias no deseadas como features
    drop_cols = ["open", "high", "low", "close", "volume",
                 "sma_5", "sma_10", "sma_20", "sma_50", "sma_200",
                 "ema_5", "ema_10", "ema_20", "ema_50",
                 "bb_upper", "bb_lower", "vol_sma20",
                 "macd_sig", "atr_14"]
    out.drop(columns=[c for c in drop_cols if c in out.columns], inplace=True)

    return out.dropna()


def get_feature_names(df_features: pd.DataFrame) -> list[str]:
    """Retorna la lista de nombres de features (excluye 'target')."""
    return [c for c in df_features.columns if c != "target"]


# ── Helpers ──────────────────────────────────────────────────────────────────

def _rsi(series: pd.Series, period: int = 14) -> pd.Series:
    delta  = series.diff()
    gain   = delta.clip(lower=0).rolling(period).mean()
    loss   = (-delta.clip(upper=0)).rolling(period).mean()
    rs     = gain / (loss + 1e-10)
    return 100 - (100 / (1 + rs))
