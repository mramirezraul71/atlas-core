"""Ichimoku Kinko Hyo: nube, Tenkan, Kijun y confianza 0–1 para el último cierre."""
from __future__ import annotations

from typing import Literal

import pandas as pd

CloudPos = Literal["above", "inside", "below"]


def add_ichimoku_columns(df: pd.DataFrame) -> pd.DataFrame:
    """Añade tenkan, kijun, senkou_a, senkou_b (estándar 9/26/52, desplazamiento 26)."""
    need = {"high", "low", "close"}
    if not need.issubset(df.columns):
        return df
    out = df.copy()
    high = out["high"].astype(float)
    low = out["low"].astype(float)

    nine_high = high.rolling(9, min_periods=9).max()
    nine_low = low.rolling(9, min_periods=9).min()
    out["tenkan"] = (nine_high + nine_low) / 2.0

    tw26_high = high.rolling(26, min_periods=26).max()
    tw26_low = low.rolling(26, min_periods=26).min()
    out["kijun"] = (tw26_high + tw26_low) / 2.0

    out["senkou_a"] = ((out["tenkan"] + out["kijun"]) / 2.0).shift(26)
    fifty_two_high = high.rolling(52, min_periods=52).max()
    fifty_two_low = low.rolling(52, min_periods=52).min()
    out["senkou_b"] = ((fifty_two_high + fifty_two_low) / 2.0).shift(26)
    return out


def ichimoku_cloud_position(price: float, senkou_a: float, senkou_b: float) -> CloudPos | None:
    """Posición del precio respecto a la nube (span A/B)."""
    if not all(map(pd.notna, (price, senkou_a, senkou_b))):
        return None
    top = max(float(senkou_a), float(senkou_b))
    bot = min(float(senkou_a), float(senkou_b))
    if float(price) > top:
        return "above"
    if float(price) < bot:
        return "below"
    return "inside"


def ichimoku_signal(price: float, tenkan: float, kijun: float, senkou_a: float, senkou_b: float) -> int:
    """Señal discreta −10…+10 (precio vs nube; Tenkan vs Kijun)."""
    cloud = ichimoku_cloud_position(price, senkou_a, senkou_b)
    if cloud is None:
        return 0
    if cloud == "above":
        return 10 if tenkan > kijun else 5
    if cloud == "below":
        return -10 if tenkan < kijun else -5
    return 0


def last_ichimoku_confidence(df: pd.DataFrame) -> float:
    """Confianza 0–1 a partir de la última fila con columnas Ichimoku."""
    need = {"close", "tenkan", "kijun", "senkou_a", "senkou_b"}
    if not need.issubset(df.columns) or len(df) < 1:
        return 0.5
    row = df.iloc[-1]
    sig = ichimoku_signal(
        float(row["close"]),
        float(row["tenkan"]),
        float(row["kijun"]),
        float(row["senkou_a"]),
        float(row["senkou_b"]),
    )
    return max(0.0, min(1.0, (float(sig) + 10.0) / 20.0))
