"""Análisis ligero de ciclos en precio (autocorrelación de retornos + Hurst heurístico).

Sin dependencias pesadas: sirve como filtro estructural previo a señales de trading,
no como predictor único. Complementa el régimen ML y el contexto heurístico.
"""
from __future__ import annotations

import math
from typing import Any, Sequence

import numpy as np

try:
    from atlas_code_quant.models.regime_classifier import RegimeClassifier
except Exception:  # pragma: no cover
    RegimeClassifier = None  # type: ignore[misc,assignment]


def _hurst_proxy(closes: np.ndarray) -> float:
    if RegimeClassifier is not None:
        try:
            return float(RegimeClassifier._compute_hurst_simple(closes.astype(float)))
        except Exception:
            pass
    return 0.5


def _best_autocorr_lag(returns: np.ndarray, min_lag: int, max_lag: int) -> tuple[int, float]:
    n = len(returns)
    if n < max_lag * 2:
        return 0, 0.0
    max_lag = min(max_lag, n // 2 - 1)
    best_lag, best_ac = 0, 0.0
    for lag in range(min_lag, max_lag + 1):
        a = returns[lag:]
        b = returns[:-lag]
        if len(a) < 10:
            break
        ac = float(np.corrcoef(a, b)[0, 1])
        if math.isfinite(ac) and ac > best_ac:
            best_ac, best_lag = ac, lag
    return best_lag, best_ac


def analyze_price_cycles(
    closes: Sequence[float],
    *,
    min_lag: int = 5,
    max_lag: int = 80,
) -> dict[str, Any]:
    """Devuelve pistas de periodicidad y mean-reversion vs trending.

    - ``dominant_lag_bars``: desfase (en barras) con mayor autocorrelación en retornos log.
    - ``cycle_strength``: autocorrelación máxima encontrada (0–1 aprox.).
    - ``mean_reversion_friendly``: Hurst bajo + ciclo moderado (heurística operativa).
    """
    arr = np.asarray(list(closes), dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size < max(min_lag * 3, 48):
        return {
            "available": False,
            "reason": "insufficient_bars",
            "n": int(arr.size),
        }

    log_p = np.log(arr + 1e-12)
    r = np.diff(log_p)
    if len(r) < max_lag + 5:
        return {"available": False, "reason": "insufficient_returns", "n": int(arr.size)}

    lag, ac = _best_autocorr_lag(r, min_lag=min_lag, max_lag=min(max_lag, len(r) // 2 - 1))
    hurst = _hurst_proxy(arr)
    strength = max(0.0, min(1.0, abs(ac)))

    # Ciclo “moderado”: autocorr positiva en retornos sugiere inercia a corto plazo;
    # mean-reversion clásico suele asociarse a Hurst < 0.5 en el precio.
    mean_rev = hurst < 0.48 and 0.12 <= strength <= 0.55
    trending_bias = hurst >= 0.52 and strength >= 0.18

    hint = "neutral"
    if mean_rev and not trending_bias:
        hint = "mean_reversion_friendly"
    elif trending_bias:
        hint = "trending_bias"
    elif strength < 0.08:
        hint = "noise_like"

    return {
        "available": True,
        "n": int(arr.size),
        "hurst_exponent": round(hurst, 4),
        "dominant_lag_bars": int(lag),
        "cycle_strength": round(strength, 4),
        "autocorr_at_lag": round(ac, 4) if lag > 0 else 0.0,
        "mean_reversion_friendly": bool(mean_rev),
        "trending_bias": bool(trending_bias),
        "cycle_hint": hint,
    }
