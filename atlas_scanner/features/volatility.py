from __future__ import annotations

from collections.abc import Mapping, Sequence

from atlas_scanner.models import SymbolSnapshot


def compute_volatility(symbol: SymbolSnapshot) -> float:
    _ = symbol
    return 0.0


def compute_iv_rank(iv_history: Sequence[float], current_iv: float) -> float:
    """
    Compute IV Rank (0-100) as position of current_iv within iv_history range.

    Returns 0.0 when inputs are insufficient or range is zero.
    """
    if not iv_history:
        return 0.0
    min_iv = min(iv_history)
    max_iv = max(iv_history)
    if max_iv <= min_iv:
        return 0.0
    rank = 100.0 * (current_iv - min_iv) / (max_iv - min_iv)
    if rank < 0.0:
        return 0.0
    if rank > 100.0:
        return 100.0
    return rank


def compute_iv_percentile(iv_history: Sequence[float], current_iv: float) -> float:
    """
    Compute IV Percentile (0-100) as percentage of iv_history <= current_iv.

    Returns 0.0 when iv_history is empty.
    """
    if not iv_history:
        return 0.0
    count = sum(1 for value in iv_history if value <= current_iv)
    return 100.0 * count / len(iv_history)


def compute_vrp(iv_annualized: float, rv_annualized: float) -> float:
    """Compute volatility risk premium as IV - RV."""
    return iv_annualized - rv_annualized


def compute_vrp_series(
    iv_series: Mapping[str, float],
    rv_series: Mapping[str, float],
) -> dict[str, float]:
    """
    Compute VRP for keys present in both iv_series and rv_series.
    """
    return {
        key: compute_vrp(iv_annualized=iv_series[key], rv_annualized=rv_series[key])
        for key in iv_series.keys()
        if key in rv_series
    }

