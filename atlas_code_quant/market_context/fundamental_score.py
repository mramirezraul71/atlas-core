"""
Fundamentales para equities vía yfinance (Fase 3, opt-in).
"""
from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger("quant.fundamental_score")


def fetch_fundamental_data(symbol: str) -> dict[str, Any]:
    out: dict[str, Any] = {
        "per": None,
        "pbr": None,
        "roe": None,
        "debt_to_equity": None,
        "dividend_yield": 0.0,
        "market_cap": None,
    }
    try:
        import yfinance as yf  # type: ignore

        t = yf.Ticker(symbol)
        info = t.info or {}
        out["per"] = info.get("trailingPE")
        out["pbr"] = info.get("priceToBook")
        out["roe"] = info.get("returnOnEquity")
        out["debt_to_equity"] = info.get("debtToEquity")
        out["dividend_yield"] = float(info.get("dividendYield") or 0.0)
        out["market_cap"] = info.get("marketCap")
    except Exception as exc:
        logger.debug("fetch_fundamental_data %s: %s", symbol, exc)
    return out


def calculate_fundamental_score(symbol: str, sector_medians: dict[str, float] | None = None) -> float:
    data = fetch_fundamental_data(symbol)
    score = 0.0
    per_m = (sector_medians or {}).get("per")
    per = data.get("per")
    if isinstance(per, (int, float)) and per == per:
        if per_m and isinstance(per_m, (int, float)) and per_m > 0:
            if per < per_m * 0.8:
                score += 15.0
            elif per > per_m * 1.5:
                score -= 10.0
        elif per < 15.0:
            score += 10.0
    roe = data.get("roe")
    if isinstance(roe, (int, float)) and roe == roe:
        if roe > 0.15:
            score += 10.0
        elif roe < 0.05:
            score -= 5.0
    de = data.get("debt_to_equity")
    if isinstance(de, (int, float)) and de == de:
        if de < 1.0:
            score += 10.0
        elif de > 2.0:
            score -= 5.0
    if float(data.get("dividend_yield") or 0.0) > 0.02:
        score += 5.0
    return max(0.0, min(score, 50.0))


def apply_fundamental_filter(fundamental_score: float, min_score: float = 25.0, reject_below: float = 15.0) -> str:
    if fundamental_score < reject_below:
        return "reject"
    if fundamental_score < min_score:
        return "downweight"
    return "accept"
