"""Scoring y contratos canónicos para el Options Trading Engine (paper).

Módulo diseñado para ser multi-activo y configurable por familia de activo sin
acoplarse a un broker específico.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Literal, Optional

from pydantic import BaseModel, Field, field_validator

try:  # pragma: no cover - fallback opcional para entornos offline
    import yfinance as yf
except Exception:  # pragma: no cover
    yf = None


AssetFamily = Literal[
    "INDEX",
    "ETF",
    "LARGE_CAP_EQUITY",
    "HIGH_BETA_EQUITY",
    "SECTOR_ETF",
    "EVENT_DRIVEN_EQUITY",
]


@dataclass
class VolData:
    iv_rank: float
    vrp_20d: float
    term_structure_slope: float
    skew_25d: float


@dataclass
class GexData:
    net_gex: float
    gamma_flip_distance_pct: float
    max_pain_distance_pct: float


@dataclass
class OiData:
    oi_change_1d_pct: float
    call_put_volume_ratio: float
    max_pain_distance_pct: float


@dataclass
class PriceRegime:
    adx: float
    ema_alignment: str
    breakout_status: str


@dataclass
class GlobalRegime:
    event_risk: bool
    regime_id: str


SCORING_CONFIG: dict[str, dict[str, float]] = {
    "vol": {
        "iv_rank_high_min": 55,
        "iv_rank_high_max": 85,
        "iv_rank_moderate_min": 40,
        "vrp_strong": 8.0,
        "vrp_moderate": 4.0,
        "term_contango_min": 0.5,
        "skew_neutral_max": 8.0,
        "event_risk_multiplier": 0.4,
        "weight": 0.40,
    },
    "gamma": {
        "net_gex_threshold": 0,
        "gamma_flip_max_dist": 1.5,
        "max_pain_max_dist": 1.5,
        "event_risk_multiplier": 0.6,
        "weight": 0.20,
    },
    "oi": {
        "max_pain_max_dist": 1.2,
        "oi_change_min": 10.0,
        "call_put_neutral_min": 0.8,
        "call_put_neutral_max": 1.2,
        "event_risk_multiplier": 0.5,
        "weight": 0.15,
    },
    "price": {
        "adx_ranging_max": 25,
        "adx_trending_min": 30,
        "event_risk_multiplier": 0.7,
        "weight": 0.25,
    },
}


ASSET_FAMILY_SCORE_CONFIG: dict[AssetFamily, dict[str, float]] = {
    "INDEX": {"min_total_score": 75, "score_multiplier": 1.05},
    "ETF": {"min_total_score": 76, "score_multiplier": 1.02},
    "LARGE_CAP_EQUITY": {"min_total_score": 80, "score_multiplier": 1.00},
    "HIGH_BETA_EQUITY": {"min_total_score": 84, "score_multiplier": 0.96},
    "SECTOR_ETF": {"min_total_score": 78, "score_multiplier": 1.01},
    "EVENT_DRIVEN_EQUITY": {"min_total_score": 88, "score_multiplier": 0.90},
}


class Leg(BaseModel):
    side: Literal["buy", "sell"]
    type: Literal["call", "put"]
    strike: float
    delta: float = Field(..., ge=-1.0, le=1.0)


class OptionStructure(BaseModel):
    symbol: str
    strategy: str
    expiry: str
    legs: List[Leg]
    contracts: int = Field(..., gt=0)

    @field_validator("legs")
    @classmethod
    def validate_legs(cls, v: List[Leg]) -> List[Leg]:
        if not 2 <= len(v) <= 4:
            raise ValueError("Estrategias soportadas deben tener entre 2 y 4 legs")
        return v


def _clip_score(value: float) -> float:
    return max(0.0, min(100.0, float(value)))


def calculate_vol_score(vol_data: VolData, global_regime: GlobalRegime, config: dict[str, Any] | None = None) -> float:
    cfg = (config or SCORING_CONFIG).get("vol", {})
    score = 0.0
    iv_rank = float(vol_data.iv_rank)
    vrp = float(vol_data.vrp_20d)
    slope = float(vol_data.term_structure_slope)
    skew = abs(float(vol_data.skew_25d))

    if cfg["iv_rank_high_min"] <= iv_rank <= cfg["iv_rank_high_max"]:
        score += 40.0
    elif iv_rank >= cfg["iv_rank_moderate_min"]:
        score += 24.0
    else:
        score += 10.0

    if vrp >= cfg["vrp_strong"]:
        score += 30.0
    elif vrp >= cfg["vrp_moderate"]:
        score += 20.0
    elif vrp > 0:
        score += 12.0

    if slope >= cfg["term_contango_min"]:
        score += 20.0
    if skew <= cfg["skew_neutral_max"]:
        score += 10.0

    if global_regime.event_risk:
        score *= float(cfg.get("event_risk_multiplier", 1.0))
    return _clip_score(score)


def calculate_gamma_score(gex_data: GexData, global_regime: GlobalRegime, config: dict[str, Any] | None = None) -> float:
    cfg = (config or SCORING_CONFIG).get("gamma", {})
    score = 0.0
    net_gex = float(gex_data.net_gex)
    gamma_flip_dist = abs(float(gex_data.gamma_flip_distance_pct))
    max_pain_dist = abs(float(gex_data.max_pain_distance_pct))

    if net_gex <= float(cfg["net_gex_threshold"]):
        score += 40.0
    else:
        score += 20.0
    if gamma_flip_dist <= float(cfg["gamma_flip_max_dist"]):
        score += 30.0
    if max_pain_dist <= float(cfg["max_pain_max_dist"]):
        score += 30.0
    if global_regime.event_risk:
        score *= float(cfg.get("event_risk_multiplier", 1.0))
    return _clip_score(score)


def calculate_oi_score(oi_data: OiData, global_regime: GlobalRegime, config: dict[str, Any] | None = None) -> float:
    cfg = (config or SCORING_CONFIG).get("oi", {})
    score = 0.0
    oi_change = float(oi_data.oi_change_1d_pct)
    cp = float(oi_data.call_put_volume_ratio)
    max_pain_dist = abs(float(oi_data.max_pain_distance_pct))

    if oi_change >= float(cfg["oi_change_min"]):
        score += 40.0
    elif oi_change > 0:
        score += 20.0

    if float(cfg["call_put_neutral_min"]) <= cp <= float(cfg["call_put_neutral_max"]):
        score += 30.0
    elif 0.6 <= cp <= 1.4:
        score += 18.0

    if max_pain_dist <= float(cfg["max_pain_max_dist"]):
        score += 30.0
    if global_regime.event_risk:
        score *= float(cfg.get("event_risk_multiplier", 1.0))
    return _clip_score(score)


def calculate_price_score(
    price_regime: PriceRegime,
    global_regime: GlobalRegime,
    config: dict[str, Any] | None = None,
) -> float:
    cfg = (config or SCORING_CONFIG).get("price", {})
    score = 0.0
    adx = float(price_regime.adx)
    breakout = str(price_regime.breakout_status or "").lower()
    ema = str(price_regime.ema_alignment or "").lower()

    if adx <= float(cfg["adx_ranging_max"]):
        score += 35.0
    elif adx >= float(cfg["adx_trending_min"]):
        score += 35.0
    else:
        score += 18.0

    if "breakout" in breakout:
        score += 35.0
    elif "range" in breakout or "inside" in breakout:
        score += 20.0
    else:
        score += 10.0

    if "bull" in ema or "bear" in ema or "stacked" in ema:
        score += 30.0
    elif "mixed" in ema:
        score += 15.0

    if global_regime.event_risk:
        score *= float(cfg.get("event_risk_multiplier", 1.0))
    return _clip_score(score)


def combine_scores(
    *,
    vol_score: float,
    gamma_score: float,
    oi_score: float,
    price_score: float,
    asset_family: AssetFamily,
    config: dict[str, Any] | None = None,
) -> float:
    cfg = config or SCORING_CONFIG
    weighted = (
        float(vol_score) * float(cfg["vol"]["weight"])
        + float(gamma_score) * float(cfg["gamma"]["weight"])
        + float(oi_score) * float(cfg["oi"]["weight"])
        + float(price_score) * float(cfg["price"]["weight"])
    )
    family_cfg = ASSET_FAMILY_SCORE_CONFIG.get(asset_family, {"score_multiplier": 1.0})
    return _clip_score(weighted * float(family_cfg.get("score_multiplier", 1.0)))


def get_min_total_score_for_asset_family(asset_family: str) -> float:
    try:
        fam = asset_family if asset_family in ASSET_FAMILY_SCORE_CONFIG else "LARGE_CAP_EQUITY"
        return float(ASSET_FAMILY_SCORE_CONFIG[fam]["min_total_score"])
    except Exception:
        return 80.0


SHORT_PREMIUM_STRATEGIES = {
    "IRON_CONDOR",
    "CREDIT_SPREAD",
    "PUT_CREDIT_SPREAD",
    "CALL_CREDIT_SPREAD",
    "BUTTERFLY",
}
LONG_PREMIUM_STRATEGIES = {
    "LONG_CALL",
    "LONG_PUT",
    "DEBIT_SPREAD",
    "CALL_DEBIT_SPREAD",
    "PUT_DEBIT_SPREAD",
}


def matches_context_strategy_map(
    *,
    strategy: str,
    vol_data: VolData,
    gex_data: GexData,
    oi_data: OiData,
    price_regime: PriceRegime,
    global_regime: GlobalRegime,
    asset_family: str,
    earnings_days: Optional[int] = None,
    spread_pct: Optional[float] = None,
) -> bool:
    s = str(strategy or "").upper()
    event_driven = str(asset_family or "").upper() == "EVENT_DRIVEN_EQUITY"
    earnings_soon = earnings_days is not None and earnings_days <= 5
    high_spread = spread_pct is not None and spread_pct > 0.03
    if s in SHORT_PREMIUM_STRATEGIES:
        if global_regime.event_risk:
            return False
        if event_driven and earnings_soon:
            return False
        if high_spread and str(asset_family).upper() in {"HIGH_BETA_EQUITY", "EVENT_DRIVEN_EQUITY"}:
            return False
        return (
            float(vol_data.iv_rank) >= 50.0
            and float(vol_data.vrp_20d) > 0
            and float(vol_data.term_structure_slope) >= 0
            and "range" in str(price_regime.breakout_status).lower()
            and float(gex_data.net_gex) <= 0
            and abs(float(gex_data.max_pain_distance_pct)) <= 2.0
        )
    if s in LONG_PREMIUM_STRATEGIES:
        return (
            float(vol_data.iv_rank) <= 45.0
            and float(vol_data.vrp_20d) <= 5.0
            and ("breakout" in str(price_regime.breakout_status).lower() or float(price_regime.adx) >= 30.0)
            and abs(float(gex_data.gamma_flip_distance_pct)) <= 3.0
        )
    if s == "CALENDAR":
        return (
            abs(float(vol_data.vrp_20d)) >= 4.0
            and float(vol_data.term_structure_slope) > -1.0
            and 18.0 <= float(price_regime.adx) <= 36.0
            and float(oi_data.oi_change_1d_pct) >= 8.0
        )
    return True


def calculate_iv_rank_from_yfinance(symbol: str, period: str = "6mo") -> Optional[float]:
    """Fallback offline para estimar IV Rank aproximado desde históricos.

    No reemplaza la fuente principal de IV de ATLAS; solo cubre modo degradado.
    """
    if yf is None:
        return None
    try:  # pragma: no cover - depende de red externa
        ticker = yf.Ticker(symbol)
        hist = ticker.history(period=period, auto_adjust=True)
        if hist is None or hist.empty:
            return None
        ret = hist["Close"].pct_change().dropna()
        if ret.empty:
            return None
        rolling = ret.rolling(20).std().dropna() * (252.0**0.5) * 100.0
        if rolling.empty:
            return None
        last = float(rolling.iloc[-1])
        lo = float(rolling.min())
        hi = float(rolling.max())
        if hi <= lo:
            return 50.0
        return max(0.0, min(100.0, (last - lo) / (hi - lo) * 100.0))
    except Exception:
        return None


def generate_candidate_strategies(*_: Any, **__: Any) -> list[dict[str, Any]]:
    """Compatibilidad: el generador real vive en ``options_pipeline``."""
    return []

