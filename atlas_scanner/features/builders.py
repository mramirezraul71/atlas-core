from __future__ import annotations

from dataclasses import dataclass, replace
from collections.abc import Mapping, Sequence

from atlas_scanner.features.gamma import (
    StrikeGamma,
    classify_gamma_regime,
    find_call_wall,
    find_gamma_flip,
    find_put_wall,
)
from atlas_scanner.features.volatility import (
    compute_iv_percentile,
    compute_iv_rank,
    compute_vrp,
)
from atlas_scanner.models.domain_models import (
    CandidateOpportunity,
    GammaFeatures,
    StrikeLevel,
    VolFeatures,
)


@dataclass(frozen=True)
class VolFeatureInput:
    iv_current: float | None = None
    iv_history: Sequence[float] | None = None
    rv_annualized: Mapping[str, float] | None = None


@dataclass(frozen=True)
class GammaFeatureInput:
    strike_gamma: Sequence[StrikeGamma] | None = None
    net_gamma: float | None = None
    neutral_threshold: float = 0.0


def _mapping_float(value: object) -> float | None:
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _extract_horizon_value(values: Mapping[str, float], horizon: str) -> float | None:
    if horizon in values:
        return float(values[horizon])
    normalized = horizon.rstrip("d")
    for key in values:
        if key.rstrip("d") == normalized:
            return float(values[key])
    return None


def build_vol_features(
    *,
    iv_current: float | None,
    iv_history: Sequence[float] | None,
    rv_annualized: Mapping[str, float] | None,
) -> VolFeatures:
    iv_rank = None
    iv_percentile = None
    if iv_current is not None and iv_history:
        iv_rank = compute_iv_rank(iv_history=iv_history, current_iv=iv_current)
        iv_percentile = compute_iv_percentile(iv_history=iv_history, current_iv=iv_current)

    vrp_5d = None
    vrp_10d = None
    vrp_20d = None
    vrp_60d = None
    if iv_current is not None and rv_annualized:
        rv_5d = _extract_horizon_value(rv_annualized, "5d")
        rv_10d = _extract_horizon_value(rv_annualized, "10d")
        rv_20d = _extract_horizon_value(rv_annualized, "20d")
        rv_60d = _extract_horizon_value(rv_annualized, "60d")
        if rv_5d is not None:
            vrp_5d = compute_vrp(iv_annualized=iv_current, rv_annualized=rv_5d)
        if rv_10d is not None:
            vrp_10d = compute_vrp(iv_annualized=iv_current, rv_annualized=rv_10d)
        if rv_20d is not None:
            vrp_20d = compute_vrp(iv_annualized=iv_current, rv_annualized=rv_20d)
        if rv_60d is not None:
            vrp_60d = compute_vrp(iv_annualized=iv_current, rv_annualized=rv_60d)

    return VolFeatures(
        iv_rank_20d=iv_rank,
        iv_rank_50d=iv_rank,
        iv_rank_100d=iv_rank,
        iv_percentile=iv_percentile,
        vrp_5d=vrp_5d,
        vrp_10d=vrp_10d,
        vrp_20d=vrp_20d,
        vrp_60d=vrp_60d,
    )


def build_gamma_features(
    *,
    strike_gamma: Sequence[StrikeGamma] | None,
    net_gamma: float | None = None,
    neutral_threshold: float = 0.0,
) -> GammaFeatures:
    ordered_strikes = tuple(strike_gamma) if strike_gamma is not None else ()

    call_wall_strike = find_call_wall(ordered_strikes) if ordered_strikes else None
    put_wall_strike = find_put_wall(ordered_strikes) if ordered_strikes else None
    gamma_flip = find_gamma_flip(ordered_strikes) if ordered_strikes else None

    call_wall = None
    put_wall = None
    if call_wall_strike is not None:
        call_wall = StrikeLevel(strike=call_wall_strike, kind="CALL_WALL")
    if put_wall_strike is not None:
        put_wall = StrikeLevel(strike=put_wall_strike, kind="PUT_WALL")

    regime = None
    if net_gamma is not None:
        regime = classify_gamma_regime(
            net_gamma=net_gamma,
            neutral_threshold=neutral_threshold,
        ).value

    return GammaFeatures(
        net_gex=_mapping_float(net_gamma),
        gamma_regime=regime,
        gamma_flip_agg=gamma_flip,
        call_wall_nearest=call_wall,
        put_wall_nearest=put_wall,
    )


def enrich_candidate_with_features(
    candidate: CandidateOpportunity,
    *,
    vol_input: VolFeatureInput | None = None,
    gamma_input: GammaFeatureInput | None = None,
) -> CandidateOpportunity:
    next_vol = (
        build_vol_features(
            iv_current=vol_input.iv_current,
            iv_history=vol_input.iv_history,
            rv_annualized=vol_input.rv_annualized,
        )
        if vol_input is not None
        else candidate.vol_features
    )
    next_gamma = (
        build_gamma_features(
            strike_gamma=gamma_input.strike_gamma,
            net_gamma=gamma_input.net_gamma,
            neutral_threshold=gamma_input.neutral_threshold,
        )
        if gamma_input is not None
        else candidate.gamma_features
    )
    return replace(candidate, vol_features=next_vol, gamma_features=next_gamma)

