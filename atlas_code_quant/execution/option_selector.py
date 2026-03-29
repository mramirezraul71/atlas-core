"""Option contract selector with governance-aware strategy choice.

Given a directional signal plus IV metrics and market regime, this module:
1. Chooses a strategy family
2. Selects an expiration
3. Picks liquid strikes
4. Returns a structured selection payload

The implementation stays intentionally pure: no direct I/O, no broker calls.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

logger = logging.getLogger("atlas.execution.option_selector")

from backtesting.winning_probability import StrategyLeg, StrategyType  # noqa: F401


@dataclass
class ContractSelection:
    """Selected option contract or structure."""

    strategy_type: str
    legs: list[dict]
    option_symbol: str = ""
    estimated_debit: float = 0.0
    estimated_credit: float = 0.0
    selected_dte: int = 0
    anchor_strike: float = 0.0
    width_pct: float = 0.0
    iv_rank_at_selection: float = 0.0
    selection_score: float = 0.0
    governance: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    is_valid: bool = True


def _normalize_thesis(thesis: str | None, regime: str, direction: str) -> str:
    raw = (thesis or "").strip().lower()
    if raw:
        return raw
    if (regime or "").upper() == "SIDEWAYS":
        return "range_bound"
    if (direction or "").upper() in {"BUY", "SELL"}:
        return "directional_controlled"
    return "directional_controlled"


def _governance_reasons(
    *,
    direction: str,
    regime: str,
    iv_rank: float,
    iv_hv_ratio: float,
    thesis: str,
    event_near: bool,
    liquidity_score: float,
    skew_pct: float,
    term_structure_slope: float,
    strategy: str,
) -> list[str]:
    reasons: list[str] = []
    if iv_rank >= 60 or iv_hv_ratio >= 1.2:
        reasons.append("High implied volatility favors defined-risk structures and selective premium selling.")
    elif iv_rank <= 25 and iv_hv_ratio <= 1.0:
        reasons.append("Low implied volatility supports long premium or debit structures.")
    else:
        reasons.append("Mid implied volatility favors controlled-risk structures over binary premium bets.")

    if thesis == "neutral_income":
        reasons.append("Range-compression thesis favors neutral theta structures.")
    elif thesis in {"directional_explosive", "vol_expansion", "event_driven"}:
        reasons.append("Expansion thesis avoids aggressive premium selling into gap risk.")
    else:
        reasons.append("Controlled directional thesis favors defined-risk delta exposure.")

    if event_near:
        reasons.append("Nearby event penalizes short-premium selection because gap risk increases.")
    if liquidity_score < 0.5:
        reasons.append("Weak liquidity penalizes credit selling because entry/exit spreads become fragile.")
    if abs(skew_pct) > 0.15:
        reasons.append("Marked skew argues against overly symmetric structures.")
    if term_structure_slope > 1.05:
        reasons.append("Upward term structure warns against relying only on spot IV level.")

    reasons.append(f"Chosen structure: {strategy}")
    return reasons


def describe_strategy_governance(
    *,
    direction: str,
    regime: str,
    iv_rank: float,
    iv_hv_ratio: float,
    strategy: str,
    thesis: str | None = None,
    event_near: bool = False,
    liquidity_score: float = 1.0,
    skew_pct: float = 0.0,
    term_structure_slope: float = 1.0,
) -> dict:
    normalized_thesis = _normalize_thesis(thesis, regime, direction)
    premium_stance = "neutral"
    if strategy in {"bull_put_credit_spread", "bear_call_credit_spread", "iron_condor", "iron_butterfly"}:
        premium_stance = "sell_premium_defined_risk"
    elif strategy in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        premium_stance = "debit_defined_risk"
    elif strategy in {"long_call", "long_put"}:
        premium_stance = "long_premium"

    if strategy in {"iron_condor", "iron_butterfly"}:
        strategy_family = "neutral_theta"
    elif strategy in {"bull_put_credit_spread", "bear_call_credit_spread"}:
        strategy_family = "directional_credit"
    elif strategy in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        strategy_family = "directional_debit"
    else:
        strategy_family = "directional_long_premium"

    volatility_posture = "balanced"
    if premium_stance == "sell_premium_defined_risk":
        volatility_posture = "short_volatility_defined_risk"
    elif premium_stance == "long_premium":
        volatility_posture = "long_volatility"

    return {
        "thesis": normalized_thesis,
        "event_near": bool(event_near),
        "liquidity_score": round(max(0.0, min(liquidity_score, 1.0)), 3),
        "skew_pct": round(skew_pct, 4),
        "term_structure_slope": round(term_structure_slope, 4),
        "premium_stance": premium_stance,
        "volatility_posture": volatility_posture,
        "strategy_family": strategy_family,
        "reasons": _governance_reasons(
            direction=direction,
            regime=regime,
            iv_rank=iv_rank,
            iv_hv_ratio=iv_hv_ratio,
            thesis=normalized_thesis,
            event_near=event_near,
            liquidity_score=liquidity_score,
            skew_pct=skew_pct,
            term_structure_slope=term_structure_slope,
            strategy=strategy,
        ),
    }


def pick_strategy(
    direction: str,
    regime: str,
    iv_rank: float,
    iv_hv_ratio: float,
    *,
    thesis: str | None = None,
    event_near: bool = False,
    liquidity_score: float = 1.0,
    skew_pct: float = 0.0,
    term_structure_slope: float = 1.0,
    prefer_defined_risk: bool = True,
) -> str:
    """Choose an option structure from currently supported executable families."""

    direction = direction.upper()
    regime = regime.upper() if regime else "SIDEWAYS"
    normalized_thesis = _normalize_thesis(thesis, regime, direction)
    is_bull = direction == "BUY" or regime in {"BULL", "TREND"}
    is_bear = direction == "SELL" or regime == "BEAR"
    is_side = regime == "SIDEWAYS"
    high_iv = iv_rank >= 55 or iv_hv_ratio >= 1.2
    low_iv = iv_rank <= 25 and iv_hv_ratio <= 1.0
    liquidity_weak = liquidity_score < 0.5
    balanced_skew = abs(skew_pct) <= 0.12
    event_driven = bool(event_near or normalized_thesis in {"directional_explosive", "vol_expansion", "event_driven"})
    theta_friendly = normalized_thesis in {"neutral_income", "vol_contraction", "range_bound"} or is_side

    if theta_friendly and not event_driven and high_iv and not liquidity_weak:
        if normalized_thesis == "neutral_income" and iv_rank >= 75 and balanced_skew and prefer_defined_risk:
            return "iron_butterfly"
        return "iron_condor"

    if is_bull:
        if event_driven:
            return "long_call" if low_iv else "bull_call_debit_spread"
        if high_iv and not liquidity_weak and prefer_defined_risk:
            return "bull_put_credit_spread"
        if low_iv:
            return "long_call" if iv_rank < 20 else "bull_call_debit_spread"
        return "bull_call_debit_spread"

    if is_bear:
        if event_driven:
            return "long_put" if low_iv else "bear_put_debit_spread"
        if high_iv and not liquidity_weak and prefer_defined_risk:
            return "bear_call_credit_spread"
        if low_iv:
            return "long_put" if iv_rank < 20 else "bear_put_debit_spread"
        return "bear_put_debit_spread"

    if theta_friendly and term_structure_slope > 1.05 and not liquidity_weak:
        return "iron_condor"
    return "iron_condor"


def pick_expiration(
    expirations: list[str],
    target_dte: int = 30,
    min_dte: int = 7,
    max_dte: int = 60,
) -> str | None:
    """Pick the expiration closest to target DTE within bounds."""
    from datetime import date, datetime

    today = date.today()
    candidates = []
    for exp in expirations:
        try:
            exp_date = datetime.strptime(exp, "%Y-%m-%d").date()
            dte = (exp_date - today).days
            if min_dte <= dte <= max_dte:
                candidates.append((abs(dte - target_dte), dte, exp))
        except ValueError:
            continue
    if not candidates:
        return None
    candidates.sort()
    return candidates[0][2]


def pick_strike(
    chain_options: list[dict],
    spot: float,
    option_type: str,
    target_delta: float = 0.40,
    min_oi: int = 100,
    min_volume: int = 5,
    max_spread_pct: float = 0.20,
) -> dict | None:
    """Pick the liquid contract closest to target delta."""
    filtered = []
    for opt in chain_options:
        try:
            oi = float(opt.get("open_interest") or 0)
            vol = float(opt.get("volume") or 0)
            bid = float(opt.get("bid") or 0)
            ask = float(opt.get("ask") or 0)
            mid = (bid + ask) / 2
            spread = (ask - bid) / mid if mid > 0 else 1.0
            delta = abs(float(opt.get("greeks", {}).get("delta") or 0))
            otype = (opt.get("option_type") or "").lower()
            if otype != option_type.lower():
                continue
            if oi < min_oi or vol < min_volume or spread > max_spread_pct:
                continue
            filtered.append((abs(delta - target_delta), opt))
        except (TypeError, ValueError):
            continue

    if not filtered:
        return None
    filtered.sort(key=lambda item: item[0])
    return filtered[0][1]


def build_legs_for_strategy(
    strategy_type: str,
    chain_calls: list[dict],
    chain_puts: list[dict],
    spot: float,
    expiration: str,
    width_pct: float = 0.03,
    min_oi: int = 100,
) -> list[dict]:
    """Build strategy legs for supported executable structures."""

    def _leg(opt: dict, side: str) -> dict:
        return {
            "side": side,
            "option_type": opt.get("option_type", ""),
            "strike": float(opt.get("strike") or 0),
            "symbol": opt.get("symbol", ""),
            "expiration": expiration,
            "bid": float(opt.get("bid") or 0),
            "ask": float(opt.get("ask") or 0),
            "mid": (float(opt.get("bid") or 0) + float(opt.get("ask") or 0)) / 2,
            "iv": float((opt.get("greeks") or {}).get("mid_iv") or 0),
            "delta": float((opt.get("greeks") or {}).get("delta") or 0),
            "open_interest": float(opt.get("open_interest") or 0),
        }

    if strategy_type == "long_call":
        call = pick_strike(chain_calls, spot, "call", target_delta=0.40, min_oi=min_oi)
        return [_leg(call, "long")] if call else []

    if strategy_type == "long_put":
        put = pick_strike(chain_puts, spot, "put", target_delta=0.40, min_oi=min_oi)
        return [_leg(put, "long")] if put else []

    if strategy_type == "bull_call_debit_spread":
        long_call = pick_strike(chain_calls, spot, "call", target_delta=0.45, min_oi=min_oi)
        short_call = pick_strike(chain_calls, spot, "call", target_delta=0.25, min_oi=min_oi)
        if long_call and short_call and long_call["symbol"] != short_call["symbol"]:
            return [_leg(long_call, "long"), _leg(short_call, "short")]
        return [_leg(long_call, "long")] if long_call else []

    if strategy_type == "bear_put_debit_spread":
        long_put = pick_strike(chain_puts, spot, "put", target_delta=0.45, min_oi=min_oi)
        short_put = pick_strike(chain_puts, spot, "put", target_delta=0.25, min_oi=min_oi)
        if long_put and short_put and long_put["symbol"] != short_put["symbol"]:
            return [_leg(long_put, "long"), _leg(short_put, "short")]
        return [_leg(long_put, "long")] if long_put else []

    if strategy_type == "bull_put_credit_spread":
        short_put = pick_strike(chain_puts, spot, "put", target_delta=0.30, min_oi=min_oi)
        long_put = pick_strike(chain_puts, spot, "put", target_delta=0.15, min_oi=min_oi)
        if short_put and long_put and short_put["symbol"] != long_put["symbol"]:
            return [_leg(short_put, "short"), _leg(long_put, "long")]
        return []

    if strategy_type == "bear_call_credit_spread":
        short_call = pick_strike(chain_calls, spot, "call", target_delta=0.30, min_oi=min_oi)
        long_call = pick_strike(chain_calls, spot, "call", target_delta=0.15, min_oi=min_oi)
        if short_call and long_call and short_call["symbol"] != long_call["symbol"]:
            return [_leg(short_call, "short"), _leg(long_call, "long")]
        return []

    if strategy_type == "iron_condor":
        short_put = pick_strike(chain_puts, spot, "put", target_delta=0.25, min_oi=min_oi)
        long_put = pick_strike(chain_puts, spot, "put", target_delta=0.12, min_oi=min_oi)
        short_call = pick_strike(chain_calls, spot, "call", target_delta=0.25, min_oi=min_oi)
        long_call = pick_strike(chain_calls, spot, "call", target_delta=0.12, min_oi=min_oi)
        legs = []
        if short_put:
            legs.append(_leg(short_put, "short"))
        if long_put:
            legs.append(_leg(long_put, "long"))
        if short_call:
            legs.append(_leg(short_call, "short"))
        if long_call:
            legs.append(_leg(long_call, "long"))
        return legs if len(legs) == 4 else []

    if strategy_type == "iron_butterfly":
        atm_call = pick_strike(chain_calls, spot, "call", target_delta=0.50, min_oi=min_oi)
        atm_put = pick_strike(chain_puts, spot, "put", target_delta=0.50, min_oi=min_oi)
        wing_call = pick_strike(chain_calls, spot, "call", target_delta=0.15, min_oi=min_oi)
        wing_put = pick_strike(chain_puts, spot, "put", target_delta=0.15, min_oi=min_oi)
        legs = []
        if atm_call:
            legs.append(_leg(atm_call, "short"))
        if atm_put:
            legs.append(_leg(atm_put, "short"))
        if wing_call:
            legs.append(_leg(wing_call, "long"))
        if wing_put:
            legs.append(_leg(wing_put, "long"))
        return legs if len(legs) == 4 else []

    logger.warning("Strategy not implemented in build_legs_for_strategy: %s", strategy_type)
    return []


def select_option_contract(
    symbol: str,
    direction: str,
    spot: float,
    iv_rank: float,
    iv_hv_ratio: float,
    regime: str,
    expirations: list[str],
    chain_by_exp: dict[str, dict],
    strategy_hint: str | None = None,
    min_dte: int = 14,
    max_dte: int = 45,
    target_dte: int = 30,
    min_oi: int = 100,
    min_volume: int = 5,
    max_spread_pct: float = 0.20,
    width_pct: float = 0.03,
    thesis: str | None = None,
    event_near: bool = False,
    liquidity_score: float = 1.0,
    skew_pct: float = 0.0,
    term_structure_slope: float = 1.0,
    prefer_defined_risk: bool = True,
) -> ContractSelection:
    """Full selection: strategy -> expiration -> strikes -> legs."""

    warnings: list[str] = []
    strategy = strategy_hint or pick_strategy(
        direction,
        regime,
        iv_rank,
        iv_hv_ratio,
        thesis=thesis,
        event_near=event_near,
        liquidity_score=liquidity_score,
        skew_pct=skew_pct,
        term_structure_slope=term_structure_slope,
        prefer_defined_risk=prefer_defined_risk,
    )
    governance = describe_strategy_governance(
        direction=direction,
        regime=regime,
        iv_rank=iv_rank,
        iv_hv_ratio=iv_hv_ratio,
        strategy=strategy,
        thesis=thesis,
        event_near=event_near,
        liquidity_score=liquidity_score,
        skew_pct=skew_pct,
        term_structure_slope=term_structure_slope,
    )
    logger.info("[OPT] %s -> strategy=%s iv_rank=%.1f regime=%s", symbol, strategy, iv_rank, regime)

    exp = pick_expiration(expirations, target_dte=target_dte, min_dte=min_dte, max_dte=max_dte)
    if not exp:
        warnings.append(f"No expiration available between {min_dte}-{max_dte} DTE")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    chain = chain_by_exp.get(exp, {})
    chain_calls = chain.get("calls") or chain.get("options", {}).get("option", [])
    chain_puts = chain.get("puts") or []
    if not chain_puts and chain_calls:
        chain_puts = [opt for opt in chain_calls if (opt.get("option_type") or "").lower() == "put"]
        chain_calls = [opt for opt in chain_calls if (opt.get("option_type") or "").lower() == "call"]

    if not chain_calls and not chain_puts:
        warnings.append(f"Empty chain for {symbol} exp={exp}")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    legs = build_legs_for_strategy(strategy, chain_calls, chain_puts, spot, exp, width_pct=width_pct, min_oi=min_oi)
    if not legs:
        warnings.append(f"No liquid contracts found for {strategy}")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    net = sum((leg["mid"] if leg["side"] == "short" else -leg["mid"]) for leg in legs)
    debit = max(0.0, -net)
    credit = max(0.0, net)
    anchor = legs[0].get("strike", spot)
    avg_iv = sum(leg.get("iv", 0) for leg in legs) / max(len(legs), 1)
    score = min(1.0, (iv_rank / 100) * 0.4 + (1 - max(0, avg_iv - 0.3)) * 0.3 + 0.3)
    option_symbol = legs[0].get("symbol", "") if len(legs) == 1 else ""

    from datetime import date, datetime

    dte = (datetime.strptime(exp, "%Y-%m-%d").date() - date.today()).days

    return ContractSelection(
        strategy_type=strategy,
        legs=legs,
        option_symbol=option_symbol,
        estimated_debit=round(debit, 4),
        estimated_credit=round(credit, 4),
        selected_dte=dte,
        anchor_strike=anchor,
        width_pct=width_pct,
        iv_rank_at_selection=iv_rank,
        selection_score=round(score, 3),
        governance=governance,
        warnings=warnings,
        is_valid=True,
    )
