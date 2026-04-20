"""Option contract selector with governance-aware strategy choice.

Integración Fase A: ``iv_rank`` / ``iv_hv_ratio`` deben alimentarse desde datos reales;
ver ``options.iv_rank_calculator.IVRankCalculator`` (cadena Tradier + histórico precios).

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
    aliases = {
        "calendar": "time_spread",
        "calendar_spread": "time_spread",
        "diagonal": "time_spread",
        "diagonal_spread": "time_spread",
        "time_spread": "time_spread",
        "theta_carry": "time_spread",
        "term_structure": "time_spread",
        "neutral_theta": "neutral_income",
        "income": "neutral_income",
        "range": "range_bound",
        "range_trade": "range_bound",
        "hedge": "hedged_overlay",
        "hedged": "hedged_overlay",
        "protective": "hedged_overlay",
        "protective_put": "hedged_overlay",
        "covered_call": "hedged_overlay",
        "collar": "hedged_overlay",
    }
    if raw:
        return aliases.get(raw, raw)
    if (regime or "").upper() == "SIDEWAYS":
        return "range_bound"
    if (direction or "").upper() in {"BUY", "SELL"}:
        return "directional_controlled"
    return "directional_controlled"


_TIME_SPREAD_STRATEGIES = {
    "call_calendar_spread",
    "put_calendar_spread",
    "call_diagonal_debit_spread",
    "put_diagonal_debit_spread",
}

# Estrategias donde la policy de expected move (bandas alrededor del spot) aplica a patas cortas.
_EM_GATE_STRATEGIES = frozenset(
    {"iron_condor", "iron_butterfly", "bull_put_credit_spread", "bear_call_credit_spread"}
)


def normalize_gamma_regime(value: str | None) -> str | None:
    """Normaliza la etiqueta de régimen gamma (viene del briefing/contexto; no se calcula aquí).

    Valores canónicos: ``long_gamma``, ``short_gamma``, ``flip_zone``, ``unknown``.
    Cualquier otro valor se mapea a ``unknown`` para mantener decisiones conservadoras.
    """
    if value is None:
        return None
    raw = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "longgamma": "long_gamma",
        "shortgamma": "short_gamma",
        "flipzone": "flip_zone",
        "flip": "flip_zone",
    }
    raw = aliases.get(raw, raw)
    if raw in {"long_gamma", "short_gamma", "flip_zone", "unknown"}:
        return raw
    return "unknown"


def normalize_dte_mode(value: str | None) -> str | None:
    """Normaliza el modo de horizonte (viene del briefing/contexto; no se calcula aquí).

    Taxonomía: ``0dte`` | ``1to7`` | ``8to21`` | ``22plus``. Otros valores → ``unknown``.
    """
    if value is None:
        return None
    raw = str(value).strip().lower().replace(" ", "")
    aliases = {
        "0-dte": "0dte",
        "odte": "0dte",
        "0": "0dte",
        "1-7": "1to7",
        "8-21": "8to21",
        "22+": "22plus",
        "22_plus": "22plus",
    }
    norm = aliases.get(raw, raw)
    if norm in {"0dte", "1to7", "8to21", "22plus"}:
        return norm
    return "unknown"


def evaluate_expected_move_policy(
    strategy_type: str,
    legs: list[dict],
    spot: float,
    expected_move: float | None,
) -> dict:
    """Evalúa si las patas **cortas** de estructuras de crédito quedan fuera de la banda ``spot ± expected_move``.

    ``expected_move`` debe estar en las mismas unidades que ``spot`` y strikes (p. ej. dólares en el subyacente).
    Si no aplica (estrategia sin gate EM o ``expected_move`` ausente), ``em_policy_ok`` es ``None``.
    """
    result: dict = {
        "em_policy_ok": None,
        "em_expected_move": expected_move,
        "em_short_strikes": [],
        "em_notes": [],
    }
    if expected_move is None:
        return result
    try:
        em = float(expected_move)
    except (TypeError, ValueError):
        result["em_notes"].append("expected_move_unparseable")
        return result
    if em <= 0 or spot <= 0:
        result["em_notes"].append("expected_move_or_spot_invalid")
        return result
    if strategy_type not in _EM_GATE_STRATEGIES or not legs:
        result["em_notes"].append("em_not_applicable_to_strategy")
        return result

    lower = spot - em
    upper = spot + em
    shorts = [leg for leg in legs if leg.get("side") == "short"]
    ok = True
    for leg in shorts:
        strike = float(leg.get("strike") or 0.0)
        otype = str(leg.get("option_type") or "").lower()
        if otype == "call":
            outside = strike > upper
            if not outside:
                ok = False
            result["em_short_strikes"].append({"option_type": "call", "strike": strike, "em_band_ok": outside})
        elif otype == "put":
            outside = strike < lower
            if not outside:
                ok = False
            result["em_short_strikes"].append({"option_type": "put", "strike": strike, "em_band_ok": outside})
    result["em_policy_ok"] = ok
    result["em_notes"].append(
        "short_strikes_outside_expected_move_band" if ok else "short_strike_inside_expected_move_band"
    )
    return result


def _zero_dte_replace_time_spread(
    chosen: str,
    *,
    direction: str,
    regime: str,
    iv_rank: float,
    iv_hv_ratio: float,
    thesis: str | None,
    event_near: bool,
    liquidity_score: float,
    prefer_defined_risk: bool,
) -> str:
    """Matriz 0DTE: sin calendarios/diagonales multi-expiración → verticales / condor / singles."""
    if chosen not in _TIME_SPREAD_STRATEGIES:
        return chosen
    direction = direction.upper()
    regime = regime.upper() if regime else "SIDEWAYS"
    normalized_thesis = _normalize_thesis(thesis, regime, direction)
    is_bull = direction == "BUY" or regime in {"BULL", "TREND"}
    is_bear = direction == "SELL" or regime == "BEAR"
    is_side = regime == "SIDEWAYS"
    high_iv = iv_rank >= 55 or iv_hv_ratio >= 1.2
    low_iv = iv_rank <= 25 and iv_hv_ratio <= 1.0
    liquidity_weak = liquidity_score < 0.5
    event_driven = bool(
        event_near or normalized_thesis in {"directional_explosive", "vol_expansion", "event_driven"}
    )
    if is_side:
        return "iron_condor"
    if is_bull:
        if event_driven:
            return "long_call" if low_iv else "bull_call_debit_spread"
        if high_iv and not liquidity_weak and prefer_defined_risk:
            return "bull_put_credit_spread"
        return "bull_call_debit_spread"
    if is_bear:
        if event_driven:
            return "long_put" if low_iv else "bear_put_debit_spread"
        if high_iv and not liquidity_weak and prefer_defined_risk:
            return "bear_call_credit_spread"
        return "bear_put_debit_spread"
    return "iron_condor"


def _apply_gamma_regime_overlay(
    chosen: str,
    *,
    direction: str,
    regime: str,
    iv_rank: float,
    iv_hv_ratio: float,
    liquidity_score: float,
    prefer_defined_risk: bool,
    gamma_regime: str | None,
) -> str:
    """Ajustes por ``gamma_regime`` (después de 0DTE)."""
    gm = normalize_gamma_regime(gamma_regime)
    if gm is None or gm == "unknown":
        return chosen
    direction = direction.upper()
    regime = regime.upper() if regime else "SIDEWAYS"
    is_bull = direction == "BUY" or regime in {"BULL", "TREND"}
    is_bear = direction == "SELL" or regime == "BEAR"
    high_iv = iv_rank >= 55 or iv_hv_ratio >= 1.2
    liquidity_weak = liquidity_score < 0.5
    out = chosen

    if gm == "long_gamma":
        # Dealer long gamma: favorecer venta de prima acotada / theta neutra en índices cuando el IV lo permite.
        if high_iv and prefer_defined_risk and not liquidity_weak:
            if is_bull and out in {"bull_call_debit_spread", "long_call"}:
                out = "bull_put_credit_spread"
            elif is_bear and out in {"bear_put_debit_spread", "long_put"}:
                out = "bear_call_credit_spread"
    elif gm == "short_gamma":
        # Dealer short gamma: castigar créditos neutros amplios; favorecer débito direccional.
        if out in {"iron_condor", "iron_butterfly"}:
            out = "bull_call_debit_spread" if is_bull else "bear_put_debit_spread"
        elif out in {"bull_put_credit_spread", "bear_call_credit_spread"}:
            out = "bull_call_debit_spread" if is_bull else "bear_put_debit_spread"
    elif gm == "flip_zone":
        # Zona de pin / flip: estructuras más simples que la mariposa completa.
        if out == "iron_butterfly":
            out = "iron_condor"
    return out


def _preferred_but_unavailable_strategy(thesis: str, direction: str) -> str | None:
    if thesis == "hedged_overlay":
        return "protective_put" if (direction or "").upper() == "BUY" else "collar"
    return None


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
    gamma_regime: str | None = None,
    expected_move: float | None = None,
    dte_mode: str | None = None,
    em_policy_ok: bool | None = None,
) -> dict:
    normalized_thesis = _normalize_thesis(thesis, regime, direction)
    premium_stance = "neutral"
    if strategy in {"bull_put_credit_spread", "bear_call_credit_spread", "iron_condor", "iron_butterfly"}:
        premium_stance = "sell_premium_defined_risk"
    elif strategy in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        premium_stance = "debit_defined_risk"
    elif strategy in {"long_call", "long_put"}:
        premium_stance = "long_premium"
    elif strategy in _TIME_SPREAD_STRATEGIES:
        premium_stance = "time_spread_defined_risk"

    if strategy in {"iron_condor", "iron_butterfly"}:
        strategy_family = "neutral_theta"
    elif strategy in {"bull_put_credit_spread", "bear_call_credit_spread"}:
        strategy_family = "directional_credit"
    elif strategy in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        strategy_family = "directional_debit"
    elif strategy in _TIME_SPREAD_STRATEGIES:
        strategy_family = "term_structure_time_spread"
    else:
        strategy_family = "directional_long_premium"

    volatility_posture = "balanced"
    if premium_stance == "sell_premium_defined_risk":
        volatility_posture = "short_volatility_defined_risk"
    elif premium_stance == "long_premium":
        volatility_posture = "long_volatility"
    elif premium_stance == "time_spread_defined_risk":
        volatility_posture = "long_back_vega_short_front_theta"

    gm = normalize_gamma_regime(gamma_regime)
    dm = normalize_dte_mode(dte_mode)
    return {
        "thesis": normalized_thesis,
        "event_near": bool(event_near),
        "liquidity_score": round(max(0.0, min(liquidity_score, 1.0)), 3),
        "skew_pct": round(skew_pct, 4),
        "term_structure_slope": round(term_structure_slope, 4),
        "gamma_regime": gm,
        "expected_move": expected_move,
        "dte_mode": dm,
        "em_policy_ok": em_policy_ok,
        "premium_stance": premium_stance,
        "volatility_posture": volatility_posture,
        "strategy_family": strategy_family,
        "preferred_but_unavailable_strategy": _preferred_but_unavailable_strategy(normalized_thesis, direction),
        "benchmark_framework": {
            "iv_drives_credit_vs_debit": True,
            "term_structure_drives_time_spreads": True,
            "hedging_requires_stock_context": normalized_thesis == "hedged_overlay",
        },
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


def _pick_strategy_core(
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
    """Selección base (sin overlays gamma / DTE)."""

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
    term_structure_friendly = term_structure_slope >= 1.03 and not liquidity_weak and not event_driven

    if normalized_thesis == "time_spread" and term_structure_friendly:
        if is_side:
            return "call_calendar_spread" if direction != "SELL" else "put_calendar_spread"
        if is_bull:
            return "call_diagonal_debit_spread"
        if is_bear:
            return "put_diagonal_debit_spread"
        return "call_calendar_spread" if direction != "SELL" else "put_calendar_spread"

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

    if theta_friendly and term_structure_friendly and not high_iv:
        return "call_calendar_spread" if direction != "SELL" else "put_calendar_spread"
    return "iron_condor"


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
    gamma_regime: str | None = None,
    dte_mode: str | None = None,
) -> str:
    """Elige una estructura ejecutable.

    ``gamma_regime`` y ``dte_mode`` son etiquetas del contexto (briefing); no se infieren aquí.

    Matriz auditable (resumen):
    - **dte_mode == \"0dte\"**: sustituye calendarios/diagonales por condor/verticales/singles
      (sin expiraciones múltiples en la misma estructura lógica).
    - **gamma_regime**: ``long_gamma`` favorece créditos definidos con IV alto;
      ``short_gamma`` desplaza desde theta neutro hacia débito direccional;
      ``flip_zone`` simplifica (p. ej. mariposa → condor).
    """
    chosen = _pick_strategy_core(
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
    dm = normalize_dte_mode(dte_mode)
    if dm == "0dte":
        chosen = _zero_dte_replace_time_spread(
            chosen,
            direction=direction,
            regime=regime,
            iv_rank=iv_rank,
            iv_hv_ratio=iv_hv_ratio,
            thesis=thesis,
            event_near=event_near,
            liquidity_score=liquidity_score,
            prefer_defined_risk=prefer_defined_risk,
        )
    chosen = _apply_gamma_regime_overlay(
        chosen,
        direction=direction,
        regime=regime,
        iv_rank=iv_rank,
        iv_hv_ratio=iv_hv_ratio,
        liquidity_score=liquidity_score,
        prefer_defined_risk=prefer_defined_risk,
        gamma_regime=gamma_regime,
    )
    return chosen


def pick_expiration_pair(
    expirations: list[str],
    *,
    front_target_dte: int = 21,
    back_target_dte: int = 45,
    min_front_dte: int = 7,
    max_back_dte: int = 90,
    min_gap_dte: int = 7,
) -> tuple[str | None, str | None]:
    from datetime import date, datetime

    today = date.today()
    dtes: list[tuple[int, str]] = []
    for exp in expirations:
        try:
            exp_date = datetime.strptime(exp, "%Y-%m-%d").date()
        except ValueError:
            continue
        dte = (exp_date - today).days
        if dte >= min_front_dte:
            dtes.append((dte, exp))
    if not dtes:
        return None, None

    front_candidates = [(abs(dte - front_target_dte), dte, exp) for dte, exp in dtes]
    front_candidates.sort()
    for _, front_dte, front_exp in front_candidates:
        back_candidates = [
            (abs(dte - back_target_dte), dte, exp)
            for dte, exp in dtes
            if dte >= max(front_dte + min_gap_dte, back_target_dte - (back_target_dte - front_target_dte))
            and dte <= max_back_dte
        ]
        if back_candidates:
            back_candidates.sort()
            return front_exp, back_candidates[0][2]
    return None, None


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
    back_expiration: str | None = None,
    back_chain_calls: list[dict] | None = None,
    back_chain_puts: list[dict] | None = None,
    width_pct: float = 0.03,
    min_oi: int = 100,
    *,
    expected_move: float | None = None,
    em_metadata: dict | None = None,
) -> list[dict]:
    """Build strategy legs for supported executable structures.

    Si ``em_metadata`` es un ``dict``, se rellena con el resultado de
    :func:`evaluate_expected_move_policy` (gate de strikes vs ``spot ± expected_move`` en créditos).
    """

    def _finalize_legs(legs: list[dict]) -> list[dict]:
        if em_metadata is not None:
            em_metadata.clear()
            em_metadata.update(evaluate_expected_move_policy(strategy_type, legs, spot, expected_move))
        return legs

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

    def _pick_same_or_closest_strike(options: list[dict], *, strike: float, option_type: str, min_oi: int) -> dict | None:
        candidates: list[tuple[float, dict]] = []
        for opt in options:
            try:
                if str(opt.get("option_type") or "").lower() != option_type.lower():
                    continue
                if float(opt.get("open_interest") or 0) < min_oi:
                    continue
                candidates.append((abs(float(opt.get("strike") or 0) - strike), opt))
            except (TypeError, ValueError):
                continue
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def _shared_center_pair(*, min_oi_required: int) -> tuple[dict | None, dict | None, float | None]:
        call_by_strike: dict[float, dict] = {}
        put_by_strike: dict[float, dict] = {}
        for opt in chain_calls:
            try:
                if str(opt.get("option_type") or "").lower() != "call":
                    continue
                if float(opt.get("open_interest") or 0) < min_oi_required:
                    continue
                call_by_strike[float(opt.get("strike") or 0.0)] = opt
            except (TypeError, ValueError):
                continue
        for opt in chain_puts:
            try:
                if str(opt.get("option_type") or "").lower() != "put":
                    continue
                if float(opt.get("open_interest") or 0) < min_oi_required:
                    continue
                put_by_strike[float(opt.get("strike") or 0.0)] = opt
            except (TypeError, ValueError):
                continue
        shared = sorted(set(call_by_strike) & set(put_by_strike), key=lambda strike: abs(strike - spot))
        if not shared:
            return None, None, None
        center = shared[0]
        return call_by_strike.get(center), put_by_strike.get(center), center

    def _symmetric_wings(*, center_strike: float, min_oi_required: int) -> tuple[dict | None, dict | None]:
        call_candidates = []
        put_candidates = []
        for opt in chain_calls:
            try:
                strike = float(opt.get("strike") or 0.0)
                if str(opt.get("option_type") or "").lower() != "call" or strike <= center_strike:
                    continue
                if float(opt.get("open_interest") or 0) < min_oi_required:
                    continue
                call_candidates.append((strike, opt))
            except (TypeError, ValueError):
                continue
        for opt in chain_puts:
            try:
                strike = float(opt.get("strike") or 0.0)
                if str(opt.get("option_type") or "").lower() != "put" or strike >= center_strike:
                    continue
                if float(opt.get("open_interest") or 0) < min_oi_required:
                    continue
                put_candidates.append((strike, opt))
            except (TypeError, ValueError):
                continue

        if not call_candidates or not put_candidates:
            return None, None

        provisional_call = pick_strike(
            [opt for _, opt in call_candidates],
            spot,
            "call",
            target_delta=0.15,
            min_oi=min_oi_required,
        )
        provisional_put = pick_strike(
            [opt for _, opt in put_candidates],
            spot,
            "put",
            target_delta=0.15,
            min_oi=min_oi_required,
        )
        target_distance = max(spot * max(width_pct, 0.005), 1.0)
        if provisional_call is not None:
            target_distance = abs(float(provisional_call.get("strike") or center_strike) - center_strike)
        if provisional_put is not None:
            put_distance = abs(center_strike - float(provisional_put.get("strike") or center_strike))
            target_distance = (target_distance + put_distance) / 2.0 if target_distance > 0 else put_distance

        call_by_distance = {
            round(strike - center_strike, 6): opt
            for strike, opt in call_candidates
        }
        put_by_distance = {
            round(center_strike - strike, 6): opt
            for strike, opt in put_candidates
        }
        common_distances = sorted(
            set(call_by_distance) & set(put_by_distance),
            key=lambda distance: abs(distance - target_distance),
        )
        if common_distances:
            chosen = common_distances[0]
            return call_by_distance[chosen], put_by_distance[chosen]

        nearest_call = min(call_candidates, key=lambda item: abs((item[0] - center_strike) - target_distance))[1]
        nearest_put = min(put_candidates, key=lambda item: abs((center_strike - item[0]) - target_distance))[1]
        return nearest_call, nearest_put

    if strategy_type == "long_call":
        call = pick_strike(chain_calls, spot, "call", target_delta=0.40, min_oi=min_oi)
        return _finalize_legs([_leg(call, "long")]) if call else _finalize_legs([])

    if strategy_type == "long_put":
        put = pick_strike(chain_puts, spot, "put", target_delta=0.40, min_oi=min_oi)
        return _finalize_legs([_leg(put, "long")]) if put else _finalize_legs([])

    if strategy_type == "bull_call_debit_spread":
        long_call = pick_strike(chain_calls, spot, "call", target_delta=0.45, min_oi=min_oi)
        short_call = pick_strike(chain_calls, spot, "call", target_delta=0.25, min_oi=min_oi)
        if long_call and short_call and long_call["symbol"] != short_call["symbol"]:
            return _finalize_legs([_leg(long_call, "long"), _leg(short_call, "short")])
        return _finalize_legs([_leg(long_call, "long")]) if long_call else _finalize_legs([])

    if strategy_type == "bear_put_debit_spread":
        long_put = pick_strike(chain_puts, spot, "put", target_delta=0.45, min_oi=min_oi)
        short_put = pick_strike(chain_puts, spot, "put", target_delta=0.25, min_oi=min_oi)
        if long_put and short_put and long_put["symbol"] != short_put["symbol"]:
            return _finalize_legs([_leg(long_put, "long"), _leg(short_put, "short")])
        return _finalize_legs([_leg(long_put, "long")]) if long_put else _finalize_legs([])

    if strategy_type == "bull_put_credit_spread":
        short_put = pick_strike(chain_puts, spot, "put", target_delta=0.30, min_oi=min_oi)
        long_put = pick_strike(chain_puts, spot, "put", target_delta=0.15, min_oi=min_oi)
        if short_put and long_put and short_put["symbol"] != long_put["symbol"]:
            return _finalize_legs([_leg(short_put, "short"), _leg(long_put, "long")])
        return _finalize_legs([])

    if strategy_type == "bear_call_credit_spread":
        short_call = pick_strike(chain_calls, spot, "call", target_delta=0.30, min_oi=min_oi)
        long_call = pick_strike(chain_calls, spot, "call", target_delta=0.15, min_oi=min_oi)
        if short_call and long_call and short_call["symbol"] != long_call["symbol"]:
            return _finalize_legs([_leg(short_call, "short"), _leg(long_call, "long")])
        return _finalize_legs([])

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
        return _finalize_legs(legs) if len(legs) == 4 else _finalize_legs([])

    if strategy_type == "iron_butterfly":
        atm_call, atm_put, center_strike = _shared_center_pair(min_oi_required=min_oi)
        if center_strike is None or atm_call is None or atm_put is None:
            return _finalize_legs([])
        wing_call, wing_put = _symmetric_wings(center_strike=center_strike, min_oi_required=min_oi)
        legs = []
        if atm_call:
            legs.append(_leg(atm_call, "short"))
        if atm_put:
            legs.append(_leg(atm_put, "short"))
        if wing_call:
            legs.append(_leg(wing_call, "long"))
        if wing_put:
            legs.append(_leg(wing_put, "long"))
        return _finalize_legs(legs) if len(legs) == 4 else _finalize_legs([])

    if strategy_type == "call_calendar_spread":
        front_call = pick_strike(chain_calls, spot, "call", target_delta=0.50, min_oi=min_oi)
        back_calls = back_chain_calls or []
        if front_call and back_calls and back_expiration:
            back_call = _pick_same_or_closest_strike(
                back_calls,
                strike=float(front_call.get("strike") or spot),
                option_type="call",
                min_oi=min_oi,
            )
            if back_call:
                return _finalize_legs(
                    [
                        _leg(front_call, "short"),
                        {**_leg(back_call, "long"), "expiration": back_expiration},
                    ]
                )
        return _finalize_legs([])

    if strategy_type == "put_calendar_spread":
        front_put = pick_strike(chain_puts, spot, "put", target_delta=0.50, min_oi=min_oi)
        back_puts = back_chain_puts or []
        if front_put and back_puts and back_expiration:
            back_put = _pick_same_or_closest_strike(
                back_puts,
                strike=float(front_put.get("strike") or spot),
                option_type="put",
                min_oi=min_oi,
            )
            if back_put:
                return _finalize_legs(
                    [
                        _leg(front_put, "short"),
                        {**_leg(back_put, "long"), "expiration": back_expiration},
                    ]
                )
        return _finalize_legs([])

    if strategy_type == "call_diagonal_debit_spread":
        front_short = pick_strike(chain_calls, spot, "call", target_delta=0.30, min_oi=min_oi)
        back_calls = back_chain_calls or []
        if front_short and back_calls and back_expiration:
            eligible = [
                opt for opt in back_calls
                if str(opt.get("option_type") or "").lower() == "call"
                and float(opt.get("open_interest") or 0) >= min_oi
                and float(opt.get("strike") or 0) <= float(front_short.get("strike") or spot)
            ]
            back_long = pick_strike(eligible or back_calls, spot, "call", target_delta=0.55, min_oi=min_oi)
            if back_long:
                return _finalize_legs(
                    [
                        _leg(front_short, "short"),
                        {**_leg(back_long, "long"), "expiration": back_expiration},
                    ]
                )
        return _finalize_legs([])

    if strategy_type == "put_diagonal_debit_spread":
        front_short = pick_strike(chain_puts, spot, "put", target_delta=0.30, min_oi=min_oi)
        back_puts = back_chain_puts or []
        if front_short and back_puts and back_expiration:
            eligible = [
                opt for opt in back_puts
                if str(opt.get("option_type") or "").lower() == "put"
                and float(opt.get("open_interest") or 0) >= min_oi
                and float(opt.get("strike") or 0) >= float(front_short.get("strike") or spot)
            ]
            back_long = pick_strike(eligible or back_puts, spot, "put", target_delta=0.55, min_oi=min_oi)
            if back_long:
                return _finalize_legs(
                    [
                        _leg(front_short, "short"),
                        {**_leg(back_long, "long"), "expiration": back_expiration},
                    ]
                )
        return _finalize_legs([])

    logger.warning("Strategy not implemented in build_legs_for_strategy: %s", strategy_type)
    return _finalize_legs([])


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
    *,
    gamma_regime: str | None = None,
    expected_move: float | None = None,
    dte_mode: str | None = None,
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
        gamma_regime=gamma_regime,
        dte_mode=dte_mode,
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
        gamma_regime=gamma_regime,
        expected_move=expected_move,
        dte_mode=dte_mode,
        em_policy_ok=None,
    )
    logger.info("[OPT] %s -> strategy=%s iv_rank=%.1f regime=%s", symbol, strategy, iv_rank, regime)

    exp = pick_expiration(expirations, target_dte=target_dte, min_dte=min_dte, max_dte=max_dte)
    front_exp = back_exp = None
    if strategy in _TIME_SPREAD_STRATEGIES:
        front_exp, back_exp = pick_expiration_pair(
            expirations,
            front_target_dte=max(min_dte, max(7, min(target_dte, 30) - 10)),
            back_target_dte=max(target_dte, 35),
            min_front_dte=max(7, min_dte // 2),
            max_back_dte=max(max_dte + 30, target_dte + 15),
            min_gap_dte=7,
        )
        if not front_exp or not back_exp:
            warnings.append("No expiration pair available for calendar/diagonal structure.")
            return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)
        exp = front_exp
    if not exp:
        warnings.append(f"No expiration available between {min_dte}-{max_dte} DTE")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    def _chain_side(expiration: str) -> tuple[list[dict], list[dict]]:
        chain = chain_by_exp.get(expiration, {})
        calls = chain.get("calls") or chain.get("options", {}).get("option", [])
        puts = chain.get("puts") or []
        if not puts and calls:
            puts = [opt for opt in calls if (opt.get("option_type") or "").lower() == "put"]
            calls = [opt for opt in calls if (opt.get("option_type") or "").lower() == "call"]
        return calls, puts

    chain_calls, chain_puts = _chain_side(exp)
    if not chain_calls and not chain_puts:
        warnings.append(f"Empty chain for {symbol} exp={exp}")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    back_chain_calls: list[dict] = []
    back_chain_puts: list[dict] = []
    if back_exp:
        back_chain_calls, back_chain_puts = _chain_side(back_exp)
        if not back_chain_calls and not back_chain_puts:
            warnings.append(f"Empty back chain for {symbol} exp={back_exp}")
            return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    em_meta: dict = {}
    legs = build_legs_for_strategy(
        strategy,
        chain_calls,
        chain_puts,
        spot,
        exp,
        back_expiration=back_exp,
        back_chain_calls=back_chain_calls,
        back_chain_puts=back_chain_puts,
        width_pct=width_pct,
        min_oi=min_oi,
        expected_move=expected_move,
        em_metadata=em_meta,
    )
    if em_meta:
        governance = {
            **governance,
            **{k: em_meta[k] for k in em_meta if str(k).startswith("em_")},
        }
    if not legs:
        warnings.append(f"No liquid contracts found for {strategy}")
        return ContractSelection(strategy_type=strategy, legs=[], governance=governance, warnings=warnings, is_valid=False)

    net = sum((leg["mid"] if leg["side"] == "short" else -leg["mid"]) for leg in legs)
    debit = max(0.0, -net)
    credit = max(0.0, net)
    anchor = legs[0].get("strike", spot)
    avg_iv = sum(leg.get("iv", 0) for leg in legs) / max(len(legs), 1)
    liquidity_component = max(0.0, min(liquidity_score, 1.0))
    event_penalty = 0.12 if event_near and strategy in {"iron_condor", "iron_butterfly", "bull_put_credit_spread", "bear_call_credit_spread"} else 0.0
    term_structure_bonus = 0.10 if strategy in _TIME_SPREAD_STRATEGIES and term_structure_slope >= 1.03 else 0.0
    strategy_fit = 0.55
    if strategy in {"bull_put_credit_spread", "bear_call_credit_spread", "iron_condor", "iron_butterfly"}:
        strategy_fit = 0.85 if (iv_rank >= 55 or iv_hv_ratio >= 1.2) else 0.45
    elif strategy in {"long_call", "long_put", "bull_call_debit_spread", "bear_put_debit_spread"}:
        strategy_fit = 0.85 if (iv_rank <= 35 or iv_hv_ratio <= 1.05 or event_near) else 0.60
    elif strategy in _TIME_SPREAD_STRATEGIES:
        strategy_fit = 0.85 if term_structure_slope >= 1.03 and not event_near else 0.55
    score = max(
        0.0,
        min(
            1.0,
            (strategy_fit * 0.40)
            + (liquidity_component * 0.20)
            + (max(0.0, 1 - max(0, avg_iv - 0.35)) * 0.15)
            + (min(iv_rank / 100, 1.0) * 0.10)
            + (term_structure_bonus)
            + 0.15
            - event_penalty,
        ),
    )
    option_symbol = legs[0].get("symbol", "") if len(legs) == 1 else ""

    from datetime import date, datetime

    dte = (datetime.strptime(back_exp or exp, "%Y-%m-%d").date() - date.today()).days
    if back_exp:
        governance = {
            **governance,
            "front_expiration": front_exp,
            "back_expiration": back_exp,
            "time_spread_gap_dte": (
                (datetime.strptime(back_exp, "%Y-%m-%d").date() - datetime.strptime(front_exp, "%Y-%m-%d").date()).days
                if front_exp
                else None
            ),
        }

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
