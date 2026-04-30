"""Briefing de sesión (Fase A): JSON programático para guiar selección de estrategia (paper).

No genera PDF ni narrativa de terminal; solo métricas y recomendación vía ``pick_strategy``.
"""
from __future__ import annotations

import logging
import math
from datetime import date
from typing import Any, Callable, Protocol

logger = logging.getLogger("atlas.options.session_briefing")

try:
    from execution.option_selector import (
        normalize_dte_mode,
        normalize_gamma_regime,
        pick_strategy,
    )
except ModuleNotFoundError:  # pragma: no cover
    from atlas_code_quant.execution.option_selector import (
        normalize_dte_mode,
        normalize_gamma_regime,
        pick_strategy,
    )

_TIME_SPREAD_NAMES = frozenset(
    {
        "call_calendar_spread",
        "put_calendar_spread",
        "call_diagonal_debit_spread",
        "put_diagonal_debit_spread",
    }
)


class _SupportsIVRank(Protocol):
    def get_iv_rank(self, symbol: str, dte_target: int | None = None) -> dict[str, Any]: ...


def _effective_dte_days_for_mode(dte_mode: str | None) -> int:
    """Días de calendario representativos para escalar el movimiento esperado (documentación / EM)."""
    if dte_mode == "0dte":
        return 1
    if dte_mode == "1to7":
        return 4
    if dte_mode == "8to21":
        return 14
    if dte_mode == "22plus":
        return 30
    return 14


def derive_dte_mode_from_days_to_event(days: int | None) -> str | None:
    """Deriva taxonomía ``0dte`` / ``1to7`` / ``8to21`` / ``22plus`` desde días hasta evento o expiración objetivo.

    - ``days <= 0`` → ``0dte``
    - ``1..7`` → ``1to7``
    - ``8..21`` → ``8to21``
    - ``22+`` → ``22plus``
    """
    if days is None:
        return None
    if days <= 0:
        return "0dte"
    if days <= 7:
        return "1to7"
    if days <= 21:
        return "8to21"
    return "22plus"


def _dte_target_for_iv_chain(dte_mode: str | None) -> int | None:
    """DTE objetivo para alinear cadena de IV con el horizonte del briefing."""
    if dte_mode == "0dte":
        return 1
    if dte_mode == "1to7":
        return 5
    if dte_mode == "8to21":
        return 14
    if dte_mode == "22plus":
        return 30
    return 21


def _normalize_selector_direction_regime(direction: str, regime: str) -> tuple[str, str]:
    """Mapea etiquetas de briefing a ``pick_strategy`` (BUY/SELL + BULL/BEAR/SIDEWAYS/TREND)."""
    d = (direction or "").strip().lower()
    r_raw = (regime or "").strip().lower()
    regime_aliases = {
        "ranging": "SIDEWAYS",
        "range": "SIDEWAYS",
        "sideways": "SIDEWAYS",
        "choppy": "SIDEWAYS",
        "bull": "BULL",
        "bullish": "BULL",
        "bear": "BEAR",
        "bearish": "BEAR",
        "neutral": "SIDEWAYS",
        "trend": "TREND",
    }
    regime_u = regime_aliases.get(r_raw, (regime or "SIDEWAYS").upper())
    if regime_u not in {"BULL", "BEAR", "SIDEWAYS", "TREND"}:
        regime_u = "SIDEWAYS"

    if d in {"bull", "bullish", "long", "buy", "alcista"}:
        return "BUY", "BULL" if regime_u == "SIDEWAYS" else regime_u
    if d in {"bear", "bearish", "short", "sell", "bajista"}:
        return "SELL", "BEAR" if regime_u == "SIDEWAYS" else regime_u
    if d in {"neutral", "flat", "sideways"}:
        return "BUY", "SIDEWAYS"
    # default: directional hint from regime
    if regime_u == "BEAR":
        return "SELL", "BEAR"
    if regime_u == "BULL":
        return "BUY", "BULL"
    return "BUY", regime_u


def strategy_to_recommended_family(strategy: str) -> str:
    """Familia macro para UI/paper (no coincide 1:1 con governance interno)."""
    if strategy in {"iron_condor", "iron_butterfly"}:
        return "credit_neutral"
    if strategy in {"bull_put_credit_spread", "bear_call_credit_spread"}:
        return "credit_directional"
    if strategy in {"bull_call_debit_spread", "bear_put_debit_spread"}:
        return "debit_directional"
    if strategy in {"long_call", "long_put"}:
        return "long_premium"
    if strategy in _TIME_SPREAD_NAMES:
        return "time_structure"
    return "other"


def _expected_move_sigma(
    spot: float,
    iv_annual: float,
    *,
    horizon_trading_days: int,
) -> tuple[float, dict[str, Any]]:
    """1σ movimiento en precio: ``spot * iv * sqrt(horizon/252)``.

    Para 1 día de trading: ``spot * iv / sqrt(252)`` equivale a ``horizon_trading_days=1``.
    """
    h = max(1, int(horizon_trading_days))
    em = float(spot) * float(iv_annual) * math.sqrt(h / 252.0)
    meta = {
        "method": "sigma_price_spot_iv_sqrt_horizon_over_252",
        "horizon_trading_days": h,
        "formula": "expected_move = spot * iv_annual * sqrt(horizon_trading_days / 252)",
    }
    return em, meta


def _normalize_levels(levels: list[float] | None) -> list[float]:
    if not levels:
        return []
    out: list[float] = []
    for v in levels:
        try:
            fv = float(v)
        except (TypeError, ValueError):
            continue
        if fv > 0:
            out.append(fv)
    out = sorted(set(out))
    return out


def _market_bias(direction: str, regime: str, gamma_regime: str, dte_mode: str, event_risk_flag: bool) -> str:
    d = (direction or "").strip().lower()
    r = (regime or "").strip().lower()
    gm = (gamma_regime or "unknown").strip().lower()
    dm = (dte_mode or "").strip().lower()

    base = "neutral"
    if d in {"bull", "bullish", "buy", "alcista", "long"}:
        base = "bullish"
    elif d in {"bear", "bearish", "sell", "bajista", "short"}:
        base = "bearish"
    elif r in {"bull", "bullish"}:
        base = "bullish"
    elif r in {"bear", "bearish"}:
        base = "bearish"
    elif r in {"ranging", "range", "sideways", "neutral"}:
        base = "neutral"

    if base == "neutral" and gm == "long_gamma":
        base = "neutral_mean_reversion"
    if base in {"bullish", "bearish"} and gm == "flip_zone":
        base = "neutral_to_bullish" if base == "bullish" else "neutral_to_bearish"
    if event_risk_flag and dm == "0dte":
        base = f"{base}_event_risk_0dte"
    return base


def _risk_posture(gamma_regime: str, dte_mode: str, event_risk_flag: bool) -> str:
    gm = (gamma_regime or "unknown").strip().lower()
    dm = (dte_mode or "").strip().lower()
    if dm == "0dte":
        return "high_caution_0dte" if (event_risk_flag or gm in {"short_gamma", "flip_zone"}) else "balanced_0dte"
    if event_risk_flag:
        return "defensive"
    if gm == "short_gamma":
        return "opportunistic"
    if gm == "long_gamma":
        return "balanced"
    return "balanced"


def _strategy_candidates(
    recommended: str,
    *,
    selector_direction: str,
    selector_regime: str,
    gamma_regime: str,
    dte_mode: str,
) -> list[str]:
    """Lista determinista (2–4) que incluye el recomendado."""
    rec = str(recommended or "").strip()
    gm = (gamma_regime or "unknown").strip().lower()
    dm = (dte_mode or "").strip().lower()
    is_bull = selector_direction == "BUY" or selector_regime in {"BULL", "TREND"}
    is_bear = selector_direction == "SELL" or selector_regime == "BEAR"
    is_side = selector_regime == "SIDEWAYS"

    out: list[str] = []

    # 0DTE: priorizar estructuras simples.
    if dm == "0dte":
        if is_side:
            out = ["iron_condor", "bull_put_credit_spread", "bear_call_credit_spread"]
        elif is_bull:
            out = ["bull_put_credit_spread", "bull_call_debit_spread", "long_call"]
        elif is_bear:
            out = ["bear_call_credit_spread", "bear_put_debit_spread", "long_put"]
        else:
            out = ["iron_condor", "bull_put_credit_spread"]
    else:
        # Swing: permitir time structure y overlays según gamma.
        if is_side:
            out = ["iron_condor", "iron_butterfly", "call_calendar_spread"]
        elif is_bull:
            out = ["bull_put_credit_spread", "bull_call_debit_spread", "call_diagonal_debit_spread"]
        elif is_bear:
            out = ["bear_call_credit_spread", "bear_put_debit_spread", "put_diagonal_debit_spread"]
        else:
            out = ["iron_condor", "bull_put_credit_spread", "bear_call_credit_spread"]

    if gm == "short_gamma":
        # Empujar alternativas de débito.
        out = [s for s in out if s not in {"iron_condor", "iron_butterfly"}]
        out.append("bull_call_debit_spread" if is_bull else "bear_put_debit_spread")
    elif gm == "long_gamma":
        # Incluir neutral/credit si no está.
        for s in ("iron_condor", "bull_put_credit_spread", "bear_call_credit_spread"):
            if s not in out:
                out.append(s)

    # Dedup + asegurar recomendado primero.
    dedup: list[str] = []
    for s in [rec, *out]:
        if not s:
            continue
        if s not in dedup:
            dedup.append(s)
    return dedup[:4]


class SessionBriefingEngine:
    """Genera un briefing cuantitativo mínimo para la sesión (paper / test)."""

    def __init__(
        self,
        iv_rank_calculator: _SupportsIVRank,
        *,
        pick_strategy_fn: Callable[..., str] | None = None,
        option_selector: Callable[..., str] | None = None,
        default_liquidity_score: float = 0.85,
        default_skew_pct: float = 0.0,
        default_term_structure_slope: float = 1.0,
    ) -> None:
        self._iv = iv_rank_calculator
        _picker = pick_strategy_fn or option_selector
        if pick_strategy_fn is not None and option_selector is not None:
            raise ValueError("Usar solo uno de pick_strategy_fn u option_selector.")
        self._pick = _picker or pick_strategy
        self._liquidity = default_liquidity_score
        self._skew = default_skew_pct
        self._term_slope = default_term_structure_slope

    def build_briefing(
        self,
        symbol: str,
        *,
        direction: str,
        regime: str,
        gamma_regime: str | None = None,
        dte_mode: str | None = None,
        days_to_event: int | None = None,
        event_near: bool = False,
        spot: float | None = None,
        support_levels: list[float] | None = None,
        resistance_levels: list[float] | None = None,
        breach_context: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        sym = str(symbol or "").strip().upper()
        session_date = date.today().isoformat()
        quality_flags: list[str] = []
        notes: list[str] = []
        operational_notes: list[str] = []

        # --- gamma_regime ---
        gamma_in = gamma_regime
        if gamma_in is None or str(gamma_in).strip() == "":
            gamma_eff = "unknown"
            quality_flags.append("no_gamma_regime")
            notes.append("gamma_regime no provisto; usando 'unknown' (preferir input del desk/modelo).")
        else:
            gamma_eff = normalize_gamma_regime(str(gamma_in)) or "unknown"
            if gamma_eff == "unknown":
                quality_flags.append("gamma_regime_unrecognized")
                notes.append("gamma_regime no reconocido; tratado como 'unknown'.")

        # --- dte_mode ---
        dte_derived = False
        if dte_mode is None or str(dte_mode).strip() == "":
            derived = derive_dte_mode_from_days_to_event(days_to_event)
            if derived is not None:
                dte_eff = derived
                dte_derived = True
                quality_flags.append("dte_mode_derived_from_days_to_event")
                notes.append(f"dte_mode derivado de days_to_event={days_to_event} → {dte_eff}.")
            else:
                dte_eff = "8to21"
                quality_flags.append("dte_mode_defaulted")
                notes.append("dte_mode no provisto; usando horizonte por defecto 8to21 (swing).")
        else:
            dte_eff = normalize_dte_mode(str(dte_mode)) or "unknown"
            if dte_eff == "unknown":
                quality_flags.append("dte_mode_unrecognized")
                dte_eff = "8to21"
                notes.append("dte_mode no reconocido; fallback 8to21.")

        dte_target = _dte_target_for_iv_chain(dte_eff)

        # --- IV rank ---
        iv_payload = self._iv.get_iv_rank(sym, dte_target=dte_target)
        iv_quality = str(iv_payload.get("quality") or "")
        iv_current = iv_payload.get("iv_current")
        iv_rank = iv_payload.get("iv_rank")
        iv_hv_ratio = iv_payload.get("iv_hv_ratio")
        spot_eff = spot if spot is not None else iv_payload.get("spot")

        if iv_quality and iv_quality != "ok":
            quality_flags.append(f"iv_rank_quality_{iv_quality}")
            if iv_quality == "approx":
                quality_flags.append("approx_iv_rank")
            notes.append(f"IV rank proxy calidad={iv_quality} (ver IVRankCalculator).")

        if iv_rank is None:
            iv_rank = 50.0
            quality_flags.append("iv_rank_fallback_mid")
            notes.append("iv_rank ausente; usando 50.0 para pick_strategy.")
        if iv_hv_ratio is None:
            iv_hv_ratio = 1.0
            quality_flags.append("iv_hv_ratio_fallback_unit")
            notes.append("iv_hv_ratio ausente; usando 1.0.")

        try:
            spot_f = float(spot_eff) if spot_eff is not None else 0.0
        except (TypeError, ValueError):
            spot_f = 0.0
        if spot_f <= 0:
            quality_flags.append("spot_unavailable")
            notes.append("Spot no disponible; expected_move no calculado.")
            em_val = None
            em_meta: dict[str, Any] = {"method": "skipped", "reason": "no_spot"}
        else:
            try:
                iv_ann = float(iv_current) if iv_current is not None else 0.0
            except (TypeError, ValueError):
                iv_ann = 0.0
            if iv_ann <= 0:
                quality_flags.append("iv_current_missing")
                notes.append("iv_current ausente; expected_move no calculado.")
                em_val = None
                em_meta = {"method": "skipped", "reason": "no_iv_current"}
            else:
                horizon = _effective_dte_days_for_mode(dte_eff)
                em_val, em_meta = _expected_move_sigma(spot_f, iv_ann, horizon_trading_days=horizon)
                em_meta["iv_annual_used"] = round(iv_ann, 6)
                notes.append(
                    f"Expected move 1σ ≈ {em_val:.2f} (spot={spot_f:.2f}, horizonte ~{horizon} días hábiles referencia)."
                )
                operational_notes.append("expected_move_range_available_for_strike_gating")

        sel_dir, sel_reg = _normalize_selector_direction_regime(direction, regime)
        strat = self._pick(
            sel_dir,
            sel_reg,
            float(iv_rank),
            float(iv_hv_ratio),
            thesis=None,
            event_near=event_near,
            liquidity_score=self._liquidity,
            skew_pct=self._skew,
            term_structure_slope=self._term_slope,
            prefer_defined_risk=True,
            gamma_regime=gamma_eff if gamma_eff != "unknown" else None,
            dte_mode=dte_eff,
        )
        family = strategy_to_recommended_family(strat)

        # Event risk flag (ventana corta por days_to_event).
        event_risk_flag = bool(event_near or (days_to_event is not None and days_to_event <= 7))
        if event_risk_flag:
            operational_notes.append("event_risk_flag_true_tighter_controls_suggested")

        visual_breach_flag = bool((breach_context or {}).get("breach_detected"))
        if visual_breach_flag:
            operational_notes.append("visual_breach_context_detected")
            quality_flags.append("visual_breach_context_present")

        # Expected move range.
        expected_move_range: dict[str, float] | None = None
        if spot_f > 0 and em_val is not None:
            expected_move_range = {"lower": round(spot_f - float(em_val), 4), "upper": round(spot_f + float(em_val), 4)}

        # Levels (externos).
        supports = _normalize_levels(support_levels)
        resistances = _normalize_levels(resistance_levels)
        if supports or resistances:
            operational_notes.append("structural_levels_provided_externally")
        else:
            operational_notes.append("no_structural_levels_provided")
            quality_flags.append("no_structural_levels_provided")

        market_bias = _market_bias(direction, regime, gamma_eff, dte_eff, event_risk_flag)
        risk_posture = _risk_posture(gamma_eff, dte_eff, event_risk_flag)
        candidates = _strategy_candidates(
            strat,
            selector_direction=sel_dir,
            selector_regime=sel_reg,
            gamma_regime=gamma_eff,
            dte_mode=dte_eff,
        )

        if gamma_eff == "long_gamma":
            operational_notes.append("long_gamma_favors_mean_reversion_structures")
        elif gamma_eff == "short_gamma":
            operational_notes.append("short_gamma_favors_directional_debit_structures")
        if dte_eff == "0dte":
            operational_notes.append("0dte_requires_stricter_exit_discipline")

        if iv_rank < 30:
            notes.append("IV rank bajo → el selector suele favorecer débitos / long premium según régimen.")
        elif iv_rank > 60:
            notes.append("IV rank alto → estructuras de venta de prima / neutral pueden ser competitivas.")

        out: dict[str, Any] = {
            "symbol": sym,
            "session_date": session_date,
            "direction": (direction or "").strip().lower(),
            "regime": (regime or "").strip().lower(),
            "gamma_regime": gamma_eff,
            "dte_mode": dte_eff,
            "dte_mode_derived": dte_derived,
            "iv_rank": round(float(iv_rank), 2),
            "iv_hv_ratio": round(float(iv_hv_ratio), 4),
            "iv_current": iv_current,
            "iv_rank_payload_quality": iv_quality or None,
            "expected_move": None if em_val is None else round(float(em_val), 4),
            "expected_move_meta": em_meta,
            "spot": round(spot_f, 4) if spot_f > 0 else None,
            "expected_move_range": expected_move_range,
            "support_levels": supports,
            "resistance_levels": resistances,
            "market_bias": market_bias,
            "risk_posture": risk_posture,
            "event_risk_flag": event_risk_flag,
            "visual_breach_flag": visual_breach_flag,
            "breach_context": breach_context or {},
            "selector_direction": sel_dir,
            "selector_regime": sel_reg,
            "recommended_strategy": strat,
            "recommended_family": family,
            "strategy_candidates": candidates,
            "notes": notes,
            "operational_notes": operational_notes,
            "quality_flags": quality_flags,
            "iv_source": {
                "quality": iv_quality or None,
                "method": iv_payload.get("method"),
                "expiration": iv_payload.get("expiration"),
                "dte": iv_payload.get("dte"),
                "error": iv_payload.get("error"),
            },
        }
        return out
