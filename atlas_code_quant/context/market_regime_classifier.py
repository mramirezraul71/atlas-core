from __future__ import annotations

from dataclasses import dataclass
from typing import Any

try:
    from config.settings import settings as _quant_settings
except Exception:  # pragma: no cover
    _quant_settings = None  # type: ignore[assignment]


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if result != result or result in (float("inf"), float("-inf")):
        return default
    return result


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class RegimeSignal:
    label: str
    score_pct: float
    reason: str


class MarketRegimeClassifier:
    """Context-first regime classifier for Atlas.

    The classifier is intentionally heuristic and traceable. It consumes the
    candidate payload produced by the scanner/selector and emits a normalized
    market regime view that can be used as a hard gate before evaluation.
    """

    def __init__(self) -> None:
        if _quant_settings is not None:
            self._hyst_margin = float(getattr(_quant_settings, "context_regime_score_hysteresis", 10.0))
            self._strong_score = float(getattr(_quant_settings, "context_regime_strong_score", 72.0))
        else:
            self._hyst_margin = 10.0
            self._strong_score = 72.0
        self._last_primary_label: str | None = None
        self._last_primary_score: float = 0.0

    def reset_hysteresis(self) -> None:
        self._last_primary_label = None
        self._last_primary_score = 0.0

    def _stabilize_primary(self, label_new: str, score_new: float) -> tuple[str, bool]:
        """Evita saltar de régimen por diferencias pequeñas de score entre candidatos."""
        if self._last_primary_label is None:
            self._last_primary_label = label_new
            self._last_primary_score = score_new
            return label_new, False
        if label_new == self._last_primary_label:
            self._last_primary_score = max(self._last_primary_score, score_new)
            return label_new, False
        if score_new - self._last_primary_score >= self._hyst_margin or score_new >= self._strong_score:
            self._last_primary_label = label_new
            self._last_primary_score = score_new
            return label_new, False
        return self._last_primary_label, True

    def classify(self, candidate: dict[str, Any]) -> dict[str, Any]:
        direction = str(candidate.get("direction") or "").strip().lower()
        regime_hint = str(
            candidate.get("market_regime")
            or candidate.get("regime")
            or candidate.get("market_state")
            or ""
        ).strip().lower()
        timeframe = str(candidate.get("timeframe") or "1h").strip().lower()
        selection_score = _clip(_safe_float(candidate.get("selection_score"), 0.0), 0.0, 100.0)
        local_win_rate = _clip(_safe_float(candidate.get("local_win_rate_pct"), 0.0), 0.0, 100.0)
        predicted_move = max(_safe_float(candidate.get("predicted_move_pct"), 0.0), 0.0)
        relative_strength = _safe_float(candidate.get("relative_strength_pct"), 0.0)
        iv_rank = _clip(
            _safe_float(candidate.get("iv_rank"), _safe_float(candidate.get("iv_rank_pct"), 35.0)),
            0.0,
            100.0,
        )
        iv_hv_ratio = max(_safe_float(candidate.get("iv_hv_ratio"), 1.0), 0.0)
        liquidity_score = _clip(_safe_float(candidate.get("liquidity_score"), 0.75), 0.0, 1.0)
        event_near = bool(candidate.get("event_near") or candidate.get("earnings_near"))
        has_options = bool(candidate.get("has_options")) if candidate.get("has_options") is not None else None

        signals: list[RegimeSignal] = []
        trend_score = 45.0
        lateral_score = 30.0
        high_vol_score = 10.0
        low_liq_score = 0.0
        macro_event_score = 0.0
        risk_extreme_score = 0.0

        if regime_hint in {"bull", "trend", "uptrend"}:
            trend_score += 25.0
        elif regime_hint in {"bear", "downtrend"}:
            trend_score += 22.0
        elif regime_hint in {"sideways", "range", "lateral"}:
            lateral_score += 30.0

        if abs(relative_strength) >= 3.0:
            trend_score += 12.0
        if predicted_move <= 0.8:
            lateral_score += 10.0
        if predicted_move >= 2.25:
            high_vol_score += 22.0
        if iv_rank >= 70.0 or iv_hv_ratio >= 1.35:
            high_vol_score += 24.0
        if liquidity_score < 0.55:
            low_liq_score += 70.0
        elif liquidity_score < 0.75:
            low_liq_score += 35.0
        if event_near:
            macro_event_score += 65.0
            high_vol_score += 10.0
        if selection_score < 60.0 or local_win_rate < 52.0:
            risk_extreme_score += 18.0
        if liquidity_score < 0.45 and predicted_move >= 2.0:
            risk_extreme_score += 38.0
        if event_near and iv_rank >= 75.0:
            risk_extreme_score += 25.0

        trend_score = _clip(trend_score, 0.0, 100.0)
        lateral_score = _clip(lateral_score, 0.0, 100.0)
        high_vol_score = _clip(high_vol_score, 0.0, 100.0)
        low_liq_score = _clip(low_liq_score, 0.0, 100.0)
        macro_event_score = _clip(macro_event_score, 0.0, 100.0)
        risk_extreme_score = _clip(risk_extreme_score, 0.0, 100.0)

        if trend_score >= 55.0:
            reason = f"regime_hint={regime_hint or 'none'}, rs={relative_strength:.2f}, move={predicted_move:.2f}%"
            signals.append(RegimeSignal("trending", trend_score, reason))
        if lateral_score >= 50.0:
            reason = f"move={predicted_move:.2f}%, timeframe={timeframe}, regime_hint={regime_hint or 'none'}"
            signals.append(RegimeSignal("sideways", lateral_score, reason))
        if high_vol_score >= 45.0:
            reason = f"iv_rank={iv_rank:.1f}, iv_hv_ratio={iv_hv_ratio:.2f}, move={predicted_move:.2f}%"
            signals.append(RegimeSignal("high_volatility", high_vol_score, reason))
        if low_liq_score >= 35.0:
            reason = f"liquidity_score={liquidity_score:.2f}"
            signals.append(RegimeSignal("low_liquidity", low_liq_score, reason))
        if macro_event_score >= 40.0:
            reason = "event_near=True"
            signals.append(RegimeSignal("macro_event", macro_event_score, reason))
        if risk_extreme_score >= 40.0:
            reason = (
                f"selection={selection_score:.1f}, local_win={local_win_rate:.1f}, "
                f"liquidity={liquidity_score:.2f}, iv_rank={iv_rank:.1f}"
            )
            signals.append(RegimeSignal("risk_extreme", risk_extreme_score, reason))

        if not signals:
            signals.append(RegimeSignal("trending" if direction == "alcista" else "sideways", 50.0, "fallback"))

        ranked = sorted(signals, key=lambda item: item.score_pct, reverse=True)
        winner = ranked[0]
        stable_label, hyst_applied = self._stabilize_primary(winner.label, winner.score_pct)
        primary = next((s for s in ranked if s.label == stable_label), None)
        if primary is None:
            primary = RegimeSignal(
                stable_label,
                self._last_primary_score,
                f"hysteresis_hold(raw={winner.label}@{winner.score_pct:.1f})",
            )
        confidence = _clip(
            primary.score_pct * 0.55
            + selection_score * 0.2
            + local_win_rate * 0.15
            + liquidity_score * 100.0 * 0.1,
            0.0,
            100.0,
        )

        risk_flags: list[str] = []
        if low_liq_score >= 50.0:
            risk_flags.append("low_liquidity")
        if high_vol_score >= 70.0:
            risk_flags.append("high_volatility")
        if macro_event_score >= 50.0:
            risk_flags.append("macro_event")
        if risk_extreme_score >= 45.0:
            risk_flags.append("risk_extreme")
        if has_options is False and timeframe in {"5m", "15m"}:
            risk_flags.append("thin_optionability")

        return {
            "generated_at": None,
            "primary_regime": primary.label,
            "primary_regime_raw": winner.label,
            "context_hysteresis_applied": hyst_applied,
            "confidence_pct": round(confidence, 2),
            "signals": [
                {
                    "label": signal.label,
                    "score_pct": round(signal.score_pct, 2),
                    "reason": signal.reason,
                }
                for signal in ranked
            ],
            "trend_state": {
                "score_pct": round(trend_score, 2),
                "direction": direction or ("alcista" if regime_hint in {"bull", "trend", "uptrend"} else "neutral"),
            },
            "volatility_state": {
                "score_pct": round(high_vol_score, 2),
                "iv_rank_pct": round(iv_rank, 2),
                "iv_hv_ratio": round(iv_hv_ratio, 3),
            },
            "liquidity_state": {
                "score_pct": round(low_liq_score, 2),
                "liquidity_score": round(liquidity_score, 3),
            },
            "macro_event_state": {
                "score_pct": round(macro_event_score, 2),
                "event_near": event_near,
            },
            "risk_extreme_state": {
                "score_pct": round(risk_extreme_score, 2),
                "flags": risk_flags,
            },
            "states": {
                "trending": trend_score >= 55.0,
                "sideways": lateral_score >= 50.0,
                "high_volatility": high_vol_score >= 45.0,
                "low_liquidity": low_liq_score >= 35.0,
                "macro_event": macro_event_score >= 40.0,
                "risk_extreme": risk_extreme_score >= 40.0,
            },
        }
