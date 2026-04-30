"""OptionsIntentRouter v0: traduce briefing en intención operativa compacta."""
from __future__ import annotations

from typing import Any

from atlas_code_quant.options.session_briefing import strategy_to_recommended_family

_KNOWN_FAMILIES = [
    "credit_neutral",
    "credit_directional",
    "debit_directional",
    "long_premium",
    "time_structure",
    "other",
]

# Referencia estable para otros módulos (p. ej. orquestador paper) sin acoplar a ``_KNOWN_FAMILIES``.
KNOWN_STRATEGY_FAMILIES = list(_KNOWN_FAMILIES)


class OptionsIntentRouter:
    """Fusiona contexto de briefing + señal visual en un intent pre-selector."""

    def build_intent(
        self,
        briefing: dict[str, Any],
        *,
        manual_overrides: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        b = briefing or {}
        symbol = str(b.get("symbol") or "").upper()
        direction = str(b.get("direction") or "").strip().lower()
        regime = str(b.get("regime") or "").strip().lower()
        gamma_regime = str(b.get("gamma_regime") or "").strip().lower()
        dte_mode = str(b.get("dte_mode") or "").strip().lower()
        event_risk_flag = bool(b.get("event_risk_flag"))
        market_bias = str(b.get("market_bias") or "neutral")
        risk_posture = str(b.get("risk_posture") or "balanced")
        recommended_strategy = str(b.get("recommended_strategy") or "")
        recommended_family = str(b.get("recommended_family") or "") or (
            strategy_to_recommended_family(recommended_strategy) if recommended_strategy else "other"
        )
        briefing_candidates = list(b.get("strategy_candidates") or [])
        visual_breach_flag = bool(b.get("visual_breach_flag"))
        breach_context = b.get("breach_context") or {}
        if isinstance(breach_context, dict):
            visual_breach_flag = visual_breach_flag or bool(breach_context.get("breach_detected"))

        intent_notes: list[str] = []
        quality_flags: list[str] = []

        selector_inputs = {
            "direction": direction,
            "regime": regime,
            "gamma_regime": gamma_regime,
            "dte_mode": dte_mode,
            "event_near": event_risk_flag,
        }

        # Calidad mínima de input.
        if not direction:
            quality_flags.append("missing_direction")
        if not regime:
            quality_flags.append("missing_regime")
        if not gamma_regime or gamma_regime == "unknown":
            quality_flags.append("missing_gamma_regime")
        if not dte_mode:
            quality_flags.append("missing_dte_mode")
        if dte_mode == "0dte" and not visual_breach_flag and not breach_context:
            quality_flags.append("briefing_visual_context_missing")

        # Defaults.
        allow_entry = True
        force_no_trade = False
        suppress_short_premium = False
        prefer_defined_risk = True  # Índices SPX/SPY: defecto conservador.

        # Reglas de riesgo principal.
        if visual_breach_flag and dte_mode == "0dte":
            allow_entry = False
            intent_notes.append("visual_breach_detected_reduce_aggression")
            if event_risk_flag or "high_caution" in risk_posture:
                force_no_trade = True
        if event_risk_flag and visual_breach_flag:
            force_no_trade = True
        if force_no_trade:
            allow_entry = False
            intent_notes.append("force_no_trade_active")

        if visual_breach_flag:
            suppress_short_premium = True
        if gamma_regime == "short_gamma" and dte_mode == "0dte":
            suppress_short_premium = True
            intent_notes.append("short_gamma_0dte_discourages_short_premium")
        if event_risk_flag and dte_mode == "0dte":
            suppress_short_premium = True
            intent_notes.append("event_risk_requires_tighter_controls")

        # Familias preferidas/bloqueadas.
        preferred_families: list[str] = []
        blocked_families: list[str] = []
        directional = direction in {"bull", "bullish", "buy", "alcista", "long", "bear", "bearish", "sell", "bajista", "short"} or regime in {"bull", "bear", "trend"}
        if gamma_regime == "long_gamma" and regime in {"ranging", "range", "sideways", "neutral"}:
            preferred_families = ["credit_neutral", "credit_directional"]
            intent_notes.append("long_gamma_ranging_favors_mean_reversion")
        elif gamma_regime == "short_gamma" and directional:
            preferred_families = ["debit_directional", "long_premium"]
        elif recommended_family:
            preferred_families = [recommended_family]

        if event_risk_flag:
            # Penaliza estructuras que pueden exigir gestión fina de vega/tiempo.
            blocked_families.append("time_structure")
        if visual_breach_flag and dte_mode == "0dte":
            blocked_families.append("credit_neutral")
        blocked_families = self._dedup(blocked_families)
        preferred_families = [fam for fam in self._dedup(preferred_families) if fam not in blocked_families]

        if force_no_trade:
            preferred_families = []
            blocked_families = list(_KNOWN_FAMILIES)

        entry_bias = self._entry_bias(direction=direction, market_bias=market_bias)

        # Candidatos: briefing + recomendado, filtrado por familias bloqueadas.
        candidates = self._dedup([recommended_strategy, *briefing_candidates])
        filtered_candidates = [
            s for s in candidates if strategy_to_recommended_family(str(s)) not in set(blocked_families)
        ]
        if not filtered_candidates and not force_no_trade and recommended_strategy:
            # fallback suave: mantener recomendada si no hay otra alternativa y no hay bloqueo total.
            if strategy_to_recommended_family(recommended_strategy) not in set(blocked_families):
                filtered_candidates = [recommended_strategy]

        intent = {
            "symbol": symbol,
            "allow_entry": bool(allow_entry),
            "suppress_short_premium": bool(suppress_short_premium),
            "prefer_defined_risk": bool(prefer_defined_risk),
            "force_no_trade": bool(force_no_trade),
            "entry_bias": entry_bias,
            "market_bias": market_bias,
            "risk_posture": risk_posture,
            "selector_inputs": selector_inputs,
            "preferred_families": preferred_families,
            "blocked_families": blocked_families,
            "strategy_candidates": filtered_candidates,
            "recommended_strategy": (
                recommended_strategy
                if recommended_strategy and strategy_to_recommended_family(recommended_strategy) not in set(blocked_families)
                else None
            ),
            "intent_notes": self._dedup(intent_notes),
            "intent_quality_flags": self._dedup(quality_flags),
        }

        # Overrides explícitos al final.
        if manual_overrides:
            self._apply_overrides(intent, manual_overrides)
            notes = list(intent.get("intent_notes") or [])
            notes.append("manual_override_applied")
            intent["intent_notes"] = self._dedup(notes)
        return intent

    def _apply_overrides(self, intent: dict[str, Any], overrides: dict[str, Any]) -> None:
        allowed_keys = {
            "allow_entry",
            "force_no_trade",
            "suppress_short_premium",
            "prefer_defined_risk",
            "preferred_families",
            "blocked_families",
        }
        for key in allowed_keys:
            if key not in overrides:
                continue
            value = overrides[key]
            if key in {"preferred_families", "blocked_families"}:
                intent[key] = self._dedup([str(v) for v in (value or [])])
            else:
                intent[key] = bool(value)

    @staticmethod
    def _entry_bias(*, direction: str, market_bias: str) -> str:
        d = (direction or "").lower()
        mb = (market_bias or "").lower()
        if "neutral_to_bullish" in mb:
            return "neutral_to_bullish"
        if "neutral_to_bearish" in mb:
            return "neutral_to_bearish"
        if d in {"bull", "bullish", "buy", "alcista", "long"} or "bull" in mb:
            return "bullish"
        if d in {"bear", "bearish", "sell", "bajista", "short"} or "bear" in mb:
            return "bearish"
        return "neutral"

    @staticmethod
    def _dedup(items: list[Any]) -> list[Any]:
        out: list[Any] = []
        seen = set()
        for item in items:
            key = item
            if key in seen:
                continue
            seen.add(key)
            out.append(item)
        return out
