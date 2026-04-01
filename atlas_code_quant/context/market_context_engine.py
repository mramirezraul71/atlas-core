from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Any

try:
    from atlas_code_quant.context.market_regime_classifier import MarketRegimeClassifier
    from atlas_code_quant.knowledge.knowledge_base import get_knowledge_base
except ModuleNotFoundError:  # pragma: no cover - uvicorn cwd fallback
    from context.market_regime_classifier import MarketRegimeClassifier
    from knowledge.knowledge_base import get_knowledge_base


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


class MarketContextEngine:
    """Builds a pre-trade market context snapshot and gate decision."""

    def __init__(self, root_path: Path | None = None) -> None:
        self.root_path = root_path or Path(__file__).resolve().parents[2]
        self.classifier = MarketRegimeClassifier()
        self.knowledge = get_knowledge_base()

    def build(self, *, candidate: dict[str, Any], tracker_summary: dict[str, Any] | None = None) -> dict[str, Any]:
        tracker_summary = tracker_summary or {}
        symbol = str(candidate.get("symbol") or "").upper()
        timeframe = str(candidate.get("timeframe") or "1h")
        method = str(candidate.get("strategy_key") or candidate.get("strategy_label") or "unknown")
        direction = str(candidate.get("direction") or "neutral").lower()
        selection_score = _clip(_safe_float(candidate.get("selection_score"), 0.0), 0.0, 100.0)
        local_win_rate = _clip(_safe_float(candidate.get("local_win_rate_pct"), 0.0), 0.0, 100.0)
        predicted_move = max(_safe_float(candidate.get("predicted_move_pct"), 0.0), 0.0)
        relative_strength = _safe_float(candidate.get("relative_strength_pct"), 0.0)
        liquidity_score = _clip(_safe_float(candidate.get("liquidity_score"), 0.75), 0.0, 1.0)
        event_near = bool(candidate.get("event_near") or candidate.get("earnings_near"))
        regime_payload = self.classifier.classify(candidate)
        regime_payload["generated_at"] = datetime.utcnow().isoformat()

        advisory = self.knowledge.advisory_context(
            method=method,
            timeframe=timeframe,
            topic="market regime",
            max_sources=5,
        )
        method_sources = self.knowledge.sources_for_method(method)[:3]
        timeframe_sources = self.knowledge.sources_for_timeframe(timeframe)[:3]

        macro_bias = "cautious"
        if regime_payload["states"]["macro_event"] or regime_payload["states"]["risk_extreme"]:
            macro_bias = "defensive"
        elif regime_payload["states"]["trending"] and not regime_payload["states"]["high_volatility"]:
            macro_bias = "supportive"

        breadth_proxy = tracker_summary.get("totals") or {}
        total_positions = int(breadth_proxy.get("positions") or 0)
        global_risk_score = _clip(
            regime_payload["volatility_state"]["score_pct"] * 0.45
            + (100.0 - liquidity_score * 100.0) * 0.25
            + (20.0 if event_near else 0.0)
            + (15.0 if total_positions > 20 else 0.0),
            0.0,
            100.0,
        )
        global_state = "risk_off" if global_risk_score >= 60.0 else "mixed" if global_risk_score >= 35.0 else "risk_on"

        session_phase = self._session_phase(candidate)
        premarket = self._premarket_intraday(candidate=candidate, session_phase=session_phase)
        asset_profile = self._asset_history_profile(candidate=candidate, regime=regime_payload)
        historical_analogies = self._historical_analogies(
            candidate=candidate,
            regime=regime_payload,
            asset_profile=asset_profile,
            advisory=advisory,
        )
        risk_assessment = self._risk_assessment(
            candidate=candidate,
            regime=regime_payload,
            premarket=premarket,
            advisory=advisory,
            global_risk_score=global_risk_score,
        )
        gate = self._decision_gate(
            regime=regime_payload,
            risk_assessment=risk_assessment,
            advisory=advisory,
            premarket=premarket,
        )

        confidence = _clip(
            selection_score * 0.25
            + local_win_rate * 0.2
            + regime_payload["confidence_pct"] * 0.25
            + advisory.get("academic_support_score", 0.0) * 0.15
            + risk_assessment["clarity_score_pct"] * 0.15,
            0.0,
            100.0,
        )

        return {
            "generated_at": datetime.utcnow().isoformat(),
            "symbol": symbol,
            "timeframe": timeframe,
            "method": method,
            "regime": regime_payload,
            "macro_bias": {
                "state": macro_bias,
                "inflation": "unknown_live",
                "rates": "unknown_live",
                "policy": "caution_when_unknown",
                "confidence_pct": round(_clip(regime_payload["macro_event_state"]["score_pct"] * 0.6 + 40.0, 0.0, 100.0), 2),
            },
            "global_market": {
                "state": global_state,
                "risk_score_pct": round(global_risk_score, 2),
                "breadth_proxy_positions": total_positions,
                "relative_strength_pct": round(relative_strength, 2),
                "liquidity_score": round(liquidity_score, 3),
            },
            "asset_state": {
                "direction": direction,
                "selection_score_pct": round(selection_score, 2),
                "local_win_rate_pct": round(local_win_rate, 2),
                "predicted_move_pct": round(predicted_move, 2),
                "event_near": event_near,
                "profile": asset_profile,
            },
            "premarket_intraday": premarket,
            "historical_analogies": historical_analogies,
            "risk_assessment": risk_assessment,
            "confidence_pct": round(confidence, 2),
            "decision_gate": gate,
            "source_trace": {
                "knowledge_method_sources": [source.get("id") for source in method_sources],
                "knowledge_timeframe_sources": [source.get("id") for source in timeframe_sources],
                "knowledge_warnings": list(advisory.get("warnings") or []),
                "taxonomy": "macro->global->asset->session->history->gate",
            },
            "context_report": self._context_report(
                symbol=symbol,
                regime=regime_payload,
                macro_bias=macro_bias,
                global_state=global_state,
                premarket=premarket,
                historical_analogies=historical_analogies,
                risk_assessment=risk_assessment,
                gate=gate,
                confidence=confidence,
            ),
        }

    def _session_phase(self, candidate: dict[str, Any]) -> str:
        timeframe = str(candidate.get("timeframe") or "1h").lower()
        if timeframe in {"5m", "15m"}:
            return "intraday_opening"
        if timeframe in {"1h", "4h"}:
            return "intraday"
        return "swing"

    def _premarket_intraday(self, *, candidate: dict[str, Any], session_phase: str) -> dict[str, Any]:
        predicted_move = max(_safe_float(candidate.get("predicted_move_pct"), 0.0), 0.0)
        order_flow = candidate.get("order_flow") or {}
        confirmation = candidate.get("confirmation") or {}
        flow_conf = _clip(_safe_float(order_flow.get("confidence_pct"), _safe_float(order_flow.get("score_pct"), 50.0)), 0.0, 100.0)
        higher_tf = str(confirmation.get("higher_timeframe") or "1h")
        gap_state = "neutral"
        if predicted_move >= 2.0:
            gap_state = "expansion_risk"
        elif predicted_move >= 1.0:
            gap_state = "tradable_gap"
        session_bias = "accepted" if flow_conf >= 60.0 else "uncertain" if flow_conf >= 35.0 else "rejected"
        return {
            "session_phase": session_phase,
            "gap_state": gap_state,
            "volume_quality": "strong" if flow_conf >= 70.0 else "mixed" if flow_conf >= 45.0 else "weak",
            "relative_strength_state": "strong" if _safe_float(candidate.get("relative_strength_pct"), 0.0) >= 0.0 else "weak",
            "acceptance_state": session_bias,
            "higher_timeframe": higher_tf,
            "intraday_structure": "breakout_or_trend" if flow_conf >= 65.0 else "rotation_or_chop",
        }

    def _asset_history_profile(self, *, candidate: dict[str, Any], regime: dict[str, Any]) -> dict[str, Any]:
        symbol = str(candidate.get("symbol") or "").upper()
        timeframe = str(candidate.get("timeframe") or "1h")
        event_near = bool(candidate.get("event_near") or candidate.get("earnings_near"))
        profile_key = "earnings_gap_profile" if event_near else "trend_follow_profile" if regime["states"]["trending"] else "range_response_profile"
        return {
            "profile_key": profile_key,
            "asset_class": str(candidate.get("asset_class") or ("optionable_equity" if candidate.get("has_options") else "equity")),
            "session_tendency": "open_drive" if timeframe in {"5m", "15m"} else "mid_session_continuation" if timeframe in {"1h", "4h"} else "multi_session_drift",
            "news_sensitivity": "high" if event_near else "medium",
            "shock_behavior": "gap_and_go" if regime["states"]["high_volatility"] else "mean_revert_after_open",
            "profile_path": f"asset_history_profiles/{profile_key}.md",
            "symbol_reference": symbol,
        }

    def _historical_analogies(
        self,
        *,
        candidate: dict[str, Any],
        regime: dict[str, Any],
        asset_profile: dict[str, Any],
        advisory: dict[str, Any],
    ) -> dict[str, Any]:
        analogies: list[dict[str, Any]] = []
        if regime["states"]["trending"]:
            analogies.append(
                {
                    "label": "trend_persistence_setup",
                    "relevance_pct": 78.0,
                    "why": "sesgo, fuerza relativa y regimen sugieren continuidad mas que reversión temprana",
                }
            )
        if regime["states"]["high_volatility"]:
            analogies.append(
                {
                    "label": "volatility_expansion_event",
                    "relevance_pct": 74.0,
                    "why": "la volatilidad actual exige confirmar aceptacion y no comprar ruptura ciega",
                }
            )
        if not analogies:
            analogies.append(
                {
                    "label": "mixed_context_low_edge",
                    "relevance_pct": 55.0,
                    "why": "el contexto no se parece a una familia historica limpia de edge alto",
                }
            )
        return {
            "matched_profile": asset_profile["profile_key"],
            "analogies": analogies[:3],
            "academic_hint": list(advisory.get("key_insights") or [])[:2],
        }

    def _risk_assessment(
        self,
        *,
        candidate: dict[str, Any],
        regime: dict[str, Any],
        premarket: dict[str, Any],
        advisory: dict[str, Any],
        global_risk_score: float,
    ) -> dict[str, Any]:
        warnings: list[str] = []
        uncertainty_score = 0.0
        if regime["states"]["macro_event"]:
            warnings.append("evento macro o corporativo cercano")
            uncertainty_score += 30.0
        if regime["states"]["low_liquidity"]:
            warnings.append("liquidez insuficiente")
            uncertainty_score += 25.0
        if premarket["acceptance_state"] != "accepted":
            warnings.append("la sesion actual no confirma aceptacion limpia")
            uncertainty_score += 15.0
        if any("IC" in warning or "ic" in warning for warning in (advisory.get("warnings") or [])):
            warnings.append("evidencia estadistica en vivo insuficiente")
            uncertainty_score += 10.0
        if _safe_float(candidate.get("local_win_rate_pct"), 0.0) < 55.0:
            warnings.append("win rate local todavia debil")
            uncertainty_score += 8.0

        quality = _clip(100.0 - uncertainty_score - global_risk_score * 0.25, 0.0, 100.0)
        clarity = _clip(quality * 0.7 + regime["confidence_pct"] * 0.3, 0.0, 100.0)
        return {
            "signal_quality_score_pct": round(quality, 2),
            "uncertainty_score_pct": round(_clip(uncertainty_score, 0.0, 100.0), 2),
            "clarity_score_pct": round(clarity, 2),
            "adverse_conditions": warnings,
        }

    def _decision_gate(
        self,
        *,
        regime: dict[str, Any],
        risk_assessment: dict[str, Any],
        advisory: dict[str, Any],
        premarket: dict[str, Any],
    ) -> dict[str, Any]:
        reasons: list[str] = []
        warnings: list[str] = list(advisory.get("warnings") or [])
        blocked = False
        degraded = False
        action = "allow"

        if regime["states"]["risk_extreme"]:
            blocked = True
            reasons.append("regimen clasificado como riesgo extremo")
        if regime["states"]["low_liquidity"]:
            blocked = True
            reasons.append("liquidez demasiado baja para ejecutar con disciplina")
        if regime["states"]["macro_event"] and risk_assessment["clarity_score_pct"] < 70.0:
            blocked = True
            reasons.append("evento macro cercano con claridad insuficiente")
        if premarket["acceptance_state"] == "rejected":
            degraded = True
            reasons.append("la sesion actual rechaza el nivel o la direccion")
        if risk_assessment["clarity_score_pct"] < 55.0:
            blocked = True
            reasons.append("contexto ambiguo: claridad insuficiente")
        elif risk_assessment["clarity_score_pct"] < 70.0:
            degraded = True
            reasons.append("contexto util pero no limpio; exigir confirmacion extra")

        if blocked:
            action = "block"
        elif degraded:
            action = "degrade"

        return {
            "action": action,
            "blocked": blocked,
            "degraded": degraded,
            "reasons": reasons,
            "warnings": warnings,
            "confidence_pct": round(risk_assessment["clarity_score_pct"], 2),
        }

    def _context_report(
        self,
        *,
        symbol: str,
        regime: dict[str, Any],
        macro_bias: str,
        global_state: str,
        premarket: dict[str, Any],
        historical_analogies: dict[str, Any],
        risk_assessment: dict[str, Any],
        gate: dict[str, Any],
        confidence: float,
    ) -> dict[str, Any]:
        return {
            "symbol": symbol,
            "current_regime": regime["primary_regime"],
            "macro_bias": macro_bias,
            "global_market_state": global_state,
            "premarket_intraday_state": premarket["acceptance_state"],
            "historical_match": historical_analogies["matched_profile"],
            "main_risks": list(risk_assessment["adverse_conditions"]),
            "confidence_pct": round(confidence, 2),
            "permission": "block" if gate["blocked"] else "degrade" if gate["degraded"] else "allow",
        }
