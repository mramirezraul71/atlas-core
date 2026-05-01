"""Comparative shadow decisioning for paper autonomy modes.

This module keeps deterministic baseline as source-of-truth while attaching
an advisory shadow path and final resolution trace for observability/learning.
"""
from __future__ import annotations

import hashlib
from datetime import datetime, timezone
from typing import Any

try:
    from local_models.integrations import classify_journal_event
except ModuleNotFoundError:  # pragma: no cover
    from atlas_code_quant.local_models.integrations import classify_journal_event

from atlas_code_quant.operations.authority_transition_contract import (
    build_authority_transition_assessment,
)
from atlas_code_quant.operations.live_guardrails import evaluate_live_guardrails
from atlas_code_quant.operations.live_transition_realism import (
    estimate_live_transition_realism,
)


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _decision_id(symbol: str, action: str, trace_seed: str) -> str:
    raw = f"{symbol}|{action}|{trace_seed}|{_utc_now_iso()}"
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()[:20]


def _stable_short_hash(raw: str) -> str:
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()[:16]


def _policy_variant_for_mode(
    *,
    auton_mode: str,
    candidate: dict[str, Any],
    fallback_used: bool = False,
) -> str:
    mode = str(auton_mode or "").strip().lower()
    if mode == "paper_aggressive":
        score = float(candidate.get("selection_score") or 0.0)
        if score >= 85.0:
            return "high_conviction_v1"
        if fallback_used:
            return "aggressive_local_models_v1"
        return "aggressive_v1"
    return "baseline_v1"


def _cohort_id(*, policy_variant: str, session: str, regime: str) -> str:
    seed = f"{policy_variant}|{session}|{regime}"
    return f"cohort_{_stable_short_hash(seed)}"


def classify_error_taxonomy_v2(reasons: list[str]) -> dict[str, int]:
    buckets = {
        "timing_error": 0,
        "exit_error": 0,
        "validation_error": 0,
        "selection_error": 0,
        "sizing_error": 0,
        "regime_mismatch": 0,
        "advisory_conflict": 0,
        "missed_opportunity": 0,
    }
    lowered = [str(r or "").strip().lower() for r in reasons]
    for reason in lowered:
        if not reason:
            continue
        if "market_closed" in reason or "timing" in reason:
            buckets["timing_error"] += 1
        if "exit" in reason or "de_risk" in reason:
            buckets["exit_error"] += 1
        if "validation" in reason or "spread" in reason or "drift" in reason:
            buckets["validation_error"] += 1
        if "open_symbol_guard" in reason or "options_only_requires_options" in reason or "selection" in reason:
            buckets["selection_error"] += 1
            buckets["missed_opportunity"] += 1
        if "max open positions" in reason or "size" in reason or "sizing" in reason:
            buckets["sizing_error"] += 1
        if "regime" in reason:
            buckets["regime_mismatch"] += 1
        if "advisory" in reason or "local_model" in reason:
            buckets["advisory_conflict"] += 1
    return buckets


def _paper_realism_snapshot(candidate: dict[str, Any], order_seed: dict[str, Any], score: float) -> dict[str, Any]:
    asset_class = str(order_seed.get("asset_class") or candidate.get("asset_class") or "equity").lower()
    spread_hint = float(candidate.get("spread_pct") or 0.15)
    estimated_slippage_bps = max(2.0, min(40.0, spread_hint * 100.0))
    partial_fill_risk = max(0.01, min(0.75, spread_hint))
    fill_quality_assumption = "good" if score >= 80 else "normal" if score >= 65 else "degraded"
    fee_model = "paper_options_flat" if asset_class in {"option", "multileg", "combo"} else "paper_equity_flat"
    return {
        "estimated_slippage_bps": round(estimated_slippage_bps, 3),
        "fee_model": fee_model,
        "fill_quality_assumption": fill_quality_assumption,
        "partial_fill_risk": round(partial_fill_risk, 4),
    }


def _extract_visual_context(visual_payload: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(visual_payload, dict):
        return None
    ctx = visual_payload.get("visual_decision_context")
    if isinstance(ctx, dict):
        return ctx
    if not visual_payload:
        return None
    return {
        "symbol": visual_payload.get("symbol"),
        "timeframe": visual_payload.get("timeframe"),
        "chart_bias": visual_payload.get("chart_bias"),
        "pattern_signals": visual_payload.get("pattern_signals") or [],
        "confluence_score": visual_payload.get("confluence_score"),
        "confluence_bucket": visual_payload.get("confluence_bucket", "unknown"),
        "fusion_decision_reason": visual_payload.get("fusion_decision_reason"),
        "recommended_action": visual_payload.get("recommended_action", "manual_review"),
        "seasonality_context": visual_payload.get("seasonality_context") or {},
        "multi_timeframe_view": visual_payload.get("multi_timeframe_view") or {},
        "fusion_components": visual_payload.get("fusion_components") or {},
        "operational_context": {
            "navigation_ok": bool(visual_payload.get("navigation_ok", False)),
            "capture_ok": bool(visual_payload.get("capture_ok", False)),
            "symbol_verified": bool(visual_payload.get("symbol_verified", False)),
            "timeframe_verified": bool(visual_payload.get("timeframe_verified", False)),
        },
    }


def build_baseline_decision(
    *,
    candidate: dict[str, Any],
    order_seed: dict[str, Any],
    action: str,
    auton_mode: str,
    policy_variant: str = "deterministic_baseline",
    opportunity_id: str | None = None,
    trace_id: str | None = None,
    visual_context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    symbol = str(candidate.get("symbol") or order_seed.get("symbol") or "").strip().upper()
    score = float(candidate.get("selection_score") or 0.0)
    strategy_hint = (
        candidate.get("strategy_type")
        or order_seed.get("strategy_type")
        or order_seed.get("asset_class")
        or "unknown"
    )
    session = str(candidate.get("timeframe") or "intraday")
    regime = str(candidate.get("market_regime") or candidate.get("regime") or "unknown")
    variant = policy_variant
    if variant == "deterministic_baseline":
        variant = _policy_variant_for_mode(auton_mode=auton_mode, candidate=candidate, fallback_used=False)
    cohort = _cohort_id(policy_variant=variant, session=session, regime=regime)
    trace_seed = (
        str(trace_id)
        if trace_id
        else f"{symbol}|{session}|{variant}|{score:.4f}|{candidate.get('method') or 'na'}"
    )
    trace_ref = str(trace_id or f"tr_{_stable_short_hash(trace_seed)}")
    decision_id = _decision_id(symbol, action, trace_ref)
    opp_id = str(opportunity_id or f"opp_{_stable_short_hash(f'{trace_ref}|{symbol}|{session}')}")
    visual_payload = dict(visual_context or {})
    visual_ctx = _extract_visual_context(visual_payload)
    fusion_score = visual_ctx.get("confluence_score") if isinstance(visual_ctx, dict) else None
    fusion_reason = visual_ctx.get("fusion_decision_reason") if isinstance(visual_ctx, dict) else None
    recommended_action = (
        str(visual_ctx.get("recommended_action") or "manual_review")
        if isinstance(visual_ctx, dict)
        else "manual_review"
    )
    confluence_bucket = (
        str(visual_ctx.get("confluence_bucket") or "unknown")
        if isinstance(visual_ctx, dict)
        else "unknown"
    )
    seasonality_context = (
        dict(visual_ctx.get("seasonality_context") or {})
        if isinstance(visual_ctx, dict)
        else {}
    )
    multi_timeframe_view = (
        dict(visual_ctx.get("multi_timeframe_view") or {})
        if isinstance(visual_ctx, dict)
        else {}
    )
    operational_context = (
        dict(visual_ctx.get("operational_context") or {})
        if isinstance(visual_ctx, dict)
        else {}
    )
    fusion_components = (
        dict(visual_ctx.get("fusion_components") or {})
        if isinstance(visual_ctx, dict)
        else {}
    )
    return {
        "decision_id": decision_id,
        "trace_id": trace_ref,
        "opportunity_id": opp_id,
        "timestamp_utc": _utc_now_iso(),
        "symbol": symbol,
        "action": str(action),
        "auton_mode": str(auton_mode),
        "policy_variant": variant,
        "cohort_id": cohort,
        "baseline_score": score,
        "session": session,
        "regime": regime,
        "strategy_hint": str(strategy_hint),
        "paper_realism": _paper_realism_snapshot(candidate, order_seed, score),
        "visual_evidence": visual_payload,
        "visual_context": visual_ctx,
        "fusion_score": fusion_score,
        "fusion_decision_reason": fusion_reason,
        "confluence_bucket": confluence_bucket,
        "recommended_action": recommended_action,
        "seasonality_context": seasonality_context,
        "multi_timeframe_view": multi_timeframe_view,
        "operational_context": operational_context,
        "fusion_components": fusion_components,
        "baseline_reasons": ["deterministic_rank_order"],
        "candidate": {
            "selection_score": score,
            "method": candidate.get("method"),
            "timeframe": candidate.get("timeframe"),
            "direction": candidate.get("direction"),
            "local_win_rate_pct": candidate.get("local_win_rate_pct"),
        },
    }


def build_advisory_shadow(*, baseline: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    payload = {
        "event_type": "shadow_advisory_request",
        "symbol": baseline.get("symbol"),
        "action": baseline.get("action"),
        "baseline_score": baseline.get("baseline_score"),
        "strategy_hint": baseline.get("strategy_hint"),
        "candidate": {
            "selection_score": candidate.get("selection_score"),
            "direction": candidate.get("direction"),
            "method": candidate.get("method"),
            "timeframe": candidate.get("timeframe"),
        },
        "visual_evidence": baseline.get("visual_evidence") if isinstance(baseline.get("visual_evidence"), dict) else {},
    }
    advisory = classify_journal_event(payload)
    label = str(advisory.get("label") or "neutral").lower()
    confidence = float(advisory.get("confidence") or 0.0)
    # Advisory is non-authoritative in phase 1; it only proposes.
    if label == "incident" and confidence >= 0.7:
        proposed = "discard"
        rationale = "high_confidence_incident_signal"
    elif label == "trading_event":
        proposed = "accept"
        rationale = "event_alignment"
    else:
        proposed = "accept"
        rationale = "non_blocking_shadow_default"
    return {
        "decision_id": baseline.get("decision_id"),
        "trace_id": baseline.get("trace_id"),
        "opportunity_id": baseline.get("opportunity_id"),
        "timestamp_utc": _utc_now_iso(),
        "cohort_id": baseline.get("cohort_id"),
        "policy_variant": _policy_variant_for_mode(
            auton_mode=str(baseline.get("auton_mode") or "off"),
            candidate=candidate,
            fallback_used=bool(advisory.get("used_fallback", False)),
        ),
        "requested_model": advisory.get("requested_model") or advisory.get("model"),
        "provider_role": advisory.get("provider_role", "classifier"),
        "used_fallback": bool(advisory.get("used_fallback", False)),
        "fallback_reason": advisory.get("fallback_reason"),
        "label": label,
        "confidence": confidence,
        "summary": advisory.get("summary"),
        "tags": advisory.get("tags") or [],
        "proposed_decision": proposed,
        "proposed_rationale": rationale,
        "raw": advisory,
        "visual_evidence": baseline.get("visual_evidence") if isinstance(baseline.get("visual_evidence"), dict) else {},
        "visual_context": baseline.get("visual_context") if isinstance(baseline.get("visual_context"), dict) else None,
        "fusion_score": baseline.get("fusion_score"),
        "fusion_decision_reason": baseline.get("fusion_decision_reason"),
        "confluence_bucket": baseline.get("confluence_bucket", "unknown"),
        "recommended_action": baseline.get("recommended_action", "manual_review"),
        "seasonality_context": baseline.get("seasonality_context") if isinstance(baseline.get("seasonality_context"), dict) else {},
        "multi_timeframe_view": baseline.get("multi_timeframe_view") if isinstance(baseline.get("multi_timeframe_view"), dict) else {},
        "operational_context": baseline.get("operational_context") if isinstance(baseline.get("operational_context"), dict) else {},
        "fusion_components": baseline.get("fusion_components") if isinstance(baseline.get("fusion_components"), dict) else {},
    }


def resolve_final_paper_decision(
    *,
    baseline: dict[str, Any],
    advisory: dict[str, Any],
    evaluate_result: dict[str, Any] | None,
    skip_reason: str | None = None,
) -> dict[str, Any]:
    result = evaluate_result or {}
    blocked = bool(result.get("blocked", False))
    reasons = [str(r) for r in (result.get("reasons") or [])]
    if skip_reason:
        blocked = True
        reasons = [str(skip_reason)]
    if str(advisory.get("proposed_decision") or "") == "discard" and not blocked:
        reasons.append("advisory_conflict_non_blocking")
    executed = not blocked
    taxonomy = classify_error_taxonomy_v2(reasons)
    realism = baseline.get("paper_realism") if isinstance(baseline.get("paper_realism"), dict) else {}
    visual = baseline.get("visual_evidence") if isinstance(baseline.get("visual_evidence"), dict) else {}
    visual_context = baseline.get("visual_context") if isinstance(baseline.get("visual_context"), dict) else _extract_visual_context(visual)
    fusion_score = visual_context.get("confluence_score") if isinstance(visual_context, dict) else None
    fusion_reason = visual_context.get("fusion_decision_reason") if isinstance(visual_context, dict) else None
    recommended_action = (
        str(visual_context.get("recommended_action") or "manual_review")
        if isinstance(visual_context, dict)
        else "manual_review"
    )
    confluence_bucket = (
        str(visual_context.get("confluence_bucket") or "unknown")
        if isinstance(visual_context, dict)
        else "unknown"
    )
    seasonality_context = (
        dict(visual_context.get("seasonality_context") or {})
        if isinstance(visual_context, dict)
        else {}
    )
    multi_timeframe_view = (
        dict(visual_context.get("multi_timeframe_view") or {})
        if isinstance(visual_context, dict)
        else {}
    )
    operational_context = (
        dict(visual_context.get("operational_context") or {})
        if isinstance(visual_context, dict)
        else {}
    )
    fusion_components = (
        dict(visual_context.get("fusion_components") or {})
        if isinstance(visual_context, dict)
        else {}
    )
    visual_conf = float(visual.get("visual_confidence") or 0.0)
    no_visual_decision = "accept" if not blocked else "discard"
    with_visual_decision = "accept" if (not blocked and visual_conf >= 0.35) else "discard"

    modulation = {
        "recommended_action": recommended_action,
        "aggressiveness_multiplier": 1.0,
        "advisory_only": True,
    }
    if recommended_action == "reduce_aggressiveness":
        modulation["aggressiveness_multiplier"] = 0.75
    elif recommended_action == "postpone":
        modulation["aggressiveness_multiplier"] = 0.5
    elif recommended_action in {"manual_review", "discard"}:
        modulation["aggressiveness_multiplier"] = 0.35
    realism_transition = estimate_live_transition_realism(
        baseline_score=float(baseline.get("baseline_score") or 0.0),
        paper_realism=realism,
        operational_context=operational_context,
    )
    guardrail_state = evaluate_live_guardrails(
        {
            "fallback_rate": 1.0 if bool(advisory.get("used_fallback")) else 0.0,
            "visual_reliability_ratio": float(visual_conf),
            "seasonality_conflict_rate": 1.0
            if str(seasonality_context.get("seasonal_bias") or "unknown") == "hostile"
            else 0.0,
            "mtf_conflict_rate": 1.0
            if str(multi_timeframe_view.get("alignment") or "unknown") == "mixed"
            else 0.0,
            "realism_penalty_score": float(realism_transition.get("paper_live_drift_score") or 0.0),
            "error_taxonomy_rate": 1.0 if reasons else 0.0,
        }
    )
    live_risk_flags: list[str] = []
    if realism_transition.get("paper_live_drift_bucket") == "high":
        live_risk_flags.append("high_paper_live_drift")
    if guardrail_state.get("pause_conditions_triggered"):
        live_risk_flags.extend([str(x) for x in guardrail_state.get("pause_conditions_triggered") or []])
    if str(multi_timeframe_view.get("alignment") or "unknown") == "mixed":
        live_risk_flags.append("mtf_conflict")
    if str(seasonality_context.get("seasonal_bias") or "unknown") == "hostile":
        live_risk_flags.append("hostile_seasonality")

    required_operator_checks = [
        "confirm_market_microstructure",
        "confirm_liquidity_snapshot",
        "confirm_no_operational_alerts",
    ]
    live_readiness_candidate = (
        (not blocked)
        and recommended_action in {"execute", "reduce_aggressiveness"}
        and not bool(guardrail_state.get("pause_recommended"))
        and float(realism_transition.get("paper_live_drift_score") or 1.0) <= 0.45
    )
    live_readiness_reason = (
        "shadow_live_candidate_pass"
        if live_readiness_candidate
        else "shadow_live_candidate_blocked_by_guardrails_or_drift"
    )
    authority_transition = build_authority_transition_assessment(
        initiator="system",
        authority_level="paper_aggressive",
        recommendation="supervised_live_candidate" if live_readiness_candidate else "stay_paper",
        transition_reason=live_readiness_reason,
        rollback_ready=bool(guardrail_state.get("rollback_ready")),
        emergency_stop_ready=bool(guardrail_state.get("emergency_stop_ready")),
        required_human_ack=True,
        additional_checks=required_operator_checks,
    )
    final = {
        "decision_id": baseline.get("decision_id"),
        "trace_id": baseline.get("trace_id"),
        "opportunity_id": baseline.get("opportunity_id"),
        "timestamp_utc": _utc_now_iso(),
        "symbol": baseline.get("symbol"),
        "cohort_id": baseline.get("cohort_id"),
        "policy_variant": advisory.get("policy_variant") or baseline.get("policy_variant"),
        "baseline_decision": "accept" if str(baseline.get("action")) in {"preview", "submit"} else "discard",
        "advisory_decision": advisory.get("proposed_decision"),
        "advisory_rationale": advisory.get("proposed_rationale"),
        "executed_in_paper": executed,
        "final_decision": "accept" if executed else "discard",
        "reasons": reasons,
        "baseline_score": float(baseline.get("baseline_score") or 0.0),
        "session": baseline.get("session"),
        "regime": baseline.get("regime"),
        "strategy_hint": baseline.get("strategy_hint"),
        "used_fallback": bool(advisory.get("used_fallback", False)),
        "requested_model": advisory.get("requested_model"),
        "provider_role": advisory.get("provider_role"),
        "visual_confirmation": bool(visual_conf >= 0.55),
        "visual_context": visual_context,
        "fusion_score": fusion_score,
        "fusion_decision_reason": fusion_reason,
        "confluence_bucket": confluence_bucket,
        "recommended_action": recommended_action,
        "seasonality_context": seasonality_context,
        "multi_timeframe_view": multi_timeframe_view,
        "operational_context": operational_context,
        "fusion_components": fusion_components,
        "paper_advisory_modulation": modulation,
        "live_transition_realism": realism_transition,
        "live_guardrails": guardrail_state,
        "live_readiness_candidate": live_readiness_candidate,
        "live_readiness_reason": live_readiness_reason,
        "live_risk_flags": sorted(set(live_risk_flags)),
        "required_operator_checks": required_operator_checks,
        "authority_transition_assessment": authority_transition,
        "decision_without_visual": {
            "decision": no_visual_decision,
            "visual_context": None,
            "fusion_score": None,
            "fusion_decision_reason": None,
            "confluence_bucket": "unknown",
            "recommended_action": "manual_review",
            "seasonality_context": {},
            "multi_timeframe_view": {},
            "operational_context": {},
            "fusion_components": {},
            "live_readiness_candidate": False,
        },
        "decision_with_visual": {
            "decision": with_visual_decision,
            "visual_context": visual_context,
            "fusion_score": fusion_score,
            "fusion_decision_reason": fusion_reason,
            "confluence_bucket": confluence_bucket,
            "recommended_action": recommended_action,
            "seasonality_context": seasonality_context,
            "multi_timeframe_view": multi_timeframe_view,
            "operational_context": operational_context,
            "fusion_components": fusion_components,
            "live_readiness_candidate": live_readiness_candidate,
        },
        "error_taxonomy_v2": taxonomy,
        "paper_realism": {
            "estimated_slippage_bps": float(realism.get("estimated_slippage_bps") or 0.0),
            "fee_model": str(realism.get("fee_model") or "paper_flat"),
            "fill_quality_assumption": str(realism.get("fill_quality_assumption") or "normal"),
            "partial_fill_risk": float(realism.get("partial_fill_risk") or 0.0),
        },
        "visual_evidence": visual,
    }
    return final


def record_shadow_triplet(
    *,
    event_store: Any,
    baseline: dict[str, Any],
    advisory: dict[str, Any],
    final: dict[str, Any],
) -> None:
    event_store.append("decision.baseline", baseline, source="paper_shadow")
    event_store.append("decision.advisory.local_models", advisory, source="paper_shadow")
    event_store.append("decision.final", final, source="paper_shadow")
    if isinstance(final.get("authority_transition_assessment"), dict):
        event_store.append(
            "authority.transition.assessment",
            final.get("authority_transition_assessment"),
            source="paper_shadow",
        )
    if isinstance(final.get("live_transition_realism"), dict):
        event_store.append(
            "live.readiness.assessment",
            {
                "decision_id": final.get("decision_id"),
                "trace_id": final.get("trace_id"),
                "opportunity_id": final.get("opportunity_id"),
                "live_readiness_candidate": bool(final.get("live_readiness_candidate")),
                "live_readiness_reason": final.get("live_readiness_reason"),
                "live_risk_flags": final.get("live_risk_flags") or [],
                "required_operator_checks": final.get("required_operator_checks") or [],
                "realism": final.get("live_transition_realism"),
                "guardrails": final.get("live_guardrails"),
            },
            source="paper_shadow",
        )
