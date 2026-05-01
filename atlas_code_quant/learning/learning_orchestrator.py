"""LearningOrchestrator — cerebro del loop de aprendizaje cerrado de ATLAS-Quant.

Conecta todos los subsistemas de aprendizaje en un ciclo estadístico riguroso:

    1. Reconciliation (cada 5 min):
       - Detecta posiciones cerradas en el journal desde el último ciclo.
       - Cierra señales pendientes en ICSignalTracker vía fuzzy-match.
       - Construye TradeEvent y los pasa a AtlasLearningBrain.record_trade().

    2. IC Policy Update (post-reconciliation, si hubo nuevos outcomes):
       - Calcula IC por método usando Spearman rank correlation.
       - Traduce IC a multiplicadores de tamaño y umbrales de score (Grinold-Kahn).
       - Escribe PolicySnapshot actualizado en PolicyManager.

    3. Daily Analysis (una vez al día, horario configurable):
       - Ejecuta AtlasLearningBrain.run_daily_analysis().
       - Aplica políticas propuestas vía AtlasLearningBrain.update_policies().
       - Genera y guarda SystemReadinessReport (criterios para ir a live).

Marco teórico:
    - Grinold & Kahn (2000): IC como medida de calidad de señal.
    - Van Tharp (1999): R-múltiplos como unidad de aprendizaje.
    - Lopez de Prado (2018): validación OOS, Deflated Sharpe.
    - Kelly (1956) modificado: f* = IC / σ — half-Kelly para robustez.

El orquestador es NO BLOQUEANTE: todos los errores son capturados y
logeados sin interrumpir el auto-cycle principal.
"""
from __future__ import annotations

import asyncio
import logging
import math
import uuid
from datetime import date, datetime, timedelta, timezone
from typing import Any

logger = logging.getLogger(__name__)

# ── Umbrales IC (Grinold-Kahn) ────────────────────────────────────────────────
_IC_PAUSE_THRESHOLD = 0.02       # IC < 0.02 → pausa método (sin edge)
_IC_MEANINGFUL = 0.05            # IC >= 0.05 → edge operativo mínimo
_IC_STRONG = 0.10                # IC >= 0.10 → escalar posición

# ── Kelly fractions (half-Kelly para robustez) ────────────────────────────────
_SIZE_MULTIPLIER_PAUSE = 0.0     # Método sin evidencia
_SIZE_MULTIPLIER_WEAK = 0.5      # Edge por debajo del umbral mínimo
_SIZE_MULTIPLIER_NORMAL = 1.0    # Edge confirmado
_SIZE_MULTIPLIER_STRONG = 1.5    # Edge fuerte (capped en 1.5 por gestión de riesgo)

# ── Score thresholds para filtrar señales del scanner ─────────────────────────
_SCORE_THRESHOLD_PAUSE = 0.82    # Muy exigente — casi no pasa señales
_SCORE_THRESHOLD_TIGHT = 0.72    # Exigente — solo señales fuertes
_SCORE_THRESHOLD_NORMAL = 0.62   # Normal
_SCORE_THRESHOLD_RELAXED = 0.55  # Relajado — IC fuerte, aceptar más oportunidades

# ── Estado global del orquestador ────────────────────────────────────────────
_STATE: dict[str, Any] = {
    "last_reconcile_at": None,
    "last_daily_analysis_at": None,
    "last_daily_analysis_date": None,
    "reconcile_count": 0,
    "trades_processed_total": 0,
    "ic_updates_total": 0,
    "policy_updates": 0,
    "last_ic_by_method": {},
    "last_readiness_report": None,
    "last_reconciliation_status": None,
    "last_advisory_uplift": None,
    "running": False,
    "errors": [],
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _utcnow() -> datetime:
    return datetime.now(timezone.utc)


def _safe_float(v: Any, default: float = 0.0) -> float:
    try:
        out = float(v)
        if math.isnan(out) or math.isinf(out):
            return default
        return out
    except (TypeError, ValueError):
        return default


def _ic_to_size_multiplier(ic: float | None) -> float:
    """Traduce IC a multiplicador de tamaño de posición (half-Kelly)."""
    if ic is None:
        return _SIZE_MULTIPLIER_WEAK
    if ic < _IC_PAUSE_THRESHOLD:
        return _SIZE_MULTIPLIER_PAUSE
    if ic < _IC_MEANINGFUL:
        return _SIZE_MULTIPLIER_WEAK
    if ic < _IC_STRONG:
        return _SIZE_MULTIPLIER_NORMAL
    return _SIZE_MULTIPLIER_STRONG


def _ic_to_score_threshold(ic: float | None, n_obs: int) -> float:
    """Traduce IC a umbral de score mínimo para aceptar señales del scanner."""
    # Con n < 10, ser conservador independientemente del IC
    if n_obs < 10:
        return _SCORE_THRESHOLD_TIGHT
    if ic is None or ic < _IC_PAUSE_THRESHOLD:
        return _SCORE_THRESHOLD_PAUSE
    if ic < _IC_MEANINGFUL:
        return _SCORE_THRESHOLD_TIGHT
    if ic < _IC_STRONG:
        return _SCORE_THRESHOLD_NORMAL
    return _SCORE_THRESHOLD_RELAXED


def _build_trade_event_from_journal(entry: Any) -> Any | None:
    """Construye TradeEvent desde una fila de TradingJournal.

    Retorna None si faltan campos críticos.
    """
    try:
        from atlas_code_quant.learning.trade_events import TradeEvent

        entry_price = _safe_float(entry.entry_price)
        exit_price = _safe_float(entry.exit_price)
        risk_at_entry = _safe_float(entry.risk_at_entry)

        if entry_price <= 0 or exit_price <= 0:
            return None

        # R-inicial: riesgo absoluto por unidad de precio
        r_initial = abs(risk_at_entry) if risk_at_entry > 0 else abs(entry_price * 0.02)

        # R realizado: ganancia/pérdida normalizada por R
        price_delta = exit_price - entry_price
        # Para shorts: invertir el signo
        side = str(entry.strategy_type or "equity_long").lower()
        is_short = any(x in side for x in ("short", "put", "bear"))
        if is_short:
            price_delta = -price_delta
        r_realized = price_delta / r_initial if r_initial > 0 else 0.0

        entry_time = entry.entry_time
        if entry_time and entry_time.tzinfo is None:
            entry_time = entry_time.replace(tzinfo=timezone.utc)
        exit_time = entry.exit_time
        if exit_time and exit_time.tzinfo is None:
            exit_time = exit_time.replace(tzinfo=timezone.utc)

        if not entry_time or not exit_time:
            return None

        # Determinar asset_class desde strategy_type
        st = str(entry.strategy_type or "equity_long").lower()
        if "option" in st or "call" in st or "put" in st or "spread" in st or "condor" in st:
            asset_class = "option"
        elif "future" in st:
            asset_class = "future"
        elif "crypto" in st:
            asset_class = "crypto"
        else:
            asset_class = "equity"

        trade_side = "sell" if is_short else "buy"

        return TradeEvent(
            trade_id=str(entry.journal_key or uuid.uuid4()),
            symbol=str(entry.symbol or "UNKNOWN").upper(),
            asset_class=asset_class,
            side=trade_side,
            entry_time=entry_time,
            exit_time=exit_time,
            timeframe="1d",
            entry_price=entry_price,
            exit_price=exit_price,
            stop_loss_price=entry_price - r_initial if trade_side == "buy" else entry_price + r_initial,
            r_initial=r_initial,
            r_realized=round(r_realized, 4),
            mae_r=0.0,   # No disponible sin tick history
            mfe_r=0.0,
            setup_type=str(entry.strategy_type or "unknown"),
            regime="UNKNOWN",
            exit_type="closed",
            iv_rank=_safe_float(entry.iv_rank),
            capital_at_entry=_safe_float(entry.entry_notional, 100_000.0),
            position_size=_safe_float(entry.entry_notional),
            commission=_safe_float(entry.fees),
            notes=str(entry.post_mortem_text or "")[:200],
        )
    except Exception as e:
        logger.debug("[orchestrator] build_trade_event failed: %s", e)
        return None


def _reconciliation_status_for_learning_gate() -> str:
    """Resolve reconciliation status for learning gate logging."""
    try:
        from operations.operation_center import OperationCenter
    except Exception:
        try:
            from atlas_code_quant.operations.operation_center import OperationCenter
        except Exception:
            return "UNKNOWN"

    try:
        snapshot = OperationCenter().status_lite()
    except Exception:
        logger.debug("[orchestrator] unable to pull operation status for reconciliation gate", exc_info=True)
        return "UNKNOWN"

    monitor = snapshot.get("monitor_summary") or {}
    reconciliation = monitor.get("reconciliation") or {}
    raw_status = (
        reconciliation.get("reconciliation_status")
        or reconciliation.get("status")
        or reconciliation.get("state")
        or ""
    )
    normalized = str(raw_status).strip().lower()
    if normalized in {"ok", "healthy"}:
        return "OK"
    if not normalized:
        return "UNKNOWN"
    return normalized.upper()


def compute_advisory_uplift(window_days: int = 7) -> dict[str, Any]:
    """Compute baseline/advisory/final paper decision deltas from EventStore."""
    from atlas_code_quant.learning.event_store import get_event_store

    since = (_utcnow() - timedelta(days=max(1, int(window_days)))).isoformat()
    events = get_event_store().query(topic="decision.final", since=since, limit=5000)
    total = 0
    executed = 0
    baseline_accept = 0
    advisory_accept = 0
    fallback_used = 0
    traceable = 0
    visual_confirmed = 0
    visual_decisions = 0
    by_path: dict[str, dict[str, float]] = {}
    setup_stats: dict[str, dict[str, int]] = {}
    cohort_stats: dict[str, dict[str, Any]] = {}
    reason_buckets = {
        "timing_error": 0,
        "exit_error": 0,
        "validation_error": 0,
        "selection_error": 0,
        "sizing_error": 0,
        "regime_mismatch": 0,
        "advisory_conflict": 0,
        "missed_opportunity": 0,
    }
    confluence_bucket_counts = {"high": 0, "medium": 0, "low": 0, "unknown": 0}
    seasonal_bias_counts = {"favorable": 0, "neutral": 0, "hostile": 0, "unknown": 0}
    mtf_alignment_counts = {
        "aligned_bullish": 0,
        "aligned_bearish": 0,
        "mixed": 0,
        "unknown": 0,
    }
    recommended_action_counts = {
        "execute": 0,
        "reduce_aggressiveness": 0,
        "postpone": 0,
        "discard": 0,
        "manual_review": 0,
    }
    outcome_by_seasonal_bias: dict[str, dict[str, float]] = {}
    outcome_by_mtf_alignment: dict[str, dict[str, float]] = {}
    outcome_by_recommended_action: dict[str, dict[str, float]] = {}
    for ev in events:
        data = ev.get("data") if isinstance(ev.get("data"), dict) else {}
        total += 1
        if str(data.get("baseline_decision") or "") == "accept":
            baseline_accept += 1
        if str(data.get("advisory_decision") or "") == "accept":
            advisory_accept += 1
        if bool(data.get("executed_in_paper")):
            executed += 1
        if bool(data.get("used_fallback")):
            fallback_used += 1
        if data.get("visual_confirmation") is not None:
            visual_decisions += 1
            if bool(data.get("visual_confirmation")):
                visual_confirmed += 1
        if data.get("opportunity_id") and data.get("trace_id") and data.get("decision_id"):
            traceable += 1
        confluence_bucket = str(
            data.get("confluence_bucket")
            or ((data.get("visual_context") or {}).get("confluence_bucket") if isinstance(data.get("visual_context"), dict) else None)
            or ((data.get("visual_evidence") or {}).get("confluence_bucket") if isinstance(data.get("visual_evidence"), dict) else None)
            or "unknown"
        ).lower()
        if confluence_bucket not in confluence_bucket_counts:
            confluence_bucket = "unknown"
        confluence_bucket_counts[confluence_bucket] += 1
        seasonal_bias = str(
            data.get("seasonality_context", {}).get("seasonal_bias")
            if isinstance(data.get("seasonality_context"), dict)
            else (
                (data.get("visual_context") or {}).get("seasonality_context", {}).get("seasonal_bias")
                if isinstance(data.get("visual_context"), dict)
                else "unknown"
            )
        ).lower()
        if seasonal_bias not in seasonal_bias_counts:
            seasonal_bias = "unknown"
        seasonal_bias_counts[seasonal_bias] += 1

        mtf_alignment = str(
            data.get("multi_timeframe_view", {}).get("alignment")
            if isinstance(data.get("multi_timeframe_view"), dict)
            else (
                (data.get("visual_context") or {}).get("multi_timeframe_view", {}).get("alignment")
                if isinstance(data.get("visual_context"), dict)
                else "unknown"
            )
        ).lower()
        if mtf_alignment not in mtf_alignment_counts:
            mtf_alignment = "unknown"
        mtf_alignment_counts[mtf_alignment] += 1

        recommended_action = str(data.get("recommended_action") or "manual_review").lower()
        if recommended_action not in recommended_action_counts:
            recommended_action = "manual_review"
        recommended_action_counts[recommended_action] += 1

        for key, table, label in (
            (seasonal_bias, outcome_by_seasonal_bias, "seasonal_bias"),
            (mtf_alignment, outcome_by_mtf_alignment, "mtf_alignment"),
            (recommended_action, outcome_by_recommended_action, "recommended_action"),
        ):
            row = table.setdefault(key, {"segment": key, "n": 0.0, "exec": 0.0, "label": label})
            row["n"] += 1.0
            if bool(data.get("executed_in_paper")):
                row["exec"] += 1.0
        strategy = str(data.get("strategy_hint") or data.get("strategy") or "unknown")
        ss = setup_stats.setdefault(strategy, {"n": 0, "exec": 0})
        ss["n"] += 1
        if bool(data.get("executed_in_paper")):
            ss["exec"] += 1
        tax = data.get("error_taxonomy_v2")
        if isinstance(tax, dict):
            for bucket in reason_buckets:
                reason_buckets[bucket] += max(0, int(tax.get(bucket) or 0))
        else:
            for r in [str(x).lower() for x in (data.get("reasons") or [])]:
                if "timing" in r or "market_closed" in r:
                    reason_buckets["timing_error"] += 1
                if "exit" in r or "de_risk" in r:
                    reason_buckets["exit_error"] += 1
                if "validation" in r or "spread" in r or "drift" in r:
                    reason_buckets["validation_error"] += 1
                if "open_symbol_guard" in r or "options_only_requires_options" in r:
                    reason_buckets["selection_error"] += 1
                    reason_buckets["missed_opportunity"] += 1
                if "size" in r or "max open positions" in r:
                    reason_buckets["sizing_error"] += 1
                if "regime" in r:
                    reason_buckets["regime_mismatch"] += 1
                if "advisory" in r:
                    reason_buckets["advisory_conflict"] += 1
        cohort_id = str(data.get("cohort_id") or "cohort_unknown")
        policy_variant = str(data.get("policy_variant") or "baseline_v1")
        regime = str(data.get("regime") or "unknown")
        cohort_key = f"{policy_variant}|{cohort_id}|{regime}"
        cstats = cohort_stats.setdefault(
            cohort_key,
            {"policy_variant": policy_variant, "cohort_id": cohort_id, "regime": regime, "n": 0, "exec": 0},
        )
        cstats["n"] += 1
        if bool(data.get("executed_in_paper")):
            cstats["exec"] += 1
        path = str(data.get("requested_model") or "none")
        row = by_path.setdefault(path, {"n": 0.0, "exec": 0.0})
        row["n"] += 1.0
        if bool(data.get("executed_in_paper")):
            row["exec"] += 1.0
    baseline_rate = (baseline_accept / total) if total > 0 else 0.0
    advisory_rate = (advisory_accept / total) if total > 0 else 0.0
    exec_rate = (executed / total) if total > 0 else 0.0
    by_model_path = {
        model: ((vals["exec"] / vals["n"]) - baseline_rate) if vals["n"] > 0 else 0.0
        for model, vals in by_path.items()
    }
    cohort_metrics = []
    for row in cohort_stats.values():
        n = int(row["n"])
        exec_rate_local = (int(row["exec"]) / n) if n > 0 else 0.0
        cohort_metrics.append(
            {
                "policy_variant": row["policy_variant"],
                "cohort_id": row["cohort_id"],
                "regime": row["regime"],
                "samples": n,
                "execution_rate": exec_rate_local,
                "delta_vs_baseline": exec_rate_local - baseline_rate,
            }
        )
    cohort_metrics.sort(key=lambda item: (item["delta_vs_baseline"], item["samples"]), reverse=True)
    setup_ranked = sorted(
        (
            {
                "strategy": strategy,
                "samples": vals["n"],
                "execution_rate": (vals["exec"] / vals["n"]) if vals["n"] else 0.0,
            }
            for strategy, vals in setup_stats.items()
        ),
        key=lambda row: (row["execution_rate"], row["samples"]),
        reverse=True,
    )
    return {
        "window_days": max(1, int(window_days)),
        "total_decisions": total,
        "executed_total": executed,
        "baseline_accept_rate": baseline_rate,
        "advisory_accept_rate": advisory_rate,
        "execution_rate": exec_rate,
        "advisory_uplift_ratio": advisory_rate - baseline_rate,
        "fallback_rate": (fallback_used / total) if total > 0 else 0.0,
        "traceability_coverage_ratio": (traceable / total) if total > 0 else 0.0,
        "visual_confirmation_ratio": (visual_confirmed / visual_decisions) if visual_decisions > 0 else 0.0,
        "by_model_path": by_model_path,
        "cohort_metrics": cohort_metrics,
        "confluence_bucket_counts": confluence_bucket_counts,
        "seasonal_bias_counts": seasonal_bias_counts,
        "mtf_alignment_counts": mtf_alignment_counts,
        "recommended_action_counts": recommended_action_counts,
        "outcome_by_seasonal_bias": [
            {
                "seasonal_bias": key,
                "samples": int(vals["n"]),
                "execution_rate": (vals["exec"] / vals["n"]) if vals["n"] > 0 else 0.0,
            }
            for key, vals in outcome_by_seasonal_bias.items()
        ],
        "outcome_by_mtf_alignment": [
            {
                "mtf_alignment": key,
                "samples": int(vals["n"]),
                "execution_rate": (vals["exec"] / vals["n"]) if vals["n"] > 0 else 0.0,
            }
            for key, vals in outcome_by_mtf_alignment.items()
        ],
        "outcome_by_recommended_action": [
            {
                "recommended_action": key,
                "samples": int(vals["n"]),
                "execution_rate": (vals["exec"] / vals["n"]) if vals["n"] > 0 else 0.0,
            }
            for key, vals in outcome_by_recommended_action.items()
        ],
        "strong_setups": setup_ranked[:3],
        "weak_setups": list(reversed(setup_ranked[-3:])),
        "error_buckets": reason_buckets,
    }


def build_transition_readiness_insights(uplift_payload: dict[str, Any] | None) -> dict[str, Any]:
    payload = dict(uplift_payload or {})
    total = int(payload.get("total_decisions") or 0)
    confluence = payload.get("confluence_bucket_counts") if isinstance(payload.get("confluence_bucket_counts"), dict) else {}
    seasonality = payload.get("seasonal_bias_counts") if isinstance(payload.get("seasonal_bias_counts"), dict) else {}
    mtf = payload.get("mtf_alignment_counts") if isinstance(payload.get("mtf_alignment_counts"), dict) else {}
    actions = payload.get("recommended_action_counts") if isinstance(payload.get("recommended_action_counts"), dict) else {}
    errors = payload.get("error_buckets") if isinstance(payload.get("error_buckets"), dict) else {}
    fallback_rate = float(payload.get("fallback_rate") or 0.0)

    def _ratio(count: float | int) -> float:
        if total <= 0:
            return 0.0
        return float(count) / float(total)

    visual_reliability = 1.0 - _ratio(confluence.get("unknown", 0))
    temporal_consistency = 1.0 - _ratio(errors.get("timing_error", 0))
    seasonality_conflict_rate = _ratio(seasonality.get("hostile", 0))
    mtf_conflict_rate = _ratio(mtf.get("mixed", 0))
    low_confluence_execution_rate = _ratio(confluence.get("low", 0))
    hostile_execution_rate = seasonality_conflict_rate
    mixed_execution_rate = mtf_conflict_rate
    realism_penalty_score = min(
        1.0,
        (fallback_rate * 0.35)
        + (seasonality_conflict_rate * 0.25)
        + (mtf_conflict_rate * 0.20)
        + (low_confluence_execution_rate * 0.20),
    )
    execution_ratio = float(payload.get("execution_rate") or 0.0)
    runtime_stability_score = max(0.0, 1.0 - float(payload.get("fallback_rate") or 0.0))
    recommended_manual_ratio = _ratio(actions.get("manual_review", 0))
    return {
        "runtime_stability_score": round(runtime_stability_score, 4),
        "visual_reliability_ratio": round(visual_reliability, 4),
        "temporal_consistency_score": round(temporal_consistency, 4),
        "seasonality_conflict_rate": round(seasonality_conflict_rate, 4),
        "mtf_conflict_rate": round(mtf_conflict_rate, 4),
        "low_confluence_execution_rate": round(low_confluence_execution_rate, 4),
        "seasonal_hostile_execution_rate": round(hostile_execution_rate, 4),
        "mtf_mixed_execution_rate": round(mixed_execution_rate, 4),
        "realism_penalty_score": round(realism_penalty_score, 4),
        "error_taxonomy_rate": round(_ratio(sum(float(v or 0.0) for v in errors.values())), 4),
        "manual_review_ratio": round(recommended_manual_ratio, 4),
        "execution_ratio": round(execution_ratio, 4),
        "transition_ready_variants": [
            row.get("policy_variant")
            for row in (payload.get("cohort_metrics") or [])
            if isinstance(row, dict) and float(row.get("delta_vs_baseline") or 0.0) > 0.03 and int(row.get("samples") or 0) >= 20
        ],
    }


def build_live_readiness_events(
    *,
    promotion: dict[str, Any],
    readiness_insights: dict[str, Any],
    guardrails: dict[str, Any],
) -> list[dict[str, Any]]:
    events = [
        {
            "topic": "live.readiness.assessment",
            "data": {
                "insights": readiness_insights,
                "guardrails": guardrails,
                "recommendation": promotion.get("recommendation"),
                "ready_for_supervised_live": bool(promotion.get("ready_for_supervised_live")),
                "ready_for_guarded_live": bool(promotion.get("ready_for_guarded_live")),
                "ready_for_full_live": bool(promotion.get("ready_for_full_live")),
            },
        }
    ]
    if bool(promotion.get("ready_for_supervised_live")) or bool(promotion.get("ready_for_guarded_live")):
        events.append(
            {
                "topic": "live.readiness.ready_candidate",
                "data": {
                    "current_stage": promotion.get("current_stage"),
                    "target_stage": promotion.get("target_stage"),
                    "recommendation": promotion.get("recommendation"),
                    "readiness_score": (promotion.get("scorecard") or {}).get("readiness_score"),
                    "blocking_reasons": promotion.get("blocking_reasons") or [],
                    "warnings": promotion.get("warnings") or [],
                },
            }
        )
    return events


# ── Reconciliación de posiciones cerradas ─────────────────────────────────────

async def reconcile_closed_positions() -> dict[str, Any]:
    """Detecta posiciones cerradas en el journal y cierra sus señales IC.

    Consulta TradingJournal por entradas con status='closed', exit_price IS NOT NULL
    y que no hayan sido procesadas en los últimos 7 días.

    Retorna resumen de la reconciliación.
    """
    from atlas_code_quant.journal.db import session_scope
    from atlas_code_quant.journal.models import TradingJournal
    from atlas_code_quant.learning.ic_signal_tracker import get_ic_tracker
    from atlas_code_quant.learning.atlas_learning_brain import AtlasLearningBrain
    from sqlalchemy import select

    ic_tracker = get_ic_tracker()
    brain = _get_learning_brain()

    cutoff = _utcnow() - timedelta(days=7)
    reconciled = 0
    ic_updated = 0
    brain_fed = 0
    errors_local: list[str] = []

    try:
        async def _query() -> list[Any]:
            return await asyncio.to_thread(_query_closed_sync, cutoff)

        closed_entries = await _query()
    except Exception as e:
        logger.warning("[orchestrator] journal query failed: %s", e)
        return {"reconciled": 0, "ic_updated": 0, "brain_fed": 0, "error": str(e)}

    for entry in closed_entries:
        try:
            exit_price = _safe_float(entry.exit_price)
            if exit_price <= 0:
                continue

            # IC Tracker: cerrar señal pendiente por fuzzy match
            updated_sid = ic_tracker.update_pending_outcome(
                symbol=str(entry.symbol).upper(),
                method=str(entry.strategy_type or "unknown"),
                entry_price=_safe_float(entry.entry_price),
                recorded_near=entry.entry_time.isoformat() if entry.entry_time else None,
                exit_price=exit_price,
            )
            if updated_sid:
                ic_updated += 1
                logger.debug(
                    "[orchestrator] IC signal closed: %s %s → signal_id=%s",
                    entry.symbol, entry.strategy_type, updated_sid,
                )

            # AtlasLearningBrain: construir TradeEvent y registrar
            trade_event = _build_trade_event_from_journal(entry)
            if trade_event:
                await asyncio.to_thread(brain.record_trade, trade_event)
                brain_fed += 1

            reconciled += 1

        except Exception as e:
            errors_local.append(f"{entry.symbol}: {e}")
            logger.debug("[orchestrator] entry reconcile error: %s", e)

    return {
        "closed_found": len(closed_entries),
        "reconciled": reconciled,
        "ic_updated": ic_updated,
        "brain_fed": brain_fed,
        "errors": len(errors_local),
    }


def _query_closed_sync(cutoff: datetime) -> list[Any]:
    """Consulta sincrónica al journal de posiciones cerradas."""
    from atlas_code_quant.journal.db import session_scope
    from atlas_code_quant.journal.models import TradingJournal
    from sqlalchemy import select

    results = []
    with session_scope() as sess:
        stmt = (
            select(TradingJournal)
            .where(
                TradingJournal.status == "closed",
                TradingJournal.exit_price.is_not(None),
                TradingJournal.exit_time >= cutoff,
            )
            .order_by(TradingJournal.exit_time.asc())
            .limit(200)
        )
        rows = sess.execute(stmt).scalars().all()
        # Necesitamos los datos ANTES de cerrar la sesión
        for row in rows:
            results.append(_journal_snapshot(row))
    return results


class _JournalSnapshot:
    """Snapshot inmutable de una fila de TradingJournal."""
    __slots__ = (
        "journal_key", "symbol", "strategy_type", "side",
        "entry_price", "exit_price", "entry_time", "exit_time",
        "entry_notional", "risk_at_entry", "realized_pnl",
        "fees", "iv_rank", "post_mortem_text",
    )

    def __init__(self, row: Any) -> None:
        self.journal_key = row.journal_key
        self.symbol = row.symbol
        self.strategy_type = row.strategy_type
        self.side = getattr(row, "side", "buy")
        self.entry_price = row.entry_price
        self.exit_price = row.exit_price
        self.entry_time = row.entry_time
        self.exit_time = row.exit_time
        self.entry_notional = row.entry_notional
        self.risk_at_entry = row.risk_at_entry
        self.realized_pnl = row.realized_pnl
        self.fees = row.fees
        self.iv_rank = row.iv_rank
        self.post_mortem_text = row.post_mortem_text


def _journal_snapshot(row: Any) -> _JournalSnapshot:
    return _JournalSnapshot(row)


# ── Aplicar IC a las políticas ────────────────────────────────────────────────

def apply_ic_to_policy() -> dict[str, Any]:
    """Calcula IC por método y actualiza PolicySnapshot en PolicyManager.

    Implementa el framework Grinold-Kahn:
    - IC < 0.02 → pausa método (size = 0, threshold elevado)
    - IC [0.02, 0.05) → reducción (half-Kelly)
    - IC [0.05, 0.10) → normal
    - IC >= 0.10 → scale-up (capped en 1.5x)

    También ajusta score_thresholds: si el IC es bajo, exigimos señales
    con score más alto para compensar la incertidumbre estadística.
    """
    from atlas_code_quant.learning.ic_signal_tracker import get_ic_tracker
    from atlas_code_quant.learning.policy_manager import PolicyManager
    from atlas_code_quant.learning.trade_events import PolicySnapshot, PolicyAction
    from pathlib import Path

    tracker = get_ic_tracker()
    summary = tracker.summary()
    by_method = summary.get("by_method") or {}

    if not by_method:
        return {"updated": False, "reason": "no IC data by method yet"}

    size_multipliers: dict[str, float] = {}
    score_thresholds: dict[str, float] = {}
    disabled_setups: list[str] = []
    enabled_setups: list[str] = []
    policy_log: list[str] = []

    for method, ic_result in by_method.items():
        ic_val = ic_result.get("ic")
        n_obs = ic_result.get("n_observations", 0)
        status = ic_result.get("ic_status", "insufficient_data")

        size_mult = _ic_to_size_multiplier(ic_val)
        score_thresh = _ic_to_score_threshold(ic_val, n_obs)

        size_multipliers[method] = size_mult
        score_thresholds[method] = score_thresh

        if size_mult == 0.0 and n_obs >= 10:
            disabled_setups.append(method)
            policy_log.append(
                f"PAUSA {method}: IC={ic_val:.4f} n={n_obs} — sin edge estadístico"
            )
        elif size_mult >= _SIZE_MULTIPLIER_NORMAL:
            enabled_setups.append(method)
            policy_log.append(
                f"OK {method}: IC={ic_val:.4f} n={n_obs} → size×{size_mult:.1f}"
            )
        else:
            policy_log.append(
                f"REDUCIDO {method}: IC={ic_val:.4f} n={n_obs} → size×{size_mult:.1f} threshold={score_thresh:.2f}"
            )

    # Cargar PolicyManager y actualizar snapshot
    policy_path = _get_policy_path()
    pm = PolicyManager(storage_path=policy_path)
    current = pm.get_snapshot()

    updated_snapshot = PolicySnapshot(
        enabled_setups=enabled_setups or current.enabled_setups,
        disabled_setups=disabled_setups,
        size_multipliers={**current.size_multipliers, **size_multipliers},
        score_thresholds={**current.score_thresholds, **score_thresholds},
        global_score_threshold=current.global_score_threshold,
        score_boosts=current.score_boosts,
        manual_review_setups=current.manual_review_setups,
        last_updated=_utcnow(),
        last_update_source="ic_orchestrator",
    )
    pm.apply_snapshot(updated_snapshot)

    _STATE["last_ic_by_method"] = {
        m: {"ic": r.get("ic"), "n": r.get("n_observations"), "size_mult": size_multipliers.get(m)}
        for m, r in by_method.items()
    }
    _STATE["policy_updates"] += 1

    return {
        "updated": True,
        "methods_updated": list(size_multipliers.keys()),
        "disabled_setups": disabled_setups,
        "policy_log": policy_log,
        "size_multipliers": size_multipliers,
        "score_thresholds": score_thresholds,
    }


# ── Análisis diario ───────────────────────────────────────────────────────────

async def run_daily_analysis() -> dict[str, Any]:
    """Ejecuta el análisis diario completo: métricas, patrones, readiness."""
    brain = _get_learning_brain()
    from atlas_code_quant.learning.event_store import get_event_store
    from atlas_code_quant.operations.authority_transition_contract import (
        build_authority_transition_assessment,
    )
    from atlas_code_quant.operations.live_guardrails import evaluate_live_guardrails
    from atlas_code_quant.operations.promotion_framework import evaluate_promotion_stage

    try:
        report = await asyncio.to_thread(brain.run_daily_analysis)
        policies_applied = await asyncio.to_thread(brain.update_policies, report)

        readiness = await asyncio.to_thread(brain.is_system_ready_for_live)

        _STATE["last_daily_analysis_at"] = _utcnow().isoformat()
        _STATE["last_daily_analysis_date"] = date.today().isoformat()
        uplift = _STATE.get("last_advisory_uplift") if isinstance(_STATE.get("last_advisory_uplift"), dict) else {}
        readiness_insights = build_transition_readiness_insights(uplift)
        promotion = evaluate_promotion_stage(
            metrics={
                "n_trades_analyzed": report.n_trades_analyzed,
                "n_trades_evaluated": readiness.n_trades_evaluated,
                "profit_factor": report.global_metrics.profit_factor,
                "stability_score": report.stability_score,
                "win_rate_pct": float(getattr(report.global_metrics, "win_rate_pct", 0.0) or 0.0),
                "fallback_rate": float(
                    ((uplift or {}).get("fallback_rate", 0.0))  # backward-safe
                ),
                "cohort_metrics": ((uplift or {}).get("cohort_metrics") or []),
                **readiness_insights,
            },
            stage="paper_aggressive",
        )
        guardrail_snapshot = evaluate_live_guardrails(
            {
                "fallback_rate": float((uplift or {}).get("fallback_rate", 0.0)),
                "visual_reliability_ratio": float(readiness_insights.get("visual_reliability_ratio", 0.0)),
                "seasonality_conflict_rate": float(readiness_insights.get("seasonality_conflict_rate", 0.0)),
                "mtf_conflict_rate": float(readiness_insights.get("mtf_conflict_rate", 0.0)),
                "realism_penalty_score": float(readiness_insights.get("realism_penalty_score", 0.0)),
                "error_taxonomy_rate": float(readiness_insights.get("error_taxonomy_rate", 0.0)),
            }
        )
        authority_assessment = build_authority_transition_assessment(
            initiator="system",
            authority_level="paper_aggressive",
            recommendation=str(promotion.get("recommendation") or "stay_paper"),
            transition_reason="daily_promotion_assessment",
            rollback_ready=bool(guardrail_snapshot.get("rollback_ready")),
            emergency_stop_ready=bool(guardrail_snapshot.get("emergency_stop_ready")),
            required_human_ack=True,
            additional_checks=["operator_ack_required", "guardrail_pause_conditions_review"],
        )
        _STATE["last_readiness_report"] = {
            "is_ready": readiness.ready,
            "criteria_passed": readiness.passed,
            "criteria_failed": readiness.failed,
            "n_trades_evaluated": readiness.n_trades_evaluated,
            "summary": readiness.summary,
            "next_step": readiness.next_step,
            "promotion_framework": promotion,
            "guardrails": guardrail_snapshot,
            "authority_transition_assessment": authority_assessment,
            "transition_readiness_insights": readiness_insights,
        }
        try:
            es = get_event_store()
            es.append(
                "promotion.scorecard",
                {
                    "current_stage": promotion.get("current_stage"),
                    "target_stage": promotion.get("target_stage"),
                    "recommendation": promotion.get("recommendation"),
                    "scorecard": promotion.get("scorecard"),
                    "passed_checks": promotion.get("passed_checks"),
                    "failed_checks": promotion.get("failed_checks"),
                    "warnings": promotion.get("warnings"),
                },
                source="learning_orchestrator",
            )
            es.append(
                "authority.transition.assessment",
                authority_assessment,
                source="learning_orchestrator",
            )
            for ev in build_live_readiness_events(
                promotion=promotion,
                readiness_insights=readiness_insights,
                guardrails=guardrail_snapshot,
            ):
                es.append(
                    ev["topic"],
                    ev["data"],
                    source="learning_orchestrator",
                )
        except Exception:
            logger.debug("[orchestrator] event_store append for transition readiness failed", exc_info=True)

        logger.info(
            "[orchestrator] daily_analysis: n_trades=%d PF=%.2f stability=%.2f "
            "policies=%d readiness=%s",
            report.n_trades_analyzed,
            report.global_metrics.profit_factor,
            report.stability_score,
            len(report.proposed_policies),
            readiness.ready,
        )

        return {
            "n_trades_analyzed": report.n_trades_analyzed,
            "profit_factor": report.global_metrics.profit_factor,
            "stability_score": report.stability_score,
            "proposed_policies": len(report.proposed_policies),
            "n_trades_evaluated": readiness.n_trades_evaluated,
            "is_ready_for_live": readiness.ready,
            "promotion_framework": promotion,
            "transition_readiness_insights": readiness_insights,
            "authority_transition_assessment": authority_assessment,
            "guardrails": guardrail_snapshot,
        }
    except Exception as e:
        logger.warning("[orchestrator] daily_analysis failed: %s", e)
        return {"error": str(e)}


# ── Loop principal ────────────────────────────────────────────────────────────

async def run_learning_loop(
    reconcile_interval_sec: int = 300,
    daily_analysis_hour_utc: int = 21,   # ~17:00 ET = 21:00 UTC
) -> None:
    """Loop de aprendizaje autónomo. Se ejecuta como tarea de fondo asyncio.

    Args:
        reconcile_interval_sec: Cada cuántos segundos reconciliar (default 5 min).
        daily_analysis_hour_utc: Hora UTC para disparar análisis diario (default 21h = 17h ET).
    """
    _STATE["running"] = True
    logger.info(
        "[orchestrator] Learning loop iniciado: reconcile_interval=%ds daily_hour=%dUTC",
        reconcile_interval_sec, daily_analysis_hour_utc,
    )

    while _STATE["running"]:
        try:
            # ── Reconciliación ────────────────────────────────────────────
            recon_result = await reconcile_closed_positions()
            _STATE["last_reconcile_at"] = _utcnow().isoformat()
            _STATE["reconcile_count"] += 1
            _STATE["trades_processed_total"] += recon_result.get("reconciled", 0)
            _STATE["ic_updates_total"] += recon_result.get("ic_updated", 0)
            reconciliation_status = _reconciliation_status_for_learning_gate()
            _STATE["last_reconciliation_status"] = reconciliation_status
            if reconciliation_status != "OK":
                logger.error("Learning blocked: reconciliation_status = %s", reconciliation_status)
            else:
                logger.info("Reconciliation OK, learning enabled")

            if recon_result.get("ic_updated", 0) > 0:
                # Hubo nuevos outcomes → actualizar políticas IC
                ic_result = await asyncio.to_thread(apply_ic_to_policy)
                logger.info(
                    "[orchestrator] IC policy updated: %d methods, disabled=%s",
                    len(ic_result.get("methods_updated", [])),
                    ic_result.get("disabled_setups", []),
                )

            uplift = await asyncio.to_thread(compute_advisory_uplift, 7)
            _STATE["last_advisory_uplift"] = uplift
            try:
                from atlas_code_quant.options.options_engine_metrics import update_advisory_uplift_metrics

                update_advisory_uplift_metrics(uplift)
            except Exception:
                pass

            # ── Análisis diario (una vez por día a la hora configurada) ───
            now_utc = _utcnow()
            last_analysis_date = _STATE.get("last_daily_analysis_date")
            if (
                now_utc.hour == daily_analysis_hour_utc
                and str(now_utc.date()) != str(last_analysis_date)
            ):
                logger.info("[orchestrator] Triggering daily analysis for %s", now_utc.date())
                await run_daily_analysis()

        except asyncio.CancelledError:
            break
        except Exception as e:
            err_msg = f"{_utcnow().isoformat()}: {e}"
            _STATE["errors"].append(err_msg)
            if len(_STATE["errors"]) > 50:
                _STATE["errors"] = _STATE["errors"][-50:]
            logger.warning("[orchestrator] loop error: %s", e)

        await asyncio.sleep(reconcile_interval_sec)

    _STATE["running"] = False
    logger.info("[orchestrator] Learning loop detenido.")


def stop_learning_loop() -> None:
    """Señaliza al loop que debe detenerse."""
    _STATE["running"] = False


def get_orchestrator_status() -> dict[str, Any]:
    """Estado actual del orquestador para endpoints REST y monitoreo."""
    return {
        "running": _STATE["running"],
        "reconcile_count": _STATE["reconcile_count"],
        "trades_processed_total": _STATE["trades_processed_total"],
        "ic_updates_total": _STATE["ic_updates_total"],
        "policy_updates": _STATE["policy_updates"],
        "last_reconcile_at": _STATE["last_reconcile_at"],
        "last_daily_analysis_at": _STATE["last_daily_analysis_at"],
        "last_daily_analysis_date": _STATE["last_daily_analysis_date"],
        "last_ic_by_method": _STATE["last_ic_by_method"],
        "last_readiness_report": _STATE["last_readiness_report"],
        "last_reconciliation_status": _STATE["last_reconciliation_status"],
        "last_advisory_uplift": _STATE["last_advisory_uplift"],
        "recent_errors": _STATE["errors"][-5:] if _STATE["errors"] else [],
    }


# ── Singleton del learning brain ──────────────────────────────────────────────

_LEARNING_BRAIN_INSTANCE = None


def _get_learning_brain():
    """Retorna singleton de AtlasLearningBrain."""
    global _LEARNING_BRAIN_INSTANCE
    if _LEARNING_BRAIN_INSTANCE is None:
        from atlas_code_quant.learning.atlas_learning_brain import AtlasLearningBrain
        policy_path = _get_policy_path()
        _LEARNING_BRAIN_INSTANCE = AtlasLearningBrain(policy_path=policy_path)
    return _LEARNING_BRAIN_INSTANCE


def _get_policy_path():
    """Ruta estándar al JSON de políticas."""
    from pathlib import Path
    base = Path(__file__).resolve().parent.parent
    policy_dir = base / "data" / "learning"
    policy_dir.mkdir(parents=True, exist_ok=True)
    return policy_dir / "adaptive_policy_snapshot.json"
