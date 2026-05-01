"""Priorización de oportunidades, riesgos y foco de posiciones."""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo

from notifications.models import LearningPhase, PrioritizedBriefing, BriefingKind


def _safe_float(x: Any, d: float = 0.0) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return d


def _configured_market_tz(settings: Any) -> tuple[object, str]:
    tz_name = str(getattr(settings, "notify_tz", "") or "America/New_York").strip() or "America/New_York"
    try:
        return ZoneInfo(tz_name), tz_name
    except Exception:
        return timezone.utc, "UTC"


def _daily_row_key(row: dict[str, Any]) -> str:
    raw = str(row.get("day") or row.get("date") or "").strip()
    return raw[:10] if raw else ""


def _select_eod_day_row(daily_rows: list[dict[str, Any]], settings: Any) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    tzinfo, tz_name = _configured_market_tz(settings)
    session_day = datetime.now(tzinfo).date().isoformat()
    utc_day = datetime.now(timezone.utc).date().isoformat()
    normalized = [(key, row) for row in daily_rows if isinstance(row, dict) for key in [_daily_row_key(row)] if key]

    for key, row in normalized:
        if key == session_day:
            return row, {"session_day": session_day, "matched_day": key, "match_source": "session_day_exact", "timezone": tz_name}
    for key, row in normalized:
        if key == utc_day:
            return row, {"session_day": session_day, "matched_day": key, "match_source": "utc_day_fallback", "timezone": tz_name}
    if normalized:
        key, row = max(normalized, key=lambda item: item[0])
        return row, {"session_day": session_day, "matched_day": key, "match_source": "latest_row_fallback", "timezone": tz_name}
    return None, {"session_day": session_day, "matched_day": None, "match_source": "no_daily_rows", "timezone": tz_name}


def classify_learning_phase(
    orchestrator: dict[str, Any],
    *,
    reconcile_interval_sec: int = 300,
) -> tuple[LearningPhase, list[str]]:
    notes: list[str] = []
    if not orchestrator:
        return LearningPhase.UNKNOWN, ["orchestrator: sin estado"]
    errs = orchestrator.get("recent_errors") or []
    if errs:
        notes.append(f"errores recientes en loop: {len(errs)}")
    running = bool(orchestrator.get("running"))
    last_rec = orchestrator.get("last_reconcile_at")
    trades = int(orchestrator.get("trades_processed_total") or 0)
    if errs and len(errs) >= 3:
        return LearningPhase.DEGRADED, notes
    if not running:
        return LearningPhase.STALLED, notes + ["loop no marcado como running"]
    if last_rec:
        try:
            ts = datetime.fromisoformat(str(last_rec).replace("Z", "+00:00"))
            age = (datetime.now(timezone.utc) - ts).total_seconds()
            if age > reconcile_interval_sec * 3 + 120:
                return LearningPhase.STALLED, notes + [f"reconcile antiguo ({int(age)}s)"]
        except Exception:
            pass
    if trades < 15:
        return LearningPhase.WARMING_UP, notes + ["pocas operaciones reconciliadas aún"]
    lr = orchestrator.get("last_readiness_report")
    if isinstance(lr, dict) and lr.get("is_ready") is False:
        notes.append("readiness report: sistema no listo para live según cerebro")
    return LearningPhase.READY, notes


def adaptive_learning_headline(snapshot: dict[str, Any] | None) -> dict[str, Any]:
    if not snapshot or not isinstance(snapshot, dict):
        return {"reliable": False, "headline": "Adaptive policy: snapshot no disponible", "bias": {}}
    reliable = True
    strategies = snapshot.get("strategies") or snapshot.get("by_strategy") or {}
    bias: dict[str, str] = {}
    if isinstance(strategies, dict):
        for k, v in list(strategies.items())[:12]:
            if isinstance(v, dict):
                wr = v.get("win_rate_pct") or v.get("win_rate")
                n = v.get("n") or v.get("samples") or 0
                if _safe_float(n, 0) < 3:
                    reliable = False
                bias[str(k)] = f"n={n} wr={wr}"
    summ = snapshot.get("summary") if isinstance(snapshot.get("summary"), dict) else {}
    return {
        "reliable": reliable and bool(summ or strategies),
        "headline": summ.get("headline") or "Adaptive policy cargada",
        "bias": bias,
        "generated_at": snapshot.get("generated_at") or snapshot.get("updated_at"),
    }


def rank_scanner_opportunities(
    scanner_report: dict[str, Any],
    *,
    max_n: int,
    min_score: float = 0.0,
) -> list[dict[str, Any]]:
    cands = list(scanner_report.get("candidates") or [])
    scored: list[tuple[float, dict[str, Any]]] = []
    for c in cands:
        if not isinstance(c, dict):
            continue
        sc = _safe_float(c.get("selection_score"), 0.0)
        if sc < min_score:
            continue
        scored.append((sc, c))
    scored.sort(key=lambda x: x[0], reverse=True)
    out: list[dict[str, Any]] = []
    for sc, c in scored[:max_n]:
        out.append(
            {
                "symbol": str(c.get("symbol") or c.get("underlying") or "?").upper(),
                "selection_score": round(sc, 2),
                "timeframe": c.get("timeframe"),
                "method": c.get("method") or c.get("primary_method"),
                "win_rate_local": c.get("win_rate_local") or c.get("local_win_rate_pct"),
                "profit_factor_local": c.get("profit_factor_local") or c.get("local_profit_factor"),
                "expectancy_hint": c.get("expectancy_r") or c.get("expected_r"),
            }
        )
    return out


def build_risk_list(
    ctx: dict[str, Any],
    *,
    max_n: int = 8,
) -> list[str]:
    risks: list[str] = []
    r = ctx.get("readiness") or {}
    if not r.get("ready"):
        for reason in r.get("reasons_not_ready") or []:
            risks.append(f"Readiness: {reason}")
    oc = ctx.get("operation_lite") or {}
    fs = oc.get("failsafe") or {}
    if fs.get("active"):
        risks.append(f"Failsafe activo: {fs.get('reason')}")
    n_err = int(fs.get("operational_error_count") or 0)
    lim = int(fs.get("operational_error_limit") or 0)
    if n_err and lim and n_err >= max(1, lim - 1):
        risks.append(f"Errores operativos elevados ({n_err}/{lim})")
    sc = ctx.get("scanner") or {}
    st = sc.get("status") or {}
    if st.get("last_error"):
        risks.append(f"Scanner: {st.get('last_error')}")
    snap = ctx.get("canonical_snapshot") or {}
    for s in (snap.get("strategies") or [])[:8]:
        if isinstance(s, dict) and s.get("alert"):
            risks.append(f"Cuenta {s.get('underlying')}: {s.get('alert')}")
    phase, ln = classify_learning_phase(ctx.get("learning_orchestrator") or {})
    if phase == LearningPhase.DEGRADED:
        risks.append("Aprendizaje: fase DEGRADED — revisar errores del orquestador")
    elif phase == LearningPhase.STALLED:
        risks.append("Aprendizaje: reconciliación posiblemente detenida o lenta")
    for note in ln:
        if note not in str(risks):
            risks.append(f"Learning: {note}")
    return risks[:max_n]


def rank_positions_for_attention(
    snapshot: dict[str, Any],
    *,
    max_n: int,
) -> list[dict[str, Any]]:
    positions = list(snapshot.get("positions") or [])
    scored: list[tuple[float, dict[str, Any]]] = []
    for p in positions:
        if not isinstance(p, dict):
            continue
        u = abs(_safe_float(p.get("unrealized_pnl") or p.get("open_pnl")))
        heat = abs(_safe_float(p.get("symbol_heat_pct") or 0.0))
        urgency = u + heat * 50.0
        scored.append(
            (
                urgency,
                {
                    "symbol": str(p.get("symbol") or p.get("underlying") or "?").upper(),
                    "unrealized_pnl": p.get("unrealized_pnl") or p.get("open_pnl"),
                    "strategy_type": p.get("strategy_type"),
                    "days_open": p.get("days_open") or p.get("holding_days"),
                },
            )
        )
    scored.sort(key=lambda x: x[0], reverse=True)
    return [x[1] for x in scored[:max_n]]


def build_session_plan(
    ctx: dict[str, Any],
    opportunities: list[dict[str, Any]],
    risks: list[str],
) -> list[str]:
    lines: list[str] = []
    ready = bool((ctx.get("readiness") or {}).get("ready"))
    lines.append("1) Confirmar readiness y visión antes de tamaños." if ready else "1) Resolver readiness antes de operar tamaño real.")
    if opportunities:
        top = opportunities[0]
        lines.append(
            f"2) Priorizar revisión: {top.get('symbol')} (score {top.get('selection_score')}) "
            f"alineado con contexto y política IC."
        )
    else:
        lines.append("2) Sin candidatos destacados del scanner; operar solo setups de alta convicción manual.")
    if any("Drawdown" in r or "Circuit" in r for r in risks):
        lines.append("3) Riesgo: reducir agresividad y revisar circuit breakers.")
    else:
        lines.append("3) Mantener disciplina de salida (governance + time stop).")
    lines.append("4) Registrar decisiones en journal para cerrar el loop de aprendizaje.")
    return lines


def open_close_criteria(settings: Any) -> dict[str, list[str]]:
    tp_r = _safe_float(getattr(settings, "exit_governance_take_profit_r", 0.75), 0.75)
    open_c = [
        "Readiness OK y proveedor de visión acorde al modo.",
        "Señal con score ≥ umbral dinámico del scanner / política IC.",
        "Spread y drift de entrada dentro de límites (entry validation).",
    ]
    close_c = [
        f"Take profit / hard loss R según exit governance (TP R≈{tp_r}).",
        "Time stop y trailing floor si aplica.",
        "Cierre si thesis drift o heat de símbolo excede política.",
    ]
    return {"open": open_c, "close": close_c}


def build_prioritized_premarket(ctx: dict[str, Any], settings: Any) -> PrioritizedBriefing:
    max_opp = int(getattr(settings, "notify_max_opportunities", 5))
    max_pos = int(getattr(settings, "notify_max_positions", 8))
    scanner_rep = ctx.get("scanner") or {}
    opps = rank_scanner_opportunities(scanner_rep, max_n=max_opp)
    risks = build_risk_list(ctx)
    snap = ctx.get("canonical_snapshot") or {}
    pos = rank_positions_for_attention(snap, max_n=max_pos)
    orch = ctx.get("learning_orchestrator") or {}
    phase, phase_notes = classify_learning_phase(orch)
    adaptive = adaptive_learning_headline(ctx.get("adaptive_snapshot"))
    learning_summary = {
        "phase": phase.value,
        "phase_notes": phase_notes,
        "orchestrator": {
            "reconcile_count": orch.get("reconcile_count"),
            "trades_processed_total": orch.get("trades_processed_total"),
            "last_ic_by_method": orch.get("last_ic_by_method"),
            "readiness_live": (orch.get("last_readiness_report") or {}).get("is_ready"),
        },
        "adaptive": adaptive,
        "policy_can_weight": phase == LearningPhase.READY and adaptive.get("reliable"),
    }
    plan = build_session_plan(ctx, opps, risks)
    criteria = open_close_criteria(settings)
    headline = "Preapertura — briefing operativo ATLAS Quant"
    return PrioritizedBriefing(
        kind=BriefingKind.PREMARKET,
        headline=headline,
        opportunities=opps,
        risks=risks,
        positions_focus=pos,
        learning_summary=learning_summary,
        session_plan=plan,
        open_close_criteria=criteria,
        metrics={
            "readiness_ready": bool((ctx.get("readiness") or {}).get("ready")),
            "scanner_running": bool((scanner_rep.get("status") or {}).get("running")),
            "open_positions": len(snap.get("positions") or []),
        },
        raw_refs={"generated_at": ctx.get("generated_at")},
    )


def build_prioritized_eod(ctx: dict[str, Any], settings: Any) -> PrioritizedBriefing:
    analytics = ctx.get("journal_analytics") or {}
    daily = list(analytics.get("daily_pnl") or [])
    day_row, day_match = _select_eod_day_row(daily, settings)
    pnl_day = _safe_float((day_row or {}).get("pnl") or (day_row or {}).get("daily_pnl"), 0.0)
    strat = list(analytics.get("strategy_performance") or [])
    best = max(strat, key=lambda s: _safe_float(s.get("profit_factor"), 0.0)) if strat else {}
    worst = min(strat, key=lambda s: _safe_float(s.get("profit_factor"), 999.0)) if strat else {}
    r_dist = analytics.get("r_distribution") or {}
    risks = build_risk_list(ctx, max_n=5)
    if day_match["match_source"] != "session_day_exact":
        risks.append(
            "EoD: PnL diario estimado sin match exacto de sesion; revisar timezone/journal antes de usarlo contablemente."
        )
    recommendations: list[str] = []
    if pnl_day < 0:
        recommendations.append("PnL diario negativo: reducir tamaño mañana y revisar estrategias con peor PF.")
    if best:
        recommendations.append(
            f"Reforzar lo que funciona: {best.get('strategy_type')} PF≈{best.get('profit_factor')}"
        )
    if worst and worst != best:
        recommendations.append(
            f"Vigilar o recortar: {worst.get('strategy_type')} PF≈{worst.get('profit_factor')}"
        )
    recommendations.append("Revisar causas de salida en journal y ajustar governance si hay salidas reactivas.")
    learning_summary = {
        "phase": classify_learning_phase(ctx.get("learning_orchestrator") or {})[0].value,
        "adaptive": adaptive_learning_headline(ctx.get("adaptive_snapshot")),
    }
    return PrioritizedBriefing(
        kind=BriefingKind.EOD,
        headline="Cierre de sesión — resumen operativo ATLAS Quant",
        opportunities=[],
        risks=risks,
        positions_focus=rank_positions_for_attention(ctx.get("canonical_snapshot") or {}, max_n=settings.notify_max_positions),
        learning_summary=learning_summary,
        session_plan=recommendations,
        open_close_criteria={},
        metrics={
            "pnl_day_estimate": round(pnl_day, 2),
            "pnl_day_session_date": day_match["session_day"],
            "pnl_day_matched_date": day_match["matched_day"],
            "pnl_day_match_source": day_match["match_source"],
            "pnl_day_timezone": day_match["timezone"],
            "trades_analytics_rows": int(analytics.get("row_count") or 0),
            "r_distribution_summary": str(r_dist)[:200] if r_dist else "",
        },
        raw_refs={"analytics_generated_at": analytics.get("generated_at"), "eod_day_match": day_match},
    )


def load_adaptive_snapshot_file(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
