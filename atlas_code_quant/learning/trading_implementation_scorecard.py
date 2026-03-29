from __future__ import annotations

import json
import re
import sqlite3
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from atlas_code_quant.learning.trading_self_audit_protocol import IMPLEMENTATION_SCORECARD_METRICS
from atlas_code_quant.operations.brain_bridge import QuantBrainBridge


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _clamp_pct(value: float) -> float:
    return round(max(0.0, min(100.0, value)), 2)


def _ratio_pct(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return 0.0
    return _clamp_pct((numerator / denominator) * 100.0)


def _score_status(score: float) -> str:
    if score >= 85.0:
        return "healthy"
    if score >= 70.0:
        return "workable"
    if score >= 50.0:
        return "watch"
    if score >= 30.0:
        return "weak"
    return "critical"


def _read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        for encoding in ("utf-8", "utf-8-sig"):
            try:
                payload = json.loads(path.read_text(encoding=encoding))
                return payload if isinstance(payload, dict) else {}
            except UnicodeDecodeError:
                continue
            except json.JSONDecodeError:
                continue
        return {}
    except (OSError, json.JSONDecodeError):
        return {}


def _parse_datetime(value: Any) -> datetime | None:
    if not value:
        return None
    try:
        normalized = str(value).replace("Z", "+00:00")
        return datetime.fromisoformat(normalized)
    except ValueError:
        return None


def _glob_exists(root: Path, pattern: str) -> bool:
    return any(root.glob(pattern))


def _count_event_kinds(log_path: Path, interesting_kinds: set[str]) -> dict[str, int]:
    counts = {kind: 0 for kind in interesting_kinds}
    if not log_path.exists():
        return counts
    with log_path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                payload = json.loads(line)
            except json.JSONDecodeError:
                continue
            kind = str(payload.get("kind") or "")
            if kind in counts:
                counts[kind] += 1
    return counts


def _compute_process_compliance(protocol: dict[str, Any]) -> dict[str, Any]:
    lifecycle = protocol.get("lifecycle") or []
    status_points = {
        "baseline_hardened": 1.0,
        "active_focus": 0.85,
        "pending_extension": 0.25,
    }
    total_points = 0.0
    for stage in lifecycle:
        total_points += status_points.get(str(stage.get("status") or ""), 0.0)
    max_points = max(len(lifecycle), 1)
    score = _clamp_pct((total_points / max_points) * 100.0)
    stage_statuses = {str(stage.get("stage") or "?"): str(stage.get("status") or "unknown") for stage in lifecycle}
    return {
        "name": "process_compliance_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "stage_count": len(lifecycle),
            "audited_or_active_stages": sum(
                1 for stage in lifecycle if str(stage.get("status") or "") in {"baseline_hardened", "active_focus"}
            ),
            "stage_statuses": stage_statuses,
            "formula": "baseline_hardened=1.0, active_focus=0.85, pending_extension=0.25",
        },
    }


def _compute_artifact_coverage(root: Path) -> dict[str, Any]:
    checks = {
        "protocol_module": (root / "atlas_code_quant/learning/trading_self_audit_protocol.py").exists(),
        "protocol_record_script": (root / "atlas_code_quant/scripts/record_trading_self_audit_note.py").exists(),
        "protocol_json": (root / "reports/trading_self_audit_protocol.json").exists(),
        "framework_report": _glob_exists(root, "reports/atlas_quant_trading_process_audit_framework_*.md"),
        "scanner_audit_report": _glob_exists(root, "reports/atlas_quant_scanner_metric_audit_*.md"),
        "entry_audit_report": _glob_exists(root, "reports/atlas_quant_entry_validation_audit_*.md"),
        "execution_audit_report": _glob_exists(root, "reports/atlas_quant_execution_quality_audit_*.md"),
        "position_management_audit_report": _glob_exists(root, "reports/atlas_quant_position_management_audit_*.md"),
        "exit_governance_audit_report": _glob_exists(root, "reports/atlas_quant_exit_governance_audit_*.md"),
        "post_trade_learning_audit_report": _glob_exists(root, "reports/atlas_quant_post_trade_learning_audit_*.md"),
        "external_benchmark_report": _glob_exists(root, "reports/atlas_quant_external_benchmark_confrontation_*.md"),
    }
    score = _ratio_pct(sum(1 for ok in checks.values() if ok), len(checks))
    return {
        "name": "artifact_coverage_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "checks": checks,
        },
    }


def _compute_memory_persistence(root: Path, brain_state: dict[str, Any]) -> dict[str, Any]:
    total_events = int(brain_state.get("total_events") or 0)
    delivered_events = int(brain_state.get("delivered_events") or 0)
    delivery_ratio_pct = _ratio_pct(delivered_events, total_events)
    last_memory_ok = bool(brain_state.get("last_memory_ok"))
    last_bitacora_ok = bool(brain_state.get("last_bitacora_ok"))
    events_path = Path(str(brain_state.get("events_path") or root / "atlas_code_quant/logs/quant_brain_bridge.jsonl"))
    note_counts = _count_event_kinds(
        events_path,
        {"trading_self_audit_protocol", "trading_implementation_scorecard"},
    )
    note_presence_score = 100.0 if note_counts["trading_self_audit_protocol"] > 0 else 0.0
    score = _clamp_pct(
        (delivery_ratio_pct * 0.70)
        + (15.0 if last_memory_ok else 0.0)
        + (10.0 if last_bitacora_ok else 0.0)
        + (note_presence_score * 0.05)
    )
    return {
        "name": "memory_persistence_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "delivered_events": delivered_events,
            "total_events": total_events,
            "queued_local_only_events": int(brain_state.get("queued_local_only_events") or 0),
            "delivery_ratio_pct": delivery_ratio_pct,
            "last_memory_ok": last_memory_ok,
            "last_bitacora_ok": last_bitacora_ok,
            "protocol_note_events": note_counts["trading_self_audit_protocol"],
            "scorecard_note_events": note_counts["trading_implementation_scorecard"],
        },
    }


def _compute_external_benchmark_coverage(root: Path, protocol: dict[str, Any]) -> dict[str, Any]:
    lifecycle = protocol.get("lifecycle") or []
    audited_stages = {
        str(stage.get("stage") or "")
        for stage in lifecycle
        if str(stage.get("status") or "") in {"baseline_hardened", "active_focus"}
    }
    sources = protocol.get("external_benchmark_sources") or []
    covered_stages: set[str] = set()
    for source in sources:
        for used_for in source.get("used_for") or []:
            used_for = str(used_for)
            if used_for in audited_stages:
                covered_stages.add(used_for)
    stage_coverage_pct = _ratio_pct(len(covered_stages), max(len(audited_stages), 1))
    source_depth_pct = _ratio_pct(min(len(sources), 12), 12)
    translation_checks = {
        "scanner_selection": _glob_exists(root, "reports/atlas_quant_scanner_metric_audit_*.md"),
        "entry_validation": _glob_exists(root, "reports/atlas_quant_entry_validation_audit_*.md"),
        "execution_quality": _glob_exists(root, "reports/atlas_quant_execution_quality_audit_*.md"),
        "position_management": _glob_exists(root, "reports/atlas_quant_position_management_audit_*.md"),
        "exit_governance": _glob_exists(root, "reports/atlas_quant_exit_governance_audit_*.md"),
        "post_trade_learning": _glob_exists(root, "reports/atlas_quant_post_trade_learning_audit_*.md"),
    }
    translated_audited = sum(1 for stage in audited_stages if translation_checks.get(stage, False))
    translation_pct = _ratio_pct(translated_audited, max(len(audited_stages), 1))
    score = _clamp_pct((stage_coverage_pct * 0.50) + (source_depth_pct * 0.25) + (translation_pct * 0.25))
    return {
        "name": "external_benchmark_coverage_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "audited_stages": sorted(audited_stages),
            "covered_audited_stages": sorted(covered_stages),
            "registered_source_count": len(sources),
            "stage_coverage_pct": stage_coverage_pct,
            "source_depth_pct": source_depth_pct,
            "translation_pct": translation_pct,
        },
    }


def _compute_observability_feedback(grafana_check: dict[str, Any]) -> dict[str, Any]:
    if not grafana_check:
        return {
            "name": "observability_feedback_score",
            "value": 0.0,
            "status": _score_status(0.0),
            "details": {
                "report_available": False,
                "note": (
                    "No se encontro el chequeo operativo de Grafana/Prometheus. "
                    "Sin esa verificacion, la observabilidad no puede entrar en el scorecard."
                ),
            },
        }

    verification = grafana_check.get("verification") or {}
    reload = grafana_check.get("reload") or {}
    health = (grafana_check.get("raw") or {}).get("health") or {}
    provisioning_rebuild = grafana_check.get("provisioning_rebuild") or {}

    health_ok = bool(health.get("ok"))
    datasource_ok = bool(verification.get("datasource_ok"))
    dashboards_ok = bool(verification.get("dashboards_ok"))
    alerting_ready = bool(verification.get("alerting_ready"))
    provisioning_ok = bool(provisioning_rebuild.get("ok"))

    foundation_checks = [health_ok, datasource_ok, dashboards_ok, alerting_ready, provisioning_ok]
    foundation_score_pct = _ratio_pct(sum(1 for ok in foundation_checks if ok), len(foundation_checks))

    reload_checks = [
        bool((reload.get("datasources") or {}).get("ok")),
        bool((reload.get("dashboards") or {}).get("ok")),
        bool((reload.get("alerting") or {}).get("ok")),
    ]
    reload_resilience_pct = _ratio_pct(sum(1 for ok in reload_checks if ok), len(reload_checks))

    checked_at = _parse_datetime(grafana_check.get("checked_at"))
    freshness_pct = 0.0
    age_hours = None
    if checked_at is not None:
        if checked_at.tzinfo is None:
            checked_at = checked_at.replace(tzinfo=timezone.utc)
        age_hours = max((datetime.now(timezone.utc) - checked_at.astimezone(timezone.utc)).total_seconds() / 3600.0, 0.0)
        if age_hours <= 6:
            freshness_pct = 100.0
        elif age_hours <= 24:
            freshness_pct = 75.0
        elif age_hours <= 72:
            freshness_pct = 40.0
        else:
            freshness_pct = 0.0

    score = _clamp_pct((foundation_score_pct * 0.60) + (reload_resilience_pct * 0.20) + (freshness_pct * 0.20))
    return {
        "name": "observability_feedback_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "report_available": True,
            "overall_status": grafana_check.get("overall_status") or "unknown",
            "grafana_version": grafana_check.get("grafana_version"),
            "telegram_env_ready": bool(grafana_check.get("telegram_env_ready")),
            "health_ok": health_ok,
            "datasource_ok": datasource_ok,
            "dashboards_ok": dashboards_ok,
            "alerting_ready": alerting_ready,
            "provisioning_rebuild_ok": provisioning_ok,
            "reload_resilience_pct": reload_resilience_pct,
            "foundation_score_pct": foundation_score_pct,
            "freshness_pct": freshness_pct,
            "age_hours": None if age_hours is None else round(age_hours, 2),
        },
    }


def _compute_journal_operational_indicators(journal_db_path: Path, external_translation_score: float) -> dict[str, Any]:
    if not journal_db_path.exists():
        score = 0.0
        return {
            "name": "implementation_usefulness_score",
            "value": score,
            "status": _score_status(score),
            "details": {
                "journal_available": False,
                "external_learning_translation_score": external_translation_score,
                "note": "No se encontro el journal, por lo que la utilidad real no puede evaluarse.",
            },
        }

    with sqlite3.connect(journal_db_path) as conn:
        cur = conn.cursor()
        open_total = int(cur.execute("select count(*) from trading_journal where status='open'").fetchone()[0] or 0)
        open_untracked = int(
            cur.execute(
                "select count(*) from trading_journal where status='open' and strategy_type='untracked'"
            ).fetchone()[0]
            or 0
        )
        closed_total = int(cur.execute("select count(*) from trading_journal where status='closed'").fetchone()[0] or 0)
        open_pnl = float(
            cur.execute("select coalesce(sum(unrealized_pnl), 0.0) from trading_journal where status='open'").fetchone()[0]
            or 0.0
        )
        realized_pnl = float(
            cur.execute("select coalesce(sum(realized_pnl), 0.0) from trading_journal where status='closed'").fetchone()[0]
            or 0.0
        )
        # Sub-indicadores de proceso: post_mortem coverage y distribución de exits
        has_post_mortem_col = _journal_has_column(cur, "trading_journal", "post_mortem_json")
        closed_with_postmortem = 0
        if has_post_mortem_col and closed_total > 0:
            closed_with_postmortem = int(
                cur.execute(
                    "select count(*) from trading_journal where status='closed' and post_mortem_json is not null and post_mortem_json!='null'"
                ).fetchone()[0]
                or 0
            )
        # Entradas recientes (últimas 100) con atribución correcta
        has_updated_at = _journal_has_column(cur, "trading_journal", "updated_at")
        recent_attributed = 0
        recent_total = 0
        if has_updated_at:
            try:
                rows = cur.execute(
                    "select strategy_type from trading_journal order by updated_at desc limit 100"
                ).fetchall()
                recent_total = len(rows)
                recent_attributed = sum(
                    1 for r in rows
                    if r[0] not in (None, "untracked", "unknown", "")
                )
            except Exception:
                pass

    attributed_open_positions_pct = _ratio_pct(max(open_total - open_untracked, 0), open_total) if open_total else 100.0
    open_untracked_ratio_pct = _ratio_pct(open_untracked, open_total) if open_total else 0.0
    evidence_sufficiency_score = _ratio_pct(min(closed_total, 20), 20)
    post_mortem_coverage_pct = _ratio_pct(closed_with_postmortem, closed_total) if closed_total else 0.0
    recent_attribution_pct = _ratio_pct(recent_attributed, recent_total) if recent_total else 0.0

    # Fórmula principal: preservada para comparación histórica
    score = _clamp_pct(
        (attributed_open_positions_pct * 0.45)
        + (evidence_sufficiency_score * 0.35)
        + (external_translation_score * 0.20)
    )
    return {
        "name": "implementation_usefulness_score",
        "value": score,
        "status": _score_status(score),
        "details": {
            "journal_available": True,
            "open_total": open_total,
            "open_untracked": open_untracked,
            "open_untracked_ratio_pct": open_untracked_ratio_pct,
            "attributed_open_positions_pct": attributed_open_positions_pct,
            "closed_total": closed_total,
            "evidence_sufficiency_score": evidence_sufficiency_score,
            "open_pnl": round(open_pnl, 2),
            "realized_pnl": round(realized_pnl, 2),
            "post_mortem_coverage_pct": post_mortem_coverage_pct,
            "recent_attribution_pct": recent_attribution_pct,
            "recent_sample_size": recent_total,
            "external_learning_translation_score": external_translation_score,
            "note": (
                "Este score no afirma edge de mercado. Mide si ya hay evidencia operativa suficiente para decir que "
                "lo implantado esta sirviendo y siendo absorbido por el sistema vivo."
            ),
        },
    }


def _journal_has_column(cur: Any, table: str, column: str) -> bool:
    """Verifica si una columna existe en la tabla del journal."""
    try:
        cols = [row[1] for row in cur.execute(f"pragma table_info({table})").fetchall()]
        return column in cols
    except Exception:
        return False


def _compute_signal_ic_quality(root: Path) -> dict[str, Any]:
    """Mide la calidad predictiva del scanner via Information Coefficient (IC).

    IC = correlación de rango entre predicted_move_pct y retorno real observado.
    Referencia: Grinold & Kahn, Active Portfolio Management (2000).
        IC > 0.05 meaningful; IC > 0.10 fuerte; t-stat >= 2.0 requerido.
    """
    ic_tracker_path = root / "atlas_code_quant" / "data" / "learning" / "ic_tracker.json"
    if not ic_tracker_path.exists():
        return {
            "name": "signal_ic_quality_score",
            "value": 0.0,
            "status": "insufficient_data",
            "details": {
                "ic_tracker_available": False,
                "note": (
                    "No hay tracker de IC disponible. "
                    "El ICSignalTracker se activa cuando el loop registra señales con outcome real."
                ),
            },
        }
    try:
        data = json.loads(ic_tracker_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        data = {}

    signals = data.get("signals") or {}
    total = len(signals)
    with_outcome = sum(1 for s in signals.values() if s.get("outcome_available"))

    if with_outcome < 5:
        score = 5.0 if total > 0 else 0.0  # crédito mínimo por tener el tracker activo
        return {
            "name": "signal_ic_quality_score",
            "value": score,
            "status": "insufficient_data",
            "details": {
                "ic_tracker_available": True,
                "total_signals": total,
                "signals_with_outcome": with_outcome,
                "min_required": 5,
                "note": (
                    f"Solo {with_outcome} señales con outcome. "
                    "Se necesitan al menos 5 para calcular IC, 30 para señal meaningfully útil."
                ),
            },
        }

    # Calcular IC via el tracker
    try:
        from atlas_code_quant.learning.ic_signal_tracker import ICSignalTracker
        tracker = ICSignalTracker(tracker_path=ic_tracker_path)
        overall = tracker.compute_ic()
        ic = overall.get("ic")
        t_stat = overall.get("t_stat") or 0.0
        ic_status = overall.get("ic_status", "insufficient_data")
        by_method_summary = {
            m: {"ic": r.get("ic"), "t_stat": r.get("t_stat"), "n": r.get("n_observations"), "status": r.get("ic_status")}
            for m, r in (tracker.summary().get("by_method") or {}).items()
        }
    except Exception as exc:
        return {
            "name": "signal_ic_quality_score",
            "value": 0.0,
            "status": "error",
            "details": {"error": str(exc)},
        }

    # Mapeo IC → score
    if ic is None or t_stat is None:
        score = 10.0
    elif t_stat < 2.0:
        score = 20.0  # tracking activo pero aún no significativo
    elif ic < 0:
        score = 5.0   # señal predictiva negativa — revisar
    elif ic < 0.05:
        score = 35.0  # por debajo de umbral meaningful pero ya significativo
    elif ic < 0.10:
        score = 65.0  # meaningful per Grinold-Kahn
    else:
        score = 100.0  # fuerte

    return {
        "name": "signal_ic_quality_score",
        "value": _clamp_pct(score),
        "status": _score_status(score),
        "details": {
            "ic_tracker_available": True,
            "total_signals": total,
            "signals_with_outcome": with_outcome,
            "overall_ic": ic,
            "overall_t_stat": t_stat,
            "ic_status": ic_status,
            "by_method": by_method_summary,
            "benchmark": {
                "ic_meaningful": 0.05,
                "ic_strong": 0.10,
                "t_stat_min": 2.0,
                "source": "Grinold & Kahn — Active Portfolio Management (2000)",
            },
            "note": overall.get("interpretation", ""),
        },
    }


def _parse_pytest_summary(output: str) -> dict[str, Any]:
    passed = sum(int(match) for match in re.findall(r"(\d+)\s+passed", output))
    failed = sum(int(match) for match in re.findall(r"(\d+)\s+failed", output))
    errors = sum(int(match) for match in re.findall(r"(\d+)\s+error", output))
    skipped = sum(int(match) for match in re.findall(r"(\d+)\s+skipped", output))
    total = passed + failed + errors
    score = 0.0 if total == 0 else _ratio_pct(passed, total)
    return {
        "mode": "executed",
        "score": score,
        "passed": passed,
        "failed": failed,
        "errors": errors,
        "skipped": skipped,
        "summary": output.strip().splitlines()[-1] if output.strip() else "",
    }


def run_guardrail_pytest(root: Path, test_paths: list[str]) -> dict[str, Any]:
    cmd = [sys.executable, "-m", "pytest", "-q", *test_paths]
    result = subprocess.run(
        cmd,
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )
    output = "\n".join(chunk for chunk in [result.stdout, result.stderr] if chunk)
    parsed = _parse_pytest_summary(output)
    parsed["returncode"] = result.returncode
    parsed["command"] = cmd
    parsed["raw_output"] = output
    return parsed


def _compute_test_guardrail_score(root: Path, pytest_result: dict[str, Any] | None) -> dict[str, Any]:
    expected_tests = {
        "protocol": (root / "atlas_code_quant/tests/test_trading_self_audit_protocol.py").exists(),
        "scorecard": (root / "atlas_code_quant/tests/test_trading_implementation_scorecard.py").exists(),
        "position_management": (root / "atlas_code_quant/tests/test_position_management_snapshot.py").exists(),
        "scanner": (root / "atlas_code_quant/tests/test_scanner_metric_recalibration.py").exists(),
        "operation_status": (root / "atlas_code_quant/tests/test_operation_center_status.py").exists(),
        "operation_guards": (root / "atlas_code_quant/tests/test_operation_center_autonomous_guards.py").exists(),
        "ic_signal_tracker": (root / "atlas_code_quant/tests/test_ic_signal_tracker.py").exists(),
    }
    presence_score = _ratio_pct(sum(1 for ok in expected_tests.values() if ok), len(expected_tests))
    if pytest_result:
        score = _clamp_pct((pytest_result["score"] * 0.80) + (presence_score * 0.20))
        details = {
            "mode": "executed",
            "presence_score": presence_score,
            "expected_tests": expected_tests,
            "pytest": pytest_result,
        }
    else:
        score = presence_score
        details = {
            "mode": "presence_only",
            "presence_score": presence_score,
            "expected_tests": expected_tests,
        }
    return {
        "name": "test_guardrail_score",
        "value": score,
        "status": _score_status(score),
        "details": details,
    }


def _build_next_actions(metric_map: dict[str, dict[str, Any]]) -> list[str]:
    actions: list[str] = []
    usefulness = metric_map["implementation_usefulness_score"]
    memory = metric_map["memory_persistence_score"]
    process = metric_map["process_compliance_score"]
    observability = metric_map.get("observability_feedback_score")

    if usefulness["details"].get("open_untracked_ratio_pct", 0.0) > 0:
        actions.append("Cerrar la brecha de estrategia no atribuida: el libro abierto no debe seguir en untracked.")
    if usefulness["details"].get("closed_total", 0) < 20:
        actions.append("Aun no hay muestra cerrada suficiente: usar paper controlado y no declarar mejora de edge todavia.")
    if memory["details"].get("delivery_ratio_pct", 0.0) < 80.0:
        actions.append("Mejorar la entrega a memoria/bitacora: la absorcion real del aprendizaje todavia es irregular.")
    if process["value"] < 70.0:
        actions.append("Completar el ciclo metodologico: revisar por que alguna etapa sigue sin consolidarse en baseline_hardened.")
    if observability and observability["value"] < 80.0:
        actions.append("Subir la confiabilidad de observabilidad: Grafana/Prometheus aun no devuelven feedback lo bastante robusto.")
    if observability and observability["details"].get("reload_resilience_pct", 0.0) < 100.0:
        actions.append("Investigar por que alguna recarga Admin API de Grafana sigue siendo fragil aunque los dashboards esten visibles.")
    ic_metric = metric_map.get("signal_ic_quality_score")
    if ic_metric:
        ic_status = ic_metric.get("status", "")
        with_outcome = ic_metric["details"].get("signals_with_outcome", 0)
        overall_ic = ic_metric["details"].get("overall_ic")
        if not ic_metric["details"].get("ic_tracker_available"):
            actions.append("Activar ICSignalTracker: el loop debe llamar tracker.record_signal() al evaluar candidatos.")
        elif with_outcome < 5:
            actions.append(
                f"Acumular outcomes en ICSignalTracker ({with_outcome} actuales, se necesitan >=5): "
                "asegurarse de que update_outcome() se llama al cerrar posiciones."
            )
        elif ic_status in {"not_significant", "insufficient_data"}:
            actions.append(
                f"IC aun no significativo (n={with_outcome}): continuar paper-trading para llegar a n>=30 "
                "antes de evaluar calidad predictiva del scanner."
            )
        elif ic_status == "negative":
            actions.append(
                f"IC negativo detectado (IC={overall_ic:.4f}): revisar si la prediccion de direccion del scanner "
                "esta invertida o si hay un bias en el calculo de predicted_move_pct."
            )
        elif ic_status == "weak":
            actions.append(
                f"IC debil (IC={overall_ic:.4f if overall_ic else '?'}): revisar filtros de regimen y calidad de "
                "la prediccion antes de escalar el loop."
            )
    if not actions:
        actions.append("Mantener observacion controlada y exigir evidencia before-after antes de promover a live.")
    return actions


def build_trading_implementation_scorecard(
    *,
    root: Path,
    protocol_path: Path,
    journal_db_path: Path,
    brain_state_path: Path,
    grafana_check_path: Path | None = None,
    pytest_result: dict[str, Any] | None = None,
) -> dict[str, Any]:
    protocol = _read_json(protocol_path)
    brain_state = _read_json(brain_state_path)
    grafana_check = _read_json(grafana_check_path) if grafana_check_path else {}

    process_compliance = _compute_process_compliance(protocol)
    artifact_coverage = _compute_artifact_coverage(root)
    memory_persistence = _compute_memory_persistence(root, brain_state)
    external_benchmark = _compute_external_benchmark_coverage(root, protocol)
    observability_feedback = _compute_observability_feedback(grafana_check)
    test_guardrails = _compute_test_guardrail_score(root, pytest_result)
    usefulness = _compute_journal_operational_indicators(
        journal_db_path,
        external_translation_score=float(external_benchmark["details"]["translation_pct"]),
    )
    signal_ic_quality = _compute_signal_ic_quality(root)

    metrics = [
        process_compliance,
        artifact_coverage,
        memory_persistence,
        external_benchmark,
        observability_feedback,
        test_guardrails,
        usefulness,
        signal_ic_quality,
    ]
    metric_map = {metric["name"]: metric for metric in metrics}

    return {
        "generated_at": _utcnow_iso(),
        "topic": "atlas_quant_trading_implementation_scorecard",
        "summary": (
            "Scorecard operativo para medir dos cosas separadas: cuanto del metodo ya esta implantado y cuanta "
            "evidencia real existe de que esa implantacion esta sirviendo."
        ),
        "metric_catalog": IMPLEMENTATION_SCORECARD_METRICS,
        "headline": {
            "atlas_process_compliance_score": process_compliance["value"],
            "atlas_process_compliance_status": process_compliance["status"],
            "atlas_implementation_usefulness_score": usefulness["value"],
            "atlas_implementation_usefulness_status": usefulness["status"],
        },
        "metrics": metric_map,
        "supporting_indicators": {
            "brain_delivery_ratio_pct": memory_persistence["details"]["delivery_ratio_pct"],
            "attributed_open_positions_pct": usefulness["details"].get("attributed_open_positions_pct", 0.0),
            "open_untracked_ratio_pct": usefulness["details"].get("open_untracked_ratio_pct", 0.0),
            "evidence_sufficiency_score": usefulness["details"].get("evidence_sufficiency_score", 0.0),
            "post_mortem_coverage_pct": usefulness["details"].get("post_mortem_coverage_pct", 0.0),
            "recent_attribution_pct": usefulness["details"].get("recent_attribution_pct", 0.0),
            "external_learning_translation_score": usefulness["details"].get(
                "external_learning_translation_score",
                0.0,
            ),
            "observability_feedback_score": observability_feedback["value"],
            "grafana_reload_resilience_pct": observability_feedback["details"].get("reload_resilience_pct", 0.0),
            "grafana_alerting_ready_pct": 100.0 if observability_feedback["details"].get("alerting_ready") else 0.0,
            "signal_ic_quality_score": signal_ic_quality["value"],
            "ic_tracker_signals_with_outcome": signal_ic_quality["details"].get("signals_with_outcome", 0),
            "ic_overall": signal_ic_quality["details"].get("overall_ic"),
            "ic_status": signal_ic_quality["details"].get("ic_status", signal_ic_quality.get("status", "insufficient_data")),
        },
        "next_actions": _build_next_actions(metric_map),
    }


def write_trading_implementation_scorecard_json(payload: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    return path


def format_trading_implementation_scorecard_markdown(payload: dict[str, Any]) -> str:
    headline = payload["headline"]
    metrics = payload["metrics"]
    indicators = payload["supporting_indicators"]
    lines = [
        "# ATLAS Quant - Trading Implementation Scorecard",
        "",
        f"- Generated at: `{payload['generated_at']}`",
        f"- Process compliance: `{headline['atlas_process_compliance_score']}/100` ({headline['atlas_process_compliance_status']})",
        f"- Implementation usefulness: `{headline['atlas_implementation_usefulness_score']}/100` ({headline['atlas_implementation_usefulness_status']})",
        "",
        "## Metrics",
        "",
    ]
    for metric_name in [
        "process_compliance_score",
        "artifact_coverage_score",
        "memory_persistence_score",
        "external_benchmark_coverage_score",
        "observability_feedback_score",
        "test_guardrail_score",
        "implementation_usefulness_score",
        "signal_ic_quality_score",
    ]:
        if metric_name not in metrics:
            continue
        metric = metrics[metric_name]
        goal = next(
            (item["goal"] for item in payload["metric_catalog"] if item["name"] == metric_name),
            "",
        )
        lines.append(f"- `{metric_name}`: `{metric['value']}/100` ({metric['status']}) - {goal}")

    lines.extend(
        [
            "",
            "## Supporting Indicators",
            "",
            f"- `brain_delivery_ratio_pct`: `{indicators['brain_delivery_ratio_pct']}`",
            f"- `attributed_open_positions_pct`: `{indicators['attributed_open_positions_pct']}`",
            f"- `open_untracked_ratio_pct`: `{indicators['open_untracked_ratio_pct']}`",
            f"- `evidence_sufficiency_score`: `{indicators['evidence_sufficiency_score']}`",
            f"- `post_mortem_coverage_pct`: `{indicators['post_mortem_coverage_pct']}`",
            f"- `recent_attribution_pct`: `{indicators['recent_attribution_pct']}`",
            f"- `external_learning_translation_score`: `{indicators['external_learning_translation_score']}`",
            f"- `observability_feedback_score`: `{indicators['observability_feedback_score']}`",
            f"- `grafana_reload_resilience_pct`: `{indicators['grafana_reload_resilience_pct']}`",
            f"- `grafana_alerting_ready_pct`: `{indicators['grafana_alerting_ready_pct']}`",
            f"- `signal_ic_quality_score`: `{indicators['signal_ic_quality_score']}` (IC={indicators['ic_overall']}, status={indicators['ic_status']}, n={indicators['ic_tracker_signals_with_outcome']})",
            "",
            "## Interpretation",
            "",
            f"- El proceso va por `{headline['atlas_process_compliance_score']}/100`: esto mide implantacion metodologica, no edge de mercado.",
            f"- La utilidad va por `{headline['atlas_implementation_usefulness_score']}/100`: esto mide si ya hay evidencia suficiente de que lo implementado esta mejorando el sistema vivo.",
            f"- La observabilidad va por `{metrics['observability_feedback_score']['value']}/100`: esto mide si el tablero operativo realmente esta devolviendo feedback confiable para vigilar la implantacion.",
            f"- La calidad de señal IC va por `{metrics['signal_ic_quality_score']['value']}/100`: esto mide el poder predictivo real del scanner (IC Grinold-Kahn). 0=sin datos, 100=IC>=0.10 significativo.",
            "",
            "## Next Actions",
            "",
        ]
    )
    for action in payload["next_actions"]:
        lines.append(f"- {action}")
    return "\n".join(lines) + "\n"


def write_trading_implementation_scorecard_report(payload: dict[str, Any], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(format_trading_implementation_scorecard_markdown(payload), encoding="utf-8")
    return path


def persist_trading_implementation_scorecard(
    payload: dict[str, Any],
    *,
    bridge: QuantBrainBridge | None = None,
    report_path: str | None = None,
) -> dict[str, Any]:
    note = dict(payload)
    if report_path:
        note["report_path"] = report_path
    description = (
        "Scorecard medible para saber si ATLAS esta cumpliendo el metodo implantado, si la memoria lo esta absorbiendo "
        "y si ya existe evidencia operativa de utilidad real."
    )
    memory_bridge = bridge or QuantBrainBridge()
    return memory_bridge.emit(
        kind="trading_implementation_scorecard",
        level="info",
        message="[QUANT][LEARNING] Scorecard medible de cumplimiento e impacto registrado",
        tags=[
            "quant",
            "learning",
            "scorecard",
            "implementation",
            "pre_live",
        ],
        data=note,
        description=description,
        context=json.dumps(note, ensure_ascii=False),
        outcome="scorecard_registered",
        memorize=True,
    )
