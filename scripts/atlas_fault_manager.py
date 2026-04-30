#!/usr/bin/env python3
"""
ATLAS Fault Manager

Coordinador nativo de fault management para ATLAS:

1. Toma un snapshot normalizado del sistema.
2. Detecta incidentes accionables con heals seguros conocidos.
3. Opcionalmente ejecuta un ciclo ANS seguro.
4. Revalida el estado del sistema.
5. Produce un reporte estructurado y lo enlaza con bitácora/Telegram.

No reemplaza la estructura actual del repo. Se monta por encima de los checks,
heals y límites que ATLAS ya tiene.
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPT_DIR = Path(__file__).resolve().parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from modules.humanoid.ans.engine import CHECK_DEFAULT_HEALS, SAFE_HEALS, run_ans_cycle
from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.ans.reporter import notify_telegram
from atlas_fault_playbooks import (STATE_PATH as PLAYBOOK_STATE_PATH,
                                   execute_playbook, plan_playbook_for_event)
from atlas_fault_snapshot import (
    EVENTS_PATH,
    SNAPSHOT_PATH,
    _run_ans_checks,
    _run_comms_health,
    _run_gateway_binding_health,
    _run_global_health,
    _run_robot_autonomy_health,
    _run_scheduler_runtime_health,
    _run_service_health,
    _run_supervisor_status,
    _write_json,
)

STATE_DIR = REPO_ROOT / "state"
REPORT_PATH = STATE_DIR / "atlas_fault_manager_report.json"
LATEST_PATH = STATE_DIR / "atlas_fault_manager_latest.json"

SEVERITY_ORDER = {
    "info": 0,
    "warning": 1,
    "degraded": 2,
    "critical": 3,
    "emergency": 4,
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _severity_counts(events: List[Dict[str, Any]]) -> Dict[str, int]:
    return {
        sev: sum(1 for event in events if event.get("severity") == sev)
        for sev in ("info", "warning", "degraded", "critical", "emergency")
    }


def _collect_snapshot(write: bool = True) -> Dict[str, Any]:
    events: List[Dict[str, Any]] = []
    events.extend(_run_ans_checks())
    events.extend(_run_service_health())
    events.extend(_run_gateway_binding_health())
    events.extend(_run_global_health())
    events.extend(_run_supervisor_status())
    events.extend(_run_comms_health())
    events.extend(_run_robot_autonomy_health())
    events.extend(_run_scheduler_runtime_health())
    snapshot = {
        "generated_at": _now_iso(),
        "event_count": len(events),
        "severity_counts": _severity_counts(events),
        "events": events,
    }
    if write:
        _write_json(SNAPSHOT_PATH, snapshot)
        _write_json(
            EVENTS_PATH,
            {"generated_at": snapshot["generated_at"], "events": events},
        )
    return snapshot


def _event_key(event: Dict[str, Any]) -> Tuple[str, str]:
    return (
        str(event.get("component_id") or ""),
        str(event.get("fault_code") or ""),
    )


def _candidate_from_event(event: Dict[str, Any]) -> Dict[str, Any] | None:
    severity = str(event.get("severity") or "info")
    if SEVERITY_ORDER.get(severity, 0) < SEVERITY_ORDER["degraded"]:
        return None

    raw = ((event.get("metadata") or {}).get("raw") or {})
    check_id = raw.get("check_id") or event.get("component_id") or ""
    suggested = list(raw.get("suggested_heals") or [])
    if not suggested and check_id:
        suggested = list(CHECK_DEFAULT_HEALS.get(str(check_id), []) or [])

    safe_heals = [heal_id for heal_id in suggested if heal_id in SAFE_HEALS]
    playbook = plan_playbook_for_event(event)
    if not safe_heals and not playbook:
        return None

    return {
        "component_id": event.get("component_id"),
        "check_id": check_id,
        "domain": event.get("domain"),
        "severity": severity,
        "fault_code": event.get("fault_code"),
        "symptom": event.get("symptom"),
        "recoverability": event.get("recoverability"),
        "suggested_heals": suggested,
        "safe_heals": safe_heals,
        "playbook": playbook,
    }


def _collect_candidates(snapshot: Dict[str, Any]) -> List[Dict[str, Any]]:
    candidates: List[Dict[str, Any]] = []
    for event in snapshot.get("events", []):
        candidate = _candidate_from_event(event)
        if candidate:
            candidates.append(candidate)
    return candidates


def _compare_snapshots(
    before: Dict[str, Any], after: Dict[str, Any]
) -> Dict[str, Any]:
    before_map = {_event_key(event): event for event in before.get("events", [])}
    after_map = {_event_key(event): event for event in after.get("events", [])}

    resolved: List[Dict[str, Any]] = []
    improved: List[Dict[str, Any]] = []
    unchanged: List[Dict[str, Any]] = []
    worsened: List[Dict[str, Any]] = []
    new_events: List[Dict[str, Any]] = []

    for key, before_event in before_map.items():
        after_event = after_map.get(key)
        if after_event is None:
            resolved.append(before_event)
            continue
        before_rank = SEVERITY_ORDER.get(before_event.get("severity", "info"), 0)
        after_rank = SEVERITY_ORDER.get(after_event.get("severity", "info"), 0)
        if after_rank < before_rank:
            improved.append(
                {
                    "before": before_event,
                    "after": after_event,
                }
            )
        elif after_rank > before_rank:
            worsened.append(
                {
                    "before": before_event,
                    "after": after_event,
                }
            )
        else:
            unchanged.append(after_event)

    for key, after_event in after_map.items():
        if key not in before_map:
            new_events.append(after_event)

    return {
        "resolved_count": len(resolved),
        "improved_count": len(improved),
        "unchanged_count": len(unchanged),
        "worsened_count": len(worsened),
        "new_count": len(new_events),
        "resolved": resolved,
        "improved": improved,
        "unchanged": unchanged,
        "worsened": worsened,
        "new_events": new_events,
    }


def _manager_status(
    after: Dict[str, Any],
    comparison: Dict[str, Any],
    ans_result: Dict[str, Any] | None,
    playbook_results: List[Dict[str, Any]],
) -> str:
    counts = after.get("severity_counts", {})
    if counts.get("emergency", 0) or counts.get("critical", 0):
        return "critical"
    if after.get("event_count", 0) == 0:
        return "healthy"
    if comparison.get("resolved_count", 0) or comparison.get("improved_count", 0):
        return "improving"
    if any(result.get("ok") for result in playbook_results or []):
        return "degraded"
    if ans_result and ans_result.get("actions_taken"):
        return "degraded"
    return "observed"


def _emit_bitacora(message: str, ok: bool) -> None:
    append_evolution_log(message=message, ok=ok, source="fault_manager")


def _emit_telegram(message: str, severity: str) -> bool:
    try:
        return bool(notify_telegram(message, severity=severity))
    except Exception:
        return False


def _build_summary(
    *,
    mode: str,
    before: Dict[str, Any],
    after: Dict[str, Any],
    comparison: Dict[str, Any],
    candidates: List[Dict[str, Any]],
    ans_result: Dict[str, Any] | None,
    manager_status: str,
    playbook_results: List[Dict[str, Any]],
) -> str:
    before_counts = before.get("severity_counts", {})
    after_counts = after.get("severity_counts", {})
    actions = len((ans_result or {}).get("actions_taken") or [])
    playbook_actions = len(playbook_results or [])
    return (
        f"[FAULT_MANAGER] mode={mode} status={manager_status} "
        f"pre={before.get('event_count', 0)} "
        f"post={after.get('event_count', 0)} "
        f"actionable={len(candidates)} actions={actions} playbooks={playbook_actions} "
        f"resolved={comparison.get('resolved_count', 0)} "
        f"improved={comparison.get('improved_count', 0)} "
        f"worsened={comparison.get('worsened_count', 0)} "
        f"critical_pre={before_counts.get('critical', 0)} "
        f"critical_post={after_counts.get('critical', 0)} "
        f"emergency_post={after_counts.get('emergency', 0)}"
    )


def run_fault_manager(
    *,
    mode: str,
    timeout_sec: int,
    emit: bool,
    allow_disruptive_playbooks: bool = False,
) -> Dict[str, Any]:
    before = _collect_snapshot(write=True)
    candidates = _collect_candidates(before)

    ans_result: Dict[str, Any] | None = None
    playbook_results: List[Dict[str, Any]] = []
    skipped_disruptive_playbooks: List[Dict[str, Any]] = []
    if mode == "heal-safe" and candidates:
        seen_signatures: set[str] = set()
        for candidate in candidates:
            playbook = candidate.get("playbook")
            signature = str((playbook or {}).get("signature") or "")
            if not playbook or signature in seen_signatures:
                continue
            seen_signatures.add(signature)
            if (
                playbook.get("execution_class") == "disruptive"
                and not allow_disruptive_playbooks
            ):
                skipped_disruptive_playbooks.append(
                    {
                        "playbook_id": playbook.get("playbook_id"),
                        "signature": signature,
                        "component_id": playbook.get("component_id"),
                        "fault_code": playbook.get("fault_code"),
                        "reason": "disruptive_playbook_blocked_by_policy",
                    }
                )
                continue
            playbook_results.append(execute_playbook(playbook, emit=emit))
        if any(candidate.get("safe_heals") for candidate in candidates):
            ans_result = run_ans_cycle(mode="auto", timeout_sec=timeout_sec)

    after = _collect_snapshot(write=True)
    comparison = _compare_snapshots(before, after)
    manager_status = _manager_status(after, comparison, ans_result, playbook_results)

    report = {
        "generated_at": _now_iso(),
        "mode": mode,
        "manager_status": manager_status,
        "before": {
            "generated_at": before.get("generated_at"),
            "event_count": before.get("event_count"),
            "severity_counts": before.get("severity_counts"),
        },
        "after": {
            "generated_at": after.get("generated_at"),
            "event_count": after.get("event_count"),
            "severity_counts": after.get("severity_counts"),
        },
        "actionable_candidates": candidates,
        "playbook_results": playbook_results,
        "skipped_disruptive_playbooks": skipped_disruptive_playbooks,
        "ans_cycle": ans_result
        and {
            "mode": ans_result.get("mode"),
            "issues_count": ans_result.get("issues_count"),
            "actions_taken": ans_result.get("actions_taken"),
            "incidents_created": ans_result.get("incidents_created"),
            "report_path": ans_result.get("report_path"),
            "ms": ans_result.get("ms"),
        },
        "comparison": comparison,
        "artifacts": {
            "fault_snapshot": str(SNAPSHOT_PATH),
            "fault_events_latest": str(EVENTS_PATH),
            "fault_manager_report": str(REPORT_PATH),
            "fault_playbook_state": str(PLAYBOOK_STATE_PATH),
        },
    }
    _write_json(REPORT_PATH, report)
    _write_json(
        LATEST_PATH,
        {
            "generated_at": report["generated_at"],
            "manager_status": manager_status,
            "mode": mode,
            "allow_disruptive_playbooks": allow_disruptive_playbooks,
            "before": report["before"],
            "after": report["after"],
            "comparison": {
                "resolved_count": comparison["resolved_count"],
                "improved_count": comparison["improved_count"],
                "worsened_count": comparison["worsened_count"],
                "new_count": comparison["new_count"],
            },
            "playbook_results_count": len(playbook_results),
            "skipped_disruptive_playbooks_count": len(skipped_disruptive_playbooks),
        },
    )

    summary = _build_summary(
        mode=mode,
        before=before,
        after=after,
        comparison=comparison,
        candidates=candidates,
        ans_result=ans_result,
        manager_status=manager_status,
        playbook_results=playbook_results,
    )
    report["summary"] = summary

    if emit:
        ok = manager_status not in {"critical"}
        _emit_bitacora(summary, ok=ok)
        if after["severity_counts"].get("critical", 0) or after["severity_counts"].get(
            "emergency", 0
        ):
            _emit_telegram(f"ATLAS FAULT MANAGER\n{summary}", severity="critical")

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="ATLAS Fault Manager")
    parser.add_argument(
        "--mode",
        choices=("detect-only", "heal-safe"),
        default="detect-only",
        help="detect-only solo normaliza y reporta; heal-safe usa ANS si hay heals seguros accionables",
    )
    parser.add_argument(
        "--timeout-sec",
        type=int,
        default=30,
        help="Timeout para el ciclo ANS cuando se ejecuta heal-safe",
    )
    parser.add_argument(
        "--no-emit",
        action="store_true",
        help="No enviar eventos a bitácora/Telegram",
    )
    parser.add_argument(
        "--allow-disruptive-playbooks",
        action="store_true",
        help="Permitir playbooks disruptivos como reinicios de servicios durante heal-safe",
    )
    parser.add_argument("--json", action="store_true", help="Imprimir JSON resumido")
    args = parser.parse_args()

    report = run_fault_manager(
        mode=args.mode,
        timeout_sec=args.timeout_sec,
        emit=not args.no_emit,
        allow_disruptive_playbooks=args.allow_disruptive_playbooks,
    )

    if args.json:
        print(
            json.dumps(
                {
                    "generated_at": report["generated_at"],
                    "mode": report["mode"],
                    "manager_status": report["manager_status"],
                    "before": report["before"],
                    "after": report["after"],
                    "comparison": {
                        "resolved_count": report["comparison"]["resolved_count"],
                        "improved_count": report["comparison"]["improved_count"],
                        "worsened_count": report["comparison"]["worsened_count"],
                        "new_count": report["comparison"]["new_count"],
                    },
                    "actionable_candidates": len(report["actionable_candidates"]),
                    "playbook_actions": len(report.get("playbook_results") or []),
                    "skipped_disruptive_playbooks": len(
                        report.get("skipped_disruptive_playbooks") or []
                    ),
                    "ans_actions": len(
                        ((report.get("ans_cycle") or {}).get("actions_taken") or [])
                    ),
                    "artifacts": report["artifacts"],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
    else:
        print(report["summary"])
        print(f"Report: {REPORT_PATH}")
        print(f"Latest: {LATEST_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
