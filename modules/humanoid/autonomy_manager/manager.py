from __future__ import annotations

import asyncio
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List
from uuid import uuid4

from modules.humanoid.ans.evolution_bitacora import append_evolution_log
from modules.humanoid.notify import send_telegram

from . import storage
from .domain_strategies import build_domain_strategy
from .planner import build_plan
from .policy_engine import evaluate_policy
from .runtime_model import build_runtime_model

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from atlas_fault_manager import _compare_snapshots
from atlas_fault_playbooks import execute_playbook
from modules.humanoid.ans.engine import run_ans_cycle


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _emit_bitacora(message: str, ok: bool) -> None:
    append_evolution_log(message=message, ok=ok, source="autonomy_manager")


def _emit_telegram(message: str, enabled: bool = True) -> bool:
    if not enabled:
        return False
    try:
        return bool(asyncio.run(send_telegram(message)))
    except Exception:
        return False


def _manager_status(
    before: Dict[str, Any], after: Dict[str, Any], executed: int, failed: int
) -> str:
    before_counts = before.get("severity_counts") or {}
    after_counts = after.get("severity_counts") or {}
    if int(after_counts.get("emergency", 0) or 0) > 0:
        return "emergency"
    if int(after_counts.get("critical", 0) or 0) > 0:
        return "critical"
    if int(after.get("event_count") or 0) == 0:
        return "healthy"
    if int(after.get("event_count") or 0) < int(before.get("event_count") or 0):
        return "improving"
    if executed > 0 and failed == 0:
        return "stabilizing"
    return "observed"


def _summary(report: Dict[str, Any]) -> str:
    return (
        f"[AUTONOMY_MANAGER] mode={report.get('trigger_mode')} "
        f"status={report.get('manager_status')} "
        f"policy={((report.get('policy') or {}).get('mode'))} "
        f"pre={((report.get('before') or {}).get('event_count'))} "
        f"post={((report.get('after') or {}).get('event_count'))} "
        f"planned={report.get('actions_planned')} "
        f"executed={report.get('actions_executed')} "
        f"ok={report.get('actions_succeeded')} "
        f"failed={report.get('actions_failed')}"
    )


def _write_live_state(report: Dict[str, Any]) -> None:
    storage.write_json(storage.EXECUTION_PATH, report)
    storage.write_json(storage.LATEST_PATH, report)


def _telegram_should_emit(report: Dict[str, Any], previous: Dict[str, Any]) -> bool:
    after = report.get("after") or {}
    severity_counts = after.get("severity_counts") or {}
    previous_policy = ((previous.get("policy") or {}).get("mode")) or previous.get(
        "policy_mode"
    )
    current_policy = ((report.get("policy") or {}).get("mode")) or report.get(
        "policy_mode"
    )
    previous_status = previous.get("manager_status")
    current_status = report.get("manager_status")
    return any(
        [
            not previous,
            previous_policy != current_policy,
            previous_status != current_status,
            int(report.get("actions_executed") or 0) > 0,
            int(report.get("actions_failed") or 0) > 0,
            int(severity_counts.get("critical", 0) or 0) > 0,
            int(severity_counts.get("emergency", 0) or 0) > 0,
        ]
    )


def _domain_event_count(snapshot: Dict[str, Any], domain_name: str) -> int:
    total = 0
    for event in snapshot.get("events") or []:
        if str(event.get("domain") or "") == domain_name:
            total += 1
    return total


def _task_priority(action: Dict[str, Any]) -> str:
    risk = str(action.get("risk") or "low").lower()
    if risk == "high":
        return "critical"
    if risk == "medium":
        return "high"
    return "medium"


def _task_detail(action: Dict[str, Any], result: Dict[str, Any] | None = None) -> str:
    parts = [
        f"target={action.get('target')}",
        f"domain={action.get('domain')}",
        f"phase={action.get('phase') or '--'}",
        f"risk={action.get('risk')}",
        f"kind={action.get('kind')}",
    ]
    strategy = action.get("domain_strategy") or {}
    if strategy.get("mode"):
        parts.append(f"strategy={strategy.get('mode')}")
    if result:
        parts.append(f"status={result.get('status')}")
        postcheck = result.get("postcheck") or {}
        if postcheck.get("outcome"):
            parts.append(f"postcheck={postcheck.get('outcome')}")
    return " ".join(str(part) for part in parts if part)


def _build_running_report(
    *,
    cycle_id: str,
    trigger_mode: str,
    runtime_before: Dict[str, Any],
    policy: Dict[str, Any],
    plan: Dict[str, Any] | None,
) -> Dict[str, Any]:
    plan_payload = plan or {
        "actions": [],
        "ai_analysis": {"used": False, "ok": False, "parsed": {}},
    }
    report = {
        "cycle_id": cycle_id,
        "generated_at": _now_iso(),
        "trigger_mode": trigger_mode,
        "execution_enabled": False,
        "manager_status": "running",
        "policy": policy,
        "plan": plan_payload,
        "before": runtime_before.get("fault_snapshot") or {},
        "control_plane": (((runtime_before.get("network") or {}).get("control_plane")) or {}),
        "after": {},
        "comparison": {},
        "actions_planned": len(plan_payload.get("actions") or []),
        "actions_executed": 0,
        "actions_succeeded": 0,
        "actions_failed": 0,
        "action_results": [],
        "domain_results": {},
        "notifications": {
            "bitacora_emitted": False,
            "telegram_attempted": False,
            "telegram_emitted": False,
        },
        "current_phase": "planning",
        "current_action": "",
        "artifacts": {
            "runtime_model": str(storage.RUNTIME_MODEL_PATH),
            "policy": str(storage.POLICY_STATE_PATH),
            "plan": str(storage.PLAN_PATH),
            "execution": str(storage.EXECUTION_PATH),
            "latest": str(storage.LATEST_PATH),
        },
    }
    report["summary"] = _summary(report)
    return report


def _ensure_action_strategy(
    action: Dict[str, Any], runtime_model: Dict[str, Any], policy: Dict[str, Any]
) -> Dict[str, Any]:
    strategy = action.get("domain_strategy") or {}
    if not strategy:
        strategy = build_domain_strategy(
            str(action.get("domain") or "operations_core"),
            runtime_model,
            policy,
            action,
        )
    action["domain_strategy"] = strategy
    return strategy


def _precheck_action(
    action: Dict[str, Any], runtime_model: Dict[str, Any], policy: Dict[str, Any]
) -> Dict[str, Any]:
    strategy = _ensure_action_strategy(action, runtime_model, policy)
    blocked_by = list(strategy.get("blocked_by") or [])
    can_execute = bool(strategy.get("can_execute"))
    passed = can_execute or str(action.get("kind") or "") == "observe_only"
    return {
        "ok": passed,
        "checked_at": _now_iso(),
        "domain": action.get("domain"),
        "strategy_mode": strategy.get("mode"),
        "checks": strategy.get("prechecks") or [],
        "blocked_by": blocked_by,
        "reason": strategy.get("reason") or "",
    }


def _postcheck_action(
    action: Dict[str, Any],
    runtime_before: Dict[str, Any],
    runtime_after: Dict[str, Any],
) -> Dict[str, Any]:
    domain_name = str(action.get("domain") or "operations_core")
    before_snapshot = runtime_before.get("fault_snapshot") or {}
    after_snapshot = runtime_after.get("fault_snapshot") or {}
    before_count = _domain_event_count(before_snapshot, domain_name)
    after_count = _domain_event_count(after_snapshot, domain_name)
    before_health = str(
        (((runtime_before.get("domains") or {}).get(domain_name)) or {}).get("health")
        or "unknown"
    )
    after_health = str(
        (((runtime_after.get("domains") or {}).get(domain_name)) or {}).get("health")
        or "unknown"
    )

    if after_count < before_count:
        outcome = "improved"
    elif after_count > before_count:
        outcome = "worsened"
    elif after_health != before_health:
        outcome = "changed"
    else:
        outcome = "no_change"

    return {
        "checked_at": _now_iso(),
        "domain": domain_name,
        "checks": ((action.get("domain_strategy") or {}).get("postchecks")) or [],
        "before_event_count": before_count,
        "after_event_count": after_count,
        "before_health": before_health,
        "after_health": after_health,
        "outcome": outcome,
    }


def _execute_action(
    action: Dict[str, Any],
    *,
    emit: bool,
    runtime_before_action: Dict[str, Any],
    policy: Dict[str, Any],
) -> Dict[str, Any]:
    started_at = _now_iso()
    strategy = _ensure_action_strategy(action, runtime_before_action, policy)
    precheck = _precheck_action(action, runtime_before_action, policy)
    kind = str(action.get("kind") or "")
    status = "done"
    ok = True
    executed = False
    result_payload: Dict[str, Any]

    if not precheck.get("ok"):
        finished_at = _now_iso()
        return {
            "status": "blocked",
            "ok": True,
            "executed": False,
            "started_at": started_at,
            "finished_at": finished_at,
            "kind": kind,
            "target": action.get("target"),
            "domain": action.get("domain"),
            "strategy": strategy,
            "precheck": precheck,
            "postcheck": {
                "checked_at": finished_at,
                "domain": action.get("domain"),
                "outcome": "blocked",
                "checks": strategy.get("postchecks") or [],
            },
            "payload": {
                "ok": True,
                "message": "action_blocked_by_strategy",
                "blocked_by": precheck.get("blocked_by") or [],
            },
        }

    try:
        if kind == "ans_cycle":
            executed = True
            ans_result = run_ans_cycle(mode="auto", timeout_sec=45)
            result_payload = {"ans_cycle": ans_result}
            ok = True
        elif kind == "playbook":
            executed = True
            playbook = action.get("playbook") or {}
            result_payload = execute_playbook(playbook, emit=emit)
            ok = bool(result_payload.get("ok"))
            if not ok:
                status = "failed"
        else:
            result_payload = {"ok": True, "message": "observe_only"}
            ok = True
    except Exception as exc:
        ok = False
        status = "failed"
        result_payload = {"ok": False, "error": str(exc)}

    runtime_after_action = build_runtime_model(refresh_faults=True)
    postcheck = _postcheck_action(action, runtime_before_action, runtime_after_action)
    if ok and postcheck.get("outcome") == "worsened":
        ok = False
        status = "postcheck_failed"

    finished_at = _now_iso()
    return {
        "status": status,
        "ok": ok,
        "executed": executed,
        "started_at": started_at,
        "finished_at": finished_at,
        "kind": kind,
        "target": action.get("target"),
        "domain": action.get("domain"),
        "strategy": strategy,
        "precheck": precheck,
        "postcheck": postcheck,
        "payload": result_payload,
    }


def _aggregate_domain_results(
    action_results: List[Dict[str, Any]], runtime_after: Dict[str, Any]
) -> Dict[str, Any]:
    aggregated: Dict[str, Dict[str, Any]] = {}
    domains_after = runtime_after.get("domains") or {}
    for item in action_results:
        action = item.get("action") or {}
        result = item.get("result") or {}
        domain_name = str(action.get("domain") or result.get("domain") or "operations_core")
        entry = aggregated.setdefault(
            domain_name,
            {
                "executed": 0,
                "blocked": 0,
                "failed": 0,
                "succeeded": 0,
                "last_status": "",
                "health_after": str((domains_after.get(domain_name) or {}).get("health") or "unknown"),
                "targets": [],
            },
        )
        if result.get("executed"):
            entry["executed"] += 1
        if result.get("status") == "blocked":
            entry["blocked"] += 1
        if not result.get("ok") and result.get("executed"):
            entry["failed"] += 1
        if result.get("ok") and result.get("executed"):
            entry["succeeded"] += 1
        entry["last_status"] = str(result.get("status") or "")
        target = str(action.get("target") or "")
        if target and target not in entry["targets"]:
            entry["targets"].append(target)
    return aggregated


def run_cycle(
    *,
    trigger_mode: str = "auto",
    emit: bool = True,
    use_ai: bool = True,
) -> Dict[str, Any]:
    cycle_id = f"cycle_{uuid4().hex[:12]}"
    previous_report = storage.read_json(storage.LATEST_PATH, default={})
    runtime_before = build_runtime_model(refresh_faults=True)
    policy = evaluate_policy(runtime_before)
    live_report = _build_running_report(
        cycle_id=cycle_id,
        trigger_mode=trigger_mode,
        runtime_before=runtime_before,
        policy=policy,
        plan=None,
    )
    live_report["current_phase"] = "planning"
    live_report["current_action"] = (
        "building plan with local AI" if use_ai else "building deterministic plan"
    )
    _write_live_state(live_report)
    plan = build_plan(runtime_before, policy, use_ai=use_ai)
    live_report["plan"] = plan
    live_report["actions_planned"] = len(plan.get("actions") or [])
    live_report["summary"] = _summary(live_report)
    _write_live_state(live_report)

    storage.timeline(
        f"Autonomy Manager cycle {cycle_id} started policy={policy.get('mode')}",
        kind="info",
        result="ok",
    )
    for action in plan.get("actions", []):
        _ensure_action_strategy(action, runtime_before, policy)
        storage.upsert_task(
            task_id=f"autonomy_{action.get('action_id')}",
            title=f"ATLAS Autonomy: {action.get('description')}",
            status="pending",
            priority=_task_priority(action),
            detail=_task_detail(action),
            action_taken="planned",
        )

    action_results: List[Dict[str, Any]] = []
    executed = 0
    succeeded = 0
    failed = 0
    execute_actions = trigger_mode != "detect-only" and bool(
        (policy.get("execution") or {}).get("allow_execution")
    )
    live_report["execution_enabled"] = execute_actions
    live_report["current_phase"] = "executing" if execute_actions else "observing"
    live_report["summary"] = _summary(live_report)
    _write_live_state(live_report)
    runtime_cursor = runtime_before

    if execute_actions:
        for action in plan.get("actions", []):
            if action.get("kind") == "observe_only":
                continue
            storage.timeline(
                f"Autonomy Manager action {action.get('kind')} domain={action.get('domain')} phase={action.get('phase') or '--'} started target={action.get('target')}",
                kind="info",
                result="running",
            )
            storage.upsert_task(
                task_id=f"autonomy_{action.get('action_id')}",
                title=f"ATLAS Autonomy: {action.get('description')}",
                status="in_progress",
                priority=_task_priority(action),
                detail=_task_detail(action),
                action_taken="running",
            )
            live_report["current_action"] = (
                f"{action.get('phase') or '--'}:{action.get('domain') or '--'}:{action.get('kind')}:{action.get('target') or '--'}"
            )
            live_report["summary"] = _summary(live_report)
            _write_live_state(live_report)
            result = _execute_action(
                action,
                emit=emit,
                runtime_before_action=runtime_cursor,
                policy=policy,
            )
            action_results.append({"action": action, "result": result})
            storage.record_action(cycle_id, action, result)
            if result.get("executed"):
                executed += 1
                if result.get("ok"):
                    succeeded += 1
                else:
                    failed += 1

            task_status = (
                "done"
                if result.get("ok") and result.get("executed")
                else "failed"
                if result.get("executed")
                else "blocked"
            )
            storage.upsert_task(
                task_id=f"autonomy_{action.get('action_id')}",
                title=f"ATLAS Autonomy: {action.get('description')}",
                status=task_status,
                priority=_task_priority(action),
                detail=_task_detail(action, result),
                action_taken=str(result.get("status") or ""),
            )
            storage.timeline(
                f"Autonomy Manager action {action.get('kind')} domain={action.get('domain')} finished status={result.get('status')} target={action.get('target')}",
                kind="error" if not result.get("ok") and result.get("executed") else "info",
                result="ok" if result.get("ok") else "failed" if result.get("executed") else "blocked",
            )
            live_report["actions_executed"] = executed
            live_report["actions_succeeded"] = succeeded
            live_report["actions_failed"] = failed
            live_report["action_results"] = action_results
            live_report["summary"] = _summary(live_report)
            _write_live_state(live_report)
            runtime_cursor = build_runtime_model(refresh_faults=False)

    runtime_after = build_runtime_model(refresh_faults=True)
    comparison = _compare_snapshots(
        runtime_before.get("fault_snapshot") or {},
        runtime_after.get("fault_snapshot") or {},
    )
    manager_status = _manager_status(
        runtime_before.get("fault_snapshot") or {},
        runtime_after.get("fault_snapshot") or {},
        executed,
        failed,
    )
    domain_results = _aggregate_domain_results(action_results, runtime_after)

    report = {
        "cycle_id": cycle_id,
        "generated_at": _now_iso(),
        "trigger_mode": trigger_mode,
        "execution_enabled": execute_actions,
        "manager_status": manager_status,
        "policy": policy,
        "plan": plan,
        "before": runtime_before.get("fault_snapshot") or {},
        "after": runtime_after.get("fault_snapshot") or {},
        "control_plane": (((runtime_after.get("network") or {}).get("control_plane")) or {}),
        "comparison": comparison,
        "actions_planned": len(plan.get("actions") or []),
        "actions_executed": executed,
        "actions_succeeded": succeeded,
        "actions_failed": failed,
        "action_results": action_results,
        "domain_results": domain_results,
        "domains_touched": sorted(domain_results.keys()),
        "notifications": {
            "bitacora_emitted": False,
            "telegram_attempted": False,
            "telegram_emitted": False,
        },
        "current_phase": "completed",
        "current_action": "",
        "artifacts": {
            "runtime_model": str(storage.RUNTIME_MODEL_PATH),
            "policy": str(storage.POLICY_STATE_PATH),
            "plan": str(storage.PLAN_PATH),
            "execution": str(storage.EXECUTION_PATH),
            "latest": str(storage.LATEST_PATH),
        },
    }
    report["summary"] = _summary(report)

    _write_live_state(report)
    storage.record_cycle(report)
    storage.append_memory(
        {
            "cycle_id": cycle_id,
            "generated_at": report["generated_at"],
            "policy_mode": policy.get("mode"),
            "manager_status": manager_status,
            "before_event_count": ((report.get("before") or {}).get("event_count")),
            "after_event_count": ((report.get("after") or {}).get("event_count")),
            "actions_executed": executed,
            "actions_failed": failed,
            "resolved_count": comparison.get("resolved_count"),
            "improved_count": comparison.get("improved_count"),
            "worsened_count": comparison.get("worsened_count"),
            "domains_touched": sorted(domain_results.keys()),
        }
    )
    storage.timeline(
        f"Autonomy Manager cycle {cycle_id} finished status={manager_status} executed={executed} failed={failed}",
        kind="warning" if manager_status in {"critical", "emergency"} else "info",
        result="ok" if failed == 0 else "degraded",
    )

    if emit:
        ok = manager_status not in {"critical", "emergency"}
        _emit_bitacora(report["summary"], ok=ok)
        report["notifications"]["bitacora_emitted"] = True
        if _telegram_should_emit(report, previous_report):
            report["notifications"]["telegram_attempted"] = True
            report["notifications"]["telegram_emitted"] = _emit_telegram(
                f"ATLAS AUTONOMY MANAGER\n{report['summary']}"
            )
        _write_live_state(report)
    return report


def get_latest_status() -> Dict[str, Any]:
    return storage.read_json(
        storage.LATEST_PATH,
        default={"ok": False, "message": "No autonomy manager cycle yet"},
    )
