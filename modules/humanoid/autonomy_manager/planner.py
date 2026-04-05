from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import sys
from typing import Any, Dict, List
from uuid import uuid4

from . import storage
from .cognition import analyze_plan

REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPTS_DIR = REPO_ROOT / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from atlas_fault_playbooks import plan_playbook_for_event

from .domain_strategies import build_domain_strategy

SEVERITY_WEIGHT = {
    "emergency": 0,
    "critical": 1,
    "degraded": 2,
    "warning": 3,
    "info": 4,
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _risk_for_severity(severity: str) -> str:
    sev = (severity or "warning").lower()
    if sev in {"emergency", "critical"}:
        return "high"
    if sev == "degraded":
        return "medium"
    return "low"


def _candidate_priority(candidate: Dict[str, Any]) -> tuple[int, int, str]:
    domain_state = candidate.get("_domain_state") or {}
    risk_score = int(domain_state.get("risk_score") or 0)
    return (
        SEVERITY_WEIGHT.get(str(candidate.get("severity") or "warning"), 9),
        -risk_score,
        str(candidate.get("component_id") or ""),
    )


def _memory_hints(candidates: List[Dict[str, Any]]) -> Dict[str, Any]:
    recent = storage.read_recent_memory(limit=25)
    hints: Dict[str, Dict[str, int]] = {}
    for entry in recent:
        status = str(entry.get("manager_status") or "")
        policy_mode = str(entry.get("policy_mode") or "")
        key = f"{policy_mode}:{status}"
        hints.setdefault(key, {"count": 0})
        hints[key]["count"] += 1
    active_faults = sorted({str(c.get("fault_code") or "") for c in candidates if c.get("fault_code")})
    return {
        "recent_cycles": len(recent),
        "policy_outcomes": hints,
        "active_faults": active_faults[:10],
    }


def _candidate_domain(candidate: Dict[str, Any]) -> str:
    return str(candidate.get("domain") or candidate.get("_domain") or "operations_core")


def _decorate_candidates(runtime_model: Dict[str, Any], candidates: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    domains = runtime_model.get("domains") or {}
    decorated: List[Dict[str, Any]] = []
    for candidate in candidates:
        item = dict(candidate)
        domain_name = _candidate_domain(item)
        item["_domain"] = domain_name
        item["_domain_state"] = domains.get(domain_name) or {}
        decorated.append(item)
    return decorated


def _synthesize_candidates_from_events(
    runtime_model: Dict[str, Any], candidates: List[Dict[str, Any]]
) -> List[Dict[str, Any]]:
    existing = {
        (
            str(candidate.get("component_id") or ""),
            str(candidate.get("fault_code") or ""),
        )
        for candidate in candidates
    }
    synthetic: List[Dict[str, Any]] = []
    events = ((runtime_model.get("fault_snapshot") or {}).get("events")) or []
    for event in events:
        key = (str(event.get("component_id") or ""), str(event.get("fault_code") or ""))
        if key in existing:
            continue
        playbook = plan_playbook_for_event(event)
        if not playbook:
            continue
        synthetic.append(
            {
                "component_id": event.get("component_id"),
                "check_id": event.get("component_id"),
                "domain": event.get("domain"),
                "severity": event.get("severity"),
                "fault_code": event.get("fault_code"),
                "symptom": event.get("symptom"),
                "recoverability": event.get("recoverability") or "auto",
                "suggested_heals": [],
                "safe_heals": [],
                "playbook": playbook,
                "_synthetic": True,
            }
        )
    return candidates + synthetic


def _build_action_from_candidate(
    candidate: Dict[str, Any],
    runtime_model: Dict[str, Any],
    policy: Dict[str, Any],
    action_index: int,
) -> Dict[str, Any] | None:
    domain_name = _candidate_domain(candidate)
    domain_policy = ((policy.get("domain_policies") or {}).get(domain_name)) or {}
    allowed = domain_policy.get("allowed_actions") or {}
    playbook = candidate.get("playbook")
    severity = str(candidate.get("severity") or "warning")

    if playbook and allowed.get("playbook_safe"):
        execution_class = str(playbook.get("execution_class") or "safe")
        if execution_class == "disruptive" and not allowed.get("playbook_disruptive"):
            return None
        action = {
            "index": action_index,
            "action_id": f"pb_{uuid4().hex[:10]}",
            "kind": "playbook",
            "phase": "",
            "domain": domain_name,
            "target": str(candidate.get("component_id") or playbook.get("component_id") or "unknown"),
            "risk": _risk_for_severity(severity),
            "description": str(playbook.get("summary") or playbook.get("playbook_id") or "playbook"),
            "why": f"Dominio {domain_name} degradado con playbook seguro disponible.",
            "preconditions": [
                "policy_allows_playbooks",
                "local_evidence_present",
                f"domain_health={candidate.get('_domain_state', {}).get('health')}",
            ],
            "post_checks": [
                "refresh_fault_snapshot",
                "compare_before_after",
            ],
            "playbook": playbook,
            "candidate": candidate,
            "execution_class": execution_class,
        }
        action["domain_strategy"] = build_domain_strategy(
            domain_name, runtime_model, policy, action
        )
        return action

    if candidate.get("safe_heals") and allowed.get("ans_cycle"):
        action = {
            "index": action_index,
            "action_id": f"ans_{uuid4().hex[:10]}",
            "kind": "ans_cycle",
            "phase": "",
            "domain": domain_name,
            "target": "atlas.ans",
            "risk": "low",
            "description": "Ejecutar un ciclo ANS seguro para heals deterministas registrados.",
            "why": f"Dominio {domain_name} permite ANS y hay heals seguros detectados.",
            "preconditions": [
                "policy_allows_ans_cycle",
                "local_evidence_present",
            ],
            "post_checks": [
                "refresh_fault_snapshot",
                "compare_before_after",
            ],
            "candidate_refs": [candidate.get("component_id")],
            "execution_class": "safe",
        }
        action["domain_strategy"] = build_domain_strategy(
            domain_name, runtime_model, policy, action
        )
        return action
    return None


def build_plan(
    runtime_model: Dict[str, Any], policy: Dict[str, Any], *, use_ai: bool = True
) -> Dict[str, Any]:
    raw_candidates = list(
        (runtime_model.get("fault_snapshot") or {}).get("actionable_candidates") or []
    )
    raw_candidates = _synthesize_candidates_from_events(runtime_model, raw_candidates)
    candidates = _decorate_candidates(runtime_model, raw_candidates)
    candidates.sort(key=_candidate_priority)
    max_actions = int((((policy.get("execution") or {}).get("max_actions")) or 3))
    domain_order = (policy.get("mission") or {}).get("priority_order") or []
    mission_phases = (policy.get("mission") or {}).get("phases") or []
    domain_rank = {name: idx for idx, name in enumerate(domain_order)}
    candidates.sort(
        key=lambda item: (
            domain_rank.get(_candidate_domain(item), 999),
            *_candidate_priority(item),
        )
    )

    actions: List[Dict[str, Any]] = []
    deferred_actions: List[Dict[str, Any]] = []
    targeted_domains: List[str] = []
    covered_domains: set[str] = set()
    if mission_phases:
        for phase in mission_phases:
            phase_name = str(phase.get("name") or "")
            phase_goal = str(phase.get("goal") or "")
            for domain_name in phase.get("domains") or []:
                if domain_name in covered_domains:
                    continue
                domain_candidates = [c for c in candidates if _candidate_domain(c) == domain_name]
                if not domain_candidates:
                    continue
                chosen = domain_candidates[0]
                action = _build_action_from_candidate(
                    chosen, runtime_model, policy, len(actions) + 1
                )
                if action:
                    action["phase"] = phase_name
                    action["mission_goal"] = phase_goal
                    actions.append(action)
                    covered_domains.add(domain_name)
                    targeted_domains.append(domain_name)
                else:
                    domain_policy = ((policy.get("domain_policies") or {}).get(domain_name)) or {}
                    deferred_actions.append(
                        {
                            "phase": phase_name,
                            "domain": domain_name,
                            "component_id": chosen.get("component_id"),
                            "reason": domain_policy.get("reason") or "blocked_by_policy",
                            "blocked_by": domain_policy.get("blocked_by") or [],
                        }
                    )
                if len(actions) >= max_actions:
                    break
            if len(actions) >= max_actions:
                break
    else:
        for candidate in candidates:
            domain_name = _candidate_domain(candidate)
            if domain_name in covered_domains:
                continue
            action = _build_action_from_candidate(
                candidate, runtime_model, policy, len(actions) + 1
            )
            if action:
                actions.append(action)
                covered_domains.add(domain_name)
                targeted_domains.append(domain_name)
            else:
                domain_policy = ((policy.get("domain_policies") or {}).get(domain_name)) or {}
                deferred_actions.append(
                    {
                        "domain": domain_name,
                        "component_id": candidate.get("component_id"),
                        "reason": domain_policy.get("reason") or "blocked_by_policy",
                        "blocked_by": domain_policy.get("blocked_by") or [],
                    }
                )
            if len(actions) >= max_actions:
                break

    if not actions:
        observe_action = {
                "index": 1,
                "action_id": f"observe_{uuid4().hex[:10]}",
                "kind": "observe_only",
                "phase": "observe_only",
                "domain": "autonomy_core",
                "target": "atlas.runtime",
                "risk": "low",
                "description": "Mantener observación y persistir estado sin ejecutar remediación.",
                "why": "Los interlocks activos aconsejan observación antes de actuar.",
                "preconditions": ["persist_runtime_model"],
                "post_checks": ["refresh_fault_snapshot"],
                "execution_class": "safe",
            }
        observe_action["domain_strategy"] = build_domain_strategy(
            "autonomy_core", runtime_model, policy, observe_action
        )
        actions.append(observe_action)

    plan = {
        "plan_id": f"plan_{uuid4().hex[:12]}",
        "generated_at": _now_iso(),
        "objective": "Reducir incidentes activos y estabilizar ATLAS con remediación segura.",
        "policy_mode": policy.get("mode"),
        "actions": actions[:max_actions],
        "domains_targeted": targeted_domains,
        "mission_phases": mission_phases,
        "current_focus": ((policy.get("mission") or {}).get("current_focus")) or "",
        "deferred_actions": deferred_actions[:12],
        "memory_hints": _memory_hints(candidates),
        "incidents_considered": [
            {
                "component_id": candidate.get("component_id"),
                "fault_code": candidate.get("fault_code"),
                "severity": candidate.get("severity"),
                "recoverability": candidate.get("recoverability"),
                "domain": _candidate_domain(candidate),
                "synthetic": bool(candidate.get("_synthetic")),
            }
            for candidate in candidates[:12]
        ],
        "ai_analysis": {"used": False, "ok": False, "parsed": {}},
    }
    if use_ai and ((policy.get("interlocks") or {}).get("allow_local_ai_analysis")):
        plan["ai_analysis"] = analyze_plan(runtime_model, policy, plan)
    storage.write_json(storage.PLAN_PATH, plan)
    return plan
