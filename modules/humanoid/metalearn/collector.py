"""Collect feedback events from approvals, GA, deploy, scheduler, router."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from . import db
from .features import extract


def _enabled() -> bool:
    return os.getenv("METALEARN_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def _audit_event_recorded(source: str, event_hash: str) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("metalearn", "system", "event_recorded", True, 0, None, {"source": source, "event_hash": event_hash}, None)
    except Exception:
        pass


def record_feedback(
    action_type: str,
    risk_level: str,
    decision: str,
    outcome: str,
    latency_ms: Optional[int] = None,
    node_id: Optional[str] = None,
    model_used: Optional[str] = None,
    payload: Optional[Dict[str, Any]] = None,
    correlation_id: Optional[str] = None,
    thread_id: Optional[str] = None,
    task_id: Optional[str] = None,
    source: str = "unknown",
    error: Any = None,
) -> bool:
    """
    Record a feedback event. Safe to call from anywhere; no-op if disabled or DB error.
    decision: approve | reject | auto | failed | expired
    outcome: ok | fail
    """
    if not _enabled():
        return False
    try:
        payload = payload or {}
        features = extract(payload, outcome=outcome, error=error)
        payload.setdefault("latency_ms", latency_ms)
        payload.setdefault("model_used", model_used)
        features_json = {**payload, **features}
        # Redact large or PII-like fields for storage
        for k in list(features_json):
            if k in ("content", "output", "prompt", "transcript") or (isinstance(features_json.get(k), str) and len(str(features_json[k])) > 500):
                features_json[k] = "[redacted]"
        row_id = db.insert_event(
            ts=_now(),
            action_type=action_type.strip()[:64],
            risk_level=(risk_level or "medium").strip().lower()[:32],
            decision=decision.strip().lower()[:32],
            outcome=outcome.strip().lower()[:16],
            latency_ms=latency_ms,
            node_id=node_id,
            model_used=model_used,
            features_json=features_json,
            correlation_id=correlation_id,
            thread_id=thread_id,
            task_id=task_id,
            source=source.strip()[:32],
        )
        _audit_event_recorded(source, str(row_id))
        return True
    except Exception:
        return False


def record_approval_resolved(
    approval_id: str,
    action: str,
    risk: str,
    decision: str,
    payload: Optional[Dict[str, Any]] = None,
) -> bool:
    """Convenience: record approval approved/rejected/expired."""
    outcome = "ok" if decision == "approved" else "fail"
    return record_feedback(
        action_type=action or "approval",
        risk_level=risk or "medium",
        decision="approve" if decision == "approved" else "reject",
        outcome=outcome,
        payload=payload or {},
        correlation_id=approval_id,
        source="approval",
    )


def record_ga_action(
    action_type: str,
    risk_level: str,
    outcome: str,
    latency_ms: Optional[int] = None,
    payload: Optional[Dict[str, Any]] = None,
    correlation_id: Optional[str] = None,
) -> bool:
    """Convenience: record GA safe action executed."""
    return record_feedback(
        action_type=action_type,
        risk_level=risk_level,
        decision="auto",
        outcome=outcome,
        latency_ms=latency_ms,
        payload=payload or {},
        correlation_id=correlation_id,
        source="ga",
    )


def record_deploy(outcome: str, payload: Optional[Dict[str, Any]] = None) -> bool:
    """Convenience: record deploy promote ok/fail."""
    return record_feedback(
        action_type="deploy_promote",
        risk_level="high",
        decision="auto",
        outcome=outcome,
        payload=payload or {},
        source="deploy",
    )


def record_scheduler_job(job_id: str, kind: str, outcome: str, latency_ms: Optional[int] = None, error: Any = None) -> bool:
    """Convenience: record scheduler job ok/fail."""
    return record_feedback(
        action_type=kind or "job",
        risk_level="medium",
        decision="auto",
        outcome=outcome,
        latency_ms=latency_ms,
        payload={"job_id": job_id},
        correlation_id=job_id,
        source="scheduler",
        error=error,
    )


def record_router_call(route: str, model_used: str, outcome: str, latency_ms: Optional[int] = None) -> bool:
    """Convenience: record LLM router call."""
    return record_feedback(
        action_type="llm",
        risk_level="low",
        decision="auto",
        outcome=outcome,
        latency_ms=latency_ms,
        node_id=None,
        model_used=model_used,
        payload={"route": route, "model_used": model_used},
        source="router",
    )
