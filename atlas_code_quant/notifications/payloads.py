"""Extracción de contexto semántico desde subsistemas existentes (sin render)."""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any

from config.settings import settings as global_settings
from learning.duckdb_analytics import get_analytics_engine
from learning.learning_orchestrator import get_orchestrator_status
from operations.readiness_payload_builder import build_readiness_fast_payload
from notifications.prioritization import load_adaptive_snapshot_file


def gather_operational_context(
    *,
    operation_center: Any,
    scanner: Any,
    canonical_service: Any,
    vision_service: Any,
    st: Any | None = None,
    account_scope: str | None = None,
) -> dict[str, Any]:
    s = st or global_settings
    scope = str(account_scope or getattr(s, "tradier_default_scope", "paper") or "paper")
    generated_at = datetime.now(timezone.utc).isoformat()
    readiness = build_readiness_fast_payload(
        operation_center=operation_center,
        vision_service=vision_service,
        st=s,
    )
    operation_lite = operation_center.status_lite()
    scanner_report = scanner.report(activity_limit=max(40, getattr(s, "notify_max_opportunities", 5) * 8))
    try:
        canonical_snapshot = canonical_service.build_snapshot(account_scope=scope)
    except Exception as exc:
        canonical_snapshot = {"error": str(exc), "positions": [], "strategies": []}
    orch = get_orchestrator_status()
    adaptive = load_adaptive_snapshot_file(s.adaptive_learning_snapshot_path)
    try:
        journal_analytics = get_analytics_engine().full_analytics(scope)
    except Exception as exc:
        journal_analytics = {"error": str(exc), "generated_at": generated_at}
    return {
        "generated_at": generated_at,
        "account_scope": scope,
        "readiness": readiness,
        "operation_lite": operation_lite,
        "scanner": scanner_report,
        "canonical_snapshot": canonical_snapshot,
        "learning_orchestrator": orch,
        "adaptive_snapshot": adaptive or {},
        "journal_analytics": journal_analytics,
    }
