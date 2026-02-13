"""Meta-learning cycle: train -> tune -> report."""
from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

from . import db
from .reporter import build_insights, export_markdown, get_latest_report_path
from .trainer import run_train
from .tuner import apply_tuning, get_current_params


_last_update_ts: Optional[float] = None
_last_changes_summary: Optional[str] = None


def _enabled() -> bool:
    return os.getenv("METALEARN_ENABLED", "true").strip().lower() in ("1", "true", "yes")


def _min_samples() -> int:
    return int(os.getenv("METALEARN_MIN_SAMPLES", "10") or 10)


def _audit(action: str, ok: bool, payload: Optional[Dict] = None, error: Optional[str] = None, ms: int = 0) -> None:
    try:
        from modules.humanoid.audit import get_audit_logger
        get_audit_logger().log_event("metalearn", "system", "cycle", action, ok, ms, error, payload, None)
    except Exception:
        pass


def run_cycle(since_ts: Optional[str] = None) -> Dict[str, Any]:
    """
    Run train -> apply tuning (if allowed and enough samples) -> export report.
    Returns {ok, sample_count, rule_count, report_path, changes_summary, error}.
    """
    global _last_update_ts, _last_changes_summary
    t0 = time.perf_counter()
    if not _enabled():
        return {"ok": False, "sample_count": 0, "rule_count": 0, "report_path": None, "changes_summary": "", "error": "METALEARN_ENABLED=false"}
    try:
        train_result = run_train(since_ts=since_ts)
        if not train_result.get("ok"):
            _audit("metalearn_cycle", False, train_result, train_result.get("error"), int((time.perf_counter() - t0) * 1000))
            return {"ok": False, "sample_count": 0, "rule_count": 0, "report_path": None, "changes_summary": "", "error": train_result.get("error")}

        sample_count = train_result.get("sample_count", 0)
        rules = db.get_rules(limit=100)
        rule_count = len(rules)
        min_samp = _min_samples()
        insufficient = sample_count < min_samp

        applied = {}
        if not insufficient and train_result.get("rules_created", 0) > 0:
            tune_result = apply_tuning(rules, snapshot_before=True)
            if tune_result.get("ok"):
                applied = tune_result.get("applied") or {}
                _last_changes_summary = train_result.get("changes_summary", "")

        params = get_current_params()
        insights = build_insights(rules, db.get_all_stats())
        model_rec = "Use learned router_hints if available; otherwise default route."
        if params.get("router_hints"):
            model_rec = f"Router hints: {params['router_hints']}"
        recommend_rollback = False  # Could set from degradation detection
        report_path = export_markdown(
            insights=insights,
            changes_applied=applied,
            model_routing_rec=model_rec,
            new_rules=rules[:5],
            recommend_rollback=recommend_rollback,
            insufficient_data=insufficient,
        )
        _last_update_ts = time.time()
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("metalearn_cycle", True, {"sample_count": sample_count, "rule_count": rule_count}, None, ms)
        return {
            "ok": True,
            "sample_count": sample_count,
            "rule_count": rule_count,
            "report_path": report_path,
            "changes_summary": _last_changes_summary or train_result.get("changes_summary", ""),
            "error": None,
        }
    except Exception as e:
        ms = int((time.perf_counter() - t0) * 1000)
        _audit("metalearn_cycle", False, {}, str(e), ms)
        return {"ok": False, "sample_count": 0, "rule_count": 0, "report_path": None, "changes_summary": "", "error": str(e)}


def get_status() -> Dict[str, Any]:
    """Status: enabled, last_update_ts, sample_count, rule_count, last_changes_summary."""
    try:
        if not _enabled():
            return {"enabled": False, "last_update_ts": _last_update_ts, "sample_count": 0, "rule_count": 0, "last_changes_summary": None, "error": None}
        sample_count = db.event_count()
        rules = db.get_rules(limit=1)
        rule_count = len(db.get_rules(limit=1000))
        return {
            "enabled": True,
            "last_update_ts": _last_update_ts,
            "sample_count": sample_count,
            "rule_count": rule_count,
            "last_changes_summary": _last_changes_summary,
            "error": None,
        }
    except Exception as e:
        return {"enabled": False, "last_update_ts": None, "sample_count": 0, "rule_count": 0, "last_changes_summary": None, "error": str(e)}
