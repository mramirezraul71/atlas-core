"""Stats update with decay + rule mining (no heavy ML)."""
from __future__ import annotations

import os
import uuid
from collections import defaultdict
from datetime import datetime, timedelta, timezone
from typing import Any, Dict, List, Tuple

from . import db


def _decay() -> float:
    return float(os.getenv("METALEARN_DECAY", "0.98") or 0.98)


def _risk_shift_max() -> float:
    return float(os.getenv("METALEARN_RISK_SHIFT_MAX", "0.15") or 0.15)


def _min_samples() -> int:
    return int(os.getenv("METALEARN_MIN_SAMPLES", "10") or 10)


def _now() -> str:
    return datetime.now(timezone.utc).isoformat()


def run_train(since_ts: Optional[str] = None) -> Dict[str, Any]:
    """
    Load events since last run (or since_ts), apply decay to stats, compute adjustments, mine rules.
    Returns {ok, sample_count, rules_created, changes_summary, error}.
    """
    try:
        if since_ts is None:
            # Default: last 7 days
            since_dt = datetime.now(timezone.utc) - timedelta(days=7)
            since_ts = since_dt.isoformat()
        events = db.get_events_since(since_ts)
        if not events:
            return {"ok": True, "sample_count": 0, "rules_created": 0, "changes_summary": "no events", "error": None}

        decay = _decay()
        risk_max = _risk_shift_max()
        min_samp = _min_samples()

        # Buckets: (action_type, risk_level) and (action_type, model_family) for router
        approve_reject: Dict[str, List[float]] = defaultdict(lambda: [0.0, 0.0])  # approve, reject
        success_fail: Dict[str, List[float]] = defaultdict(lambda: [0.0, 0.0])
        latency_sum: Dict[str, float] = defaultdict(float)
        sample_count: Dict[str, int] = defaultdict(int)

        for ev in events:
            w = 1.0
            action = (ev.get("action_type") or "unknown").strip().lower()
            risk = (ev.get("risk_level") or "medium").strip().lower()
            decision = (ev.get("decision") or "").strip().lower()
            outcome = (ev.get("outcome") or "fail").strip().lower()
            lat = ev.get("latency_ms") or 0
            model_family = (ev.get("features_json") or {}).get("model_family", "unknown")
            bucket_ar = f"ar:{action}:{risk}"
            bucket_sf = f"sf:{action}:{risk}"
            bucket_router = f"router:{action}:{model_family}"

            if decision in ("approve", "approved"):
                approve_reject[bucket_ar][0] += w
            elif decision in ("reject", "rejected", "expired"):
                approve_reject[bucket_ar][1] += w

            if outcome == "ok":
                success_fail[bucket_sf][0] += w
                success_fail[bucket_router][0] += w
            else:
                success_fail[bucket_sf][1] += w
                success_fail[bucket_router][1] += w

            latency_sum[bucket_router] += float(lat or 0)
            sample_count[bucket_ar] += 1
            sample_count[bucket_sf] += 1
            sample_count[bucket_router] += 1

        now = _now()
        # Decay pass: multiply all existing stats by decay
        conn = db._ensure()
        for row in db.get_all_stats():
            key = row["bucket_key"]
            ap = row["approve_count"] * decay
            rej = row["reject_count"] * decay
            su = row["success_count"] * decay
            fa = row["fail_count"] * decay
            tot_lat = row["total_latency_ms"] * decay
            conn.execute(
                """UPDATE learn_stats SET approve_count = ?, reject_count = ?, success_count = ?, fail_count = ?, total_latency_ms = ?, updated_ts = ? WHERE bucket_key = ?""",
                (ap, rej, su, fa, tot_lat, now, key),
            )
        conn.commit()

        for key in set(approve_reject) | set(success_fail):
            ar = approve_reject.get(key, [0, 0])
            sf = success_fail.get(key, [0, 0])
            lat = latency_sum.get(key, 0)
            n = sample_count.get(key, 0)
            db.upsert_stat(key, ar[0], ar[1], sf[0], sf[1], lat, now, sample_delta=max(1, n))

        # Mine rules: from ar: and sf: buckets
        rules: List[Dict[str, Any]] = []
        all_stats = db.get_all_stats()
        for row in all_stats:
            key = row["bucket_key"]
            if not key.startswith("ar:") and not key.startswith("sf:"):
                continue
            parts = key.split(":", 2)
            if len(parts) < 3:
                continue
            action_type = parts[1]
            risk_level = parts[2] if len(parts) > 2 else "medium"
            ap, rej = row["approve_count"], row["reject_count"]
            su, fa = row["success_count"], row["fail_count"]
            total_ar = ap + rej
            total_sf = su + fa
            if total_ar < min_samp and total_sf < min_samp:
                continue
            approve_rate = ap / total_ar if total_ar > 0 else 0
            success_rate = su / total_sf if total_sf > 0 else 0
            # Bounded risk adjust: high approve+success -> lower risk slightly; high reject -> raise risk
            risk_adjust = 0.0
            if total_ar >= min_samp:
                if approve_rate >= 0.85 and success_rate >= 0.8:
                    risk_adjust = -min(risk_max, 0.1)
                elif approve_rate <= 0.2:
                    risk_adjust = min(risk_max, 0.1)
            rules.append({
                "id": str(uuid.uuid4())[:8],
                "conditions": {"action_type": action_type, "risk_level": risk_level},
                "risk_adjust": risk_adjust,
                "router_hint": None,
                "canary_hint": None,
                "approve_rate": approve_rate,
                "success_rate": success_rate,
                "sample_count": int(row["sample_count"]),
                "created_ts": now,
                "updated_ts": now,
            })

        db.save_rules(rules, now)
        changes = f"rules={len(rules)} samples={len(events)}"
        return {"ok": True, "sample_count": len(events), "rules_created": len(rules), "changes_summary": changes, "error": None}
    except Exception as e:
        return {"ok": False, "sample_count": 0, "rules_created": 0, "changes_summary": "", "error": str(e)}


