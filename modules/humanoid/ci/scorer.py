"""Score findings: impact, risk, maintainability, dev_effort."""
from __future__ import annotations

from typing import Any, Dict, List

KIND_WEIGHTS = {
    "large_file": 0.7,
    "todo_count": 0.2,
    "ps1_no_param": 0.15,
    "high_latency": 0.8,
    "error_rate": 0.7,
    "scheduler_stopped": 0.5,
    "policy_gap": 0.6,
    "missing_timeout": 0.4,
}


def score_findings(findings: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Add score, risk, dev_effort; sort by score desc."""
    for f in findings:
        kind = f.get("kind") or ""
        base = KIND_WEIGHTS.get(kind, 0.3)
        f["score"] = float(f.get("score", base))
        f["risk"] = "high" if kind in ("high_latency", "error_rate", "scheduler_stopped") else "low"
        f["dev_effort"] = "high" if kind in ("large_file",) else "low"
    return sorted(findings, key=lambda x: -x.get("score", 0))[:20]
