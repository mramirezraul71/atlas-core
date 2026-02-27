"""Capture evidence: logs tail, metrics, pids."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List


def capture_evidence(check_id: str, details: Dict[str, Any] = None) -> Dict[str, Any]:
    evidence: Dict[str, Any] = {"check_id": check_id, "details": details or {}}
    try:
        base = (os.getenv("POLICY_ALLOWED_PATHS") or "C:\\ATLAS_PUSH").strip().split(",")[0].strip()
        log_dir = Path(base) / "logs"
        if log_dir.exists():
            tail_lines = []
            for p in sorted(log_dir.glob("*.log"), key=lambda x: x.stat().st_mtime, reverse=True)[:2]:
                try:
                    with open(p, "r", encoding="utf-8", errors="ignore") as f:
                        tail_lines.extend(f.readlines()[-20:])
                except Exception:
                    pass
            evidence["logs_tail"] = "".join(tail_lines[-30:])[:2000]
    except Exception as e:
        evidence["error"] = str(e)
    try:
        from modules.humanoid.metrics import get_metrics_store
        evidence["metrics"] = get_metrics_store().snapshot()
    except Exception:
        pass
    return evidence
