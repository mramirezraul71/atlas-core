"""Minimal markdown report for meta-learning."""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from . import db
from .tuner import get_current_params


def _report_dir() -> Path:
    d = os.getenv("METALEARN_REPORT_DIR", "C:\\ATLAS_PUSH\\snapshots\\metalearn")
    return Path(d)


def generate_report(
    insights: List[str],
    changes_applied: Dict[str, Any],
    model_routing_rec: str,
    new_rules: List[Dict[str, Any]],
    recommend_rollback: bool = False,
    insufficient_data: bool = False,
) -> str:
    lines = [
        "# Meta-Learning Report",
        f"\nGenerated: {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}\n",
        "## Top Insights",
    ]
    for s in insights[:5]:
        lines.append(f"- {s}")
    if not insights:
        lines.append("- (none)")
    lines.append("\n## Changes Applied (bounded)")
    if changes_applied:
        for k, v in changes_applied.items():
            lines.append(f"- {k}: {v}")
    else:
        lines.append("- (none)")
    lines.append("\n## Model Routing Recommendation")
    lines.append(model_routing_rec or "(none)")
    lines.append("\n## New Rules (max 5)")
    for r in new_rules[:5]:
        lines.append(f"- {r.get('id')}: {r.get('conditions')} risk_adjust={r.get('risk_adjust')} samples={r.get('sample_count')}")
    if not new_rules:
        lines.append("- (none)")
    if recommend_rollback:
        lines.append("\n## ⚠ Recommendation")
        lines.append("Degradation detected. Consider rollback via POST /metalearn/rollback (not executed automatically).")
    if insufficient_data:
        lines.append("\n## Data")
        lines.append("Insufficient samples for tuning. Reporting only.")
    return "\n".join(lines)


def export_markdown(
    insights: List[str],
    changes_applied: Dict[str, Any],
    model_routing_rec: str,
    new_rules: List[Dict[str, Any]],
    recommend_rollback: bool = False,
    insufficient_data: bool = False,
) -> Optional[str]:
    path = _report_dir()
    path.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    filepath = path / f"META_REPORT_{ts}.md"
    content = generate_report(insights, changes_applied, model_routing_rec, new_rules, recommend_rollback, insufficient_data)
    filepath.write_text(content, encoding="utf-8")
    return str(filepath)


def get_latest_report_path() -> Optional[str]:
    try:
        d = _report_dir()
        if not d.exists():
            return None
        reports = sorted(d.glob("META_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
        return str(reports[0]) if reports else None
    except Exception:
        return None


def build_insights(rules: List[Dict[str, Any]], stats: List[Dict[str, Any]]) -> List[str]:
    """Build 3-5 text insights from rules and stats."""
    insights = []
    for r in rules[:5]:
        c = r.get("conditions") or {}
        action = c.get("action_type", "?")
        ar = r.get("approve_rate", 0)
        sr = r.get("success_rate", 0)
        n = r.get("sample_count", 0)
        if ar >= 0.85 and sr >= 0.8:
            insights.append(f"Action '{action}' has high approve+success ({ar:.0%}/{sr:.0%}, n={n}); consider lowering risk tier.")
        elif ar <= 0.2:
            insights.append(f"Action '{action}' is often rejected ({ar:.0%}); risk tier may be appropriate.")
    if not insights:
        insights.append("No strong patterns yet. Collect more approval/outcome events.")
    return insights[:5]
