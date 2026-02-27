"""Generate minimal report + export markdown."""
from __future__ import annotations

import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

EXPORT_DIR = Path(os.getenv("CI_EXPORT_DIR", "C:\\ATLAS_PUSH\\snapshots\\ci"))


def build_report(plan: Dict[str, Any], executed: List[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Top 5 findings, 1-3 actions, auto executed, approvals required, evidence."""
    findings = (plan.get("findings") or [])[:5]
    steps = plan.get("steps") or []
    auto = executed or plan.get("auto_executed") or []
    required = plan.get("approvals_required") or []
    actions = []
    if findings:
        actions.append(f"Revisar {len(findings)} hallazgos prioritarios")
    if required:
        actions.append(f"Aprobar {len(required)} ítems vía POST /agent/improve/apply")
    if not actions:
        actions.append("Ninguna acción urgente")
    return {
        "top_findings": findings,
        "recommended_actions": actions[:3],
        "auto_executed": auto,
        "approvals_required": [{"item_id": r.get("item_id"), "path": (r.get("finding") or {}).get("path")} for r in required[:10]],
        "evidence": {"paths": [f.get("path") for f in findings], "plan_id": plan.get("plan_id")},
    }


def export_markdown(plan: Dict[str, Any], report: Dict[str, Any], path: str = "") -> str:
    """Write CI_REPORT_<timestamp>.md to CI_EXPORT_DIR. Returns file path."""
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_path = path or str(EXPORT_DIR / f"CI_REPORT_{ts}.md")
    lines = [
        "# CI Report",
        f"**Date:** {ts}",
        f"**Plan ID:** {plan.get('plan_id', '')}",
        "",
        "## Top findings",
    ]
    for f in report.get("top_findings") or []:
        lines.append(f"- `{f.get('path', '')}`: {f.get('detail', '')} (score: {f.get('score', 0)})")
    lines.extend(["", "## Recommended actions"])
    for a in report.get("recommended_actions") or []:
        lines.append(f"- {a}")
    lines.extend(["", "## Auto-executed", ""])
    for e in report.get("auto_executed") or []:
        lines.append(f"- {e.get('item_id', '')} @ {e.get('path', '')}")
    lines.extend(["", "## Approvals required", ""])
    for r in report.get("approvals_required") or []:
        lines.append(f"- {r.get('item_id', '')} @ {r.get('path', '')}")
    try:
        Path(out_path).write_text("\n".join(lines), encoding="utf-8")
    except Exception:
        out_path = ""
    return out_path
