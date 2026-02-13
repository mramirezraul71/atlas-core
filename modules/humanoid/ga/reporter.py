"""Minimal report generation: markdown export."""
from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from .models import ActionCandidate, ExecutionResult, Finding


def _report_dir() -> Path:
    d = os.getenv("GA_REPORT_DIR", "C:\\ATLAS_PUSH\\snapshots\\ga")
    return Path(d)


def _strict_evidence() -> bool:
    return os.getenv("GA_STRICT_EVIDENCE", "true").strip().lower() in ("1", "true", "yes")


def _format_finding(f: Finding) -> str:
    return f"- [{f.source}] {f.kind} @ {f.path}: {f.detail} (score={f.score:.2f})"


def _format_executed(r: ExecutionResult, strict: bool) -> Optional[str]:
    if strict and r.ok and r.exit_code is None:
        return None
    ev = f"exit_code={r.exit_code}" if r.exit_code is not None else ""
    if r.paths:
        ev += f" paths={r.paths}"
    return f"- {r.action_type}: ok={r.ok} {ev} ms={r.ms}"


def generate_report(
    findings: List[Finding],
    plan: Dict[str, Any],
    executed: List[ExecutionResult],
    approvals_created: List[Dict[str, Any]],
    metrics: Optional[Dict[str, Any]] = None,
    next_action: Optional[str] = None,
) -> str:
    """Build markdown report content."""
    lines: List[str] = []
    lines.append("# GA Report")
    lines.append(f"\nGenerated: {time.strftime('%Y-%m-%d %H:%M:%S UTC', time.gmtime())}\n")
    lines.append("## Top Findings")
    for f in findings[:5]:
        lines.append(_format_finding(f))
    lines.append("\n## Safe Actions Executed")
    strict = _strict_evidence()
    for r in executed:
        block = _format_executed(r, strict)
        if block:
            lines.append(block)
    if not any(_format_executed(r, strict) for r in executed):
        lines.append("- (none)")
    lines.append("\n## Approvals Required")
    for a in approvals_created:
        lines.append(f"- id={a.get('id')} risk={a.get('risk')} action={a.get('action')}")
    if not approvals_created:
        lines.append("- (none)")
    if metrics:
        lines.append("\n## Metrics Summary")
        lines.append(f"- latency: {metrics.get('latency', 'N/A')}")
        lines.append(f"- error_rate: {metrics.get('error_rate', 'N/A')}")
    if next_action:
        lines.append("\n## Next Best Action")
        lines.append(next_action)
    return "\n".join(lines)


def export_markdown(
    findings: List[Finding],
    plan: Dict[str, Any],
    executed: List[ExecutionResult],
    approvals_created: List[Dict[str, Any]],
    metrics: Optional[Dict[str, Any]] = None,
    next_action: Optional[str] = None,
) -> Optional[str]:
    """Write report to GA_REPORT_DIR. Returns path or None."""
    try:
        d = _report_dir()
        d.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
        path = d / f"GA_REPORT_{ts}.md"
        content = generate_report(
            findings=findings,
            plan=plan,
            executed=executed,
            approvals_created=approvals_created,
            metrics=metrics,
            next_action=next_action,
        )
        path.write_text(content, encoding="utf-8")
        return str(path)
    except Exception:
        return None


def get_latest_report_path() -> Optional[str]:
    """Return path of most recent GA report."""
    try:
        d = _report_dir()
        if not d.exists():
            return None
        reports = sorted(d.glob("GA_REPORT_*.md"), key=lambda p: p.stat().st_mtime, reverse=True)
        return str(reports[0]) if reports else None
    except Exception:
        return None
