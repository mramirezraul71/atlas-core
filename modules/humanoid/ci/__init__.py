"""Continuous Improvement: scan, score, plan, execute (policy-gated), report."""
from __future__ import annotations

from .models import ImprovementFinding, ImprovementPlan, PatchProposal, ApprovalItem
from .scanner import scan_repo
from .runtime_scan import scan_runtime
from .scorer import score_findings
from .planner import build_plan
from .executor import execute_plan, apply_autofix
from .reporter import build_report, export_markdown
from .policy_gate import can_autofix, classify_items
from .engine import run_improve, get_improve_status, set_improve_status, ensure_ci_jobs, get_last_plan

__all__ = [
    "ImprovementFinding", "ImprovementPlan", "PatchProposal", "ApprovalItem",
    "scan_repo", "scan_runtime", "score_findings", "build_plan",
    "execute_plan", "apply_autofix", "build_report", "export_markdown",
    "can_autofix", "classify_items", "run_improve", "get_improve_status", "set_improve_status", "ensure_ci_jobs", "get_last_plan",
]
