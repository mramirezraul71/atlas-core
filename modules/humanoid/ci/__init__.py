"""Continuous Improvement: scan, score, plan, execute (policy-gated), report."""
from __future__ import annotations

from .engine import (ensure_ci_jobs, get_improve_status, get_last_plan,
                     run_improve, set_improve_status)
from .executor import apply_autofix, execute_plan
from .models import (ApprovalItem, ImprovementFinding, ImprovementPlan,
                     PatchProposal)
from .planner import build_plan
from .policy_gate import can_autofix, classify_items
from .reporter import build_report, export_markdown
from .runtime_scan import scan_runtime
from .scanner import scan_repo
from .scorer import score_findings

__all__ = [
    "ImprovementFinding",
    "ImprovementPlan",
    "PatchProposal",
    "ApprovalItem",
    "scan_repo",
    "scan_runtime",
    "score_findings",
    "build_plan",
    "execute_plan",
    "apply_autofix",
    "build_report",
    "export_markdown",
    "can_autofix",
    "classify_items",
    "run_improve",
    "get_improve_status",
    "set_improve_status",
    "ensure_ci_jobs",
    "get_last_plan",
]
