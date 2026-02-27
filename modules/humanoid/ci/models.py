"""CI models: ImprovementFinding, ImprovementPlan, PatchProposal, ApprovalItem."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class ImprovementFinding:
    kind: str  # large_file, duplicate, todo_count, missing_timeout, policy_gap, etc.
    path: str
    detail: str
    score: float = 0.0
    risk: str = "low"
    dev_effort: str = "low"
    autofix_allowed: bool = False


@dataclass
class PatchProposal:
    finding_id: str
    action: str  # add_timeout, add_param_defaults, add_gitignore, etc.
    target_path: str
    patch_preview: str
    safe: bool = True


@dataclass
class ApprovalItem:
    item_id: str
    finding: ImprovementFinding
    proposal: Optional[PatchProposal] = None
    reason: str = ""


@dataclass
class ImprovementPlan:
    plan_id: str
    scope: str  # repo | runtime | all
    findings: List[ImprovementFinding] = field(default_factory=list)
    commits: List[Dict[str, Any]] = field(default_factory=list)
    steps: List[Dict[str, Any]] = field(default_factory=list)
    approvals_required: List[ApprovalItem] = field(default_factory=list)
    auto_executed: List[Dict[str, Any]] = field(default_factory=list)
    artifacts: List[str] = field(default_factory=list)
    created_ts: str = ""
