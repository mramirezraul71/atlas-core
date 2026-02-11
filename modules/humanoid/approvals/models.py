"""Approval queue: policy-gated items requiring human/UI/Telegram confirmation."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class ApprovalItem:
    id: str
    ts: str
    action: str
    payload: Dict[str, Any]
    risk: str  # low | medium | high
    status: str  # pending | approved | rejected
    job_id: Optional[str] = None
    run_id: Optional[int] = None
    resolved_ts: Optional[str] = None
    resolved_by: Optional[str] = None
