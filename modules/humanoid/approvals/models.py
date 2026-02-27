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
    risk: str  # low | medium | high | critical
    status: str  # pending | approved | rejected
    job_id: Optional[str] = None
    run_id: Optional[int] = None
    resolved_ts: Optional[str] = None
    resolved_by: Optional[str] = None
    risk_level: str = "medium"  # low | medium | high | critical
    requires_2fa: bool = False
    expires_at: Optional[str] = None
    approval_signature: Optional[str] = None
    origin_node_id: Optional[str] = None
    chain_hash: Optional[str] = None
