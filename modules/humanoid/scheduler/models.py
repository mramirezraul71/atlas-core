"""Scheduler job models (in-memory / API)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class JobSpec:
    """Spec for creating a job."""
    name: str
    kind: str  # update_check | llm_plan | shell_command | custom
    payload: Dict[str, Any]
    run_at: Optional[str] = None  # ISO datetime
    interval_seconds: Optional[int] = None
    max_retries: int = 3
    backoff_seconds: int = 5


@dataclass
class Job:
    """Job row (from DB)."""
    id: str
    name: str
    kind: str
    payload_json: str
    cron: Optional[str]
    run_at: Optional[str]
    interval_seconds: Optional[int]
    enabled: int
    status: str
    retries: int
    max_retries: int
    backoff_seconds: int
    last_run_ts: Optional[str]
    next_run_ts: Optional[str]
    last_error: Optional[str]
    created_ts: str
    updated_ts: str

    def payload(self) -> Dict[str, Any]:
        import json
        return json.loads(self.payload_json) if self.payload_json else {}
