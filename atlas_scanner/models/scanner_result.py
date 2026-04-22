from __future__ import annotations

from dataclasses import dataclass, field

from .candidate import CandidateOpportunity
from .scanner_metrics import ScannerRunMetrics


@dataclass(frozen=True)
class ScannerRunResult:
    snapshot_id: str
    started_at: str
    finished_at: str
    candidates: tuple[CandidateOpportunity, ...]
    metrics: ScannerRunMetrics
    meta: dict[str, object] = field(default_factory=dict)

