from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

from .candidate import CandidateOpportunity
from .scan_snapshot import ScanSnapshot
from .scanner_metrics import ScannerRunMetrics


@dataclass(frozen=True)
class ScannerRunResult:
    snapshot: ScanSnapshot
    candidates: Tuple[CandidateOpportunity, ...]
    filtered_symbols: Tuple[str, ...]
    rejected_symbols: Tuple[str, ...]
    error_symbols: Tuple[str, ...]
    metrics: ScannerRunMetrics
    data_source_path: Tuple[str, ...] = ()
    warnings: Tuple[str, ...] = ()

