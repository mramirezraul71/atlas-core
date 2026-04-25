from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ScannerRunMetrics:
    total_symbols: int
    evaluated_symbols: int
    generated_candidates: int
    avg_score: float
    p95_score: float
    error_count: int
    warnings: tuple[str, ...] = field(default_factory=tuple)

