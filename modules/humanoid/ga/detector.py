"""Detect opportunities/issues from signals."""
from __future__ import annotations

from typing import List

from .models import Finding
from .signals import collect_all_signals


def detect(scope: str = "all", max_findings: int = 10) -> List[Finding]:
    """Detect findings from all signals, sorted by score desc, limited."""
    findings = collect_all_signals(scope=scope)
    findings.sort(key=lambda f: f.score, reverse=True)
    return findings[:max_findings]
