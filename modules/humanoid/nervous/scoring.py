"""Severity -> points mapping and health score computation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional


SEVERITY_ORDER = ("low", "med", "high", "critical")


def normalize_severity(sev: Optional[str]) -> str:
    s = (sev or "low").strip().lower()
    if s in ("medium", "moderate"):
        s = "med"
    if s not in SEVERITY_ORDER:
        return "low"
    return s


def severity_points(sev: str) -> int:
    """Base penalty points by severity (embargadura del problema)."""
    s = normalize_severity(sev)
    if s == "critical":
        return 50
    if s == "high":
        return 25
    if s == "med":
        return 10
    return 3


def compute_score(base_health_score: int, open_points_total: int) -> int:
    """
    Score final 0-100:
    - se respeta el score base del healthcheck (infra + latencia + error rate)
    - se aplica penalización por señales del sistema nervioso (puntos)
    """
    derived = max(0, 100 - int(open_points_total))
    return int(min(max(0, base_health_score), derived))


@dataclass
class Signal:
    sensor_id: str
    subsystem: str
    severity: str
    points: int
    fingerprint: str
    message: str
    details: Dict[str, Any]
    suggested_heals: list[str]

