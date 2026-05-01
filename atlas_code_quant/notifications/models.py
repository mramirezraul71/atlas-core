"""Modelos y tipos para notificaciones operativas."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class BriefingKind(str, Enum):
    PREMARKET = "premarket"
    EOD = "eod"
    INTRADAY = "intraday"
    EXIT_INTEL = "exit_intelligence"
    TEST = "test"


class LearningPhase(str, Enum):
    READY = "ready"
    WARMING_UP = "warming_up"
    STALLED = "stalled"
    DEGRADED = "degraded"
    UNKNOWN = "unknown"


@dataclass
class PrioritizedBriefing:
    """Contenido ya priorizado listo para render."""
    kind: BriefingKind
    headline: str
    opportunities: list[dict[str, Any]] = field(default_factory=list)
    risks: list[str] = field(default_factory=list)
    positions_focus: list[dict[str, Any]] = field(default_factory=list)
    learning_summary: dict[str, Any] = field(default_factory=dict)
    session_plan: list[str] = field(default_factory=list)
    open_close_criteria: dict[str, list[str]] = field(default_factory=dict)
    metrics: dict[str, Any] = field(default_factory=dict)
    raw_refs: dict[str, Any] = field(default_factory=dict)


@dataclass
class DispatchRecord:
    fingerprint: str
    channels: set[str]
    ok: bool
    detail: str = ""
