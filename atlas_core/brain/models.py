"""Modelos comunes del ATLAS Brain Core."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal


@dataclass
class Event:
    source: str
    kind: str
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass
class Command:
    target: str
    action: str
    params: dict[str, Any] = field(default_factory=dict)


@dataclass
class ModuleState:
    name: str
    health: Literal["ok", "degraded", "critical"]
    mode: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class RiskState:
    name: str
    level: Literal["low", "medium", "high", "critical"]
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class SystemSnapshot:
    global_mode: str
    active_mission: str | None
    modules: dict[str, ModuleState]
    risks: dict[str, RiskState]
    meta: dict[str, Any] = field(default_factory=dict)
