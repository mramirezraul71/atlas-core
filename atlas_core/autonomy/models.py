from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Literal

AutonomyMode = Literal["manual", "semi", "auto", "experimental"]


@dataclass
class ModuleState:
    name: str
    mode: AutonomyMode
    health: Literal["ok", "degraded", "critical"]
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ModuleRisks:
    name: str
    risks: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Command:
    target: str  # nombre del módulo
    action: str  # e.g. "set_mode", "reduce_risk"
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class Snapshot:
    # {name: {"state":..., "risks":..., "capabilities":...}}
    modules: Dict[str, Dict[str, Any]]
    global_meta: Dict[str, Any] = field(default_factory=dict)

