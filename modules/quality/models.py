from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional


class POTSeverity(str, Enum):
    LOW = "low"
    MED = "med"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class POTStep:
    id: str
    name: str
    run: Callable[[Dict], str]
    fatal: bool = True


@dataclass
class POT:
    id: str
    name: str
    description: str
    severity: POTSeverity = POTSeverity.MED
    rules: List[str] = field(default_factory=list)
    steps: List[POTStep] = field(default_factory=list)
    tags: List[str] = field(default_factory=list)


@dataclass
class POTResult:
    pot_id: str
    ok: bool
    output: str
    step_outputs: List[Dict] = field(default_factory=list)
    report_path: Optional[str] = None

