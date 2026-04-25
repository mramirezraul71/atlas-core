from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RadarStreamEvent:
    seq: int
    event_type: str
    payload: dict[str, Any]
    timestamp: str
