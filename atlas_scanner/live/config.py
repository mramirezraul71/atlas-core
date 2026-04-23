from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping


@dataclass(frozen=True)
class LiveScannerConfig:
    symbols: tuple[str, ...]
    top_k: int = 20
    stale_after_seconds: int = 15
    max_event_lag_seconds: int = 5
    flow_window_seconds: int = 300
    quote_window_seconds: int = 60
    publish_interval_ms: int = 500
    health_check_interval_seconds: int = 5
    weights: Mapping[str, float] = field(
        default_factory=lambda: {
            "institutional_flow": 0.30,
            "vol": 0.20,
            "gamma": 0.20,
            "oi_flow": 0.20,
            "price": 0.10,
        }
    )
