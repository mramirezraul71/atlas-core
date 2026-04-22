from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date
from typing import Protocol


@dataclass(frozen=True)
class StrikeGammaData:
    strike: float
    call_gamma: float
    put_gamma: float


@dataclass(frozen=True)
class GammaData:
    strikes: tuple[StrikeGammaData, ...] = ()
    net_gex: float | None = None


@dataclass(frozen=True)
class OIFlowData:
    oi_change_1d_pct: float | None = None
    call_put_volume_ratio: float | None = None
    volume_imbalance: float | None = None
    call_volume: float | None = None
    put_volume: float | None = None
    meta: dict[str, float] = field(default_factory=dict)


class GammaOIProvider(Protocol):
    def get_gamma_data(self, symbol: str, as_of: date) -> GammaData:
        ...

    def get_oi_flow_data(self, symbol: str, as_of: date) -> OIFlowData:
        ...

