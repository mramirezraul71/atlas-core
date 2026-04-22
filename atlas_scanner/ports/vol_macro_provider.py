from __future__ import annotations

from dataclasses import dataclass, field
from datetime import date
from typing import Protocol


@dataclass(frozen=True)
class VolData:
    iv_history: tuple[float, ...] = ()
    iv_current: float | None = None
    rv_annualized: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class MacroData:
    vix: float | None = None
    macro_regime: str | None = None
    seasonal_factor: float | None = None


class VolMacroProvider(Protocol):
    def get_vol_data(self, symbol: str, as_of: date) -> VolData:
        ...

    def get_macro_data(self, as_of: date) -> MacroData:
        ...

