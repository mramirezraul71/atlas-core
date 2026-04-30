"""Leg: una pata individual de una posición de opciones."""
from __future__ import annotations
from dataclasses import dataclass
from .option_contract import OptionContract, OptionRight


@dataclass
class Leg:
    contract: OptionContract
    right:    OptionRight      # LONG o SHORT
    qty:      int = 1          # número de contratos

    @property
    def premium(self) -> float:
        """Crédito (+) o débito (-) por leg × qty × multiplier."""
        sign = -1 if self.right == OptionRight.LONG else 1
        return round(sign * self.contract.mid * self.qty * self.contract.multiplier, 2)

    @property
    def net_delta(self) -> float:
        sign = 1 if self.right == OptionRight.LONG else -1
        return round(sign * self.contract.greeks.delta * self.qty, 4)

    def __repr__(self) -> str:
        return (f"[{self.right.value.upper()} {self.qty}x "
                f"{self.contract.option_type.value.upper()} "
                f"@{self.contract.strike} mid:{self.contract.mid}]")
