"""
Atlas Options Brain – Fase 1
Modelo de datos canónico para un contrato de opciones.
Abstracto del proveedor: el DSL y el simulador consumen ESTE modelo,
nunca objetos crudos del API.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from datetime import date
from enum import Enum
from typing import Optional


class OptionType(str, Enum):
    CALL = "call"
    PUT  = "put"


class OptionRight(str, Enum):
    LONG  = "long"
    SHORT = "short"


@dataclass(frozen=True)
class Greeks:
    delta:  float = 0.0
    gamma:  float = 0.0
    theta:  float = 0.0
    vega:   float = 0.0
    rho:    float = 0.0
    iv:     float = 0.0   # implied volatility (decimal, ej. 0.25 = 25%)


@dataclass(frozen=True)
class OptionContract:
    """Contrato de opción normalizado (independiente del proveedor)."""
    symbol:           str            # ticker subyacente, ej. "SPY"
    option_type:      OptionType
    strike:           float
    expiration:       date
    last_price:       float          # último precio negociado
    bid:              float
    ask:              float
    volume:           int
    open_interest:    int
    greeks:           Greeks         = field(default_factory=Greeks)
    contract_symbol:  str            = ""   # ej. "SPY250620C00520000"
    multiplier:       int            = 100

    @property
    def mid(self) -> float:
        return round((self.bid + self.ask) / 2, 4)

    @property
    def spread(self) -> float:
        return round(self.ask - self.bid, 4)

    def __repr__(self) -> str:
        return (f"<{self.option_type.value.upper()} {self.symbol} "
                f"${self.strike} exp:{self.expiration} mid:{self.mid}>")


@dataclass
class OptionsChain:
    """Cadena de opciones completa para un subyacente y vencimiento."""
    symbol:     str
    expiration: date
    spot_price: float
    contracts:  list[OptionContract] = field(default_factory=list)

    def calls(self) -> list[OptionContract]:
        return [c for c in self.contracts if c.option_type == OptionType.CALL]

    def puts(self) -> list[OptionContract]:
        return [c for c in self.contracts if c.option_type == OptionType.PUT]

    def by_strike(self, strike: float, option_type: OptionType) -> Optional[OptionContract]:
        for c in self.contracts:
            if c.strike == strike and c.option_type == option_type:
                return c
        return None

    def strikes(self) -> list[float]:
        return sorted({c.strike for c in self.contracts})
