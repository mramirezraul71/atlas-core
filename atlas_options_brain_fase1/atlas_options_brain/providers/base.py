"""
Interfaz abstracta para proveedores de datos de opciones.
Cualquier proveedor (Tradier, Polygon, Tastytrade, yfinance-fallback)
debe implementar este protocolo.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
from datetime import date
from typing import Optional
from ..models.option_contract import OptionsChain


class OptionsDataProvider(ABC):
    """Interfaz común.  Atlas nunca habla directo con un proveedor."""

    @abstractmethod
    def get_chain(
        self,
        symbol: str,
        expiration: date,
        option_type: Optional[str] = None,  # "call" | "put" | None = ambas
    ) -> OptionsChain:
        ...

    @abstractmethod
    def get_expirations(self, symbol: str) -> list[date]:
        ...

    @abstractmethod
    def get_quote(self, symbol: str) -> float:
        """Precio spot del subyacente."""
        ...
