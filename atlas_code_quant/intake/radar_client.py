"""Cliente HTTP/SSE de Radar para F1.

Implementación deliberadamente conservadora:
- No inicia conexiones de fondo por defecto.
- Solo expone métodos de preparación para fases posteriores.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class RadarOpportunityClient:
    """Stub de cliente de oportunidades Radar."""

    base_url: str = "http://127.0.0.1:8791"
    enabled: bool = False
    timeout_sec: float = 5.0

    def fetch_once(self) -> list[dict]:
        """Devuelve lote vacío en F1 para evitar side effects."""
        return []

    def stream(self):
        """Iterador vacío placeholder para SSE batch futuro."""
        if False:
            yield {}
