"""Capa de intake para consumo de oportunidades Radar (F3)."""

from .opportunity import RadarOpportunity, RadarOpportunityBatch
from .radar_client import RadarClientResult, RadarOpportunityClient

__all__ = [
    "RadarOpportunity",
    "RadarOpportunityBatch",
    "RadarClientResult",
    "RadarOpportunityClient",
]
