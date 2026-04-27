"""Capa de intake para consumir oportunidades externas (Radar).

F1: scaffolding seguro. No activa consumo en runtime por defecto.
"""

from .opportunity import RadarOpportunity, RadarOpportunityBatch
from .radar_client import RadarOpportunityClient

__all__ = ["RadarOpportunity", "RadarOpportunityBatch", "RadarOpportunityClient"]
