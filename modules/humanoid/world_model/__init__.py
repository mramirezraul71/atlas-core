"""
World Model — Modelo interno del mundo de ATLAS.

Inspirado en ESWM (ICLR 2026) y RoboMemory (2025).
Mantiene un modelo actualizado del entorno (servicios, recursos, entidades)
y predice outcomes de acciones basandose en experiencia pasada.
"""
from .engine import WorldModel
from .predictor import OutcomePredictor

__all__ = ["WorldModel", "OutcomePredictor"]
