"""Nervous System (Sistema Nervioso): sensores -> señales -> score -> reflejos -> bitácora/incidentes.

Diseñado para conectar toda la estructura (API, cerebro IA, scheduler, NEXUS/Robot, disco/logs)
con un estado de salud cuantificado por puntos.
"""

from __future__ import annotations

from .engine import run_nervous_cycle, get_nervous_status

__all__ = ["run_nervous_cycle", "get_nervous_status"]

