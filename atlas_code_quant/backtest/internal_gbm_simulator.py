"""Alias canónico F1 para el simulador interno GBM.

Este módulo NO implementa QuantConnect LEAN; reutiliza el simulador interno
existente para mantener compatibilidad sin cortar imports en F1.
"""
from __future__ import annotations

from .lean_simulator import *  # noqa: F401,F403
