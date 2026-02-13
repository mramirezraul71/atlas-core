"""Auto-Plugin Expansion: si detecta tarea nueva -> crear plugin -> registrar dispatcher -> integrar router -> tests -> validar."""
from __future__ import annotations

from .engine import create_plugin, register_plugin, expand_for_task

__all__ = ["create_plugin", "register_plugin", "expand_for_task"]
