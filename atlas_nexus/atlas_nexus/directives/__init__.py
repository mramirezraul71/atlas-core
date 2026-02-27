#!/usr/bin/env python3
"""
ATLAS NEXUS - Directives Module
Sistema de gestión de directivas globales y por proyecto
"""

from .directives_manager import DirectivesManager, get_directives_manager
from .directives_manager import (
    get_global_directives,
    get_project_directives, 
    get_active_directives,
    list_projects,
    get_summary
)

__version__ = "1.0.0"
__author__ = "ATLAS NEXUS"

# Exports principales
__all__ = [
    "DirectivesManager",
    "get_directives_manager",
    "get_global_directives",
    "get_project_directives",
    "get_active_directives", 
    "list_projects",
    "get_summary"
]

# Instancia global para fácil acceso
_directives_manager = None

def get_directives_manager() -> DirectivesManager:
    """Obtener instancia global del DirectivesManager"""
    global _directives_manager
    if _directives_manager is None:
        _directives_manager = DirectivesManager()
    return _directives_manager
