"""ATLAS Self-Model: autoconocimiento del sistema.
El sistema se conoce por dentro: estructura, componentes, nervios (checks), cerebro (decisión), heals.
"""
from .manifest import (get_anatomy, get_component, get_manifest,
                       get_nervous_system)

__all__ = [
    "get_manifest",
    "get_anatomy",
    "get_nervous_system",
    "get_component",
]
