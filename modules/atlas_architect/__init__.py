"""ATLAS_ARCHITECT: Agente agentic de codificación para ATLAS_PUSH.

Componentes:
- Indexador de arquitectura (PUSH/NEXUS/ROBOT + puertos 8791/8000/8002).
- FS tools (read/write/list/create) con diffs y logging ANS.
- Ejecutor de terminal (pytest/scripts) y análisis de stdout/stderr.
- Orquestación multi-modelo vía router de IA existente (free-first, y paid si policy permite).
"""

from __future__ import annotations

from .agent import AtlasArchitect
from .indexer import ArchitectureIndex

__all__ = ["AtlasArchitect", "ArchitectureIndex"]

