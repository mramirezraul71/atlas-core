"""
Hipocampo Atlas: Sistema de memoria.

Analogo biologico: Hipocampo + corteza entorrinal
- EpisodicMemory: Memoria de experiencias
- SemanticMemory: Conocimiento conceptual
- Consolidator: Consolidacion de memorias
- HippoAPI: API unificada de acceso
"""
from .schemas import Episode, ActionRecord, Concept, Relation
from .episodic_memory import EpisodicMemory
from .semantic_memory import SemanticMemory
from .consolidator import Consolidator
from .api import HippoAPI

__all__ = [
    "Episode",
    "ActionRecord",
    "Concept",
    "Relation",
    "EpisodicMemory",
    "SemanticMemory",
    "Consolidator",
    "HippoAPI",
]
