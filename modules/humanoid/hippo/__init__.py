"""
Hipocampo Atlas: Sistema de memoria.

Analogo biologico: Hipocampo + corteza entorrinal
- EpisodicMemory: Memoria de experiencias
- SemanticMemory: Conocimiento conceptual
- Consolidator: Consolidacion de memorias
- HippoAPI: API unificada de acceso
"""
from .api import HippoAPI
from .consolidator import Consolidator
from .episodic_memory import EpisodicMemory
from .schemas import ActionRecord, Concept, Episode, Relation
from .semantic_memory import SemanticMemory

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
