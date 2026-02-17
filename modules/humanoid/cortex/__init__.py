"""
Corteza Atlas: Módulos cognitivos de alto nivel.

Análogo biológico: Corteza cerebral
- Frontal: Planificación, decisión, control ejecutivo
- Parietal: Integración sensorial, modelo espacial
- Temporal: Procesamiento auditivo, lenguaje, memoria
- Occipital: Procesamiento visual
"""
from .frontal import TaskPlanner, DecisionMaker, InhibitoryControl
from .parietal import SensoryFusion, SpatialMap, BodySchema
from .temporal import AudioProcessor, LanguageUnderstanding
from .occipital import VisionPipeline

__all__ = [
    # Frontal
    "TaskPlanner",
    "DecisionMaker", 
    "InhibitoryControl",
    # Parietal
    "SensoryFusion",
    "SpatialMap",
    "BodySchema",
    # Temporal
    "AudioProcessor",
    "LanguageUnderstanding",
    # Occipital
    "VisionPipeline",
]
