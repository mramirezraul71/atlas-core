"""
Corteza Atlas: Módulos cognitivos de alto nivel.

Análogo biológico: Corteza cerebral
- Frontal: Planificación, decisión, control ejecutivo
- Parietal: Integración sensorial, modelo espacial
- Temporal: Procesamiento auditivo, lenguaje, memoria episódica
- Occipital: Procesamiento visual, profundidad, reconocimiento
"""
from .frontal import TaskPlanner, DecisionMaker, InhibitoryControl
from .parietal import SensoryFusion, SpatialMap, BodySchema
from .temporal import AudioProcessor, LanguageUnderstanding, EpisodicRecall
from .occipital import VisionPipeline, DepthEstimation, ObjectRecognition

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
    "EpisodicRecall",
    # Occipital
    "VisionPipeline",
    "DepthEstimation",
    "ObjectRecognition",
]
