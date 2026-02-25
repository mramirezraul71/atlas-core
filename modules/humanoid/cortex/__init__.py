"""
Corteza Atlas: Módulos cognitivos de alto nivel.

Análogo biológico: Corteza cerebral
- Frontal: Planificación, decisión, control ejecutivo
- Parietal: Integración sensorial, modelo espacial
- Temporal: Procesamiento auditivo, lenguaje, memoria episódica
- Occipital: Procesamiento visual, profundidad, reconocimiento
"""
from .frontal import DecisionMaker, InhibitoryControl, TaskPlanner
from .occipital import DepthEstimation, ObjectRecognition, VisionPipeline
from .parietal import BodySchema, SensoryFusion, SpatialMap
from .temporal import AudioProcessor, EpisodicRecall, LanguageUnderstanding

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
