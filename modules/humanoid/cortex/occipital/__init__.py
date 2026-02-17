"""
Lóbulo Occipital Atlas: Procesamiento visual.

Análogo biológico: Corteza visual primaria (V1) y áreas visuales superiores
- VisionPipeline: Detección, segmentación, estimación de profundidad
"""
from .vision_pipeline import VisionPipeline

__all__ = [
    "VisionPipeline",
]
