"""
Lóbulo Occipital Atlas: Procesamiento visual.

Análogo biológico: Corteza visual primaria (V1) y áreas visuales superiores
- VisionPipeline: Detección, segmentación, estimación de profundidad
- DepthEstimation: Estimación de profundidad monocular
- ObjectRecognition: Reconocimiento de objetos específicos
"""
from .vision_pipeline import VisionPipeline
from .depth_estimation import DepthEstimation, DepthOutput
from .object_recognition import ObjectRecognition, ObjectIdentity, RecognitionOutput

__all__ = [
    "VisionPipeline",
    "DepthEstimation",
    "DepthOutput",
    "ObjectRecognition",
    "ObjectIdentity",
    "RecognitionOutput",
]
