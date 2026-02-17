"""
Lóbulo Parietal Atlas: Integración sensorial y representación espacial.

Análogo biológico: Corteza parietal posterior
- SensoryFusion: Fusión multimodal de sensores
- SpatialMap: Mapa espacial egocéntrico/alocéntrico
- BodySchema: Modelo del propio cuerpo
"""
from .sensory_fusion import SensoryFusion
from .spatial_map import SpatialMap
from .body_schema import BodySchema

__all__ = [
    "SensoryFusion",
    "SpatialMap",
    "BodySchema",
]
