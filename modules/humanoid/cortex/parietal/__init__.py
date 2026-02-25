"""
Lóbulo Parietal Atlas: Integración sensorial y representación espacial.

Análogo biológico: Corteza parietal posterior
- SensoryFusion: Fusión multimodal de sensores
- SpatialMap: Mapa espacial egocéntrico/alocéntrico
- BodySchema: Modelo del propio cuerpo
"""
from .body_schema import BodySchema
from .sensory_fusion import SensoryFusion
from .spatial_map import SpatialMap

__all__ = [
    "SensoryFusion",
    "SpatialMap",
    "BodySchema",
]
