"""ATLAS Vision Sensor.

Sensor visual autónomo para capturar la Insta360 Link 2, analizar gráficos
en pantalla y publicar eventos estructurados al bus ATLAS PUSH.
"""

from .config import VisionSensorConfig
from .main import run

__all__ = ["VisionSensorConfig", "run"]
