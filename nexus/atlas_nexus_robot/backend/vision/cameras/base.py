"""
Base abstracta para drivers de cámara.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, Optional, Tuple

import numpy as np


class CameraBase(ABC):
    """Interfaz base para todas las cámaras."""

    @abstractmethod
    def open(self) -> bool:
        """Abre el dispositivo de cámara."""
        pass

    @abstractmethod
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Lee un frame. Retorna (success, frame)."""
        pass

    @abstractmethod
    def release(self) -> None:
        """Libera el recurso."""
        pass

    @abstractmethod
    def get_properties(self) -> Dict[str, Any]:
        """Retorna propiedades: modelo, resolución, capacidades."""
        pass

    @property
    def is_opened(self) -> bool:
        return False
