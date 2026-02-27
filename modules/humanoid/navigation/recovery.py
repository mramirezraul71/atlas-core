"""
Recovery Behaviors: Comportamientos de recuperación.
=====================================================
Define comportamientos para cuando la navegación falla.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.navigation.recovery")


class RecoveryType(str, Enum):
    """Tipos de comportamiento de recuperación."""
    CLEAR_COSTMAP = "clear_costmap"
    ROTATE_IN_PLACE = "rotate_in_place"
    BACK_UP = "back_up"
    WAIT = "wait"
    REPLAN = "replan"


@dataclass
class RecoveryBehavior:
    """Un comportamiento de recuperación."""
    type: RecoveryType
    name: str
    enabled: bool = True
    max_attempts: int = 3
    parameters: Dict[str, Any] = None
    
    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}


class RecoveryManager:
    """
    Gestor de comportamientos de recuperación.
    
    Ejecuta una secuencia de comportamientos cuando la navegación falla.
    """
    
    def __init__(self):
        self._behaviors: List[RecoveryBehavior] = [
            RecoveryBehavior(
                type=RecoveryType.CLEAR_COSTMAP,
                name="Clear local costmap",
                parameters={"layer": "local"},
            ),
            RecoveryBehavior(
                type=RecoveryType.ROTATE_IN_PLACE,
                name="Rotate 360 degrees",
                parameters={"angle": 360, "speed": 0.5},
            ),
            RecoveryBehavior(
                type=RecoveryType.BACK_UP,
                name="Back up 0.3m",
                parameters={"distance": 0.3, "speed": -0.2},
            ),
            RecoveryBehavior(
                type=RecoveryType.WAIT,
                name="Wait 5 seconds",
                parameters={"duration": 5.0},
            ),
            RecoveryBehavior(
                type=RecoveryType.REPLAN,
                name="Request replan",
            ),
        ]
        
        self._current_index: int = 0
        self._attempt_counts: Dict[int, int] = {}
    
    def get_next_recovery(self) -> Optional[RecoveryBehavior]:
        """Obtiene el siguiente comportamiento de recuperación."""
        while self._current_index < len(self._behaviors):
            behavior = self._behaviors[self._current_index]
            
            if not behavior.enabled:
                self._current_index += 1
                continue
            
            attempts = self._attempt_counts.get(self._current_index, 0)
            if attempts >= behavior.max_attempts:
                self._current_index += 1
                continue
            
            self._attempt_counts[self._current_index] = attempts + 1
            return behavior
        
        return None
    
    def recovery_succeeded(self) -> None:
        """Llamar cuando la navegación se recupera."""
        self._current_index = 0
        self._attempt_counts.clear()
        _log.info("Recovery succeeded, resetting recovery sequence")
    
    def recovery_failed(self) -> None:
        """Llamar cuando una recuperación falla."""
        self._current_index += 1
        _log.warning("Recovery behavior failed, trying next")
    
    def reset(self) -> None:
        """Resetea el manager."""
        self._current_index = 0
        self._attempt_counts.clear()
    
    def add_behavior(self, behavior: RecoveryBehavior, index: int = -1) -> None:
        """Añade un comportamiento de recuperación."""
        if index < 0:
            self._behaviors.append(behavior)
        else:
            self._behaviors.insert(index, behavior)
    
    def get_behaviors(self) -> List[RecoveryBehavior]:
        """Retorna todos los comportamientos."""
        return self._behaviors.copy()
    
    def has_more_recoveries(self) -> bool:
        """Verifica si hay más comportamientos disponibles."""
        return self._current_index < len(self._behaviors)
