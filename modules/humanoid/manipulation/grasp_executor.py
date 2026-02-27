"""
Grasp Executor: Ejecución de agarres.
======================================
Ejecuta la secuencia de movimientos para agarrar.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Callable
import numpy as np

from .grasp_planner import GraspCandidate

_log = logging.getLogger("humanoid.manipulation.grasp_executor")


class GraspState(str, Enum):
    """Estados del ejecutor de agarre."""
    IDLE = "idle"
    APPROACHING = "approaching"
    CLOSING = "closing"
    GRASPED = "grasped"
    LIFTING = "lifting"
    RELEASING = "releasing"
    ERROR = "error"


@dataclass
class GraspResult:
    """Resultado de un intento de agarre."""
    success: bool
    grasp: Optional[GraspCandidate]
    state: GraspState
    elapsed_time: float
    error_message: Optional[str] = None
    force_applied: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "success": self.success,
            "state": self.state.value,
            "elapsed_time": self.elapsed_time,
            "error_message": self.error_message,
            "force_applied": self.force_applied,
        }


class GraspExecutor:
    """
    Ejecutor de agarres para ATLAS.
    
    Secuencia de ejecución:
    1. Mover a posición de aproximación
    2. Aproximar al punto de agarre
    3. Cerrar gripper/dedos
    4. Verificar agarre (fuerza)
    5. Levantar objeto
    """
    
    def __init__(self):
        self._state = GraspState.IDLE
        self._current_grasp: Optional[GraspCandidate] = None
        
        # Callbacks para control de hardware
        self._move_arm_callback: Optional[Callable] = None
        self._move_gripper_callback: Optional[Callable] = None
        self._get_force_callback: Optional[Callable] = None
        
        # Parámetros
        self._approach_speed = 0.1  # m/s
        self._grasp_force = 10.0    # N
        self._lift_height = 0.1     # m
        
        # Timeouts
        self._approach_timeout = 10.0
        self._grasp_timeout = 5.0
    
    def set_callbacks(
        self,
        move_arm: Optional[Callable] = None,
        move_gripper: Optional[Callable] = None,
        get_force: Optional[Callable] = None,
    ) -> None:
        """Configura callbacks de hardware."""
        self._move_arm_callback = move_arm
        self._move_gripper_callback = move_gripper
        self._get_force_callback = get_force
    
    def execute(self, grasp: GraspCandidate, dry_run: bool = False) -> GraspResult:
        """
        Ejecuta un agarre.
        
        Args:
            grasp: Candidato de agarre a ejecutar
            dry_run: Si True, solo simula sin mover
            
        Returns:
            Resultado del agarre
        """
        t0 = time.time()
        self._current_grasp = grasp
        
        try:
            # 1. Ir a posición de aproximación
            self._state = GraspState.APPROACHING
            approach_pos = grasp.position + grasp.approach_vector * 0.15
            
            if not dry_run:
                success = self._move_to(approach_pos, grasp.orientation)
                if not success:
                    raise RuntimeError("Failed to reach approach position")
            
            _log.info("Reached approach position")
            
            # 2. Aproximar al punto de agarre
            if not dry_run:
                success = self._move_to(grasp.position, grasp.orientation)
                if not success:
                    raise RuntimeError("Failed to reach grasp position")
            
            _log.info("Reached grasp position")
            
            # 3. Cerrar gripper
            self._state = GraspState.CLOSING
            
            if not dry_run:
                success = self._close_gripper(grasp.width)
                if not success:
                    raise RuntimeError("Failed to close gripper")
            
            # 4. Verificar agarre
            force = 0.0
            if self._get_force_callback and not dry_run:
                force = self._get_force_callback()
                if force < 1.0:  # Umbral mínimo
                    _log.warning("Grasp force too low: %.2f N", force)
            
            self._state = GraspState.GRASPED
            _log.info("Object grasped with force %.2f N", force)
            
            # 5. Levantar
            self._state = GraspState.LIFTING
            lift_pos = grasp.position.copy()
            lift_pos[2] += self._lift_height
            
            if not dry_run:
                success = self._move_to(lift_pos, grasp.orientation)
                if not success:
                    raise RuntimeError("Failed to lift object")
            
            _log.info("Object lifted")
            
            elapsed = time.time() - t0
            return GraspResult(
                success=True,
                grasp=grasp,
                state=self._state,
                elapsed_time=elapsed,
                force_applied=force,
            )
            
        except Exception as e:
            self._state = GraspState.ERROR
            elapsed = time.time() - t0
            _log.exception("Grasp execution failed: %s", e)
            return GraspResult(
                success=False,
                grasp=grasp,
                state=self._state,
                elapsed_time=elapsed,
                error_message=str(e),
            )
    
    def _move_to(self, position: np.ndarray, orientation: np.ndarray) -> bool:
        """Mueve el brazo a una posición."""
        if self._move_arm_callback:
            return self._move_arm_callback(position, orientation)
        
        # Simulación
        time.sleep(0.5)
        return True
    
    def _close_gripper(self, target_width: float) -> bool:
        """Cierra el gripper."""
        if self._move_gripper_callback:
            return self._move_gripper_callback(target_width, self._grasp_force)
        
        # Simulación
        time.sleep(0.3)
        return True
    
    def release(self) -> bool:
        """Libera el objeto."""
        self._state = GraspState.RELEASING
        
        if self._move_gripper_callback:
            success = self._move_gripper_callback(0.08, 0)  # Abrir completamente
        else:
            time.sleep(0.2)
            success = True
        
        if success:
            self._state = GraspState.IDLE
            self._current_grasp = None
        
        return success
    
    def abort(self) -> None:
        """Aborta el agarre actual."""
        _log.warning("Grasp aborted")
        self.release()
        self._state = GraspState.IDLE
    
    @property
    def state(self) -> GraspState:
        return self._state
    
    def is_grasping(self) -> bool:
        return self._state == GraspState.GRASPED
    
    def get_status(self) -> Dict[str, Any]:
        return {
            "state": self._state.value,
            "has_grasp": self._current_grasp is not None,
            "grasp": self._current_grasp.to_dict() if self._current_grasp else None,
        }
