"""
Hand Controller: Control de manos dextrosas.
=============================================
Control individual de dedos para manipulación fina.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("humanoid.manipulation.hand_controller")


@dataclass
class FingerState:
    """Estado de un dedo."""
    name: str
    joints: List[float]       # Posiciones de joints en radianes
    contact: bool = False
    force: float = 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "joints": self.joints,
            "contact": self.contact,
            "force": self.force,
        }


class HandController:
    """
    Controlador de mano diestra.
    
    Soporta:
    - Control individual de dedos
    - Poses predefinidas
    - Control de fuerza
    - Detección de contacto
    """
    
    # Dedos disponibles
    FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
    JOINTS_PER_FINGER = 4  # MCP, PIP, DIP, abd
    
    def __init__(self, hand_name: str = "right"):
        self.hand_name = hand_name
        self.num_joints = len(self.FINGERS) * self.JOINTS_PER_FINGER
        
        # Estado de cada dedo
        self._finger_states: Dict[str, FingerState] = {
            finger: FingerState(
                name=finger,
                joints=[0.0] * self.JOINTS_PER_FINGER,
            )
            for finger in self.FINGERS
        }
        
        # Límites de joints (radianes)
        self._joint_limits = {
            "min": np.array([-0.2, 0.0, 0.0, -0.3] * 5),
            "max": np.array([1.5, 1.5, 1.5, 0.3] * 5),
        }
        
        # Poses predefinidas
        self._poses = {
            "open": np.zeros(self.num_joints),
            "close": np.array([1.2, 1.2, 1.2, 0.0] * 5),
            "pinch": np.array([1.0, 0.8, 0.8, 0.2,  # thumb
                              0.8, 0.8, 0.6, 0.0,   # index
                              0.0, 0.0, 0.0, 0.0,   # middle
                              0.0, 0.0, 0.0, 0.0,   # ring
                              0.0, 0.0, 0.0, 0.0]), # pinky
            "power": np.array([1.0, 1.0, 1.0, 0.0,
                              1.0, 1.0, 1.0, 0.0,
                              1.0, 1.0, 1.0, 0.0,
                              1.0, 1.0, 1.0, 0.0,
                              1.0, 1.0, 1.0, 0.0]),
            "point": np.array([1.0, 1.0, 1.0, 0.2,  # thumb
                              0.0, 0.0, 0.0, 0.0,   # index extendido
                              1.2, 1.2, 1.2, 0.0,
                              1.2, 1.2, 1.2, 0.0,
                              1.2, 1.2, 1.2, 0.0]),
        }
    
    def set_pose(self, pose_name: str) -> bool:
        """
        Establece una pose predefinida.
        
        Args:
            pose_name: Nombre de la pose
            
        Returns:
            True si exitoso
        """
        if pose_name not in self._poses:
            _log.warning("Unknown pose: %s", pose_name)
            return False
        
        target = self._poses[pose_name]
        return self.set_joint_positions(target)
    
    def set_joint_positions(self, positions: np.ndarray) -> bool:
        """
        Establece posiciones de todos los joints.
        
        Args:
            positions: Array de posiciones (num_joints,)
            
        Returns:
            True si exitoso
        """
        if len(positions) != self.num_joints:
            _log.error("Invalid position array length: %d", len(positions))
            return False
        
        # Aplicar límites
        positions = np.clip(
            positions,
            self._joint_limits["min"],
            self._joint_limits["max"],
        )
        
        # Actualizar estado de cada dedo
        for i, finger in enumerate(self.FINGERS):
            start = i * self.JOINTS_PER_FINGER
            end = start + self.JOINTS_PER_FINGER
            self._finger_states[finger].joints = positions[start:end].tolist()
        
        _log.debug("Set hand %s to positions", self.hand_name)
        return True
    
    def set_finger(self, finger: str, positions: List[float]) -> bool:
        """
        Establece posición de un dedo específico.
        
        Args:
            finger: Nombre del dedo
            positions: Posiciones de joints
            
        Returns:
            True si exitoso
        """
        if finger not in self.FINGERS:
            _log.warning("Unknown finger: %s", finger)
            return False
        
        if len(positions) != self.JOINTS_PER_FINGER:
            _log.error("Invalid positions length for finger")
            return False
        
        self._finger_states[finger].joints = list(positions)
        return True
    
    def get_joint_positions(self) -> np.ndarray:
        """Retorna posiciones actuales de todos los joints."""
        positions = []
        for finger in self.FINGERS:
            positions.extend(self._finger_states[finger].joints)
        return np.array(positions)
    
    def get_finger_state(self, finger: str) -> Optional[FingerState]:
        """Retorna estado de un dedo."""
        return self._finger_states.get(finger)
    
    def update_contacts(self, contacts: Dict[str, bool]) -> None:
        """Actualiza estado de contacto de dedos."""
        for finger, contact in contacts.items():
            if finger in self._finger_states:
                self._finger_states[finger].contact = contact
    
    def update_forces(self, forces: Dict[str, float]) -> None:
        """Actualiza fuerzas medidas en dedos."""
        for finger, force in forces.items():
            if finger in self._finger_states:
                self._finger_states[finger].force = force
    
    def get_contacts(self) -> Dict[str, bool]:
        """Retorna estado de contacto de todos los dedos."""
        return {f: s.contact for f, s in self._finger_states.items()}
    
    def get_total_force(self) -> float:
        """Retorna fuerza total de agarre."""
        return sum(s.force for s in self._finger_states.values())
    
    def is_grasping(self, force_threshold: float = 1.0) -> bool:
        """Verifica si hay un agarre activo."""
        contacts = sum(1 for s in self._finger_states.values() if s.contact)
        total_force = self.get_total_force()
        return contacts >= 2 and total_force >= force_threshold
    
    def interpolate_to(
        self,
        target: np.ndarray,
        steps: int = 10,
    ) -> List[np.ndarray]:
        """
        Genera trayectoria interpolada.
        
        Args:
            target: Posiciones objetivo
            steps: Número de pasos
            
        Returns:
            Lista de posiciones intermedias
        """
        current = self.get_joint_positions()
        trajectory = []
        
        for i in range(steps + 1):
            t = i / steps
            pos = current + t * (target - current)
            trajectory.append(pos)
        
        return trajectory
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "hand_name": self.hand_name,
            "fingers": {f: s.to_dict() for f, s in self._finger_states.items()},
            "total_force": self.get_total_force(),
            "is_grasping": self.is_grasping(),
        }
