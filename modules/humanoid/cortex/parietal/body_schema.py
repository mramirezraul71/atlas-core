"""
BodySchema: Modelo del propio cuerpo del lóbulo parietal.

Análogo biológico: Corteza parietal (áreas 5 y 7) + corteza somatosensorial
- Representación del cuerpo y sus límites
- Propiocepción (posición de articulaciones)
- Cinemática del cuerpo
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class JointState:
    """Estado de una articulación."""
    joint_id: str
    name: str
    position: float = 0.0  # rad
    velocity: float = 0.0  # rad/s
    effort: float = 0.0  # Nm
    min_position: float = -math.pi
    max_position: float = math.pi
    max_velocity: float = 2.0  # rad/s
    max_effort: float = 50.0  # Nm
    temperature: float = 25.0
    
    def is_at_limit(self, margin: float = 0.05) -> bool:
        """Verifica si está cerca de un límite."""
        return (
            self.position <= self.min_position + margin or
            self.position >= self.max_position - margin
        )
    
    def normalized_position(self) -> float:
        """Posición normalizada entre 0 y 1."""
        range_ = self.max_position - self.min_position
        if range_ == 0:
            return 0.5
        return (self.position - self.min_position) / range_


@dataclass
class LinkState:
    """Estado de un eslabón del cuerpo."""
    link_id: str
    name: str
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, z
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # quaternion
    linear_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass
class BodyPart:
    """Parte del cuerpo con articulaciones y eslabones."""
    part_id: str
    name: str
    joints: List[str] = field(default_factory=list)  # IDs de articulaciones
    links: List[str] = field(default_factory=list)  # IDs de eslabones
    is_end_effector: bool = False


class BodySchema:
    """
    Modelo del propio cuerpo (body schema).
    
    Mantiene representación de:
    - Estructura del cuerpo (articulaciones, eslabones)
    - Estado actual (posiciones, velocidades)
    - Límites y capacidades
    - Cinemática directa
    """
    
    # Configuración por defecto para robot humanoide
    DEFAULT_JOINTS = {
        # Torso
        "torso_yaw": {"name": "Torso Yaw", "min": -1.0, "max": 1.0},
        
        # Brazo derecho
        "r_shoulder_pitch": {"name": "Right Shoulder Pitch", "min": -2.0, "max": 2.0},
        "r_shoulder_roll": {"name": "Right Shoulder Roll", "min": -1.5, "max": 1.5},
        "r_shoulder_yaw": {"name": "Right Shoulder Yaw", "min": -1.5, "max": 1.5},
        "r_elbow": {"name": "Right Elbow", "min": 0.0, "max": 2.5},
        "r_wrist_yaw": {"name": "Right Wrist Yaw", "min": -1.5, "max": 1.5},
        "r_wrist_pitch": {"name": "Right Wrist Pitch", "min": -0.5, "max": 0.5},
        "r_wrist_roll": {"name": "Right Wrist Roll", "min": -1.5, "max": 1.5},
        
        # Brazo izquierdo
        "l_shoulder_pitch": {"name": "Left Shoulder Pitch", "min": -2.0, "max": 2.0},
        "l_shoulder_roll": {"name": "Left Shoulder Roll", "min": -1.5, "max": 1.5},
        "l_shoulder_yaw": {"name": "Left Shoulder Yaw", "min": -1.5, "max": 1.5},
        "l_elbow": {"name": "Left Elbow", "min": 0.0, "max": 2.5},
        "l_wrist_yaw": {"name": "Left Wrist Yaw", "min": -1.5, "max": 1.5},
        "l_wrist_pitch": {"name": "Left Wrist Pitch", "min": -0.5, "max": 0.5},
        "l_wrist_roll": {"name": "Left Wrist Roll", "min": -1.5, "max": 1.5},
        
        # Cuello
        "neck_yaw": {"name": "Neck Yaw", "min": -1.5, "max": 1.5},
        "neck_pitch": {"name": "Neck Pitch", "min": -0.5, "max": 0.7},
        
        # Manos
        "r_gripper": {"name": "Right Gripper", "min": 0.0, "max": 1.0},
        "l_gripper": {"name": "Left Gripper", "min": 0.0, "max": 1.0},
    }
    
    DEFAULT_PARTS = {
        "head": {"name": "Head", "joints": ["neck_yaw", "neck_pitch"], "is_end_effector": False},
        "torso": {"name": "Torso", "joints": ["torso_yaw"], "is_end_effector": False},
        "right_arm": {
            "name": "Right Arm",
            "joints": ["r_shoulder_pitch", "r_shoulder_roll", "r_shoulder_yaw", "r_elbow", "r_wrist_yaw", "r_wrist_pitch", "r_wrist_roll"],
            "is_end_effector": True,
        },
        "left_arm": {
            "name": "Left Arm",
            "joints": ["l_shoulder_pitch", "l_shoulder_roll", "l_shoulder_yaw", "l_elbow", "l_wrist_yaw", "l_wrist_pitch", "l_wrist_roll"],
            "is_end_effector": True,
        },
        "right_hand": {"name": "Right Hand", "joints": ["r_gripper"], "is_end_effector": True},
        "left_hand": {"name": "Left Hand", "joints": ["l_gripper"], "is_end_effector": True},
    }
    
    def __init__(self, config: Dict[str, Any] = None):
        """
        Inicializa el esquema corporal.
        
        Args:
            config: Configuración personalizada del cuerpo
        """
        # Inicializar articulaciones
        self.joints: Dict[str, JointState] = {}
        joint_config = config.get("joints", self.DEFAULT_JOINTS) if config else self.DEFAULT_JOINTS
        
        for joint_id, jconfig in joint_config.items():
            self.joints[joint_id] = JointState(
                joint_id=joint_id,
                name=jconfig.get("name", joint_id),
                min_position=jconfig.get("min", -math.pi),
                max_position=jconfig.get("max", math.pi),
            )
        
        # Inicializar partes del cuerpo
        self.parts: Dict[str, BodyPart] = {}
        part_config = config.get("parts", self.DEFAULT_PARTS) if config else self.DEFAULT_PARTS
        
        for part_id, pconfig in part_config.items():
            self.parts[part_id] = BodyPart(
                part_id=part_id,
                name=pconfig.get("name", part_id),
                joints=pconfig.get("joints", []),
                is_end_effector=pconfig.get("is_end_effector", False),
            )
        
        # Eslabones (links)
        self.links: Dict[str, LinkState] = {}
        
        # Timestamp de última actualización
        self.last_updated_ns: int = 0
    
    def update_joint(self, joint_id: str, position: float = None, 
                    velocity: float = None, effort: float = None,
                    temperature: float = None) -> bool:
        """
        Actualiza estado de una articulación.
        
        Returns:
            True si la articulación existe
        """
        if joint_id not in self.joints:
            return False
        
        joint = self.joints[joint_id]
        
        if position is not None:
            joint.position = position
        if velocity is not None:
            joint.velocity = velocity
        if effort is not None:
            joint.effort = effort
        if temperature is not None:
            joint.temperature = temperature
        
        self.last_updated_ns = time.time_ns()
        return True
    
    def update_from_encoders(self, encoder_data: Dict[str, Dict[str, float]]) -> None:
        """
        Actualiza múltiples articulaciones desde datos de encoders.
        
        Args:
            encoder_data: {joint_id: {position, velocity, effort}}
        """
        for joint_id, data in encoder_data.items():
            self.update_joint(
                joint_id,
                position=data.get("position"),
                velocity=data.get("velocity"),
                effort=data.get("effort"),
            )
    
    def get_joint_positions(self, part_id: str = None) -> Dict[str, float]:
        """
        Obtiene posiciones de articulaciones.
        
        Args:
            part_id: Si se especifica, solo articulaciones de esa parte
        
        Returns:
            Diccionario {joint_id: position}
        """
        if part_id:
            part = self.parts.get(part_id)
            if not part:
                return {}
            joint_ids = part.joints
        else:
            joint_ids = list(self.joints.keys())
        
        return {jid: self.joints[jid].position for jid in joint_ids if jid in self.joints}
    
    def set_joint_positions(self, positions: Dict[str, float], validate: bool = True) -> Dict[str, str]:
        """
        Establece posiciones de articulaciones.
        
        Args:
            positions: {joint_id: target_position}
            validate: Si True, valida límites
        
        Returns:
            Diccionario con errores {joint_id: error_message}
        """
        errors = {}
        
        for joint_id, position in positions.items():
            if joint_id not in self.joints:
                errors[joint_id] = "Joint not found"
                continue
            
            joint = self.joints[joint_id]
            
            if validate:
                if position < joint.min_position:
                    errors[joint_id] = f"Below min limit ({joint.min_position})"
                    continue
                if position > joint.max_position:
                    errors[joint_id] = f"Above max limit ({joint.max_position})"
                    continue
            
            joint.position = position
        
        self.last_updated_ns = time.time_ns()
        return errors
    
    def get_joints_at_limits(self) -> List[str]:
        """Obtiene articulaciones que están en sus límites."""
        return [jid for jid, joint in self.joints.items() if joint.is_at_limit()]
    
    def get_overheating_joints(self, threshold: float = 60.0) -> List[str]:
        """Obtiene articulaciones con temperatura alta."""
        return [jid for jid, joint in self.joints.items() if joint.temperature > threshold]
    
    def get_high_effort_joints(self, threshold_ratio: float = 0.8) -> List[str]:
        """Obtiene articulaciones con esfuerzo alto."""
        return [
            jid for jid, joint in self.joints.items()
            if abs(joint.effort) > joint.max_effort * threshold_ratio
        ]
    
    def get_part_state(self, part_id: str) -> Dict[str, Any]:
        """Obtiene estado completo de una parte del cuerpo."""
        part = self.parts.get(part_id)
        if not part:
            return {}
        
        joint_states = {}
        for jid in part.joints:
            if jid in self.joints:
                joint = self.joints[jid]
                joint_states[jid] = {
                    "position": joint.position,
                    "velocity": joint.velocity,
                    "effort": joint.effort,
                    "normalized": joint.normalized_position(),
                    "at_limit": joint.is_at_limit(),
                }
        
        return {
            "part_id": part_id,
            "name": part.name,
            "is_end_effector": part.is_end_effector,
            "joints": joint_states,
        }
    
    def is_gripper_closed(self, hand: str = "right") -> bool:
        """Verifica si el gripper está cerrado."""
        gripper_id = f"{'r' if hand == 'right' else 'l'}_gripper"
        if gripper_id in self.joints:
            return self.joints[gripper_id].position < 0.1
        return False
    
    def is_gripper_open(self, hand: str = "right") -> bool:
        """Verifica si el gripper está abierto."""
        gripper_id = f"{'r' if hand == 'right' else 'l'}_gripper"
        if gripper_id in self.joints:
            return self.joints[gripper_id].position > 0.9
        return False
    
    def get_arm_reach(self, arm: str = "right") -> float:
        """
        Estima alcance actual del brazo basado en posición del codo.
        
        Args:
            arm: "right" o "left"
        
        Returns:
            Alcance estimado en metros (0 a 1)
        """
        prefix = "r" if arm == "right" else "l"
        elbow_id = f"{prefix}_elbow"
        
        if elbow_id in self.joints:
            elbow = self.joints[elbow_id]
            # Codo extendido = mayor alcance
            extension = 1.0 - elbow.normalized_position()
            return extension
        
        return 0.5  # Default
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte esquema a diccionario."""
        return {
            "last_updated_ns": self.last_updated_ns,
            "joints_count": len(self.joints),
            "parts_count": len(self.parts),
            "joints_at_limits": self.get_joints_at_limits(),
            "overheating_joints": self.get_overheating_joints(),
            "high_effort_joints": self.get_high_effort_joints(),
            "right_gripper_closed": self.is_gripper_closed("right"),
            "left_gripper_closed": self.is_gripper_closed("left"),
        }
