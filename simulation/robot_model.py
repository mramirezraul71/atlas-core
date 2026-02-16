"""
Robot Model: Modelo del robot para simulación.
===============================================
Carga y gestiona modelos URDF/MJCF.
"""
from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("simulation.robot_model")


@dataclass
class JointInfo:
    """Información de un joint."""
    name: str
    type: str  # revolute, prismatic, fixed
    axis: List[float]
    limits: List[float]  # [lower, upper]
    max_force: float
    max_velocity: float


@dataclass
class LinkInfo:
    """Información de un link."""
    name: str
    mass: float
    inertia: List[float]
    visual_mesh: Optional[str] = None
    collision_mesh: Optional[str] = None


@dataclass
class ModelConfig:
    """Configuración del modelo del robot."""
    name: str = "atlas_humanoid"
    num_joints: int = 20
    num_links: int = 21
    base_height: float = 0.5


class RobotModel:
    """
    Modelo del robot ATLAS para simulación.
    
    Define la cinemática, dinámica y geometría del robot.
    """
    
    def __init__(self, config: Optional[ModelConfig] = None):
        self.config = config or ModelConfig()
        self._joints: List[JointInfo] = []
        self._links: List[LinkInfo] = []
        self._loaded = False
        
        # Crear modelo por defecto
        self._create_default_model()
    
    def _create_default_model(self) -> None:
        """Crea un modelo humanoid por defecto."""
        # Definir joints del humanoid
        joint_names = [
            # Pierna derecha
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee", "right_ankle_pitch", "right_ankle_roll",
            # Pierna izquierda
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee", "left_ankle_pitch", "left_ankle_roll",
            # Torso
            "waist_yaw", "waist_pitch",
            # Brazo derecho
            "right_shoulder_pitch", "right_shoulder_roll", "right_elbow",
            # Brazo izquierdo
            "left_shoulder_pitch", "left_shoulder_roll", "left_elbow",
        ]
        
        for name in joint_names:
            self._joints.append(JointInfo(
                name=name,
                type="revolute",
                axis=[0, 0, 1],  # Simplificado
                limits=[-np.pi, np.pi],
                max_force=100.0,
                max_velocity=10.0,
            ))
        
        # Links
        link_names = ["base", "torso", "head"] + [f"link_{i}" for i in range(18)]
        for name in link_names:
            self._links.append(LinkInfo(
                name=name,
                mass=1.0,
                inertia=[0.1, 0.1, 0.1],
            ))
        
        self._loaded = True
    
    def load(self, path: str) -> Dict[str, Any]:
        """
        Carga un modelo desde archivo.
        
        Args:
            path: Ruta al archivo URDF/MJCF/JSON
            
        Returns:
            Resultado de la carga
        """
        model_path = Path(path)
        
        if not model_path.exists():
            # Usar modelo por defecto
            _log.info("Model file not found, using default humanoid model")
            return {"ok": True, "source": "default", "joints": len(self._joints)}
        
        suffix = model_path.suffix.lower()
        
        try:
            if suffix == ".json":
                return self._load_json(model_path)
            elif suffix == ".urdf":
                return self._load_urdf(model_path)
            elif suffix == ".xml":
                return self._load_mjcf(model_path)
            else:
                return {"ok": False, "error": f"Unknown format: {suffix}"}
        except Exception as e:
            _log.exception("Failed to load model: %s", e)
            return {"ok": False, "error": str(e)}
    
    def _load_json(self, path: Path) -> Dict[str, Any]:
        """Carga modelo desde JSON."""
        data = json.loads(path.read_text())
        
        self._joints = []
        for j in data.get("joints", []):
            self._joints.append(JointInfo(**j))
        
        self._links = []
        for l in data.get("links", []):
            self._links.append(LinkInfo(**l))
        
        self.config.num_joints = len(self._joints)
        self.config.num_links = len(self._links)
        self._loaded = True
        
        return {"ok": True, "source": "json", "joints": len(self._joints)}
    
    def _load_urdf(self, path: Path) -> Dict[str, Any]:
        """Carga modelo desde URDF (placeholder)."""
        # TODO: Implementar parser URDF completo
        _log.info("URDF loading not fully implemented, using default")
        return {"ok": True, "source": "urdf_placeholder", "joints": len(self._joints)}
    
    def _load_mjcf(self, path: Path) -> Dict[str, Any]:
        """Carga modelo desde MJCF (placeholder)."""
        # TODO: Implementar parser MJCF completo
        _log.info("MJCF loading not fully implemented, using default")
        return {"ok": True, "source": "mjcf_placeholder", "joints": len(self._joints)}
    
    def save(self, path: str) -> Dict[str, Any]:
        """Guarda el modelo a JSON."""
        data = {
            "name": self.config.name,
            "joints": [
                {
                    "name": j.name,
                    "type": j.type,
                    "axis": j.axis,
                    "limits": j.limits,
                    "max_force": j.max_force,
                    "max_velocity": j.max_velocity,
                }
                for j in self._joints
            ],
            "links": [
                {
                    "name": l.name,
                    "mass": l.mass,
                    "inertia": l.inertia,
                }
                for l in self._links
            ],
        }
        
        Path(path).write_text(json.dumps(data, indent=2))
        return {"ok": True, "path": path}
    
    @property
    def num_joints(self) -> int:
        return len(self._joints)
    
    @property
    def num_links(self) -> int:
        return len(self._links)
    
    def get_joint(self, name: str) -> Optional[JointInfo]:
        """Obtiene info de un joint por nombre."""
        for j in self._joints:
            if j.name == name:
                return j
        return None
    
    def get_joint_names(self) -> List[str]:
        """Retorna lista de nombres de joints."""
        return [j.name for j in self._joints]
    
    def forward_kinematics(self, joint_positions: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Calcula cinemática directa (placeholder).
        
        Args:
            joint_positions: Posiciones de joints
            
        Returns:
            Posiciones de cada link
        """
        # TODO: Implementar FK real
        positions = {}
        base_pos = np.array([0, 0, self.config.base_height])
        
        for i, link in enumerate(self._links):
            # Placeholder: posición basada en índice
            positions[link.name] = base_pos + np.array([0, 0, i * 0.1])
        
        return positions
    
    def to_dict(self) -> Dict[str, Any]:
        """Retorna info del modelo."""
        return {
            "name": self.config.name,
            "num_joints": self.num_joints,
            "num_links": self.num_links,
            "loaded": self._loaded,
            "joint_names": self.get_joint_names(),
        }
