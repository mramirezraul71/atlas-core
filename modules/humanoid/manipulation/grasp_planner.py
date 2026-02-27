"""
Grasp Planner: Planificación de agarres.
=========================================
Genera candidatos de agarre para objetos.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.manipulation.grasp_planner")


@dataclass
class GraspConfig:
    """Configuración del planificador de agarres."""
    num_candidates: int = 20
    approach_distance: float = 0.1      # Distancia de aproximación
    gripper_width: float = 0.08         # Apertura del gripper
    min_quality_threshold: float = 0.3
    collision_check: bool = True


@dataclass
class GraspCandidate:
    """Un candidato de agarre."""
    position: np.ndarray       # Posición del gripper [x, y, z]
    orientation: np.ndarray    # Orientación (quaternion)
    approach_vector: np.ndarray  # Vector de aproximación
    width: float               # Apertura requerida
    quality: float             # Score de calidad (0-1)
    grasp_type: str = "parallel"  # parallel, power, precision
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
            "approach_vector": self.approach_vector.tolist(),
            "width": self.width,
            "quality": self.quality,
            "grasp_type": self.grasp_type,
        }


class GraspPlanner:
    """
    Planificador de agarres para ATLAS.
    
    Genera candidatos de agarre basados en:
    - Geometría del objeto (point cloud)
    - Restricciones del gripper
    - Antipodal grasp sampling
    """
    
    def __init__(self, config: Optional[GraspConfig] = None):
        self.config = config or GraspConfig()
        self._object_cloud: Optional[np.ndarray] = None
        self._candidates: List[GraspCandidate] = []
    
    def plan_grasp(
        self,
        point_cloud: np.ndarray,
        normals: Optional[np.ndarray] = None,
    ) -> List[GraspCandidate]:
        """
        Planifica agarres para un objeto.
        
        Args:
            point_cloud: Puntos del objeto (N x 3)
            normals: Normales opcionales (N x 3)
            
        Returns:
            Lista de candidatos ordenados por calidad
        """
        self._object_cloud = point_cloud
        self._candidates = []
        
        if len(point_cloud) < 10:
            _log.warning("Point cloud too small for grasp planning")
            return []
        
        # Calcular normales si no se proporcionan
        if normals is None:
            normals = self._estimate_normals(point_cloud)
        
        # Generar candidatos antipodales
        candidates = self._sample_antipodal_grasps(point_cloud, normals)
        
        # Filtrar por calidad
        candidates = [c for c in candidates if c.quality >= self.config.min_quality_threshold]
        
        # Ordenar por calidad
        candidates.sort(key=lambda c: c.quality, reverse=True)
        
        self._candidates = candidates[:self.config.num_candidates]
        
        _log.info("Generated %d grasp candidates", len(self._candidates))
        
        return self._candidates
    
    def _estimate_normals(self, points: np.ndarray) -> np.ndarray:
        """Estima normales de superficie."""
        # PCA local simplificado
        normals = np.zeros_like(points)
        
        for i, p in enumerate(points):
            # Encontrar vecinos cercanos
            distances = np.linalg.norm(points - p, axis=1)
            neighbors_idx = np.argsort(distances)[1:min(10, len(points))]
            
            if len(neighbors_idx) < 3:
                normals[i] = [0, 0, 1]
                continue
            
            neighbors = points[neighbors_idx]
            
            # PCA
            centered = neighbors - np.mean(neighbors, axis=0)
            try:
                _, _, vh = np.linalg.svd(centered)
                normals[i] = vh[-1]  # Menor componente principal
            except:
                normals[i] = [0, 0, 1]
        
        return normals
    
    def _sample_antipodal_grasps(
        self,
        points: np.ndarray,
        normals: np.ndarray,
    ) -> List[GraspCandidate]:
        """Samplea agarres antipodales."""
        candidates = []
        centroid = np.mean(points, axis=0)
        
        for _ in range(self.config.num_candidates * 3):  # Oversample
            # Seleccionar dos puntos aleatorios
            idx1, idx2 = np.random.choice(len(points), 2, replace=False)
            p1, p2 = points[idx1], points[idx2]
            n1, n2 = normals[idx1], normals[idx2]
            
            # Verificar condición antipodal
            grasp_axis = p2 - p1
            grasp_width = np.linalg.norm(grasp_axis)
            
            if grasp_width > self.config.gripper_width or grasp_width < 0.02:
                continue
            
            grasp_axis_normalized = grasp_axis / grasp_width
            
            # Las normales deben apuntar aproximadamente opuestas al eje de agarre
            alignment1 = np.abs(np.dot(n1, grasp_axis_normalized))
            alignment2 = np.abs(np.dot(n2, grasp_axis_normalized))
            
            if alignment1 < 0.5 or alignment2 < 0.5:
                continue
            
            # Posición del gripper (centro del agarre)
            position = (p1 + p2) / 2
            
            # Orientación: eje z hacia abajo, eje x a lo largo del agarre
            z_axis = np.array([0, 0, -1])  # Aproximación hacia abajo
            x_axis = grasp_axis_normalized
            y_axis = np.cross(z_axis, x_axis)
            y_norm = np.linalg.norm(y_axis)
            if y_norm < 0.1:
                continue
            y_axis /= y_norm
            z_axis = np.cross(x_axis, y_axis)
            
            # Convertir a quaternion (simplificado)
            rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
            orientation = self._matrix_to_quaternion(rotation_matrix)
            
            # Calcular calidad
            quality = self._compute_grasp_quality(position, grasp_axis_normalized, points)
            
            candidates.append(GraspCandidate(
                position=position,
                orientation=orientation,
                approach_vector=-z_axis,
                width=grasp_width,
                quality=quality,
            ))
        
        return candidates
    
    def _compute_grasp_quality(
        self,
        position: np.ndarray,
        grasp_axis: np.ndarray,
        points: np.ndarray,
    ) -> float:
        """Calcula la calidad de un agarre."""
        # Factores de calidad:
        
        # 1. Proximidad al centroide
        centroid = np.mean(points, axis=0)
        dist_to_centroid = np.linalg.norm(position - centroid)
        centroid_score = np.exp(-dist_to_centroid * 2)
        
        # 2. Simetría del agarre
        distances = np.linalg.norm(points - position, axis=1)
        mean_dist = np.mean(distances)
        std_dist = np.std(distances)
        symmetry_score = 1 / (1 + std_dist / max(mean_dist, 0.01))
        
        # 3. Puntos de contacto cercanos
        contact_points = np.sum(distances < 0.03)
        contact_score = min(1.0, contact_points / 10)
        
        # Score combinado
        quality = 0.4 * centroid_score + 0.3 * symmetry_score + 0.3 * contact_score
        
        return float(quality)
    
    def _matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convierte matriz de rotación a quaternion."""
        # Algoritmo de Shepperd
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        return np.array([w, x, y, z])
    
    def get_candidates(self) -> List[GraspCandidate]:
        """Retorna los candidatos generados."""
        return self._candidates.copy()
    
    def get_best_grasp(self) -> Optional[GraspCandidate]:
        """Retorna el mejor candidato."""
        if self._candidates:
            return self._candidates[0]
        return None
