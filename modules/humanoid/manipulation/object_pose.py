"""
Object Pose Estimation: Estimación de pose de objetos.
========================================================
Detecta y estima la pose de objetos para manipulación.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.manipulation.object_pose")


@dataclass
class ObjectPose:
    """Pose estimada de un objeto."""
    position: np.ndarray       # [x, y, z]
    orientation: np.ndarray    # Quaternion [w, x, y, z]
    dimensions: np.ndarray     # [width, height, depth]
    confidence: float
    class_name: str = "unknown"
    class_id: int = -1
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
            "dimensions": self.dimensions.tolist(),
            "confidence": self.confidence,
            "class_name": self.class_name,
            "class_id": self.class_id,
        }


class ObjectPoseEstimator:
    """
    Estimador de pose de objetos.
    
    Métodos:
    - Point cloud based (ICP, RANSAC)
    - Template matching
    - Deep learning (placeholder)
    """
    
    def __init__(self):
        # Modelos de objetos conocidos
        self._known_objects: Dict[str, Dict[str, Any]] = {}
        
        # Última detección
        self._last_detections: List[ObjectPose] = []
    
    def register_object(
        self,
        name: str,
        template_cloud: Optional[np.ndarray] = None,
        dimensions: Optional[np.ndarray] = None,
    ) -> None:
        """Registra un objeto conocido."""
        self._known_objects[name] = {
            "template": template_cloud,
            "dimensions": dimensions if dimensions is not None else np.array([0.1, 0.1, 0.1]),
        }
        _log.info("Registered object: %s", name)
    
    def estimate_from_pointcloud(
        self,
        point_cloud: np.ndarray,
        object_name: Optional[str] = None,
    ) -> List[ObjectPose]:
        """
        Estima poses desde point cloud.
        
        Args:
            point_cloud: Puntos del objeto (N x 3)
            object_name: Nombre si es conocido
            
        Returns:
            Lista de poses detectadas
        """
        if len(point_cloud) < 10:
            return []
        
        detections = []
        
        # Segmentar clusters (simplificado)
        clusters = self._segment_clusters(point_cloud)
        
        for cluster in clusters:
            # Calcular bounding box
            min_pt = np.min(cluster, axis=0)
            max_pt = np.max(cluster, axis=0)
            centroid = (min_pt + max_pt) / 2
            dimensions = max_pt - min_pt
            
            # Estimar orientación (PCA)
            centered = cluster - centroid
            try:
                _, _, vh = np.linalg.svd(centered)
                # Construir rotación desde ejes principales
                orientation = self._axes_to_quaternion(vh)
            except:
                orientation = np.array([1, 0, 0, 0])
            
            # Si hay objeto conocido, refinar con ICP
            if object_name and object_name in self._known_objects:
                obj_info = self._known_objects[object_name]
                if obj_info["template"] is not None:
                    # ICP refinement (placeholder)
                    pass
                dimensions = obj_info["dimensions"]
            
            # Calcular confianza
            confidence = min(1.0, len(cluster) / 500)
            
            detections.append(ObjectPose(
                position=centroid,
                orientation=orientation,
                dimensions=dimensions,
                confidence=confidence,
                class_name=object_name or "unknown",
            ))
        
        self._last_detections = detections
        return detections
    
    def estimate_from_image(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        intrinsics: Dict[str, float],
    ) -> List[ObjectPose]:
        """
        Estima poses desde imagen RGB-D.
        
        Args:
            rgb: Imagen RGB (H x W x 3)
            depth: Mapa de profundidad (H x W)
            intrinsics: Parámetros de cámara
            
        Returns:
            Lista de poses
        """
        # Convertir a point cloud
        fx, fy = intrinsics.get("fx", 500), intrinsics.get("fy", 500)
        cx, cy = intrinsics.get("cx", 320), intrinsics.get("cy", 240)
        
        h, w = depth.shape
        v, u = np.meshgrid(np.arange(h), np.arange(w), indexing='ij')
        
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        valid = (z.reshape(-1) > 0.1) & (z.reshape(-1) < 3.0)
        points = points[valid]
        
        return self.estimate_from_pointcloud(points)
    
    def _segment_clusters(
        self,
        points: np.ndarray,
        eps: float = 0.05,
        min_points: int = 20,
    ) -> List[np.ndarray]:
        """Segmenta point cloud en clusters."""
        # DBSCAN simplificado
        clusters = []
        visited = np.zeros(len(points), dtype=bool)
        
        for i, point in enumerate(points):
            if visited[i]:
                continue
            
            # Encontrar vecinos
            distances = np.linalg.norm(points - point, axis=1)
            neighbors = np.where(distances < eps)[0]
            
            if len(neighbors) < min_points:
                continue
            
            # Expandir cluster
            cluster_indices = set(neighbors.tolist())
            visited[list(cluster_indices)] = True
            
            # Expansión simple (una iteración)
            for n_idx in list(cluster_indices):
                distances = np.linalg.norm(points - points[n_idx], axis=1)
                new_neighbors = np.where(distances < eps)[0]
                cluster_indices.update(new_neighbors.tolist())
            
            cluster = points[list(cluster_indices)]
            if len(cluster) >= min_points:
                clusters.append(cluster)
                visited[list(cluster_indices)] = True
        
        return clusters
    
    def _axes_to_quaternion(self, axes: np.ndarray) -> np.ndarray:
        """Convierte ejes principales a quaternion."""
        # Asegurar rotación propia
        R = axes.T
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1
        
        # Matriz a quaternion (algoritmo de Shepperd)
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
    
    def get_last_detections(self) -> List[ObjectPose]:
        return self._last_detections.copy()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "known_objects": list(self._known_objects.keys()),
            "last_detections": [d.to_dict() for d in self._last_detections],
        }
