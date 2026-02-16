"""
Depth Camera: Procesamiento de cámaras de profundidad.
=======================================================
Soporta Intel RealSense, Azure Kinect, etc.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.sensors.depth")


@dataclass
class DepthFrame:
    """Frame de profundidad."""
    timestamp: float
    depth: np.ndarray          # H x W, valores en metros
    color: Optional[np.ndarray] = None  # H x W x 3, RGB
    intrinsics: Optional[Dict[str, float]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "depth_shape": self.depth.shape,
            "has_color": self.color is not None,
            "intrinsics": self.intrinsics,
        }


class DepthCamera:
    """
    Procesador de cámara de profundidad.
    
    Funcionalidades:
    - Point cloud generation
    - Obstacle detection
    - Plane detection (suelo)
    - Visual odometry básica
    """
    
    def __init__(self, width: int = 640, height: int = 480):
        self.width = width
        self.height = height
        
        # Intrínsecos por defecto (RealSense D435)
        self.intrinsics = {
            "fx": 380.0,
            "fy": 380.0,
            "cx": width / 2,
            "cy": height / 2,
        }
        
        # Estado
        self._last_frame: Optional[DepthFrame] = None
        self._last_pointcloud: Optional[np.ndarray] = None
        
        # Visual odometry
        self._vo_position = np.zeros(3)
        self._vo_features: Optional[np.ndarray] = None
    
    def process(self, depth_data: np.ndarray, color_data: Optional[np.ndarray] = None) -> DepthFrame:
        """
        Procesa un frame de profundidad.
        
        Args:
            depth_data: Array de profundidad (H x W)
            color_data: Array RGB opcional (H x W x 3)
            
        Returns:
            Frame procesado
        """
        import time
        
        # Filtrar valores inválidos
        depth = depth_data.copy()
        depth[depth <= 0] = np.nan
        depth[depth > 10.0] = np.nan  # Max 10 metros
        
        # Aplicar filtro bilateral simple (promedio local)
        depth = self._median_filter(depth, kernel_size=3)
        
        frame = DepthFrame(
            timestamp=time.time(),
            depth=depth,
            color=color_data,
            intrinsics=self.intrinsics.copy(),
        )
        
        # Actualizar visual odometry
        if self._last_frame is not None:
            self._update_visual_odometry(frame)
        
        self._last_frame = frame
        
        return frame
    
    def _median_filter(self, depth: np.ndarray, kernel_size: int = 3) -> np.ndarray:
        """Filtro de mediana simple."""
        from scipy import ndimage
        try:
            return ndimage.median_filter(np.nan_to_num(depth), size=kernel_size)
        except ImportError:
            return depth  # Sin scipy, retornar sin filtrar
    
    def depth_to_pointcloud(self, frame: DepthFrame) -> np.ndarray:
        """
        Convierte depth a point cloud.
        
        Args:
            frame: Frame de profundidad
            
        Returns:
            Point cloud (N x 3)
        """
        fx = self.intrinsics["fx"]
        fy = self.intrinsics["fy"]
        cx = self.intrinsics["cx"]
        cy = self.intrinsics["cy"]
        
        # Crear grilla de coordenadas
        v, u = np.meshgrid(np.arange(frame.depth.shape[0]),
                          np.arange(frame.depth.shape[1]),
                          indexing='ij')
        
        # Convertir a 3D
        z = frame.depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # Stack y filtrar NaN
        points = np.stack([x, y, z], axis=-1)
        points = points.reshape(-1, 3)
        valid = ~np.any(np.isnan(points), axis=1)
        points = points[valid]
        
        self._last_pointcloud = points
        
        return points
    
    def detect_obstacles(self, frame: DepthFrame, min_height: float = 0.1, max_dist: float = 3.0) -> List[Dict[str, Any]]:
        """
        Detecta obstáculos en el frame.
        
        Args:
            frame: Frame de profundidad
            min_height: Altura mínima para considerar obstáculo
            max_dist: Distancia máxima a considerar
            
        Returns:
            Lista de obstáculos detectados
        """
        # Generar point cloud
        points = self.depth_to_pointcloud(frame)
        
        if len(points) == 0:
            return []
        
        # Filtrar por distancia
        points = points[points[:, 2] < max_dist]
        
        # Detectar plano del suelo (RANSAC simplificado)
        floor_y = np.percentile(points[:, 1], 90)  # Asumir suelo está abajo
        
        # Puntos sobre el suelo
        obstacles_mask = points[:, 1] < floor_y - min_height
        obstacle_points = points[obstacles_mask]
        
        if len(obstacle_points) < 10:
            return []
        
        # Clustering simple por proximidad en x-z
        obstacles = []
        # Simplificado: tomar centroide de puntos cercanos
        centroid = np.mean(obstacle_points, axis=0)
        obstacles.append({
            "position": centroid.tolist(),
            "num_points": len(obstacle_points),
            "min_distance": float(np.min(obstacle_points[:, 2])),
        })
        
        return obstacles
    
    def detect_floor(self, frame: DepthFrame) -> Optional[Dict[str, Any]]:
        """
        Detecta el plano del suelo.
        
        Returns:
            Parámetros del plano o None
        """
        points = self.depth_to_pointcloud(frame)
        
        if len(points) < 100:
            return None
        
        # RANSAC simplificado
        best_plane = None
        best_inliers = 0
        
        for _ in range(20):  # 20 iteraciones
            # Sample 3 puntos
            idx = np.random.choice(len(points), 3, replace=False)
            p1, p2, p3 = points[idx]
            
            # Calcular normal del plano
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            
            # Distancia al plano
            d = -np.dot(normal, p1)
            distances = np.abs(points @ normal + d)
            
            # Contar inliers
            inliers = np.sum(distances < 0.02)
            
            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = {"normal": normal, "d": d}
        
        if best_plane:
            return {
                "normal": best_plane["normal"].tolist(),
                "d": float(best_plane["d"]),
                "inliers": best_inliers,
                "inlier_ratio": best_inliers / len(points),
            }
        
        return None
    
    def _update_visual_odometry(self, frame: DepthFrame) -> None:
        """Actualiza visual odometry (simplificado)."""
        # Placeholder: VO real requiere feature matching
        # Aquí solo actualizamos posición basada en centroide del depth
        if self._last_frame is None:
            return
        
        # Calcular desplazamiento simple
        current_mean_depth = np.nanmean(frame.depth)
        last_mean_depth = np.nanmean(self._last_frame.depth)
        
        if not np.isnan(current_mean_depth) and not np.isnan(last_mean_depth):
            delta_z = last_mean_depth - current_mean_depth
            self._vo_position[2] += delta_z * 0.1  # Factor de escala
    
    def get_visual_odometry(self) -> Dict[str, Any]:
        """Retorna la estimación de visual odometry."""
        return {
            "position": self._vo_position.tolist(),
        }
    
    def get_last_pointcloud(self) -> Optional[np.ndarray]:
        """Retorna el último point cloud generado."""
        return self._last_pointcloud
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "width": self.width,
            "height": self.height,
            "intrinsics": self.intrinsics,
            "has_last_frame": self._last_frame is not None,
        }
