"""
SpatialMap: Mapa espacial del lóbulo parietal.

Análogo biológico: Corteza parietal posterior + células de lugar del hipocampo
- Representación egocéntrica (relativa al robot)
- Representación alocéntrica (absoluta en el mundo)
- Integración con SLAM
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


def _bitacora(msg: str, ok: bool = True) -> None:
    try:
        from modules.humanoid.ans.evolution_bitacora import append_evolution_log
        append_evolution_log(msg, ok=ok, source="cortex.parietal")
    except Exception:
        pass


@dataclass
class MapCell:
    """Celda del mapa de ocupación."""
    x: int
    y: int
    occupancy: float = 0.0  # 0=libre, 1=ocupado, 0.5=desconocido
    last_updated_ns: int = 0
    semantic_label: Optional[str] = None


@dataclass
class Landmark:
    """Punto de referencia en el mapa."""
    id: str
    name: str
    position: Tuple[float, float, float]  # x, y, z en metros
    landmark_type: str  # "object", "place", "person", "feature"
    confidence: float = 1.0
    last_seen_ns: int = 0
    properties: Dict[str, Any] = field(default_factory=dict)
    
    def distance_to(self, other_pos: Tuple[float, float, float]) -> float:
        """Calcula distancia a otra posición."""
        return math.sqrt(
            (self.position[0] - other_pos[0]) ** 2 +
            (self.position[1] - other_pos[1]) ** 2 +
            (self.position[2] - other_pos[2]) ** 2
        )


@dataclass
class SpatialRegion:
    """Región semántica del espacio."""
    id: str
    name: str
    center: Tuple[float, float, float]
    radius: float
    region_type: str  # "room", "area", "zone"
    landmarks: List[str] = field(default_factory=list)  # IDs de landmarks


class OccupancyGrid:
    """
    Mapa de ocupación 2D para navegación.
    """
    
    def __init__(self, width: int = 100, height: int = 100, resolution: float = 0.1):
        """
        Inicializa el mapa de ocupación.
        
        Args:
            width: Ancho en celdas
            height: Alto en celdas
            resolution: Metros por celda
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        
        # Origen del mapa (centro)
        self.origin_x = width // 2
        self.origin_y = height // 2
        
        # Grid de ocupación (0.5 = desconocido)
        self.grid = [[0.5 for _ in range(height)] for _ in range(width)]
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas del mundo a celdas del grid."""
        gx = int(x / self.resolution) + self.origin_x
        gy = int(y / self.resolution) + self.origin_y
        return (
            max(0, min(self.width - 1, gx)),
            max(0, min(self.height - 1, gy)),
        )
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """Convierte celdas del grid a coordenadas del mundo."""
        x = (gx - self.origin_x) * self.resolution
        y = (gy - self.origin_y) * self.resolution
        return (x, y)
    
    def set_occupancy(self, x: float, y: float, value: float) -> None:
        """Establece ocupación en posición del mundo."""
        gx, gy = self.world_to_grid(x, y)
        self.grid[gx][gy] = max(0.0, min(1.0, value))
    
    def get_occupancy(self, x: float, y: float) -> float:
        """Obtiene ocupación en posición del mundo."""
        gx, gy = self.world_to_grid(x, y)
        return self.grid[gx][gy]
    
    def is_free(self, x: float, y: float, threshold: float = 0.3) -> bool:
        """Verifica si posición está libre."""
        return self.get_occupancy(x, y) < threshold
    
    def is_occupied(self, x: float, y: float, threshold: float = 0.7) -> bool:
        """Verifica si posición está ocupada."""
        return self.get_occupancy(x, y) > threshold
    
    def update_from_scan(self, robot_pos: Tuple[float, float], scan_points: List[Tuple[float, float]], free: bool = False) -> None:
        """
        Actualiza mapa desde escaneo de sensor.
        
        Args:
            robot_pos: Posición del robot
            scan_points: Puntos detectados (obstáculos o espacio libre)
            free: Si True, los puntos son espacio libre; si False, son obstáculos
        """
        for point in scan_points:
            # Valor a asignar
            value = 0.1 if free else 0.9
            
            # Actualizar con running average
            current = self.get_occupancy(point[0], point[1])
            new_value = 0.8 * current + 0.2 * value
            self.set_occupancy(point[0], point[1], new_value)


class SpatialMap:
    """
    Mapa espacial del lóbulo parietal.
    
    Mantiene representaciones del espacio:
    - Egocéntrica: relativa a la posición del robot
    - Alocéntrica: absoluta en coordenadas del mundo
    """
    
    def __init__(self, grid_size: int = 100, resolution: float = 0.1):
        """
        Inicializa el mapa espacial.
        
        Args:
            grid_size: Tamaño del grid de ocupación
            resolution: Resolución en metros/celda
        """
        # Mapa de ocupación
        self.occupancy_grid = OccupancyGrid(grid_size, grid_size, resolution)
        
        # Landmarks conocidos
        self.landmarks: Dict[str, Landmark] = {}
        
        # Regiones semánticas
        self.regions: Dict[str, SpatialRegion] = {}
        
        # Pose actual del robot
        self.robot_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, theta
        
        # Historial de poses (para odometría)
        self.pose_history: List[Tuple[int, Tuple[float, float, float]]] = []
    
    def update_robot_pose(self, x: float, y: float, theta: float) -> None:
        """
        Actualiza pose del robot.
        
        Args:
            x, y: Posición en metros
            theta: Orientación en radianes
        """
        self.robot_pose = (x, y, theta)
        self.pose_history.append((time.time_ns(), self.robot_pose))
        
        # Limitar historial
        if len(self.pose_history) > 1000:
            self.pose_history = self.pose_history[-500:]
    
    def world_to_egocentric(self, world_x: float, world_y: float) -> Tuple[float, float]:
        """
        Convierte coordenadas del mundo a egocéntricas (relativas al robot).
        
        Returns:
            (forward, left): Distancia adelante y a la izquierda del robot
        """
        rx, ry, rtheta = self.robot_pose
        
        # Diferencia
        dx = world_x - rx
        dy = world_y - ry
        
        # Rotar al frame del robot
        cos_t = math.cos(-rtheta)
        sin_t = math.sin(-rtheta)
        
        forward = dx * cos_t - dy * sin_t
        left = dx * sin_t + dy * cos_t
        
        return (forward, left)
    
    def egocentric_to_world(self, forward: float, left: float) -> Tuple[float, float]:
        """
        Convierte coordenadas egocéntricas a coordenadas del mundo.
        
        Args:
            forward: Distancia adelante del robot
            left: Distancia a la izquierda del robot
        
        Returns:
            (x, y): Coordenadas del mundo
        """
        rx, ry, rtheta = self.robot_pose
        
        # Rotar del frame del robot al mundo
        cos_t = math.cos(rtheta)
        sin_t = math.sin(rtheta)
        
        world_x = rx + forward * cos_t - left * sin_t
        world_y = ry + forward * sin_t + left * cos_t
        
        return (world_x, world_y)
    
    def add_landmark(self, landmark: Landmark) -> None:
        """Agrega o actualiza un landmark."""
        landmark.last_seen_ns = time.time_ns()
        is_new = landmark.id not in self.landmarks
        self.landmarks[landmark.id] = landmark
        if is_new:
            _bitacora(f"Landmark added: {landmark.name} ({landmark.landmark_type}) at ({landmark.position[0]:.2f}, {landmark.position[1]:.2f}, {landmark.position[2]:.2f})")
    
    def remove_landmark(self, landmark_id: str) -> bool:
        """Remueve un landmark."""
        if landmark_id in self.landmarks:
            del self.landmarks[landmark_id]
            return True
        return False
    
    def get_nearby_landmarks(self, radius: float = 2.0) -> List[Landmark]:
        """
        Obtiene landmarks cercanos al robot.
        
        Args:
            radius: Radio de búsqueda en metros
        
        Returns:
            Lista de landmarks dentro del radio
        """
        robot_pos = (self.robot_pose[0], self.robot_pose[1], 0.0)
        nearby = []
        
        for landmark in self.landmarks.values():
            if landmark.distance_to(robot_pos) <= radius:
                nearby.append(landmark)
        
        return sorted(nearby, key=lambda l: l.distance_to(robot_pos))
    
    def find_landmark_by_name(self, name: str) -> Optional[Landmark]:
        """Busca landmark por nombre."""
        name_lower = name.lower()
        for landmark in self.landmarks.values():
            if name_lower in landmark.name.lower():
                return landmark
        return None
    
    def add_region(self, region: SpatialRegion) -> None:
        """Agrega o actualiza una región."""
        self.regions[region.id] = region
    
    def get_current_region(self) -> Optional[SpatialRegion]:
        """Obtiene la región donde está el robot."""
        robot_pos = (self.robot_pose[0], self.robot_pose[1], 0.0)
        
        for region in self.regions.values():
            dist = math.sqrt(
                (region.center[0] - robot_pos[0]) ** 2 +
                (region.center[1] - robot_pos[1]) ** 2
            )
            if dist <= region.radius:
                return region
        
        return None
    
    def update_from_detections(self, detections: List[Dict[str, Any]]) -> None:
        """
        Actualiza mapa desde detecciones de visión.
        
        Args:
            detections: Lista de detecciones con pose
        """
        for det in detections:
            if "pose" not in det:
                continue
            
            pose = det["pose"]
            landmark = Landmark(
                id=det.get("object_id", f"obj_{time.time_ns()}"),
                name=det.get("class_name", "object"),
                position=(pose.get("x", 0), pose.get("y", 0), pose.get("z", 0)),
                landmark_type="object",
                confidence=det.get("confidence", 0.5),
            )
            self.add_landmark(landmark)
    
    def get_path_clear(self, target_x: float, target_y: float, 
                       check_interval: float = 0.1) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """
        Verifica si el camino hacia el objetivo está libre.
        
        Args:
            target_x, target_y: Posición objetivo
            check_interval: Intervalo de verificación en metros
        
        Returns:
            (is_clear, first_obstacle_position)
        """
        rx, ry, _ = self.robot_pose
        
        # Distancia al objetivo
        dist = math.sqrt((target_x - rx) ** 2 + (target_y - ry) ** 2)
        
        if dist < check_interval:
            return (True, None)
        
        # Dirección normalizada
        dx = (target_x - rx) / dist
        dy = (target_y - ry) / dist
        
        # Verificar a lo largo del camino
        check_dist = check_interval
        while check_dist < dist:
            check_x = rx + dx * check_dist
            check_y = ry + dy * check_dist
            
            if self.occupancy_grid.is_occupied(check_x, check_y):
                return (False, (check_x, check_y))
            
            check_dist += check_interval
        
        return (True, None)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convierte mapa a diccionario."""
        return {
            "robot_pose": self.robot_pose,
            "landmarks_count": len(self.landmarks),
            "regions_count": len(self.regions),
            "current_region": self.get_current_region().name if self.get_current_region() else None,
            "nearby_landmarks": [l.name for l in self.get_nearby_landmarks()[:5]],
        }
