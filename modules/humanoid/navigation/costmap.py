"""
Costmap 2D: Mapa de costos para navegación.
============================================
Gestiona el mapa de costos usado para planificación.
"""
from __future__ import annotations

import logging
import threading
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.navigation.costmap")


@dataclass
class CostmapConfig:
    """Configuración del costmap."""
    width: int = 200              # Celdas
    height: int = 200             # Celdas
    resolution: float = 0.05      # m/celda
    origin: Tuple[float, float] = (-5.0, -5.0)
    inflation_radius: float = 0.3  # m
    robot_radius: float = 0.25     # m
    
    # Valores de costo
    FREE_SPACE: int = 0
    INSCRIBED_INFLATED: int = 253
    LETHAL_OBSTACLE: int = 254
    NO_INFORMATION: int = 255


class Costmap2D:
    """
    Mapa de costos 2D para navegación.
    
    Mantiene una representación de los obstáculos y el espacio libre
    con costos inflados para planificación segura.
    """
    
    def __init__(self, config: Optional[CostmapConfig] = None):
        self.config = config or CostmapConfig()
        self._lock = threading.Lock()
        
        # Inicializar costmap
        self._costmap = np.full(
            (self.config.height, self.config.width),
            self.config.NO_INFORMATION,
            dtype=np.uint8,
        )
        
        # Mapa de obstáculos estáticos
        self._static_map: Optional[np.ndarray] = None
        
        # Lista de obstáculos dinámicos
        self._dynamic_obstacles: List[Dict[str, Any]] = []
    
    def update_static_map(self, occupancy_grid: np.ndarray) -> None:
        """
        Actualiza el mapa estático desde un occupancy grid.
        
        Args:
            occupancy_grid: Array con valores -1=unknown, 0-100=probability
        """
        with self._lock:
            self._static_map = occupancy_grid.copy()
            self._rebuild_costmap()
    
    def add_obstacle(self, x: float, y: float, radius: float = 0.1, dynamic: bool = True) -> str:
        """
        Añade un obstáculo al costmap.
        
        Args:
            x, y: Posición en metros
            radius: Radio del obstáculo
            dynamic: Si es dinámico (temporal)
            
        Returns:
            ID del obstáculo
        """
        import uuid
        obs_id = str(uuid.uuid4())[:8]
        
        with self._lock:
            self._dynamic_obstacles.append({
                "id": obs_id,
                "x": x,
                "y": y,
                "radius": radius,
            })
            self._rebuild_costmap()
        
        return obs_id
    
    def remove_obstacle(self, obs_id: str) -> bool:
        """Elimina un obstáculo dinámico."""
        with self._lock:
            for i, obs in enumerate(self._dynamic_obstacles):
                if obs["id"] == obs_id:
                    del self._dynamic_obstacles[i]
                    self._rebuild_costmap()
                    return True
        return False
    
    def clear_dynamic_obstacles(self) -> None:
        """Elimina todos los obstáculos dinámicos."""
        with self._lock:
            self._dynamic_obstacles.clear()
            self._rebuild_costmap()
    
    def _rebuild_costmap(self) -> None:
        """Reconstruye el costmap completo."""
        # Empezar con mapa estático o vacío
        if self._static_map is not None:
            # Convertir occupancy (-1, 0-100) a costmap (0-255)
            self._costmap = np.where(
                self._static_map < 0,
                self.config.NO_INFORMATION,
                np.where(
                    self._static_map < 50,
                    self.config.FREE_SPACE,
                    self.config.LETHAL_OBSTACLE,
                )
            ).astype(np.uint8)
        else:
            self._costmap.fill(self.config.FREE_SPACE)
        
        # Añadir obstáculos dinámicos
        for obs in self._dynamic_obstacles:
            mx, my = self._world_to_map(obs["x"], obs["y"])
            r_cells = int(obs["radius"] / self.config.resolution) + 1
            
            for dy in range(-r_cells, r_cells + 1):
                for dx in range(-r_cells, r_cells + 1):
                    nx, ny = mx + dx, my + dy
                    if 0 <= nx < self.config.width and 0 <= ny < self.config.height:
                        dist = np.sqrt(dx*dx + dy*dy) * self.config.resolution
                        if dist <= obs["radius"]:
                            self._costmap[ny, nx] = self.config.LETHAL_OBSTACLE
        
        # Inflar obstáculos
        self._inflate()
    
    def _inflate(self) -> None:
        """Infla los obstáculos según el radio del robot."""
        total_radius = self.config.inflation_radius + self.config.robot_radius
        inflation_cells = int(total_radius / self.config.resolution)
        
        # Encontrar celdas letales
        lethal = np.where(self._costmap == self.config.LETHAL_OBSTACLE)
        
        # Crear mapa de distancias
        inflated = self._costmap.copy()
        
        for y, x in zip(lethal[0], lethal[1]):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= nx < self.config.width and 0 <= ny < self.config.height:
                        dist = np.sqrt(dx*dx + dy*dy) * self.config.resolution
                        if dist <= total_radius and dist > 0:
                            # Costo decreciente con la distancia
                            cost = int(252 * (1 - dist / total_radius))
                            inflated[ny, nx] = max(inflated[ny, nx], cost)
        
        self._costmap = inflated
    
    def get_cost(self, x: float, y: float) -> int:
        """Obtiene el costo en una posición del mundo."""
        with self._lock:
            mx, my = self._world_to_map(x, y)
            if 0 <= mx < self.config.width and 0 <= my < self.config.height:
                return int(self._costmap[my, mx])
            return self.config.NO_INFORMATION
    
    def get_costmap(self) -> np.ndarray:
        """Retorna una copia del costmap."""
        with self._lock:
            return self._costmap.copy()
    
    def _world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas mundo a mapa."""
        mx = int((x - self.config.origin[0]) / self.config.resolution)
        my = int((y - self.config.origin[1]) / self.config.resolution)
        return mx, my
    
    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convierte coordenadas mapa a mundo."""
        x = mx * self.config.resolution + self.config.origin[0]
        y = my * self.config.resolution + self.config.origin[1]
        return x, y
    
    def get_footprint_cost(self, x: float, y: float, theta: float = 0.0) -> int:
        """Obtiene el máximo costo bajo el footprint del robot."""
        with self._lock:
            # Simplified circular footprint
            max_cost = 0
            radius_cells = int(self.config.robot_radius / self.config.resolution)
            mx, my = self._world_to_map(x, y)
            
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx*dx + dy*dy <= radius_cells*radius_cells:
                        nx, ny = mx + dx, my + dy
                        if 0 <= nx < self.config.width and 0 <= ny < self.config.height:
                            max_cost = max(max_cost, int(self._costmap[ny, nx]))
            
            return max_cost
    
    def to_dict(self) -> Dict[str, Any]:
        """Retorna información del costmap."""
        return {
            "width": self.config.width,
            "height": self.config.height,
            "resolution": self.config.resolution,
            "origin": self.config.origin,
            "dynamic_obstacles": len(self._dynamic_obstacles),
        }
