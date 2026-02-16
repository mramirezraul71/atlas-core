"""
Path Planner: Planificación de trayectorias.
=============================================
Implementa algoritmos de planificación:
- A* (A-star)
- Dijkstra
- RRT (Rapidly-exploring Random Trees)
- Potential Fields
"""
from __future__ import annotations

import heapq
import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.navigation.planner")


@dataclass
class Waypoint:
    """Un punto en el camino."""
    x: float
    y: float
    theta: Optional[float] = None  # Orientación opcional
    
    def to_dict(self) -> Dict[str, Any]:
        return {"x": self.x, "y": self.y, "theta": self.theta}


@dataclass
class Path:
    """Camino planificado."""
    waypoints: List[Waypoint]
    cost: float = 0.0
    length: float = 0.0
    planning_time_ms: float = 0.0
    algorithm: str = "a_star"
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "waypoints": [w.to_dict() for w in self.waypoints],
            "cost": self.cost,
            "length": self.length,
            "planning_time_ms": self.planning_time_ms,
            "algorithm": self.algorithm,
            "num_waypoints": len(self.waypoints),
        }


class PathPlanner:
    """
    Planificador de caminos para ATLAS.
    
    Soporta múltiples algoritmos y configuraciones.
    """
    
    def __init__(self):
        self._costmap: Optional[np.ndarray] = None
        self._resolution: float = 0.05
        self._origin: Tuple[float, float] = (0.0, 0.0)
        
        # Parámetros
        self._inflation_radius: float = 0.3  # Radio de inflación de obstáculos
        self._robot_radius: float = 0.25      # Radio del robot
        
    def set_costmap(self, costmap: np.ndarray, resolution: float, origin: Tuple[float, float]):
        """Establece el costmap para planificación."""
        self._costmap = costmap
        self._resolution = resolution
        self._origin = origin
        
        # Inflar obstáculos
        self._inflate_costmap()
    
    def _inflate_costmap(self):
        """Infla los obstáculos según el radio del robot."""
        if self._costmap is None:
            return
        
        inflation_cells = int((self._inflation_radius + self._robot_radius) / self._resolution)
        inflated = self._costmap.copy()
        
        # Encontrar celdas ocupadas
        occupied = np.where(self._costmap >= 90)
        
        for y, x in zip(occupied[0], occupied[1]):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < self._costmap.shape[0] and 0 <= nx < self._costmap.shape[1]:
                        dist = math.sqrt(dy*dy + dx*dx) * self._resolution
                        if dist < self._inflation_radius + self._robot_radius:
                            # Costo decreciente con la distancia
                            cost = int(90 * (1 - dist / (self._inflation_radius + self._robot_radius)))
                            inflated[ny, nx] = max(inflated[ny, nx], cost)
        
        self._costmap = inflated
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        algorithm: str = "a_star",
    ) -> Optional[Path]:
        """
        Planifica un camino desde start hasta goal.
        
        Args:
            start: Posición inicial (x, y) en metros
            goal: Posición objetivo (x, y) en metros
            algorithm: Algoritmo a usar ("a_star", "dijkstra", "rrt")
            
        Returns:
            Path o None si no se encuentra camino
        """
        import time
        t0 = time.time()
        
        if self._costmap is None:
            _log.warning("No costmap set, cannot plan")
            return None
        
        # Convertir a coordenadas de mapa
        start_cell = self._world_to_map(start[0], start[1])
        goal_cell = self._world_to_map(goal[0], goal[1])
        
        # Verificar validez
        if not self._is_valid_cell(start_cell):
            _log.warning("Start position is invalid or occupied")
            return None
        if not self._is_valid_cell(goal_cell):
            _log.warning("Goal position is invalid or occupied")
            return None
        
        # Planificar según algoritmo
        if algorithm == "a_star":
            path_cells, cost = self._a_star(start_cell, goal_cell)
        elif algorithm == "dijkstra":
            path_cells, cost = self._dijkstra(start_cell, goal_cell)
        elif algorithm == "rrt":
            path_cells, cost = self._rrt(start_cell, goal_cell)
        else:
            path_cells, cost = self._a_star(start_cell, goal_cell)
        
        if path_cells is None:
            return None
        
        # Convertir a waypoints
        waypoints = [Waypoint(x=self._map_to_world(c[0], c[1])[0],
                              y=self._map_to_world(c[0], c[1])[1])
                     for c in path_cells]
        
        # Simplificar path (reducir waypoints redundantes)
        waypoints = self._simplify_path(waypoints)
        
        # Calcular longitud
        length = sum(
            math.sqrt((waypoints[i+1].x - waypoints[i].x)**2 + 
                      (waypoints[i+1].y - waypoints[i].y)**2)
            for i in range(len(waypoints) - 1)
        )
        
        planning_time = (time.time() - t0) * 1000
        
        return Path(
            waypoints=waypoints,
            cost=cost,
            length=length,
            planning_time_ms=planning_time,
            algorithm=algorithm,
        )
    
    def _a_star(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], float]:
        """Algoritmo A*."""
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal:
                # Reconstruir camino
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, g_score[goal]
            
            for neighbor in self._get_neighbors(current):
                # Costo de moverse al vecino
                move_cost = self._get_move_cost(current, neighbor)
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        return None, 0
    
    def _dijkstra(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[Optional[List[Tuple[int, int]]], float]:
        """Algoritmo Dijkstra."""
        # Dijkstra es A* con heurística = 0
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        cost = {start: 0}
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, cost[goal]
            
            for neighbor in self._get_neighbors(current):
                move_cost = self._get_move_cost(current, neighbor)
                tentative_cost = cost[current] + move_cost
                
                if neighbor not in cost or tentative_cost < cost[neighbor]:
                    came_from[neighbor] = current
                    cost[neighbor] = tentative_cost
                    heapq.heappush(open_set, (tentative_cost, neighbor))
        
        return None, 0
    
    def _rrt(self, start: Tuple[int, int], goal: Tuple[int, int], max_iter: int = 5000) -> Tuple[Optional[List[Tuple[int, int]]], float]:
        """Algoritmo RRT (Rapidly-exploring Random Trees)."""
        tree = {start: None}
        goal_threshold = 10  # celdas
        
        for _ in range(max_iter):
            # Sample random point (con sesgo hacia goal)
            if np.random.random() < 0.1:
                sample = goal
            else:
                sample = (
                    np.random.randint(0, self._costmap.shape[1]),
                    np.random.randint(0, self._costmap.shape[0]),
                )
            
            # Encontrar nodo más cercano
            nearest = min(tree.keys(), key=lambda n: self._distance(n, sample))
            
            # Extender hacia sample
            direction = (sample[0] - nearest[0], sample[1] - nearest[1])
            dist = math.sqrt(direction[0]**2 + direction[1]**2)
            if dist > 0:
                step = 5  # celdas
                new_x = int(nearest[0] + direction[0] / dist * min(step, dist))
                new_y = int(nearest[1] + direction[1] / dist * min(step, dist))
                new_node = (new_x, new_y)
                
                # Verificar que el camino está libre
                if self._is_path_clear(nearest, new_node):
                    tree[new_node] = nearest
                    
                    # Verificar si llegamos al goal
                    if self._distance(new_node, goal) < goal_threshold:
                        # Reconstruir camino
                        path = [new_node]
                        current = new_node
                        while tree[current] is not None:
                            current = tree[current]
                            path.append(current)
                        path.reverse()
                        path.append(goal)
                        return path, len(path) * self._resolution
        
        return None, 0
    
    def _get_neighbors(self, cell: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Obtiene los vecinos válidos de una celda."""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = cell[0] + dx, cell[1] + dy
                if self._is_valid_cell((nx, ny)):
                    neighbors.append((nx, ny))
        return neighbors
    
    def _get_move_cost(self, from_cell: Tuple[int, int], to_cell: Tuple[int, int]) -> float:
        """Calcula el costo de moverse de una celda a otra."""
        # Costo base (diagonal o recto)
        dx = abs(to_cell[0] - from_cell[0])
        dy = abs(to_cell[1] - from_cell[1])
        base_cost = math.sqrt(dx*dx + dy*dy) * self._resolution
        
        # Añadir costo del costmap
        cell_cost = self._costmap[to_cell[1], to_cell[0]]
        if cell_cost < 0:
            cell_cost = 50  # Desconocido = costo medio
        
        return base_cost * (1 + cell_cost / 100)
    
    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Heurística para A* (distancia euclidiana)."""
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2) * self._resolution
    
    def _distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Distancia euclidiana entre dos celdas."""
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
    
    def _is_valid_cell(self, cell: Tuple[int, int]) -> bool:
        """Verifica si una celda es válida para navegación."""
        x, y = cell
        if x < 0 or x >= self._costmap.shape[1]:
            return False
        if y < 0 or y >= self._costmap.shape[0]:
            return False
        # Ocupada si cost >= 90
        return self._costmap[y, x] < 90
    
    def _is_path_clear(self, from_cell: Tuple[int, int], to_cell: Tuple[int, int]) -> bool:
        """Verifica si el camino entre dos celdas está libre."""
        # Bresenham line
        x0, y0 = from_cell
        x1, y1 = to_cell
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            if not self._is_valid_cell((x0, y0)):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return True
    
    def _world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convierte coordenadas mundo a coordenadas de mapa."""
        mx = int((x - self._origin[0]) / self._resolution)
        my = int((y - self._origin[1]) / self._resolution)
        return mx, my
    
    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convierte coordenadas de mapa a coordenadas mundo."""
        x = mx * self._resolution + self._origin[0] + self._resolution / 2
        y = my * self._resolution + self._origin[1] + self._resolution / 2
        return x, y
    
    def _simplify_path(self, waypoints: List[Waypoint], tolerance: float = 0.1) -> List[Waypoint]:
        """Simplifica el camino usando Douglas-Peucker."""
        if len(waypoints) <= 2:
            return waypoints
        
        # Encontrar el punto más alejado de la línea start-end
        start = waypoints[0]
        end = waypoints[-1]
        max_dist = 0
        max_idx = 0
        
        for i in range(1, len(waypoints) - 1):
            dist = self._point_line_distance(waypoints[i], start, end)
            if dist > max_dist:
                max_dist = dist
                max_idx = i
        
        if max_dist > tolerance:
            # Recursively simplify
            left = self._simplify_path(waypoints[:max_idx + 1], tolerance)
            right = self._simplify_path(waypoints[max_idx:], tolerance)
            return left[:-1] + right
        else:
            return [start, end]
    
    def _point_line_distance(self, point: Waypoint, line_start: Waypoint, line_end: Waypoint) -> float:
        """Distancia de un punto a una línea."""
        dx = line_end.x - line_start.x
        dy = line_end.y - line_start.y
        
        if dx == 0 and dy == 0:
            return math.sqrt((point.x - line_start.x)**2 + (point.y - line_start.y)**2)
        
        t = max(0, min(1, ((point.x - line_start.x) * dx + (point.y - line_start.y) * dy) / (dx*dx + dy*dy)))
        
        proj_x = line_start.x + t * dx
        proj_y = line_start.y + t * dy
        
        return math.sqrt((point.x - proj_x)**2 + (point.y - proj_y)**2)
