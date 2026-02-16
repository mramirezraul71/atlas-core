"""
Path Controller: Control de seguimiento de trayectorias.
=========================================================
Implementa controladores:
- Pure Pursuit
- DWA (Dynamic Window Approach)
- TEB (Timed Elastic Band) - placeholder
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

_log = logging.getLogger("humanoid.navigation.controller")


@dataclass
class ControlCommand:
    """Comando de control para el robot."""
    linear_x: float = 0.0    # Velocidad lineal (m/s)
    linear_y: float = 0.0    # Velocidad lateral (m/s) - para omnidireccional
    angular_z: float = 0.0   # Velocidad angular (rad/s)
    
    def to_dict(self) -> Dict[str, float]:
        return {
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z,
        }


@dataclass
class ControllerConfig:
    """Configuración del controlador."""
    max_linear_vel: float = 0.5      # m/s
    max_angular_vel: float = 1.0     # rad/s
    min_linear_vel: float = 0.05     # m/s
    lookahead_distance: float = 0.5  # m
    goal_tolerance_xy: float = 0.1   # m
    goal_tolerance_theta: float = 0.1  # rad
    controller_frequency: float = 20.0  # Hz


class PathController:
    """
    Controlador de seguimiento de caminos.
    
    Usa Pure Pursuit como algoritmo principal.
    """
    
    def __init__(self, config: Optional[ControllerConfig] = None):
        self.config = config or ControllerConfig()
        self._path: List[Tuple[float, float]] = []
        self._current_waypoint_idx: int = 0
        self._goal_reached: bool = False
        
    def set_path(self, waypoints: List[Tuple[float, float]]) -> None:
        """Establece el camino a seguir."""
        self._path = waypoints
        self._current_waypoint_idx = 0
        self._goal_reached = False
        _log.info("Path set with %d waypoints", len(waypoints))
    
    def compute_velocity(self, pose: Tuple[float, float, float]) -> ControlCommand:
        """
        Calcula el comando de velocidad dado la pose actual.
        
        Args:
            pose: Pose actual (x, y, theta)
            
        Returns:
            Comando de control
        """
        if not self._path or self._goal_reached:
            return ControlCommand()
        
        x, y, theta = pose
        
        # Encontrar el punto de lookahead
        lookahead_point = self._find_lookahead_point(x, y)
        if lookahead_point is None:
            # Llegamos al final
            self._goal_reached = True
            return ControlCommand()
        
        # Calcular error de orientación hacia lookahead
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        target_angle = math.atan2(dy, dx)
        angle_error = self._normalize_angle(target_angle - theta)
        
        # Pure Pursuit: curvatura
        distance = math.sqrt(dx*dx + dy*dy)
        curvature = 2 * math.sin(angle_error) / max(distance, 0.1)
        
        # Calcular velocidades
        linear_vel = self.config.max_linear_vel
        
        # Reducir velocidad en curvas
        if abs(curvature) > 0.5:
            linear_vel *= 0.5
        
        # Reducir velocidad cerca del goal
        goal = self._path[-1]
        dist_to_goal = math.sqrt((goal[0] - x)**2 + (goal[1] - y)**2)
        if dist_to_goal < 0.5:
            linear_vel *= max(0.2, dist_to_goal / 0.5)
        
        # Calcular velocidad angular
        angular_vel = linear_vel * curvature
        angular_vel = max(-self.config.max_angular_vel, 
                          min(self.config.max_angular_vel, angular_vel))
        
        # Verificar si llegamos al goal
        if dist_to_goal < self.config.goal_tolerance_xy:
            self._goal_reached = True
            return ControlCommand()
        
        return ControlCommand(
            linear_x=linear_vel,
            angular_z=angular_vel,
        )
    
    def _find_lookahead_point(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """Encuentra el punto de lookahead en el camino."""
        lookahead = self.config.lookahead_distance
        
        # Buscar desde el waypoint actual
        for i in range(self._current_waypoint_idx, len(self._path)):
            wp = self._path[i]
            dist = math.sqrt((wp[0] - x)**2 + (wp[1] - y)**2)
            
            if dist >= lookahead:
                self._current_waypoint_idx = max(0, i - 1)
                return wp
        
        # Si no encontramos punto a lookahead distance, usar el último
        if self._path:
            return self._path[-1]
        
        return None
    
    def _normalize_angle(self, angle: float) -> float:
        """Normaliza ángulo a [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_goal_reached(self) -> bool:
        """Verifica si se alcanzó el objetivo."""
        return self._goal_reached
    
    def get_progress(self) -> float:
        """Retorna el progreso (0-1) en el camino."""
        if not self._path:
            return 0.0
        return self._current_waypoint_idx / len(self._path)
    
    def reset(self) -> None:
        """Resetea el controlador."""
        self._path = []
        self._current_waypoint_idx = 0
        self._goal_reached = False


class DWAController:
    """
    Dynamic Window Approach controller.
    
    Considera restricciones dinámicas del robot.
    """
    
    def __init__(self, config: Optional[ControllerConfig] = None):
        self.config = config or ControllerConfig()
        self._goal: Optional[Tuple[float, float]] = None
        
        # Restricciones dinámicas
        self.max_accel = 0.5  # m/s^2
        self.max_decel = 1.0  # m/s^2
        self.max_angular_accel = 2.0  # rad/s^2
        
        # Velocidad actual
        self._current_vel = (0.0, 0.0)  # (linear, angular)
        
        # Parámetros de evaluación
        self.heading_weight = 0.8
        self.distance_weight = 0.4
        self.velocity_weight = 0.2
    
    def set_goal(self, goal: Tuple[float, float]) -> None:
        """Establece el objetivo."""
        self._goal = goal
    
    def compute_velocity(
        self,
        pose: Tuple[float, float, float],
        obstacles: List[Tuple[float, float]] = None,
        dt: float = 0.1,
    ) -> ControlCommand:
        """
        Calcula velocidad usando DWA.
        
        Args:
            pose: Pose actual (x, y, theta)
            obstacles: Lista de obstáculos (x, y)
            dt: Delta time
            
        Returns:
            Comando de control
        """
        if self._goal is None:
            return ControlCommand()
        
        x, y, theta = pose
        obstacles = obstacles or []
        
        # Verificar si llegamos
        dist_to_goal = math.sqrt((self._goal[0] - x)**2 + (self._goal[1] - y)**2)
        if dist_to_goal < self.config.goal_tolerance_xy:
            self._current_vel = (0.0, 0.0)
            return ControlCommand()
        
        # Ventana dinámica
        v_min = max(0, self._current_vel[0] - self.max_decel * dt)
        v_max = min(self.config.max_linear_vel, self._current_vel[0] + self.max_accel * dt)
        w_min = max(-self.config.max_angular_vel, self._current_vel[1] - self.max_angular_accel * dt)
        w_max = min(self.config.max_angular_vel, self._current_vel[1] + self.max_angular_accel * dt)
        
        # Muestrear velocidades
        best_cmd = ControlCommand()
        best_score = float('-inf')
        
        for v in [v_min + i * (v_max - v_min) / 10 for i in range(11)]:
            for w in [w_min + i * (w_max - w_min) / 10 for i in range(11)]:
                # Simular trayectoria
                sim_x = x + v * math.cos(theta) * dt
                sim_y = y + v * math.sin(theta) * dt
                sim_theta = theta + w * dt
                
                # Verificar colisiones
                collision = False
                for ox, oy in obstacles:
                    if math.sqrt((sim_x - ox)**2 + (sim_y - oy)**2) < 0.3:
                        collision = True
                        break
                
                if collision:
                    continue
                
                # Calcular score
                # 1. Heading hacia goal
                goal_angle = math.atan2(self._goal[1] - sim_y, self._goal[0] - sim_x)
                heading_score = math.cos(self._normalize_angle(goal_angle - sim_theta))
                
                # 2. Distancia al goal
                dist_score = 1.0 / (1.0 + math.sqrt((self._goal[0] - sim_x)**2 + (self._goal[1] - sim_y)**2))
                
                # 3. Velocidad
                vel_score = v / self.config.max_linear_vel
                
                score = (self.heading_weight * heading_score +
                         self.distance_weight * dist_score +
                         self.velocity_weight * vel_score)
                
                if score > best_score:
                    best_score = score
                    best_cmd = ControlCommand(linear_x=v, angular_z=w)
        
        self._current_vel = (best_cmd.linear_x, best_cmd.angular_z)
        return best_cmd
    
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
