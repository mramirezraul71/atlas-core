"""
Navigation System: Sistema de navegación integrado.
====================================================
Integra SLAM, localización, planificación y control.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from .slam import SLAMEngine, SLAMConfig, SLAMMode, ScanData
from .localization import Localizer, PoseEstimate, Odometry
from .planner import PathPlanner, Path, Waypoint
from .controller import PathController, ControlCommand, ControllerConfig
from .costmap import Costmap2D, CostmapConfig
from .recovery import RecoveryManager, RecoveryBehavior

_log = logging.getLogger("humanoid.navigation.navigation_system")


class NavigationState(str, Enum):
    """Estados del sistema de navegación."""
    IDLE = "idle"
    MAPPING = "mapping"
    LOCALIZING = "localizing"
    NAVIGATING = "navigating"
    RECOVERING = "recovering"
    ERROR = "error"


@dataclass
class NavigationConfig:
    """Configuración del sistema de navegación."""
    slam_config: SLAMConfig = None
    costmap_config: CostmapConfig = None
    controller_config: ControllerConfig = None
    
    update_rate_hz: float = 20.0
    planning_rate_hz: float = 1.0
    
    def __post_init__(self):
        if self.slam_config is None:
            self.slam_config = SLAMConfig()
        if self.costmap_config is None:
            self.costmap_config = CostmapConfig()
        if self.controller_config is None:
            self.controller_config = ControllerConfig()


class NavigationSystem:
    """
    Sistema de navegación integrado para ATLAS.
    
    Proporciona una interfaz de alto nivel para:
    - Mapeo (SLAM)
    - Localización
    - Navegación a objetivos
    - Recuperación de errores
    """
    
    def __init__(self, config: Optional[NavigationConfig] = None):
        self.config = config or NavigationConfig()
        
        # Componentes
        self._slam = SLAMEngine(self.config.slam_config)
        self._localizer = Localizer()
        self._planner = PathPlanner()
        self._controller = PathController(self.config.controller_config)
        self._costmap = Costmap2D(self.config.costmap_config)
        self._recovery = RecoveryManager()
        
        # Estado
        self._state = NavigationState.IDLE
        self._lock = threading.Lock()
        self._running = False
        
        # Navegación actual
        self._current_goal: Optional[Tuple[float, float, float]] = None
        self._current_path: Optional[Path] = None
        
        # Thread de control
        self._control_thread: Optional[threading.Thread] = None
        
        # Callbacks
        self._cmd_callback: Optional[callable] = None
        
        # Estadísticas
        self._stats = {
            "goals_reached": 0,
            "goals_failed": 0,
            "recovery_attempts": 0,
            "total_distance": 0.0,
        }
    
    # ========================================================================
    # LIFECYCLE
    # ========================================================================
    
    def start(self) -> Dict[str, Any]:
        """Inicia el sistema de navegación."""
        with self._lock:
            if self._running:
                return {"ok": True, "already_running": True}
            
            self._running = True
            self._control_thread = threading.Thread(
                target=self._control_loop,
                name="NavigationSystem-Control",
                daemon=True,
            )
            self._control_thread.start()
            
            _log.info("Navigation system started")
            return {"ok": True, "state": self._state.value}
    
    def stop(self) -> Dict[str, Any]:
        """Detiene el sistema de navegación."""
        with self._lock:
            self._running = False
            self._state = NavigationState.IDLE
            self._slam.stop()
            
            if self._control_thread:
                self._control_thread.join(timeout=2.0)
            
            _log.info("Navigation system stopped")
            return {"ok": True}
    
    # ========================================================================
    # MAPPING
    # ========================================================================
    
    def start_mapping(self) -> Dict[str, Any]:
        """Inicia el modo de mapeo (SLAM)."""
        with self._lock:
            result = self._slam.start_mapping()
            if result.get("ok"):
                self._state = NavigationState.MAPPING
            return result
    
    def stop_mapping(self) -> Dict[str, Any]:
        """Detiene el mapeo y guarda el mapa."""
        with self._lock:
            result = self._slam.save_map()
            self._slam.stop()
            self._state = NavigationState.IDLE
            return result
    
    def save_map(self, path: Optional[str] = None) -> Dict[str, Any]:
        """Guarda el mapa actual."""
        return self._slam.save_map(path)
    
    def load_map(self, path: str) -> Dict[str, Any]:
        """Carga un mapa."""
        result = self._slam.load_map(path)
        if result.get("ok"):
            # Actualizar costmap con el mapa
            map_data = self._slam.get_map()
            if map_data:
                self._costmap.update_static_map(map_data.data)
                self._planner.set_costmap(
                    map_data.data,
                    map_data.resolution,
                    map_data.origin,
                )
        return result
    
    # ========================================================================
    # LOCALIZATION
    # ========================================================================
    
    def start_localization(self, map_path: Optional[str] = None) -> Dict[str, Any]:
        """Inicia la localización."""
        with self._lock:
            if map_path:
                load_result = self.load_map(map_path)
                if not load_result.get("ok"):
                    return load_result
            
            result = self._slam.start_localization()
            if result.get("ok"):
                self._state = NavigationState.LOCALIZING
            return result
    
    def set_initial_pose(self, x: float, y: float, theta: float) -> Dict[str, Any]:
        """Establece la pose inicial para localización."""
        return self._localizer.initialize_pose(x, y, theta)
    
    def get_pose(self) -> PoseEstimate:
        """Obtiene la pose actual."""
        return self._localizer.get_pose()
    
    # ========================================================================
    # NAVIGATION
    # ========================================================================
    
    def goto(self, x: float, y: float, theta: Optional[float] = None) -> Dict[str, Any]:
        """
        Navega a una posición objetivo.
        
        Args:
            x, y: Posición objetivo en metros
            theta: Orientación objetivo (opcional)
            
        Returns:
            Resultado de la solicitud de navegación
        """
        with self._lock:
            if self._state not in (NavigationState.IDLE, NavigationState.LOCALIZING, NavigationState.NAVIGATING):
                return {"ok": False, "error": f"Cannot navigate in state {self._state.value}"}
            
            # Obtener pose actual
            current_pose = self._localizer.get_pose()
            
            # Planificar camino
            path = self._planner.plan(
                start=(current_pose.x, current_pose.y),
                goal=(x, y),
                algorithm="a_star",
            )
            
            if path is None or len(path.waypoints) == 0:
                return {"ok": False, "error": "No path found to goal"}
            
            # Establecer objetivo y camino
            self._current_goal = (x, y, theta if theta else 0.0)
            self._current_path = path
            
            # Configurar controlador
            waypoints = [(w.x, w.y) for w in path.waypoints]
            self._controller.set_path(waypoints)
            
            self._state = NavigationState.NAVIGATING
            
            _log.info("Navigating to (%.2f, %.2f), path length: %.2fm", x, y, path.length)
            return {
                "ok": True,
                "goal": {"x": x, "y": y, "theta": theta},
                "path_length": path.length,
                "num_waypoints": len(path.waypoints),
            }
    
    def cancel_navigation(self) -> Dict[str, Any]:
        """Cancela la navegación actual."""
        with self._lock:
            self._current_goal = None
            self._current_path = None
            self._controller.reset()
            
            if self._state == NavigationState.NAVIGATING:
                self._state = NavigationState.LOCALIZING if self._slam.mode == SLAMMode.LOCALIZATION else NavigationState.IDLE
            
            return {"ok": True}
    
    def is_goal_reached(self) -> bool:
        """Verifica si se alcanzó el objetivo."""
        return self._controller.is_goal_reached()
    
    # ========================================================================
    # SENSOR INPUT
    # ========================================================================
    
    def process_scan(self, ranges: List[float], angles: List[float]) -> Dict[str, Any]:
        """Procesa un scan de LiDAR/depth."""
        scan = ScanData(ranges=ranges, angles=angles, timestamp=time.time())
        return self._slam.process_scan(scan)
    
    def process_odometry(self, x: float, y: float, theta: float, vx: float = 0, vy: float = 0, vtheta: float = 0) -> Dict[str, Any]:
        """Procesa datos de odometría."""
        odom = Odometry(x=x, y=y, theta=theta, vx=vx, vy=vy, vtheta=vtheta, timestamp=time.time())
        return self._localizer.update_odometry(odom)
    
    # ========================================================================
    # CONTROL
    # ========================================================================
    
    def set_command_callback(self, callback: callable) -> None:
        """Establece el callback para comandos de velocidad."""
        self._cmd_callback = callback
    
    def _control_loop(self) -> None:
        """Loop principal de control."""
        rate = 1.0 / self.config.update_rate_hz
        
        while self._running:
            try:
                with self._lock:
                    if self._state == NavigationState.NAVIGATING:
                        self._update_navigation()
                    elif self._state == NavigationState.RECOVERING:
                        self._update_recovery()
                
                time.sleep(rate)
                
            except Exception as e:
                _log.exception("Error in control loop: %s", e)
    
    def _update_navigation(self) -> None:
        """Actualiza la navegación."""
        pose = self._localizer.get_pose()
        
        # Calcular comando
        cmd = self._controller.compute_velocity((pose.x, pose.y, pose.theta))
        
        # Verificar obstáculos
        footprint_cost = self._costmap.get_footprint_cost(pose.x, pose.y, pose.theta)
        if footprint_cost >= 253:  # Obstáculo
            _log.warning("Obstacle detected, entering recovery")
            self._state = NavigationState.RECOVERING
            cmd = ControlCommand()  # Stop
        
        # Enviar comando
        if self._cmd_callback:
            self._cmd_callback(cmd)
        
        # Verificar si llegamos
        if self._controller.is_goal_reached():
            _log.info("Goal reached!")
            self._stats["goals_reached"] += 1
            self._state = NavigationState.LOCALIZING if self._slam.mode == SLAMMode.LOCALIZATION else NavigationState.IDLE
            self._current_goal = None
            self._current_path = None
    
    def _update_recovery(self) -> None:
        """Ejecuta comportamientos de recuperación."""
        recovery = self._recovery.get_next_recovery()
        
        if recovery is None:
            _log.error("All recovery behaviors failed")
            self._stats["goals_failed"] += 1
            self._state = NavigationState.ERROR
            return
        
        _log.info("Executing recovery: %s", recovery.name)
        self._stats["recovery_attempts"] += 1
        
        # Ejecutar recuperación (simplificado)
        # En implementación real, ejecutar el comportamiento completo
        time.sleep(1.0)
        
        # Intentar replantear
        if self._current_goal:
            pose = self._localizer.get_pose()
            path = self._planner.plan(
                start=(pose.x, pose.y),
                goal=(self._current_goal[0], self._current_goal[1]),
            )
            
            if path and len(path.waypoints) > 0:
                self._current_path = path
                waypoints = [(w.x, w.y) for w in path.waypoints]
                self._controller.set_path(waypoints)
                self._recovery.recovery_succeeded()
                self._state = NavigationState.NAVIGATING
            else:
                self._recovery.recovery_failed()
    
    # ========================================================================
    # STATUS
    # ========================================================================
    
    def get_status(self) -> Dict[str, Any]:
        """Obtiene el estado del sistema de navegación."""
        with self._lock:
            pose = self._localizer.get_pose()
            return {
                "state": self._state.value,
                "running": self._running,
                "slam_mode": self._slam.mode.value,
                "pose": pose.to_dict(),
                "current_goal": self._current_goal,
                "path_progress": self._controller.get_progress() if self._current_path else 0,
                "stats": self._stats.copy(),
            }
    
    def get_map_info(self) -> Dict[str, Any]:
        """Obtiene información del mapa."""
        map_data = self._slam.get_map()
        if map_data:
            return map_data.to_dict()
        return {"error": "No map available"}
    
    @property
    def state(self) -> NavigationState:
        return self._state
