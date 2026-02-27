"""
TrajectoryPlanner: Planificacion de trayectorias para control motor.

Genera trayectorias suaves y seguras para movimiento del robot.
"""
from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class TrajectoryType(str, Enum):
    """Tipos de trayectoria."""
    LINEAR = "linear"           # Movimiento lineal
    CUBIC = "cubic"             # Interpolacion cubica
    QUINTIC = "quintic"         # Interpolacion quintica (mas suave)
    TRAPEZOIDAL = "trapezoidal" # Perfil de velocidad trapezoidal
    SPLINE = "spline"           # Spline cubica
    MINIMUM_JERK = "min_jerk"   # Minimo jerk (mas natural)


@dataclass
class Pose3D:
    """Pose 3D (posicion + orientacion)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    def to_list(self) -> List[float]:
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]
    
    @classmethod
    def from_list(cls, values: List[float]) -> 'Pose3D':
        return cls(*values[:6])
    
    def distance_to(self, other: 'Pose3D') -> float:
        """Distancia euclidiana a otra pose."""
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )


@dataclass
class JointState:
    """Estado de articulaciones."""
    positions: Dict[str, float] = field(default_factory=dict)
    velocities: Dict[str, float] = field(default_factory=dict)
    efforts: Dict[str, float] = field(default_factory=dict)
    
    def get_position_array(self, joint_order: List[str]) -> List[float]:
        return [self.positions.get(j, 0.0) for j in joint_order]


@dataclass
class Waypoint:
    """Punto en una trayectoria."""
    time_from_start: float  # Segundos desde inicio
    pose: Optional[Pose3D] = None
    joints: Optional[Dict[str, float]] = None
    velocities: Optional[Dict[str, float]] = None
    accelerations: Optional[Dict[str, float]] = None
    
    def interpolate(self, other: 'Waypoint', t: float) -> 'Waypoint':
        """Interpola linealmente entre dos waypoints."""
        if t <= 0:
            return self
        if t >= 1:
            return other
        
        new_time = self.time_from_start + t * (other.time_from_start - self.time_from_start)
        
        new_joints = None
        if self.joints and other.joints:
            new_joints = {}
            for key in self.joints:
                if key in other.joints:
                    new_joints[key] = self.joints[key] + t * (other.joints[key] - self.joints[key])
        
        new_pose = None
        if self.pose and other.pose:
            new_pose = Pose3D(
                x=self.pose.x + t * (other.pose.x - self.pose.x),
                y=self.pose.y + t * (other.pose.y - self.pose.y),
                z=self.pose.z + t * (other.pose.z - self.pose.z),
                roll=self.pose.roll + t * (other.pose.roll - self.pose.roll),
                pitch=self.pose.pitch + t * (other.pose.pitch - self.pose.pitch),
                yaw=self.pose.yaw + t * (other.pose.yaw - self.pose.yaw),
            )
        
        return Waypoint(
            time_from_start=new_time,
            pose=new_pose,
            joints=new_joints,
        )


@dataclass
class Trajectory:
    """Trayectoria completa."""
    id: str
    trajectory_type: TrajectoryType
    waypoints: List[Waypoint]
    
    # Metadata
    created_at_ns: int = field(default_factory=lambda: time.time_ns())
    duration: float = 0.0
    
    # Limites aplicados
    max_velocity: float = 1.0
    max_acceleration: float = 2.0
    max_jerk: float = 5.0
    
    # Validacion
    is_valid: bool = True
    validation_errors: List[str] = field(default_factory=list)
    
    def get_duration(self) -> float:
        """Obtiene duracion total."""
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].time_from_start
    
    def sample(self, t: float) -> Optional[Waypoint]:
        """Samplea la trayectoria en tiempo t."""
        if not self.waypoints:
            return None
        
        if t <= 0:
            return self.waypoints[0]
        if t >= self.get_duration():
            return self.waypoints[-1]
        
        # Encontrar waypoints adyacentes
        for i in range(len(self.waypoints) - 1):
            if self.waypoints[i].time_from_start <= t <= self.waypoints[i + 1].time_from_start:
                t0 = self.waypoints[i].time_from_start
                t1 = self.waypoints[i + 1].time_from_start
                ratio = (t - t0) / (t1 - t0) if t1 > t0 else 0
                return self.waypoints[i].interpolate(self.waypoints[i + 1], ratio)
        
        return self.waypoints[-1]


@dataclass
class Obstacle:
    """Obstaculo para evitar."""
    id: str
    position: Pose3D
    radius: float  # Esfera envolvente
    is_dynamic: bool = False


class TrajectoryPlanner:
    """
    Planificador de trayectorias.
    
    Genera trayectorias suaves y seguras considerando:
    - Limites de velocidad/aceleracion
    - Obstaculos
    - Suavidad del movimiento
    """
    
    def __init__(
        self,
        max_velocity: float = 1.0,
        max_acceleration: float = 2.0,
        max_jerk: float = 5.0,
        collision_check: bool = True,
    ):
        """
        Inicializa el planificador.
        
        Args:
            max_velocity: Velocidad maxima (m/s o rad/s)
            max_acceleration: Aceleracion maxima
            max_jerk: Jerk maximo (derivada de aceleracion)
            collision_check: Activar verificacion de colisiones
        """
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_jerk = max_jerk
        self.collision_check = collision_check
        
        # IK solver (mock)
        self._ik_solver: Optional[Callable[[Pose3D], Dict[str, float]]] = None
        
        # Obstaculos conocidos
        self._obstacles: Dict[str, Obstacle] = {}
        
        # Historial
        self._planned_trajectories: List[Trajectory] = []
        
        # Contador
        self._trajectory_count = 0
    
    def plan_joint_trajectory(
        self,
        current_joints: Dict[str, float],
        target_joints: Dict[str, float],
        trajectory_type: TrajectoryType = TrajectoryType.QUINTIC,
        duration: Optional[float] = None,
    ) -> Trajectory:
        """
        Planifica trayectoria en espacio de articulaciones.
        
        Args:
            current_joints: Posiciones actuales
            target_joints: Posiciones objetivo
            trajectory_type: Tipo de interpolacion
            duration: Duracion (auto-calculada si None)
        
        Returns:
            Trayectoria planificada
        """
        self._trajectory_count += 1
        traj_id = f"traj_{self._trajectory_count}"
        
        # Calcular duracion si no se especifica
        if duration is None:
            max_delta = max(
                abs(target_joints.get(j, 0) - current_joints.get(j, 0))
                for j in set(current_joints) | set(target_joints)
            )
            duration = max(0.5, max_delta / self.max_velocity)
        
        # Generar waypoints segun tipo
        if trajectory_type == TrajectoryType.LINEAR:
            waypoints = self._linear_interpolation(
                current_joints, target_joints, duration
            )
        elif trajectory_type == TrajectoryType.CUBIC:
            waypoints = self._cubic_interpolation(
                current_joints, target_joints, duration
            )
        elif trajectory_type == TrajectoryType.QUINTIC:
            waypoints = self._quintic_interpolation(
                current_joints, target_joints, duration
            )
        elif trajectory_type == TrajectoryType.TRAPEZOIDAL:
            waypoints = self._trapezoidal_interpolation(
                current_joints, target_joints, duration
            )
        elif trajectory_type == TrajectoryType.MINIMUM_JERK:
            waypoints = self._minimum_jerk_interpolation(
                current_joints, target_joints, duration
            )
        else:
            waypoints = self._linear_interpolation(
                current_joints, target_joints, duration
            )
        
        trajectory = Trajectory(
            id=traj_id,
            trajectory_type=trajectory_type,
            waypoints=waypoints,
            duration=duration,
            max_velocity=self.max_velocity,
            max_acceleration=self.max_acceleration,
            max_jerk=self.max_jerk,
        )
        
        # Validar
        self._validate_trajectory(trajectory)
        
        self._planned_trajectories.append(trajectory)
        
        return trajectory
    
    def plan_cartesian_trajectory(
        self,
        current_pose: Pose3D,
        target_pose: Pose3D,
        trajectory_type: TrajectoryType = TrajectoryType.QUINTIC,
        duration: Optional[float] = None,
        num_waypoints: int = 20,
    ) -> Trajectory:
        """
        Planifica trayectoria en espacio cartesiano.
        
        Args:
            current_pose: Pose actual
            target_pose: Pose objetivo
            trajectory_type: Tipo de interpolacion
            duration: Duracion
            num_waypoints: Numero de waypoints
        
        Returns:
            Trayectoria planificada
        """
        self._trajectory_count += 1
        traj_id = f"traj_{self._trajectory_count}"
        
        # Calcular distancia
        distance = current_pose.distance_to(target_pose)
        
        # Calcular duracion
        if duration is None:
            duration = max(0.5, distance / self.max_velocity)
        
        # Generar waypoints cartesianos
        waypoints = []
        for i in range(num_waypoints + 1):
            t = i / num_waypoints
            time_val = t * duration
            
            # Interpolar pose
            if trajectory_type == TrajectoryType.MINIMUM_JERK:
                # Perfil de minimo jerk
                s = 10 * t**3 - 15 * t**4 + 6 * t**5
            elif trajectory_type == TrajectoryType.QUINTIC:
                # Quintico
                s = 6 * t**5 - 15 * t**4 + 10 * t**3
            else:
                # Lineal
                s = t
            
            pose = Pose3D(
                x=current_pose.x + s * (target_pose.x - current_pose.x),
                y=current_pose.y + s * (target_pose.y - current_pose.y),
                z=current_pose.z + s * (target_pose.z - current_pose.z),
                roll=current_pose.roll + s * (target_pose.roll - current_pose.roll),
                pitch=current_pose.pitch + s * (target_pose.pitch - current_pose.pitch),
                yaw=current_pose.yaw + s * (target_pose.yaw - current_pose.yaw),
            )
            
            # Resolver IK si hay solver
            joints = None
            if self._ik_solver:
                try:
                    joints = self._ik_solver(pose)
                except Exception as e:
                    logger.warning(f"IK failed at t={t}: {e}")
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                pose=pose,
                joints=joints,
            ))
        
        trajectory = Trajectory(
            id=traj_id,
            trajectory_type=trajectory_type,
            waypoints=waypoints,
            duration=duration,
            max_velocity=self.max_velocity,
            max_acceleration=self.max_acceleration,
        )
        
        # Verificar colisiones
        if self.collision_check:
            self._check_collisions(trajectory)
        
        self._validate_trajectory(trajectory)
        self._planned_trajectories.append(trajectory)
        
        return trajectory
    
    def _linear_interpolation(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        duration: float,
        num_points: int = 10,
    ) -> List[Waypoint]:
        """Interpolacion lineal."""
        waypoints = []
        joints = list(set(start) | set(end))
        
        for i in range(num_points + 1):
            t = i / num_points
            time_val = t * duration
            
            positions = {}
            for j in joints:
                s = start.get(j, 0)
                e = end.get(j, 0)
                positions[j] = s + t * (e - s)
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                joints=positions,
            ))
        
        return waypoints
    
    def _cubic_interpolation(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        duration: float,
        num_points: int = 20,
    ) -> List[Waypoint]:
        """Interpolacion cubica (velocidad cero en extremos)."""
        waypoints = []
        joints = list(set(start) | set(end))
        
        for i in range(num_points + 1):
            t = i / num_points
            time_val = t * duration
            
            # Polinomio cubico: s(t) = 3t^2 - 2t^3
            s = 3 * t**2 - 2 * t**3
            
            positions = {}
            for j in joints:
                p0 = start.get(j, 0)
                p1 = end.get(j, 0)
                positions[j] = p0 + s * (p1 - p0)
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                joints=positions,
            ))
        
        return waypoints
    
    def _quintic_interpolation(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        duration: float,
        num_points: int = 20,
    ) -> List[Waypoint]:
        """Interpolacion quintica (velocidad y aceleracion cero en extremos)."""
        waypoints = []
        joints = list(set(start) | set(end))
        
        for i in range(num_points + 1):
            t = i / num_points
            time_val = t * duration
            
            # Polinomio quintico: s(t) = 6t^5 - 15t^4 + 10t^3
            s = 6 * t**5 - 15 * t**4 + 10 * t**3
            
            # Velocidad normalizada
            v = 30 * t**4 - 60 * t**3 + 30 * t**2
            
            positions = {}
            velocities = {}
            for j in joints:
                p0 = start.get(j, 0)
                p1 = end.get(j, 0)
                delta = p1 - p0
                positions[j] = p0 + s * delta
                velocities[j] = v * delta / duration
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                joints=positions,
                velocities=velocities,
            ))
        
        return waypoints
    
    def _trapezoidal_interpolation(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        duration: float,
        num_points: int = 20,
    ) -> List[Waypoint]:
        """Perfil de velocidad trapezoidal."""
        waypoints = []
        joints = list(set(start) | set(end))
        
        # Tiempos de aceleracion/deceleracion (25% cada uno)
        t_accel = 0.25
        t_cruise = 0.5
        
        for i in range(num_points + 1):
            t = i / num_points
            time_val = t * duration
            
            # Perfil trapezoidal
            if t < t_accel:
                # Acelerando
                s = 2 * (t / t_accel)**2 * t_accel / 2
            elif t < t_accel + t_cruise:
                # Crucero
                s = t_accel / 2 + (t - t_accel)
            else:
                # Decelerando
                t_rel = (t - t_accel - t_cruise) / t_accel
                s = t_accel / 2 + t_cruise + t_accel * (2 * t_rel - t_rel**2) / 2
            
            # Normalizar
            s = s / (t_accel / 2 + t_cruise + t_accel / 2)
            
            positions = {}
            for j in joints:
                p0 = start.get(j, 0)
                p1 = end.get(j, 0)
                positions[j] = p0 + s * (p1 - p0)
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                joints=positions,
            ))
        
        return waypoints
    
    def _minimum_jerk_interpolation(
        self,
        start: Dict[str, float],
        end: Dict[str, float],
        duration: float,
        num_points: int = 20,
    ) -> List[Waypoint]:
        """Trayectoria de minimo jerk (movimiento mas natural)."""
        waypoints = []
        joints = list(set(start) | set(end))
        
        for i in range(num_points + 1):
            t = i / num_points
            time_val = t * duration
            
            # Perfil de minimo jerk: s(t) = 10t^3 - 15t^4 + 6t^5
            s = 10 * t**3 - 15 * t**4 + 6 * t**5
            
            # Velocidad
            v = 30 * t**2 - 60 * t**3 + 30 * t**4
            
            # Aceleracion
            a = 60 * t - 180 * t**2 + 120 * t**3
            
            positions = {}
            velocities = {}
            accelerations = {}
            
            for j in joints:
                p0 = start.get(j, 0)
                p1 = end.get(j, 0)
                delta = p1 - p0
                positions[j] = p0 + s * delta
                velocities[j] = v * delta / duration
                accelerations[j] = a * delta / (duration ** 2)
            
            waypoints.append(Waypoint(
                time_from_start=time_val,
                joints=positions,
                velocities=velocities,
                accelerations=accelerations,
            ))
        
        return waypoints
    
    def _validate_trajectory(self, trajectory: Trajectory) -> None:
        """Valida trayectoria contra limites."""
        errors = []
        
        for i, wp in enumerate(trajectory.waypoints):
            if wp.velocities:
                for joint, vel in wp.velocities.items():
                    if abs(vel) > self.max_velocity:
                        errors.append(
                            f"Waypoint {i}: velocidad {joint}={vel:.3f} > max {self.max_velocity}"
                        )
            
            if wp.accelerations:
                for joint, acc in wp.accelerations.items():
                    if abs(acc) > self.max_acceleration:
                        errors.append(
                            f"Waypoint {i}: aceleracion {joint}={acc:.3f} > max {self.max_acceleration}"
                        )
        
        trajectory.validation_errors = errors
        trajectory.is_valid = len(errors) == 0
    
    def _check_collisions(self, trajectory: Trajectory) -> None:
        """Verifica colisiones a lo largo de la trayectoria."""
        for i, wp in enumerate(trajectory.waypoints):
            if not wp.pose:
                continue
            
            for obs_id, obstacle in self._obstacles.items():
                dist = wp.pose.distance_to(obstacle.position)
                if dist < obstacle.radius:
                    trajectory.validation_errors.append(
                        f"Waypoint {i}: colision con {obs_id} a dist={dist:.3f}"
                    )
                    trajectory.is_valid = False
    
    def add_obstacle(self, obstacle: Obstacle) -> None:
        """Agrega obstaculo."""
        self._obstacles[obstacle.id] = obstacle
    
    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Elimina obstaculo."""
        if obstacle_id in self._obstacles:
            del self._obstacles[obstacle_id]
            return True
        return False
    
    def set_ik_solver(self, solver: Callable[[Pose3D], Dict[str, float]]) -> None:
        """Configura solver de cinematica inversa."""
        self._ik_solver = solver
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        valid_count = sum(1 for t in self._planned_trajectories if t.is_valid)
        return {
            "total_trajectories": self._trajectory_count,
            "valid_trajectories": valid_count,
            "invalid_trajectories": self._trajectory_count - valid_count,
            "obstacles": len(self._obstacles),
        }
