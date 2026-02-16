"""
Localization: Estimación de pose del robot.
============================================
Implementa localización usando:
- AMCL (Adaptive Monte Carlo Localization)
- EKF (Extended Kalman Filter)
- Particle Filter
"""
from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.navigation.localization")


@dataclass
class PoseEstimate:
    """Estimación de pose con incertidumbre."""
    x: float
    y: float
    theta: float
    covariance: List[float] = field(default_factory=lambda: [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.05])
    timestamp: float = 0.0
    frame_id: str = "map"
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "covariance": self.covariance,
            "timestamp": self.timestamp,
            "frame_id": self.frame_id,
        }


@dataclass
class Odometry:
    """Datos de odometría."""
    x: float
    y: float
    theta: float
    vx: float = 0.0
    vy: float = 0.0
    vtheta: float = 0.0
    timestamp: float = 0.0


class Localizer:
    """
    Sistema de localización para ATLAS.
    
    Combina múltiples fuentes de información:
    - Odometría (wheels, IMU)
    - Scan matching
    - Visual odometry
    """
    
    def __init__(self):
        self._pose = PoseEstimate(x=0.0, y=0.0, theta=0.0)
        self._last_odom: Optional[Odometry] = None
        self._lock = threading.Lock()
        self._initialized = False
        
        # Parámetros del filtro
        self._alpha = [0.1, 0.1, 0.1, 0.1]  # Noise parameters
        
        # Partículas para particle filter
        self._particles: List[np.ndarray] = []
        self._weights: List[float] = []
        self._num_particles = 500
    
    def initialize_pose(self, x: float, y: float, theta: float) -> Dict[str, Any]:
        """Inicializa la pose del robot."""
        with self._lock:
            self._pose = PoseEstimate(x=x, y=y, theta=theta)
            self._initialized = True
            
            # Inicializar partículas alrededor de la pose
            self._initialize_particles(x, y, theta)
            
            _log.info("Localizer initialized at (%.2f, %.2f, %.2f)", x, y, theta)
            return {"ok": True, "pose": self._pose.to_dict()}
    
    def _initialize_particles(self, x: float, y: float, theta: float, spread: float = 0.5):
        """Inicializa las partículas del filtro."""
        self._particles = []
        self._weights = []
        
        for _ in range(self._num_particles):
            px = x + np.random.normal(0, spread)
            py = y + np.random.normal(0, spread)
            ptheta = theta + np.random.normal(0, 0.2)
            self._particles.append(np.array([px, py, ptheta]))
            self._weights.append(1.0 / self._num_particles)
    
    def update_odometry(self, odom: Odometry) -> Dict[str, Any]:
        """
        Actualiza la pose usando odometría.
        
        Args:
            odom: Datos de odometría
            
        Returns:
            Nueva estimación de pose
        """
        with self._lock:
            if self._last_odom is None:
                self._last_odom = odom
                return {"ok": True, "pose": self._pose.to_dict()}
            
            # Calcular delta
            dx = odom.x - self._last_odom.x
            dy = odom.y - self._last_odom.y
            dtheta = self._normalize_angle(odom.theta - self._last_odom.theta)
            
            # Modelo de movimiento
            delta_trans = math.sqrt(dx*dx + dy*dy)
            delta_rot1 = math.atan2(dy, dx) - self._last_odom.theta if delta_trans > 0.001 else 0
            delta_rot2 = dtheta - delta_rot1
            
            # Actualizar pose con ruido
            noise_rot1 = self._alpha[0] * abs(delta_rot1) + self._alpha[1] * delta_trans
            noise_trans = self._alpha[2] * delta_trans + self._alpha[3] * (abs(delta_rot1) + abs(delta_rot2))
            noise_rot2 = self._alpha[0] * abs(delta_rot2) + self._alpha[1] * delta_trans
            
            # Aplicar movimiento
            self._pose.theta = self._normalize_angle(
                self._pose.theta + delta_rot1 + np.random.normal(0, noise_rot1)
            )
            self._pose.x += (delta_trans + np.random.normal(0, noise_trans)) * math.cos(self._pose.theta)
            self._pose.y += (delta_trans + np.random.normal(0, noise_trans)) * math.sin(self._pose.theta)
            self._pose.theta = self._normalize_angle(
                self._pose.theta + delta_rot2 + np.random.normal(0, noise_rot2)
            )
            self._pose.timestamp = odom.timestamp
            
            # Actualizar partículas
            self._motion_update(delta_rot1, delta_trans, delta_rot2)
            
            self._last_odom = odom
            
            return {"ok": True, "pose": self._pose.to_dict()}
    
    def _motion_update(self, delta_rot1: float, delta_trans: float, delta_rot2: float):
        """Actualiza las partículas con el modelo de movimiento."""
        for i, p in enumerate(self._particles):
            # Añadir ruido al movimiento
            noisy_rot1 = delta_rot1 + np.random.normal(0, self._alpha[0] * abs(delta_rot1))
            noisy_trans = delta_trans + np.random.normal(0, self._alpha[2] * delta_trans)
            noisy_rot2 = delta_rot2 + np.random.normal(0, self._alpha[0] * abs(delta_rot2))
            
            # Aplicar movimiento
            p[2] = self._normalize_angle(p[2] + noisy_rot1)
            p[0] += noisy_trans * math.cos(p[2])
            p[1] += noisy_trans * math.sin(p[2])
            p[2] = self._normalize_angle(p[2] + noisy_rot2)
    
    def update_measurement(self, scan_match_score: float, matched_pose: Optional[Tuple[float, float, float]] = None):
        """
        Actualiza con medición (scan matching).
        
        Args:
            scan_match_score: Score del scan matching (0-1)
            matched_pose: Pose corregida del scan matching
        """
        with self._lock:
            if matched_pose and scan_match_score > 0.5:
                # Fusionar con pose estimada
                alpha = min(0.8, scan_match_score)
                self._pose.x = (1 - alpha) * self._pose.x + alpha * matched_pose[0]
                self._pose.y = (1 - alpha) * self._pose.y + alpha * matched_pose[1]
                self._pose.theta = self._normalize_angle(
                    (1 - alpha) * self._pose.theta + alpha * matched_pose[2]
                )
                
                # Actualizar pesos de partículas
                self._measurement_update(matched_pose, scan_match_score)
                
                # Resampling si es necesario
                if self._effective_sample_size() < self._num_particles / 2:
                    self._resample()
    
    def _measurement_update(self, measured_pose: Tuple[float, float, float], score: float):
        """Actualiza los pesos de las partículas."""
        for i, p in enumerate(self._particles):
            # Distancia a la medición
            dx = p[0] - measured_pose[0]
            dy = p[1] - measured_pose[1]
            dtheta = self._normalize_angle(p[2] - measured_pose[2])
            
            # Likelihood gaussiana
            dist = math.sqrt(dx*dx + dy*dy + dtheta*dtheta)
            self._weights[i] *= math.exp(-dist * dist / (2 * 0.5 * 0.5)) * score
        
        # Normalizar pesos
        total = sum(self._weights)
        if total > 0:
            self._weights = [w / total for w in self._weights]
    
    def _effective_sample_size(self) -> float:
        """Calcula el tamaño efectivo de muestra."""
        return 1.0 / sum(w * w for w in self._weights)
    
    def _resample(self):
        """Resampling de partículas usando low-variance resampling."""
        new_particles = []
        new_weights = []
        
        r = np.random.uniform(0, 1.0 / self._num_particles)
        c = self._weights[0]
        i = 0
        
        for m in range(self._num_particles):
            u = r + m / self._num_particles
            while u > c:
                i += 1
                if i >= len(self._weights):
                    i = len(self._weights) - 1
                    break
                c += self._weights[i]
            
            # Añadir partícula con pequeño ruido
            p = self._particles[i].copy()
            p += np.random.normal(0, 0.01, 3)
            new_particles.append(p)
            new_weights.append(1.0 / self._num_particles)
        
        self._particles = new_particles
        self._weights = new_weights
    
    def _normalize_angle(self, angle: float) -> float:
        """Normaliza ángulo a [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def get_pose(self) -> PoseEstimate:
        """Retorna la pose actual."""
        with self._lock:
            return PoseEstimate(
                x=self._pose.x,
                y=self._pose.y,
                theta=self._pose.theta,
                covariance=self._pose.covariance.copy(),
                timestamp=self._pose.timestamp,
            )
    
    def get_particles(self) -> List[Dict[str, float]]:
        """Retorna las partículas para visualización."""
        with self._lock:
            return [
                {"x": p[0], "y": p[1], "theta": p[2], "weight": w}
                for p, w in zip(self._particles, self._weights)
            ]
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas del localizador."""
        with self._lock:
            return {
                "initialized": self._initialized,
                "pose": self._pose.to_dict(),
                "num_particles": len(self._particles),
                "effective_sample_size": self._effective_sample_size() if self._particles else 0,
            }
