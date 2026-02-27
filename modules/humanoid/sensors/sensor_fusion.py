"""
Sensor Fusion: Fusión multi-sensorial.
=======================================
Combina datos de múltiples sensores para estimación robusta.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Callable
import numpy as np

_log = logging.getLogger("humanoid.sensors.fusion")


@dataclass
class FusionConfig:
    """Configuración del sistema de fusión."""
    update_rate_hz: float = 100.0
    use_kalman: bool = True
    outlier_rejection: bool = True
    outlier_threshold: float = 3.0  # Desviaciones estándar
    sensor_timeout_ms: float = 500.0


@dataclass
class FusedState:
    """Estado fusionado del robot."""
    timestamp: float
    position: np.ndarray          # [x, y, z]
    orientation: np.ndarray       # Quaternion [w, x, y, z]
    linear_velocity: np.ndarray   # [vx, vy, vz]
    angular_velocity: np.ndarray  # [wx, wy, wz]
    linear_acceleration: np.ndarray
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    contacts: List[Dict[str, Any]] = field(default_factory=list)
    confidence: float = 1.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
            "linear_velocity": self.linear_velocity.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "joint_positions": self.joint_positions.tolist() if len(self.joint_positions) > 0 else [],
            "confidence": self.confidence,
            "num_contacts": len(self.contacts),
        }


class SensorFusion:
    """
    Sistema de fusión sensorial para ATLAS.
    
    Combina datos de:
    - IMU (aceleración, giroscopio)
    - Cámaras de profundidad
    - Encoders de motores
    - Sensores de fuerza/torque
    - GPS/Visual odometry (opcional)
    """
    
    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        
        # Sensores registrados
        self._sensors: Dict[str, Any] = {}
        self._sensor_data: Dict[str, Dict[str, Any]] = {}
        self._sensor_timestamps: Dict[str, float] = {}
        
        # Estado fusionado
        self._state = FusedState(
            timestamp=0.0,
            position=np.zeros(3),
            orientation=np.array([1, 0, 0, 0]),
            linear_velocity=np.zeros(3),
            angular_velocity=np.zeros(3),
            linear_acceleration=np.zeros(3),
            joint_positions=np.zeros(20),
            joint_velocities=np.zeros(20),
        )
        
        # Filtro de Kalman
        self._ekf = None
        if self.config.use_kalman:
            from .kalman_filter import ExtendedKalmanFilter
            self._ekf = ExtendedKalmanFilter(state_dim=15, measurement_dim=6)
        
        # Control
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # Callbacks
        self._callbacks: List[Callable[[FusedState], None]] = []
        
        # Estadísticas
        self._stats = {
            "updates": 0,
            "outliers_rejected": 0,
            "sensor_timeouts": 0,
        }
        
        _log.info("Sensor fusion initialized (kalman=%s)", self.config.use_kalman)
    
    def add_sensor(self, name: str, sensor: Any) -> None:
        """Registra un sensor."""
        self._sensors[name] = sensor
        self._sensor_data[name] = {}
        self._sensor_timestamps[name] = 0.0
        _log.info("Added sensor: %s", name)
    
    def remove_sensor(self, name: str) -> bool:
        """Elimina un sensor."""
        if name in self._sensors:
            del self._sensors[name]
            del self._sensor_data[name]
            del self._sensor_timestamps[name]
            return True
        return False
    
    def update_sensor(self, name: str, data: Dict[str, Any]) -> None:
        """
        Actualiza datos de un sensor.
        
        Args:
            name: Nombre del sensor
            data: Datos del sensor
        """
        if name not in self._sensors:
            _log.warning("Unknown sensor: %s", name)
            return
        
        now = time.time()
        
        # Rechazar outliers
        if self.config.outlier_rejection and self._is_outlier(name, data):
            self._stats["outliers_rejected"] += 1
            return
        
        with self._lock:
            self._sensor_data[name] = data
            self._sensor_timestamps[name] = now
    
    def _is_outlier(self, name: str, data: Dict[str, Any]) -> bool:
        """Detecta outliers en los datos."""
        prev_data = self._sensor_data.get(name)
        if not prev_data:
            return False
        
        # Comparar valores numéricos
        for key, value in data.items():
            if isinstance(value, (int, float)) and key in prev_data:
                prev_value = prev_data[key]
                if isinstance(prev_value, (int, float)):
                    diff = abs(value - prev_value)
                    # Umbral simple basado en magnitud
                    threshold = max(abs(prev_value) * 0.5, 10.0)
                    if diff > threshold:
                        return True
        
        return False
    
    def update(self) -> FusedState:
        """
        Ejecuta un paso de fusión.
        
        Returns:
            Estado fusionado actualizado
        """
        now = time.time()
        
        with self._lock:
            # Verificar timeouts de sensores
            for name, ts in self._sensor_timestamps.items():
                if now - ts > self.config.sensor_timeout_ms / 1000:
                    self._stats["sensor_timeouts"] += 1
            
            # Fusionar datos
            self._fuse_sensors()
            
            # Actualizar timestamp
            self._state.timestamp = now
            self._stats["updates"] += 1
        
        # Notificar callbacks
        for cb in self._callbacks:
            try:
                cb(self._state)
            except Exception as e:
                _log.exception("Callback error: %s", e)
        
        return self._state
    
    def _fuse_sensors(self) -> None:
        """Fusiona todos los datos de sensores."""
        # IMU
        imu_data = self._sensor_data.get("imu")
        if imu_data:
            if "acceleration" in imu_data:
                self._state.linear_acceleration = np.array(imu_data["acceleration"])
            if "angular_velocity" in imu_data:
                self._state.angular_velocity = np.array(imu_data["angular_velocity"])
            if "orientation" in imu_data:
                self._state.orientation = np.array(imu_data["orientation"])
        
        # Encoders
        encoder_data = self._sensor_data.get("encoders")
        if encoder_data:
            if "positions" in encoder_data:
                self._state.joint_positions = np.array(encoder_data["positions"])
            if "velocities" in encoder_data:
                self._state.joint_velocities = np.array(encoder_data["velocities"])
        
        # Depth camera (para posición si hay visual odometry)
        depth_data = self._sensor_data.get("depth")
        if depth_data and "visual_odometry" in depth_data:
            vo = depth_data["visual_odometry"]
            if "position" in vo:
                # Fusionar con EKF si disponible
                if self._ekf:
                    measurement = np.concatenate([
                        np.array(vo["position"]),
                        self._state.angular_velocity,
                    ])
                    self._ekf.update(measurement)
                    state_vec = self._ekf.get_state()
                    self._state.position = state_vec[:3]
                    self._state.linear_velocity = state_vec[3:6]
                else:
                    self._state.position = np.array(vo["position"])
        
        # Force/Torque
        ft_data = self._sensor_data.get("force_torque")
        if ft_data:
            contacts = []
            for key, value in ft_data.items():
                if "contact" in key.lower() and isinstance(value, dict):
                    contacts.append(value)
            self._state.contacts = contacts
        
        # Calcular confianza
        active_sensors = sum(
            1 for ts in self._sensor_timestamps.values()
            if time.time() - ts < self.config.sensor_timeout_ms / 1000
        )
        self._state.confidence = active_sensors / max(len(self._sensors), 1)
    
    def start(self) -> None:
        """Inicia el loop de fusión."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(
            target=self._fusion_loop,
            name="SensorFusion",
            daemon=True,
        )
        self._thread.start()
        _log.info("Sensor fusion loop started")
    
    def stop(self) -> None:
        """Detiene el loop de fusión."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        _log.info("Sensor fusion stopped")
    
    def _fusion_loop(self) -> None:
        """Loop principal de fusión."""
        period = 1.0 / self.config.update_rate_hz
        
        while self._running:
            t0 = time.time()
            self.update()
            
            elapsed = time.time() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def add_callback(self, callback: Callable[[FusedState], None]) -> None:
        """Registra callback para nuevos estados."""
        self._callbacks.append(callback)
    
    def get_state(self) -> FusedState:
        """Retorna el estado fusionado actual."""
        with self._lock:
            return FusedState(
                timestamp=self._state.timestamp,
                position=self._state.position.copy(),
                orientation=self._state.orientation.copy(),
                linear_velocity=self._state.linear_velocity.copy(),
                angular_velocity=self._state.angular_velocity.copy(),
                linear_acceleration=self._state.linear_acceleration.copy(),
                joint_positions=self._state.joint_positions.copy(),
                joint_velocities=self._state.joint_velocities.copy(),
                contacts=self._state.contacts.copy(),
                confidence=self._state.confidence,
            )
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas del sistema."""
        return {
            "sensors": list(self._sensors.keys()),
            "active_sensors": sum(
                1 for ts in self._sensor_timestamps.values()
                if time.time() - ts < self.config.sensor_timeout_ms / 1000
            ),
            "running": self._running,
            **self._stats,
        }
