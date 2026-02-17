"""
SensoryFusion: Fusión sensorial multimodal del lóbulo parietal.

Análogo biológico: Corteza parietal posterior (áreas 5 y 7)
- Integración de visión, propiocepción, tacto
- Construcción de modelo 3D del entorno
- Filtrado y fusión de datos sensoriales
"""
from __future__ import annotations

import logging
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class SensorData:
    """Datos de un sensor."""
    sensor_id: str
    sensor_type: str
    timestamp_ns: int
    data: Any
    confidence: float = 1.0
    
    def age_ms(self) -> float:
        """Edad de los datos en milisegundos."""
        return (time.time_ns() - self.timestamp_ns) / 1e6


@dataclass
class FusedState:
    """Estado fusionado del robot y entorno."""
    timestamp_ns: int = field(default_factory=lambda: time.time_ns())
    
    # Pose del robot
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # x, y, z
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # quaternion
    
    # Velocidades
    linear_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
    # Estado de articulaciones
    joint_positions: Dict[str, float] = field(default_factory=dict)
    joint_velocities: Dict[str, float] = field(default_factory=dict)
    joint_efforts: Dict[str, float] = field(default_factory=dict)
    
    # Fuerzas de contacto
    contact_forces: Dict[str, Tuple[float, float, float]] = field(default_factory=dict)
    
    # Confianza general
    confidence: float = 1.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp_ns": self.timestamp_ns,
            "position": self.position,
            "orientation": self.orientation,
            "linear_velocity": self.linear_velocity,
            "angular_velocity": self.angular_velocity,
            "joint_positions": self.joint_positions,
            "confidence": self.confidence,
        }


class KalmanFilter1D:
    """Filtro de Kalman simple para fusión 1D."""
    
    def __init__(self, process_noise: float = 0.01, measurement_noise: float = 0.1):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.estimate = 0.0
        self.error_covariance = 1.0
    
    def update(self, measurement: float) -> float:
        """Actualiza estimación con nueva medición."""
        # Predicción
        predicted_estimate = self.estimate
        predicted_error = self.error_covariance + self.process_noise
        
        # Actualización
        kalman_gain = predicted_error / (predicted_error + self.measurement_noise)
        self.estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.error_covariance = (1 - kalman_gain) * predicted_error
        
        return self.estimate
    
    def reset(self, value: float = 0.0) -> None:
        """Resetea el filtro."""
        self.estimate = value
        self.error_covariance = 1.0


class SensoryFusion:
    """
    Módulo de fusión sensorial del lóbulo parietal.
    
    Integra datos de múltiples sensores para construir
    una representación coherente del estado del robot y entorno.
    """
    
    def __init__(self, history_size: int = 100):
        """
        Inicializa el módulo de fusión.
        
        Args:
            history_size: Tamaño del historial de datos por sensor
        """
        self.history_size = history_size
        
        # Historial de datos por sensor
        self._sensor_history: Dict[str, deque] = {}
        
        # Filtros de Kalman por dimensión
        self._filters: Dict[str, KalmanFilter1D] = {}
        
        # Último estado fusionado
        self._fused_state: FusedState = FusedState()
        
        # Callbacks para actualizaciones
        self._callbacks: List[Callable[[FusedState], None]] = []
        
        # Pesos de confianza por tipo de sensor
        self.sensor_weights = {
            "imu": 0.9,
            "encoder": 0.95,
            "vision": 0.7,
            "force_torque": 0.85,
            "depth": 0.8,
        }
        
        # Tiempos de expiración por tipo (ms)
        self.expiration_ms = {
            "imu": 50,
            "encoder": 20,
            "vision": 200,
            "force_torque": 50,
            "depth": 200,
        }
    
    def add_sensor_data(self, data: SensorData) -> None:
        """
        Agrega datos de sensor para fusión.
        
        Args:
            data: Datos del sensor
        """
        sensor_id = data.sensor_id
        
        # Inicializar historial si es necesario
        if sensor_id not in self._sensor_history:
            self._sensor_history[sensor_id] = deque(maxlen=self.history_size)
        
        # Agregar al historial
        self._sensor_history[sensor_id].append(data)
        
        # Procesar según tipo
        self._process_sensor_data(data)
    
    def _process_sensor_data(self, data: SensorData) -> None:
        """Procesa datos de sensor específico."""
        sensor_type = data.sensor_type
        
        if sensor_type == "imu":
            self._process_imu(data)
        elif sensor_type == "encoder":
            self._process_encoder(data)
        elif sensor_type == "force_torque":
            self._process_force_torque(data)
        elif sensor_type == "vision":
            self._process_vision(data)
    
    def _process_imu(self, data: SensorData) -> None:
        """Procesa datos de IMU."""
        imu_data = data.data
        if not isinstance(imu_data, dict):
            return
        
        # Filtrar y fusionar orientación
        if "orientation" in imu_data:
            quat = imu_data["orientation"]
            if isinstance(quat, (list, tuple)) and len(quat) == 4:
                self._fused_state.orientation = tuple(quat)
        
        # Filtrar velocidades angulares
        if all(k in imu_data for k in ["gyro_x", "gyro_y", "gyro_z"]):
            for i, key in enumerate(["gyro_x", "gyro_y", "gyro_z"]):
                filter_key = f"angular_vel_{i}"
                if filter_key not in self._filters:
                    self._filters[filter_key] = KalmanFilter1D(0.001, 0.05)
                filtered = self._filters[filter_key].update(imu_data[key])
                
            self._fused_state.angular_velocity = (
                self._filters["angular_vel_0"].estimate,
                self._filters["angular_vel_1"].estimate,
                self._filters["angular_vel_2"].estimate,
            )
    
    def _process_encoder(self, data: SensorData) -> None:
        """Procesa datos de encoder."""
        enc_data = data.data
        if not isinstance(enc_data, dict):
            return
        
        joint_id = enc_data.get("joint_id", data.sensor_id)
        
        if "position" in enc_data:
            # Filtrar posición
            filter_key = f"joint_pos_{joint_id}"
            if filter_key not in self._filters:
                self._filters[filter_key] = KalmanFilter1D(0.0001, 0.01)
            filtered = self._filters[filter_key].update(enc_data["position"])
            self._fused_state.joint_positions[joint_id] = filtered
        
        if "velocity" in enc_data:
            self._fused_state.joint_velocities[joint_id] = enc_data["velocity"]
        
        if "effort" in enc_data:
            self._fused_state.joint_efforts[joint_id] = enc_data["effort"]
    
    def _process_force_torque(self, data: SensorData) -> None:
        """Procesa datos de fuerza/torque."""
        ft_data = data.data
        if not isinstance(ft_data, dict):
            return
        
        joint_id = ft_data.get("joint_id", data.sensor_id)
        
        if all(k in ft_data for k in ["force_x", "force_y", "force_z"]):
            self._fused_state.contact_forces[joint_id] = (
                ft_data["force_x"],
                ft_data["force_y"],
                ft_data["force_z"],
            )
    
    def _process_vision(self, data: SensorData) -> None:
        """Procesa datos de visión (para odometría visual)."""
        # Por ahora solo actualiza timestamp
        # Implementación completa requeriría odometría visual
        pass
    
    def fuse(self) -> FusedState:
        """
        Ejecuta fusión de todos los datos disponibles.
        
        Returns:
            Estado fusionado actualizado
        """
        # Actualizar timestamp
        self._fused_state.timestamp_ns = time.time_ns()
        
        # Calcular confianza basada en frescura de datos
        confidences = []
        for sensor_id, history in self._sensor_history.items():
            if not history:
                continue
            
            latest = history[-1]
            age_ms = latest.age_ms()
            
            # Determinar tipo de sensor
            sensor_type = latest.sensor_type
            max_age = self.expiration_ms.get(sensor_type, 100)
            
            if age_ms < max_age:
                weight = self.sensor_weights.get(sensor_type, 0.5)
                freshness = 1.0 - (age_ms / max_age)
                confidences.append(weight * freshness * latest.confidence)
        
        if confidences:
            self._fused_state.confidence = sum(confidences) / len(confidences)
        else:
            self._fused_state.confidence = 0.0
        
        # Notificar callbacks
        for callback in self._callbacks:
            try:
                callback(self._fused_state)
            except Exception as e:
                logger.error(f"Error in fusion callback: {e}")
        
        return self._fused_state
    
    def get_state(self) -> FusedState:
        """Obtiene último estado fusionado."""
        return self._fused_state
    
    def register_callback(self, callback: Callable[[FusedState], None]) -> None:
        """Registra callback para actualizaciones de estado."""
        self._callbacks.append(callback)
    
    def get_sensor_status(self) -> Dict[str, Dict[str, Any]]:
        """Obtiene estado de todos los sensores."""
        status = {}
        for sensor_id, history in self._sensor_history.items():
            if history:
                latest = history[-1]
                status[sensor_id] = {
                    "type": latest.sensor_type,
                    "age_ms": latest.age_ms(),
                    "confidence": latest.confidence,
                    "history_size": len(history),
                }
        return status
    
    def clear(self) -> None:
        """Limpia todos los datos y filtros."""
        self._sensor_history.clear()
        self._filters.clear()
        self._fused_state = FusedState()
