"""
IMU Sensor: Procesamiento de unidad de medición inercial.
==========================================================
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional
import numpy as np

_log = logging.getLogger("humanoid.sensors.imu")


@dataclass
class IMUData:
    """Datos de la IMU."""
    timestamp: float
    acceleration: np.ndarray     # [ax, ay, az] m/s^2
    angular_velocity: np.ndarray  # [wx, wy, wz] rad/s
    magnetic_field: Optional[np.ndarray] = None  # [mx, my, mz] µT
    temperature: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "acceleration": self.acceleration.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "magnetic_field": self.magnetic_field.tolist() if self.magnetic_field is not None else None,
            "temperature": self.temperature,
        }


class IMUSensor:
    """
    Procesador de datos IMU.
    
    Incluye:
    - Calibración
    - Filtrado de ruido
    - Compensación de bias
    - Estimación de orientación
    """
    
    def __init__(self):
        # Calibración
        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)
        self._accel_scale = np.ones(3)
        
        # Filtros
        self._accel_filter = LowPassFilter(alpha=0.8)
        self._gyro_filter = LowPassFilter(alpha=0.9)
        
        # Estado
        self._orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion
        self._last_timestamp = 0.0
        
        # Estadísticas
        self._sample_count = 0
    
    def calibrate(self, samples: list[IMUData]) -> Dict[str, Any]:
        """
        Calibra la IMU usando muestras estáticas.
        
        Args:
            samples: Lista de muestras tomadas en reposo
            
        Returns:
            Parámetros de calibración
        """
        if len(samples) < 10:
            return {"ok": False, "error": "Need at least 10 samples"}
        
        accels = np.array([s.acceleration for s in samples])
        gyros = np.array([s.angular_velocity for s in samples])
        
        # Bias del giroscopio (promedio en reposo)
        self._gyro_bias = np.mean(gyros, axis=0)
        
        # Bias del acelerómetro (restando gravedad)
        mean_accel = np.mean(accels, axis=0)
        gravity = np.array([0, 0, -9.81])  # Asumiendo IMU orientada correctamente
        self._accel_bias = mean_accel - gravity
        
        _log.info("IMU calibrated: gyro_bias=%s, accel_bias=%s",
                  self._gyro_bias, self._accel_bias)
        
        return {
            "ok": True,
            "gyro_bias": self._gyro_bias.tolist(),
            "accel_bias": self._accel_bias.tolist(),
            "samples_used": len(samples),
        }
    
    def process(self, raw_data: Dict[str, Any]) -> IMUData:
        """
        Procesa datos crudos de la IMU.
        
        Args:
            raw_data: Datos crudos del sensor
            
        Returns:
            Datos procesados
        """
        timestamp = raw_data.get("timestamp", 0.0)
        
        # Extraer datos
        accel = np.array(raw_data.get("acceleration", [0, 0, -9.81]))
        gyro = np.array(raw_data.get("angular_velocity", [0, 0, 0]))
        mag = raw_data.get("magnetic_field")
        temp = raw_data.get("temperature")
        
        # Aplicar calibración
        accel = (accel - self._accel_bias) * self._accel_scale
        gyro = gyro - self._gyro_bias
        
        # Filtrar
        accel = self._accel_filter.update(accel)
        gyro = self._gyro_filter.update(gyro)
        
        # Actualizar orientación
        if self._last_timestamp > 0:
            dt = timestamp - self._last_timestamp
            if 0 < dt < 1.0:  # Sanity check
                self._update_orientation(gyro, dt)
        
        self._last_timestamp = timestamp
        self._sample_count += 1
        
        return IMUData(
            timestamp=timestamp,
            acceleration=accel,
            angular_velocity=gyro,
            magnetic_field=np.array(mag) if mag else None,
            temperature=temp,
        )
    
    def _update_orientation(self, gyro: np.ndarray, dt: float) -> None:
        """Actualiza orientación integrando giroscopio."""
        # Quaternion rate
        w, x, y, z = self._orientation
        wx, wy, wz = gyro * 0.5
        
        # Derivada del quaternion
        dw = -x * wx - y * wy - z * wz
        dx = w * wx + y * wz - z * wy
        dy = w * wy + z * wx - x * wz
        dz = w * wz + x * wy - y * wx
        
        # Integrar
        self._orientation = np.array([
            w + dw * dt,
            x + dx * dt,
            y + dy * dt,
            z + dz * dt,
        ])
        
        # Normalizar
        self._orientation /= np.linalg.norm(self._orientation)
    
    def get_orientation(self) -> np.ndarray:
        """Retorna orientación actual como quaternion."""
        return self._orientation.copy()
    
    def reset_orientation(self) -> None:
        """Resetea la orientación a identidad."""
        self._orientation = np.array([1.0, 0.0, 0.0, 0.0])
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "sample_count": self._sample_count,
            "orientation": self._orientation.tolist(),
            "gyro_bias": self._gyro_bias.tolist(),
            "accel_bias": self._accel_bias.tolist(),
        }


class LowPassFilter:
    """Filtro pasa-bajos simple."""
    
    def __init__(self, alpha: float = 0.8):
        self.alpha = alpha
        self._value: Optional[np.ndarray] = None
    
    def update(self, value: np.ndarray) -> np.ndarray:
        if self._value is None:
            self._value = value.copy()
        else:
            self._value = self.alpha * self._value + (1 - self.alpha) * value
        return self._value.copy()
    
    def reset(self) -> None:
        self._value = None
