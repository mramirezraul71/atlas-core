"""
Force/Torque Sensor: Procesamiento de sensores de fuerza y torque.
===================================================================
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("humanoid.sensors.force_torque")


@dataclass
class WrenchData:
    """Datos de fuerza/torque (wrench)."""
    timestamp: float
    force: np.ndarray      # [fx, fy, fz] en N
    torque: np.ndarray     # [tx, ty, tz] en Nm
    frame_id: str = "sensor"
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "force": self.force.tolist(),
            "torque": self.torque.tolist(),
            "frame_id": self.frame_id,
        }
    
    @property
    def magnitude_force(self) -> float:
        return float(np.linalg.norm(self.force))
    
    @property
    def magnitude_torque(self) -> float:
        return float(np.linalg.norm(self.torque))


class ForceTorqueSensor:
    """
    Procesador de sensor de fuerza/torque.
    
    Aplicaciones:
    - Detección de contacto
    - Control de fuerza
    - Detección de colisiones
    """
    
    def __init__(self, name: str = "ft_sensor"):
        self.name = name
        
        # Calibración
        self._bias_force = np.zeros(3)
        self._bias_torque = np.zeros(3)
        
        # Estado
        self._last_wrench: Optional[WrenchData] = None
        self._contact_detected = False
        
        # Umbrales
        self._contact_force_threshold = 5.0  # N
        self._collision_force_threshold = 50.0  # N
        
        # Filtro
        self._filter_alpha = 0.8
        self._filtered_force = np.zeros(3)
        self._filtered_torque = np.zeros(3)
        
        # Estadísticas
        self._max_force_seen = 0.0
        self._sample_count = 0
    
    def calibrate(self, samples: List[WrenchData]) -> Dict[str, Any]:
        """
        Calibra el sensor (determina bias).
        
        Args:
            samples: Muestras tomadas sin carga
            
        Returns:
            Parámetros de calibración
        """
        if len(samples) < 5:
            return {"ok": False, "error": "Need at least 5 samples"}
        
        forces = np.array([s.force for s in samples])
        torques = np.array([s.torque for s in samples])
        
        self._bias_force = np.mean(forces, axis=0)
        self._bias_torque = np.mean(torques, axis=0)
        
        _log.info("F/T sensor calibrated: force_bias=%s, torque_bias=%s",
                  self._bias_force, self._bias_torque)
        
        return {
            "ok": True,
            "force_bias": self._bias_force.tolist(),
            "torque_bias": self._bias_torque.tolist(),
        }
    
    def process(self, raw_data: Dict[str, Any]) -> WrenchData:
        """
        Procesa datos crudos del sensor.
        
        Args:
            raw_data: Datos crudos
            
        Returns:
            Datos procesados
        """
        import time
        
        timestamp = raw_data.get("timestamp", time.time())
        force = np.array(raw_data.get("force", [0, 0, 0]))
        torque = np.array(raw_data.get("torque", [0, 0, 0]))
        frame_id = raw_data.get("frame_id", "sensor")
        
        # Aplicar calibración
        force = force - self._bias_force
        torque = torque - self._bias_torque
        
        # Filtrar
        self._filtered_force = self._filter_alpha * self._filtered_force + \
                               (1 - self._filter_alpha) * force
        self._filtered_torque = self._filter_alpha * self._filtered_torque + \
                                (1 - self._filter_alpha) * torque
        
        wrench = WrenchData(
            timestamp=timestamp,
            force=self._filtered_force.copy(),
            torque=self._filtered_torque.copy(),
            frame_id=frame_id,
        )
        
        # Detectar contacto
        self._contact_detected = wrench.magnitude_force > self._contact_force_threshold
        
        # Actualizar estadísticas
        self._max_force_seen = max(self._max_force_seen, wrench.magnitude_force)
        self._sample_count += 1
        self._last_wrench = wrench
        
        return wrench
    
    def is_contact(self) -> bool:
        """Verifica si hay contacto detectado."""
        return self._contact_detected
    
    def is_collision(self) -> bool:
        """Verifica si hay colisión (fuerza excesiva)."""
        if self._last_wrench is None:
            return False
        return self._last_wrench.magnitude_force > self._collision_force_threshold
    
    def get_contact_info(self) -> Dict[str, Any]:
        """Retorna información de contacto."""
        if self._last_wrench is None:
            return {"contact": False}
        
        return {
            "contact": self._contact_detected,
            "force_magnitude": self._last_wrench.magnitude_force,
            "force_direction": (self._last_wrench.force / 
                               max(self._last_wrench.magnitude_force, 1e-6)).tolist(),
            "torque_magnitude": self._last_wrench.magnitude_torque,
        }
    
    def reset_bias(self) -> None:
        """Resetea el bias."""
        self._bias_force = np.zeros(3)
        self._bias_torque = np.zeros(3)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "contact_detected": self._contact_detected,
            "max_force_seen": self._max_force_seen,
            "sample_count": self._sample_count,
            "bias_force": self._bias_force.tolist(),
            "bias_torque": self._bias_torque.tolist(),
        }
