"""
Motor Encoders: Procesamiento de encoders de motores.
======================================================
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import numpy as np

_log = logging.getLogger("humanoid.sensors.encoders")


@dataclass
class EncoderData:
    """Datos de encoders."""
    timestamp: float
    positions: np.ndarray    # Posiciones en radianes
    velocities: np.ndarray   # Velocidades en rad/s
    currents: Optional[np.ndarray] = None  # Corrientes en A
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "positions": self.positions.tolist(),
            "velocities": self.velocities.tolist(),
            "currents": self.currents.tolist() if self.currents is not None else None,
        }


class MotorEncoder:
    """
    Procesador de encoders de motores.
    
    Funcionalidades:
    - Cálculo de velocidad
    - Filtrado de ruido
    - Detección de fallas
    """
    
    def __init__(self, num_joints: int = 20, ticks_per_rev: int = 4096):
        self.num_joints = num_joints
        self.ticks_per_rev = ticks_per_rev
        self.radians_per_tick = 2 * np.pi / ticks_per_rev
        
        # Estado
        self._positions = np.zeros(num_joints)
        self._velocities = np.zeros(num_joints)
        self._raw_ticks = np.zeros(num_joints, dtype=np.int64)
        self._last_ticks = np.zeros(num_joints, dtype=np.int64)
        self._last_timestamp = 0.0
        
        # Filtros
        self._vel_filter_alpha = 0.7
        
        # Límites
        self._position_limits = np.array([[-np.pi, np.pi]] * num_joints)
        
        # Detección de fallas
        self._fault_threshold_vel = 50.0  # rad/s
        self._fault_flags = np.zeros(num_joints, dtype=bool)
    
    def process(self, raw_data: Dict[str, Any]) -> EncoderData:
        """
        Procesa datos crudos de encoders.
        
        Args:
            raw_data: Datos crudos (ticks o posiciones)
            
        Returns:
            Datos procesados
        """
        import time
        
        timestamp = raw_data.get("timestamp", time.time())
        dt = timestamp - self._last_timestamp if self._last_timestamp > 0 else 0.01
        
        # Obtener datos
        if "ticks" in raw_data:
            ticks = np.array(raw_data["ticks"], dtype=np.int64)
            # Calcular posición desde ticks
            positions = ticks * self.radians_per_tick
            # Calcular velocidad desde delta ticks
            if self._last_timestamp > 0 and dt > 0:
                delta_ticks = ticks - self._last_ticks
                raw_velocities = (delta_ticks * self.radians_per_tick) / dt
            else:
                raw_velocities = np.zeros(self.num_joints)
            self._last_ticks = ticks.copy()
        elif "positions" in raw_data:
            positions = np.array(raw_data["positions"])
            # Calcular velocidad desde delta posición
            if self._last_timestamp > 0 and dt > 0:
                delta_pos = positions - self._positions
                # Manejar wrap-around
                delta_pos = np.where(delta_pos > np.pi, delta_pos - 2*np.pi, delta_pos)
                delta_pos = np.where(delta_pos < -np.pi, delta_pos + 2*np.pi, delta_pos)
                raw_velocities = delta_pos / dt
            else:
                raw_velocities = np.zeros(self.num_joints)
        else:
            positions = self._positions.copy()
            raw_velocities = self._velocities.copy()
        
        # Filtrar velocidades
        velocities = self._vel_filter_alpha * self._velocities + \
                     (1 - self._vel_filter_alpha) * raw_velocities
        
        # Detectar fallas
        self._detect_faults(velocities)
        
        # Actualizar estado
        self._positions = positions.copy()
        self._velocities = velocities.copy()
        self._last_timestamp = timestamp
        
        # Corrientes si están disponibles
        currents = None
        if "currents" in raw_data:
            currents = np.array(raw_data["currents"])
        
        return EncoderData(
            timestamp=timestamp,
            positions=positions,
            velocities=velocities,
            currents=currents,
        )
    
    def _detect_faults(self, velocities: np.ndarray) -> None:
        """Detecta fallas en los motores."""
        # Velocidad excesiva
        self._fault_flags = np.abs(velocities) > self._fault_threshold_vel
        
        if np.any(self._fault_flags):
            faulted_joints = np.where(self._fault_flags)[0]
            _log.warning("Encoder fault detected on joints: %s", faulted_joints)
    
    def get_positions(self) -> np.ndarray:
        """Retorna posiciones actuales."""
        return self._positions.copy()
    
    def get_velocities(self) -> np.ndarray:
        """Retorna velocidades actuales."""
        return self._velocities.copy()
    
    def get_faults(self) -> np.ndarray:
        """Retorna flags de fallas."""
        return self._fault_flags.copy()
    
    def reset(self, joint_idx: Optional[int] = None) -> None:
        """Resetea el encoder (posición a cero)."""
        if joint_idx is not None:
            self._positions[joint_idx] = 0
            self._velocities[joint_idx] = 0
            self._fault_flags[joint_idx] = False
        else:
            self._positions.fill(0)
            self._velocities.fill(0)
            self._fault_flags.fill(False)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "num_joints": self.num_joints,
            "positions": self._positions.tolist(),
            "velocities": self._velocities.tolist(),
            "faults": self._fault_flags.tolist(),
            "num_faults": int(np.sum(self._fault_flags)),
        }
