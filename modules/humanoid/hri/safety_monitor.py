"""
Safety Monitor: Monitor de seguridad HRI.
==========================================
Garantiza interacción segura con humanos.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple
import numpy as np

_log = logging.getLogger("humanoid.hri.safety")


class SafetyLevel(str, Enum):
    """Niveles de seguridad."""
    NORMAL = "normal"
    CAUTION = "caution"
    WARNING = "warning"
    EMERGENCY = "emergency"


@dataclass
class SafetyZone:
    """Zona de seguridad alrededor del robot."""
    name: str
    radius: float  # metros
    level: SafetyLevel
    speed_limit: float  # m/s
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "radius": self.radius,
            "level": self.level.value,
            "speed_limit": self.speed_limit,
        }


class SafetyMonitor:
    """
    Monitor de seguridad para interacción humano-robot.
    
    Funcionalidades:
    - Zonas de seguridad
    - Detección de personas
    - Límites de velocidad/fuerza
    - Parada de emergencia
    """
    
    # Zonas por defecto (ISO 10218, ISO/TS 15066)
    DEFAULT_ZONES = [
        SafetyZone("emergency", 0.3, SafetyLevel.EMERGENCY, 0.0),
        SafetyZone("warning", 0.5, SafetyLevel.WARNING, 0.1),
        SafetyZone("caution", 1.0, SafetyLevel.CAUTION, 0.3),
        SafetyZone("normal", 2.0, SafetyLevel.NORMAL, 0.5),
    ]
    
    def __init__(self):
        self._zones = self.DEFAULT_ZONES.copy()
        self._current_level = SafetyLevel.NORMAL
        
        # Personas detectadas
        self._detected_persons: List[Dict[str, Any]] = []
        
        # Límites
        self._max_force = 150.0  # N (ISO/TS 15066)
        self._max_pressure = 300.0  # N/cm²
        
        # Estado
        self._emergency_stop = False
        self._speed_override: Optional[float] = None
        
        _log.info("Safety monitor initialized with %d zones", len(self._zones))
    
    def update_persons(self, persons: List[Dict[str, Any]]) -> SafetyLevel:
        """
        Actualiza detección de personas y nivel de seguridad.
        
        Args:
            persons: Lista de personas detectadas con posición
                    [{"id": int, "position": [x,y,z], "distance": float}, ...]
            
        Returns:
            Nivel de seguridad actual
        """
        self._detected_persons = persons
        
        if not persons:
            self._current_level = SafetyLevel.NORMAL
            return self._current_level
        
        # Encontrar persona más cercana
        min_distance = min(p.get("distance", float('inf')) for p in persons)
        
        # Determinar nivel según zona
        for zone in self._zones:
            if min_distance <= zone.radius:
                self._current_level = zone.level
                _log.debug("Person at %.2fm, level: %s", min_distance, zone.level.value)
                break
        else:
            self._current_level = SafetyLevel.NORMAL
        
        # Activar parada de emergencia si necesario
        if self._current_level == SafetyLevel.EMERGENCY:
            self._emergency_stop = True
            _log.warning("EMERGENCY STOP: Person too close (%.2fm)", min_distance)
        
        return self._current_level
    
    def get_speed_limit(self) -> float:
        """Retorna el límite de velocidad actual."""
        if self._emergency_stop:
            return 0.0
        
        if self._speed_override is not None:
            return self._speed_override
        
        for zone in self._zones:
            if zone.level == self._current_level:
                return zone.speed_limit
        
        return 0.5  # Default
    
    def check_force(self, force: float) -> bool:
        """
        Verifica si la fuerza está dentro de límites seguros.
        
        Args:
            force: Fuerza actual (N)
            
        Returns:
            True si es seguro
        """
        if force > self._max_force:
            _log.warning("Force limit exceeded: %.1f N > %.1f N", force, self._max_force)
            return False
        return True
    
    def check_contact(self, force: float, contact_area: float) -> bool:
        """
        Verifica presión de contacto.
        
        Args:
            force: Fuerza (N)
            contact_area: Área de contacto (cm²)
            
        Returns:
            True si es seguro
        """
        if contact_area <= 0:
            return False
        
        pressure = force / contact_area
        
        if pressure > self._max_pressure:
            _log.warning("Pressure limit exceeded: %.1f N/cm²", pressure)
            return False
        
        return True
    
    def emergency_stop(self) -> None:
        """Activa parada de emergencia."""
        self._emergency_stop = True
        _log.warning("EMERGENCY STOP ACTIVATED")
    
    def reset_emergency(self) -> bool:
        """
        Resetea parada de emergencia.
        
        Returns:
            True si se pudo resetear
        """
        # Solo resetear si no hay personas en zona de emergencia
        for p in self._detected_persons:
            if p.get("distance", float('inf')) <= self._zones[0].radius:
                _log.warning("Cannot reset: person in emergency zone")
                return False
        
        self._emergency_stop = False
        _log.info("Emergency stop reset")
        return True
    
    def is_safe_to_move(self) -> Tuple[bool, str]:
        """
        Verifica si es seguro moverse.
        
        Returns:
            (safe: bool, reason: str)
        """
        if self._emergency_stop:
            return False, "Emergency stop active"
        
        if self._current_level == SafetyLevel.EMERGENCY:
            return False, "Person in emergency zone"
        
        return True, "Safe to move"
    
    def set_zone(self, name: str, radius: float, level: SafetyLevel, speed_limit: float) -> None:
        """Configura una zona de seguridad."""
        for i, zone in enumerate(self._zones):
            if zone.name == name:
                self._zones[i] = SafetyZone(name, radius, level, speed_limit)
                return
        
        self._zones.append(SafetyZone(name, radius, level, speed_limit))
        self._zones.sort(key=lambda z: z.radius)
    
    @property
    def current_level(self) -> SafetyLevel:
        return self._current_level
    
    @property
    def is_emergency(self) -> bool:
        return self._emergency_stop
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "level": self._current_level.value,
            "emergency_stop": self._emergency_stop,
            "speed_limit": self.get_speed_limit(),
            "detected_persons": len(self._detected_persons),
            "zones": [z.to_dict() for z in self._zones],
        }
