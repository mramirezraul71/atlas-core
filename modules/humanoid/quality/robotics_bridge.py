"""
ATLAS Quality - Robotics Bridge
================================
Conecta los módulos robóticos con el Cerebro de ATLAS.

Este módulo permite:
1. Registrar eventos de navegación, manipulación, HRI
2. Monitorear estado de módulos robóticos
3. Disparar POTs automáticamente ante eventos
4. Sincronizar estado con el sistema nervioso autónomo
"""
from __future__ import annotations

import logging
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional
from dataclasses import dataclass

from .cerebro_connector import AtlasQualityBridge, CerebroEvent, get_bridge

_log = logging.getLogger("humanoid.quality.robotics_bridge")


# ============================================================================
# ROBOTICS EVENT TYPES
# ============================================================================

@dataclass
class RoboticsEvent:
    """Evento de un módulo robótico."""
    module: str  # navigation, manipulation, hri, sensors, simulation
    event_type: str  # state_change, error, completion, warning
    message: str
    data: Optional[Dict[str, Any]] = None
    severity: str = "info"  # info, warning, error, critical


# ============================================================================
# MODULE CONNECTORS
# ============================================================================

class NavigationBridge:
    """
    Conecta el módulo de navegación con el cerebro.
    
    Eventos:
    - navigation_started: Inicio de navegación
    - navigation_completed: Llegó al destino
    - navigation_failed: Falló la navegación
    - mapping_started: Inicio de mapeo SLAM
    - mapping_completed: Mapa guardado
    - localization_lost: Pérdida de localización
    """
    
    def __init__(self, bridge: Optional[AtlasQualityBridge] = None):
        self._bridge = bridge or get_bridge()
        self._active_goal = None
    
    def on_navigation_start(self, goal: Dict[str, float], path_length: float = 0) -> None:
        """Registrar inicio de navegación."""
        self._active_goal = goal
        event = CerebroEvent(
            type="navigation_started",
            source="robotics.navigation",
            message=f"[NAV] Navegando a ({goal.get('x', 0):.2f}, {goal.get('y', 0):.2f})",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "goal": goal,
                "path_length": path_length,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        self._bridge.channels.send_ops(f"Navegación iniciada: {goal}")
    
    def on_navigation_complete(self, success: bool, final_pos: Dict[str, float] = None) -> None:
        """Registrar fin de navegación."""
        status = "completada" if success else "fallida"
        event = CerebroEvent(
            type="navigation_completed",
            source="robotics.navigation",
            message=f"[NAV] Navegación {status}",
            ok=success,
            timestamp=datetime.now().isoformat(),
            metadata={
                "goal": self._active_goal,
                "final_position": final_pos,
                "success": success,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        
        if not success:
            self._bridge.cerebro.create_incident(
                check_id="navigation_failure",
                message=f"Navegación falló al llegar a {self._active_goal}",
                severity="medium"
            )
        
        self._active_goal = None
    
    def on_mapping_complete(self, map_path: str, map_size: tuple = None) -> None:
        """Registrar mapa guardado."""
        event = CerebroEvent(
            type="mapping_completed",
            source="robotics.navigation",
            message=f"[SLAM] Mapa guardado: {map_path}",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "map_path": map_path,
                "map_size": map_size,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        self._bridge.channels.send_ops(f"Mapa SLAM guardado: {map_path}")
    
    def on_localization_lost(self) -> None:
        """Registrar pérdida de localización."""
        self._bridge.cerebro.create_incident(
            check_id="localization_lost",
            message="Sistema de localización perdió seguimiento",
            severity="high"
        )
        self._bridge.channels.send_telegram_sync("⚠️ ALERTA: Localización perdida")


class ManipulationBridge:
    """
    Conecta el módulo de manipulación con el cerebro.
    
    Eventos:
    - grasp_started: Inicio de agarre
    - grasp_completed: Agarre completado
    - grasp_failed: Agarre fallido
    - object_detected: Objeto detectado
    """
    
    def __init__(self, bridge: Optional[AtlasQualityBridge] = None):
        self._bridge = bridge or get_bridge()
    
    def on_grasp_start(self, object_name: str = "unknown", position: list = None) -> None:
        """Registrar inicio de agarre."""
        event = CerebroEvent(
            type="grasp_started",
            source="robotics.manipulation",
            message=f"[GRASP] Intentando agarrar: {object_name}",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "object": object_name,
                "position": position,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
    
    def on_grasp_complete(self, success: bool, object_name: str = "unknown", force: float = 0) -> None:
        """Registrar resultado de agarre."""
        status = "exitoso" if success else "fallido"
        event = CerebroEvent(
            type="grasp_completed",
            source="robotics.manipulation",
            message=f"[GRASP] Agarre {status}: {object_name}",
            ok=success,
            timestamp=datetime.now().isoformat(),
            metadata={
                "object": object_name,
                "success": success,
                "force_applied": force,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        
        if not success:
            self._bridge.cerebro.create_incident(
                check_id="grasp_failure",
                message=f"Fallo al agarrar objeto: {object_name}",
                severity="low"
            )
    
    def on_object_detected(self, objects: List[Dict[str, Any]]) -> None:
        """Registrar detección de objetos."""
        if objects:
            event = CerebroEvent(
                type="object_detected",
                source="robotics.manipulation",
                message=f"[VISION] Detectados {len(objects)} objetos",
                ok=True,
                timestamp=datetime.now().isoformat(),
                metadata={
                    "count": len(objects),
                    "objects": [o.get("class_name", "unknown") for o in objects[:5]],
                }
            )
            self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())


class HRIBridge:
    """
    Conecta el módulo HRI con el cerebro.
    
    Eventos:
    - interaction_started: Persona detectada, interacción iniciada
    - command_received: Comando de voz/gesto recibido
    - action_executed: Acción ejecutada
    - safety_alert: Alerta de seguridad
    - emergency_stop: Parada de emergencia
    """
    
    def __init__(self, bridge: Optional[AtlasQualityBridge] = None):
        self._bridge = bridge or get_bridge()
    
    def on_interaction_start(self, person_distance: float = 0) -> None:
        """Registrar inicio de interacción."""
        event = CerebroEvent(
            type="interaction_started",
            source="robotics.hri",
            message=f"[HRI] Interacción iniciada (persona a {person_distance:.1f}m)",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "person_distance": person_distance,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
    
    def on_command_received(self, intent: str, text: str, confidence: float = 0) -> None:
        """Registrar comando recibido."""
        event = CerebroEvent(
            type="command_received",
            source="robotics.hri",
            message=f"[HRI] Comando: {intent} - '{text}'",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "intent": intent,
                "text": text,
                "confidence": confidence,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
    
    def on_safety_alert(self, level: str, reason: str) -> None:
        """Registrar alerta de seguridad."""
        event = CerebroEvent(
            type="safety_alert",
            source="robotics.hri",
            message=f"[SAFETY] Nivel {level}: {reason}",
            ok=level not in ("emergency", "warning"),
            timestamp=datetime.now().isoformat(),
            metadata={
                "level": level,
                "reason": reason,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        
        if level == "emergency":
            self._bridge.cerebro.create_incident(
                check_id="safety_emergency",
                message=f"PARADA DE EMERGENCIA: {reason}",
                severity="critical"
            )
            self._bridge.channels.send_telegram_sync(f"🔴 EMERGENCIA: {reason}")
    
    def on_emotion_detected(self, emotion: str, confidence: float = 0) -> None:
        """Registrar emoción detectada (para análisis)."""
        # Solo loggear emociones significativas
        if confidence > 0.7 and emotion not in ("neutral",):
            event = CerebroEvent(
                type="emotion_detected",
                source="robotics.hri",
                message=f"[HRI] Emoción detectada: {emotion} ({confidence:.0%})",
                ok=True,
                timestamp=datetime.now().isoformat(),
                metadata={
                    "emotion": emotion,
                    "confidence": confidence,
                }
            )
            self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())


class SensorsBridge:
    """
    Conecta el módulo de sensores con el cerebro.
    
    Eventos:
    - sensor_failure: Fallo de sensor
    - calibration_complete: Calibración completada
    - fusion_degraded: Fusión degradada (pocos sensores activos)
    """
    
    def __init__(self, bridge: Optional[AtlasQualityBridge] = None):
        self._bridge = bridge or get_bridge()
    
    def on_sensor_failure(self, sensor_name: str, error: str = "") -> None:
        """Registrar fallo de sensor."""
        self._bridge.cerebro.create_incident(
            check_id=f"sensor_{sensor_name}_failure",
            message=f"Sensor {sensor_name} falló: {error}",
            severity="medium"
        )
        self._bridge.channels.send_ops(f"⚠️ Sensor {sensor_name} falló", "error")
    
    def on_calibration_complete(self, sensor_name: str, params: Dict[str, Any] = None) -> None:
        """Registrar calibración completada."""
        event = CerebroEvent(
            type="calibration_complete",
            source="robotics.sensors",
            message=f"[SENSORS] Calibración completada: {sensor_name}",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "sensor": sensor_name,
                "params": params,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
    
    def on_fusion_status_change(self, active_sensors: int, total_sensors: int, confidence: float) -> None:
        """Registrar cambio en estado de fusión."""
        degraded = confidence < 0.5 or active_sensors < total_sensors / 2
        
        if degraded:
            self._bridge.cerebro.create_incident(
                check_id="sensor_fusion_degraded",
                message=f"Fusión sensorial degradada: {active_sensors}/{total_sensors} sensores, confianza {confidence:.0%}",
                severity="medium"
            )


class SimulationBridge:
    """
    Conecta el módulo de simulación con el cerebro.
    
    Eventos:
    - training_started: Entrenamiento iniciado
    - training_completed: Entrenamiento completado
    - episode_complete: Episodio completado (resumen)
    """
    
    def __init__(self, bridge: Optional[AtlasQualityBridge] = None):
        self._bridge = bridge or get_bridge()
    
    def on_training_start(self, task: str, max_episodes: int = 0) -> None:
        """Registrar inicio de entrenamiento."""
        event = CerebroEvent(
            type="training_started",
            source="robotics.simulation",
            message=f"[SIM] Entrenamiento iniciado: {task}",
            ok=True,
            timestamp=datetime.now().isoformat(),
            metadata={
                "task": task,
                "max_episodes": max_episodes,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        self._bridge.channels.send_ops(f"Entrenamiento iniciado: {task}")
    
    def on_training_complete(self, task: str, episodes: int, success_rate: float) -> None:
        """Registrar fin de entrenamiento."""
        event = CerebroEvent(
            type="training_completed",
            source="robotics.simulation",
            message=f"[SIM] Entrenamiento completado: {task} ({episodes} episodios, {success_rate:.0%} éxito)",
            ok=success_rate > 0.5,
            timestamp=datetime.now().isoformat(),
            metadata={
                "task": task,
                "episodes": episodes,
                "success_rate": success_rate,
            }
        )
        self._bridge.cerebro._http_request("POST", "/ans/evolution-log", event.to_dict())
        self._bridge.channels.send_telegram_sync(
            f"🤖 Entrenamiento completado: {task}\n"
            f"Episodios: {episodes}\n"
            f"Tasa de éxito: {success_rate:.0%}"
        )


# ============================================================================
# UNIFIED ROBOTICS BRIDGE
# ============================================================================

class RoboticsBridge:
    """
    Puente unificado para todos los módulos robóticos.
    
    Uso:
        from modules.humanoid.quality.robotics_bridge import get_robotics_bridge
        
        bridge = get_robotics_bridge()
        bridge.navigation.on_navigation_start({"x": 1.0, "y": 2.0})
        bridge.manipulation.on_grasp_complete(success=True, object_name="cup")
        bridge.hri.on_command_received("fetch", "tráeme un vaso")
    """
    
    def __init__(self, base_bridge: Optional[AtlasQualityBridge] = None):
        self._base = base_bridge or get_bridge()
        
        # Sub-bridges
        self.navigation = NavigationBridge(self._base)
        self.manipulation = ManipulationBridge(self._base)
        self.hri = HRIBridge(self._base)
        self.sensors = SensorsBridge(self._base)
        self.simulation = SimulationBridge(self._base)
        
        _log.info("RoboticsBridge initialized with 5 module connectors")
    
    def get_status(self) -> Dict[str, Any]:
        """Obtener estado de todos los módulos."""
        return {
            "navigation": self._check_module("modules.humanoid.navigation"),
            "manipulation": self._check_module("modules.humanoid.manipulation"),
            "hri": self._check_module("modules.humanoid.hri"),
            "sensors": self._check_module("modules.humanoid.sensors"),
            "simulation": self._check_module("simulation"),
        }
    
    def _check_module(self, module_path: str) -> Dict[str, Any]:
        """Verificar si un módulo está disponible."""
        try:
            __import__(module_path.replace(".", "."), fromlist=[""])
            return {"available": True}
        except ImportError as e:
            return {"available": False, "error": str(e)}
    
    def log_event(self, event: RoboticsEvent) -> None:
        """Registrar evento genérico."""
        cerebro_event = CerebroEvent(
            type=f"robotics_{event.event_type}",
            source=f"robotics.{event.module}",
            message=event.message,
            ok=event.severity not in ("error", "critical"),
            timestamp=datetime.now().isoformat(),
            metadata=event.data,
        )
        self._base.cerebro._http_request("POST", "/ans/evolution-log", cerebro_event.to_dict())
        
        if event.severity == "critical":
            self._base.channels.send_telegram_sync(f"🔴 {event.message}")


# ============================================================================
# SINGLETON
# ============================================================================

_robotics_bridge: Optional[RoboticsBridge] = None


def get_robotics_bridge() -> RoboticsBridge:
    """Obtener instancia del bridge robótico."""
    global _robotics_bridge
    if _robotics_bridge is None:
        _robotics_bridge = RoboticsBridge()
    return _robotics_bridge


def init_robotics_bridge() -> RoboticsBridge:
    """Inicializar y retornar el bridge (útil para startup)."""
    bridge = get_robotics_bridge()
    _log.info("Robotics bridge ready: %s", bridge.get_status())
    return bridge


__all__ = [
    "RoboticsBridge",
    "RoboticsEvent",
    "NavigationBridge",
    "ManipulationBridge",
    "HRIBridge",
    "SensorsBridge",
    "SimulationBridge",
    "get_robotics_bridge",
    "init_robotics_bridge",
]
