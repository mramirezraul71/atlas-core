"""ATLAS Communication System - Sistema unificado de comunicaciones.

Este módulo proporciona:
- CommsHub: Hub central con retry, circuit breaker y health monitoring
- TelegramBridge: Comunicación con Telegram Bot API
- WhatsApp Bridge: Comunicación vía Twilio
- Webhook Bridge: Eventos a Make.com/MakePlay
- Event Bus: Pub/sub interno mejorado con async support
- Bootstrap: Inicialización centralizada de servicios

Uso básico:
    from modules.humanoid.comms import emit, broadcast, get_hub
    
    # Emitir mensaje (usa hub central)
    emit("subsystem", "Mensaje importante", level="high")
    
    # Broadcast a todos los canales apropiados
    hub = get_hub()
    hub.broadcast("Mensaje crítico", level="critical", subsystem="approval")

Uso avanzado:
    from modules.humanoid.comms import bootstrap_comms, health_check
    
    # Inicializar todos los servicios
    result = bootstrap_comms()
    
    # Verificar salud del sistema
    health = health_check()
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from modules.humanoid.kernel import BaseModule, HealthCheckMixin

# Importaciones principales del sistema de comunicación
from .telegram_bridge import TelegramBridge
from .ops_bus import emit as ops_emit, status as ops_status, recent as ops_recent

# Nuevo sistema unificado
from .hub import (
    CommsHub,
    get_hub,
    emit as hub_emit,
    status as hub_status,
    recent as hub_recent,
    CircuitBreaker,
    RetryPolicy,
    MessageLevel,
    ChannelState,
)

from .bootstrap import (
    bootstrap_comms,
    get_status as get_bootstrap_status,
    get_service_status,
    is_service_healthy,
    restart_service,
    health_check as bootstrap_health_check,
)


class CommsModule(BaseModule, HealthCheckMixin):
    """Módulo de comunicaciones de ATLAS.
    
    Integra:
    - Hub central de comunicaciones con retry y circuit breaker
    - Telegram Bridge para mensajes y aprobaciones
    - WhatsApp Bridge (opcional, Twilio)
    - Webhooks para integraciones externas
    - Sistema de health monitoring
    """
    
    name = "comms"

    def __init__(self) -> None:
        self.telegram = TelegramBridge()
        self._hub: Optional[CommsHub] = None
        self._initialized = False

    def init(self) -> None:
        """Inicializa el módulo de comunicaciones."""
        if self._initialized:
            return
        
        try:
            # Inicializar hub
            self._hub = get_hub()
            
            # Bootstrap de servicios (sin tests para no bloquear)
            result = bootstrap_comms(skip_tests=True)
            
            if not result.get("ok"):
                import logging
                logger = logging.getLogger("atlas.comms")
                for warning in result.get("warnings", []):
                    logger.warning(f"Comms bootstrap warning: {warning}")
            
            self._initialized = True
        except Exception as e:
            import logging
            logging.getLogger("atlas.comms").error(f"Error inicializando comms: {e}")

    @property
    def hub(self) -> CommsHub:
        """Obtiene el hub de comunicaciones."""
        if self._hub is None:
            self._hub = get_hub()
        return self._hub

    def health_check(self) -> Dict[str, Any]:
        """Ejecuta health check completo del sistema de comunicaciones."""
        try:
            hub_health = self.hub.get_health()
            telegram_health = self.telegram.health_check()
            
            # Estado general
            ok = hub_health.get("ok", False) and telegram_health.get("ok", False)
            
            return {
                "ok": ok,
                "module": self.name,
                "hub": hub_health,
                "telegram": telegram_health,
                "initialized": self._initialized,
            }
        except Exception as e:
            return {
                "ok": False,
                "module": self.name,
                "error": str(e),
                "initialized": self._initialized,
            }

    def info(self) -> Dict[str, Any]:
        """Información del módulo."""
        return {
            "module": self.name,
            "version": "2.0.0",
            "features": [
                "unified_hub",
                "circuit_breaker",
                "retry_policy",
                "health_monitoring",
                "multi_channel",
                "async_support",
            ],
            "channels": ["telegram", "whatsapp", "webhook", "audio", "ops", "bitacora"],
            "initialized": self._initialized,
        }

    def emit(
        self,
        subsystem: str,
        message: str,
        level: str = "info",
        data: Optional[Dict[str, Any]] = None,
        evidence_path: str = "",
    ) -> Dict[str, Any]:
        """Emite un mensaje a través del hub central.
        
        Compatible con la interfaz de ops_bus.emit()
        """
        return self.hub.send(
            content=message,
            level=level,
            subsystem=subsystem,
            data=data,
            evidence_path=evidence_path,
        ).__dict__

    def broadcast(
        self,
        message: str,
        level: str = "info",
        subsystem: str = "system",
        **kwargs: Any,
    ) -> Dict[str, Any]:
        """Broadcast a todos los canales apropiados según nivel."""
        msg = self.hub.broadcast(
            content=message,
            level=level,
            subsystem=subsystem,
            **kwargs,
        )
        return {
            "ok": len(msg.channels_sent) > 0,
            "message_id": msg.id,
            "channels_sent": msg.channels_sent,
            "channels_failed": msg.channels_failed,
        }

    def get_recent_messages(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Obtiene mensajes recientes del hub."""
        return self.hub.get_recent_messages(limit=limit)

    def get_channel_health(self) -> Dict[str, Any]:
        """Obtiene el estado de salud de todos los canales."""
        return self.hub.get_health()


# Función de conveniencia global para emitir mensajes
def emit(
    subsystem: str,
    message: str,
    level: str = "info",
    data: Optional[Dict[str, Any]] = None,
    evidence_path: str = "",
) -> Dict[str, Any]:
    """Emite un mensaje a través del hub central de comunicaciones.
    
    Esta función es el punto de entrada recomendado para enviar mensajes
    desde cualquier parte del sistema ATLAS.
    
    Args:
        subsystem: Nombre del subsistema que emite (ej: "approval", "vision")
        message: Contenido del mensaje
        level: Nivel de importancia (debug, info, low, medium, high, critical)
        data: Datos adicionales estructurados
        evidence_path: Ruta a archivo de evidencia (screenshot, etc.)
    
    Returns:
        Dict con información del envío
    
    Ejemplo:
        emit("approval", "Nueva aprobación pendiente", level="high", data={"id": "abc123"})
    """
    hub = get_hub()
    msg = hub.send(
        content=message,
        level=level,
        subsystem=subsystem,
        data=data,
        evidence_path=evidence_path,
    )
    return {
        "ok": len(msg.channels_sent) > 0,
        "message_id": msg.id,
        "channels_sent": msg.channels_sent,
        "channels_failed": msg.channels_failed,
    }


def broadcast(
    message: str,
    level: str = "info",
    subsystem: str = "system",
    data: Optional[Dict[str, Any]] = None,
    evidence_path: str = "",
) -> Dict[str, Any]:
    """Broadcast a todos los canales apropiados según el nivel.
    
    Alias conveniente para emit() con parámetros en orden diferente.
    """
    return emit(
        subsystem=subsystem,
        message=message,
        level=level,
        data=data,
        evidence_path=evidence_path,
    )


def status() -> Dict[str, Any]:
    """Obtiene el estado del sistema de comunicaciones."""
    return get_hub().get_health()


def recent(limit: int = 50) -> List[Dict[str, Any]]:
    """Obtiene los mensajes recientes."""
    return get_hub().get_recent_messages(limit=limit)


def health_check() -> Dict[str, Any]:
    """Ejecuta health check completo del sistema."""
    return bootstrap_health_check()


# Exports públicos
__all__ = [
    # Módulo principal
    "CommsModule",
    
    # Hub y componentes
    "CommsHub",
    "get_hub",
    "CircuitBreaker",
    "RetryPolicy",
    "MessageLevel",
    "ChannelState",
    
    # Bridges
    "TelegramBridge",
    
    # Funciones de conveniencia
    "emit",
    "broadcast",
    "status",
    "recent",
    "health_check",
    
    # Bootstrap
    "bootstrap_comms",
    "get_bootstrap_status",
    "get_service_status",
    "is_service_healthy",
    "restart_service",
    
    # Legacy (compatibilidad con código existente)
    "ops_emit",
    "ops_status",
    "ops_recent",
]
