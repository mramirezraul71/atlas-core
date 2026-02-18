"""Communication Hub: punto central de comunicación de ATLAS.

Unifica todos los canales (Telegram, WhatsApp, Audio, Webhooks, EventBus interno)
con retry automático, circuit breaker, health monitoring y logging estructurado.

Uso:
    from modules.humanoid.comms.hub import CommsHub
    hub = CommsHub()
    hub.broadcast("Mensaje importante", level="high", subsystem="approval")
"""
from __future__ import annotations

import logging
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Callable, Deque, Dict, List, Optional, Set

logger = logging.getLogger("atlas.comms.hub")


class ChannelState(Enum):
    """Estado de un canal de comunicación."""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    FAILED = "failed"
    DISABLED = "disabled"


class MessageLevel(Enum):
    """Nivel de importancia del mensaje."""
    DEBUG = "debug"
    INFO = "info"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class ChannelHealth:
    """Estado de salud de un canal."""
    name: str
    state: ChannelState = ChannelState.HEALTHY
    last_success: Optional[str] = None
    last_failure: Optional[str] = None
    consecutive_failures: int = 0
    total_sent: int = 0
    total_failed: int = 0
    last_error: str = ""
    circuit_open_until: Optional[float] = None  # timestamp


@dataclass
class CommsMessage:
    """Mensaje unificado para todos los canales."""
    id: str
    content: str
    level: MessageLevel
    subsystem: str
    timestamp: str
    data: Dict[str, Any] = field(default_factory=dict)
    evidence_path: str = ""
    channels_sent: List[str] = field(default_factory=list)
    channels_failed: List[str] = field(default_factory=list)


class CircuitBreaker:
    """Circuit breaker para evitar saturar canales fallidos."""
    
    def __init__(
        self,
        failure_threshold: int = 3,
        recovery_timeout: float = 60.0,
        half_open_max_calls: int = 1,
    ):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.half_open_max_calls = half_open_max_calls
        self._failures: Dict[str, int] = {}
        self._open_until: Dict[str, float] = {}
        self._half_open_calls: Dict[str, int] = {}
        self._lock = threading.Lock()
    
    def is_open(self, channel: str) -> bool:
        """Retorna True si el circuito está abierto (no permitir llamadas)."""
        with self._lock:
            open_until = self._open_until.get(channel)
            if open_until is None:
                return False
            if time.time() >= open_until:
                # Transition to half-open
                self._half_open_calls[channel] = 0
                return False
            return True
    
    def record_success(self, channel: str) -> None:
        """Registra un éxito y cierra el circuito."""
        with self._lock:
            self._failures[channel] = 0
            self._open_until.pop(channel, None)
            self._half_open_calls.pop(channel, None)
    
    def record_failure(self, channel: str) -> None:
        """Registra un fallo y abre el circuito si supera el umbral."""
        with self._lock:
            self._failures[channel] = self._failures.get(channel, 0) + 1
            if self._failures[channel] >= self.failure_threshold:
                self._open_until[channel] = time.time() + self.recovery_timeout
                self._failures[channel] = 0
    
    def get_state(self, channel: str) -> str:
        """Retorna el estado del circuito: closed, open, half-open."""
        with self._lock:
            open_until = self._open_until.get(channel)
            if open_until is None:
                return "closed"
            if time.time() >= open_until:
                return "half-open"
            return "open"


class RetryPolicy:
    """Política de reintentos con backoff exponencial."""
    
    def __init__(
        self,
        max_retries: int = 3,
        initial_delay: float = 0.5,
        max_delay: float = 10.0,
        exponential_base: float = 2.0,
    ):
        self.max_retries = max_retries
        self.initial_delay = initial_delay
        self.max_delay = max_delay
        self.exponential_base = exponential_base
    
    def execute(
        self,
        func: Callable[[], Any],
        channel_name: str = "",
        on_retry: Optional[Callable[[int, Exception], None]] = None,
    ) -> tuple[bool, Any, Optional[Exception]]:
        """Ejecuta la función con reintentos.
        
        Returns:
            (success, result, last_exception)
        """
        last_exception: Optional[Exception] = None
        delay = self.initial_delay
        
        for attempt in range(self.max_retries + 1):
            try:
                result = func()
                return (True, result, None)
            except Exception as e:
                last_exception = e
                if attempt < self.max_retries:
                    if on_retry:
                        on_retry(attempt + 1, e)
                    time.sleep(min(delay, self.max_delay))
                    delay *= self.exponential_base
        
        return (False, None, last_exception)


class CommsHub:
    """Hub central de comunicación de ATLAS.
    
    Características:
    - Unifica todos los canales (Telegram, WhatsApp, Audio, Webhooks)
    - Circuit breaker por canal
    - Retry automático con backoff
    - Logging estructurado
    - Health monitoring en tiempo real
    - Rate limiting por canal
    - Historial de mensajes
    """
    
    # Mapeo de nivel a canales por defecto
    # Audio se incluye desde INFO para que ATLAS "hable" constantemente
    LEVEL_CHANNEL_MAP: Dict[MessageLevel, List[str]] = {
        MessageLevel.DEBUG: ["log"],
        MessageLevel.INFO: ["log", "ops", "audio"],
        MessageLevel.LOW: ["log", "ops", "audio"],
        MessageLevel.MEDIUM: ["log", "ops", "audio", "bitacora"],
        MessageLevel.HIGH: ["log", "ops", "audio", "bitacora", "telegram"],
        MessageLevel.CRITICAL: ["log", "ops", "audio", "bitacora", "telegram", "whatsapp"],
    }
    
    def __init__(self) -> None:
        self._circuit_breaker = CircuitBreaker(
            failure_threshold=3,
            recovery_timeout=60.0,
        )
        self._retry_policy = RetryPolicy(
            max_retries=2,
            initial_delay=0.5,
            max_delay=5.0,
        )
        self._channel_health: Dict[str, ChannelHealth] = {}
        self._message_history: Deque[CommsMessage] = deque(maxlen=500)
        self._lock = threading.RLock()
        self._subscribers: Dict[str, List[Callable[[CommsMessage], None]]] = {}
        self._message_counter = 0
        self._initialized = False
        
        # Rate limiting
        self._rate_windows: Dict[str, Deque[float]] = {}
        self._rate_limits: Dict[str, tuple[int, float]] = {
            "telegram": (5, 10.0),  # 5 mensajes en 10 segundos
            "whatsapp": (3, 10.0),  # 3 mensajes en 10 segundos
            "audio": (2, 5.0),      # 2 mensajes en 5 segundos
            "webhook": (10, 10.0),  # 10 eventos en 10 segundos
        }
        
        self._init_channels()
    
    def _init_channels(self) -> None:
        """Inicializa los canales disponibles."""
        channels = ["telegram", "whatsapp", "audio", "webhook", "ops", "bitacora", "log"]
        for name in channels:
            self._channel_health[name] = ChannelHealth(name=name)
            self._rate_windows[name] = deque(maxlen=100)
        self._initialized = True
    
    def _generate_message_id(self) -> str:
        """Genera un ID único para el mensaje."""
        with self._lock:
            self._message_counter += 1
            ts = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
            return f"msg-{ts}-{self._message_counter:06d}"
    
    def _check_rate_limit(self, channel: str) -> bool:
        """Verifica si el canal está dentro del rate limit."""
        if channel not in self._rate_limits:
            return True
        
        max_calls, window_sec = self._rate_limits[channel]
        now = time.time()
        
        with self._lock:
            window = self._rate_windows.get(channel, deque(maxlen=100))
            # Limpia timestamps viejos
            while window and now - window[0] > window_sec:
                window.popleft()
            
            if len(window) >= max_calls:
                return False
            
            window.append(now)
            self._rate_windows[channel] = window
            return True
    
    def _update_health(
        self,
        channel: str,
        success: bool,
        error: str = "",
    ) -> None:
        """Actualiza el estado de salud de un canal."""
        with self._lock:
            health = self._channel_health.get(channel)
            if not health:
                health = ChannelHealth(name=channel)
                self._channel_health[channel] = health
            
            now = datetime.now(timezone.utc).isoformat()
            
            if success:
                health.last_success = now
                health.consecutive_failures = 0
                health.total_sent += 1
                if health.state != ChannelState.DISABLED:
                    health.state = ChannelState.HEALTHY
                self._circuit_breaker.record_success(channel)
            else:
                health.last_failure = now
                health.consecutive_failures += 1
                health.total_failed += 1
                health.last_error = error[:500]
                self._circuit_breaker.record_failure(channel)
                
                if health.consecutive_failures >= 3:
                    health.state = ChannelState.FAILED
                elif health.consecutive_failures >= 1:
                    health.state = ChannelState.DEGRADED
    
    def _is_channel_enabled(self, channel: str) -> bool:
        """Verifica si un canal está habilitado por configuración."""
        # Mapeo de canal a variables de entorno (primera que exista)
        channel_env_map = {
            "telegram": ["OPS_TELEGRAM_ENABLED"],
            "whatsapp": ["WHATSAPP_ENABLED", "OPS_WHATSAPP_ENABLED"],
            "audio": ["OPS_AUDIO_ENABLED"],
            "webhook": ["MAKEPLAY_ENABLED"],
        }
        env_vars = channel_env_map.get(channel, [])
        if not env_vars:
            return True  # Canales sin var de env están siempre activos
        
        # Buscar primera variable configurada
        for env_var in env_vars:
            value = (os.getenv(env_var) or "").strip().lower()
            if value:
                return value in ("1", "true", "yes", "on")
        
        # Defaults si no hay ninguna configurada
        if channel in ("telegram", "audio"):
            return True
        return False
    
    def _send_telegram(self, msg: CommsMessage) -> Dict[str, Any]:
        """Envía mensaje a Telegram."""
        try:
            from modules.humanoid.comms.telegram_bridge import TelegramBridge
            from modules.humanoid.comms.ops_bus import _telegram_chat_id, _subsystem_human
            
            chat_id = _telegram_chat_id()
            if not chat_id:
                return {"ok": False, "error": "no_chat_id"}
            
            bridge = TelegramBridge()
            header = f"<b>ATLAS</b> — {_subsystem_human(msg.subsystem)}\n"
            text = header + msg.content
            
            if msg.evidence_path and str(msg.evidence_path).lower().endswith(
                (".png", ".jpg", ".jpeg", ".webp")
            ):
                result = bridge.send_photo(chat_id, str(msg.evidence_path), caption=text[:900])
            else:
                result = bridge.send(chat_id, text)
            
            return result
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def _send_whatsapp(self, msg: CommsMessage) -> Dict[str, Any]:
        """Envía mensaje a WhatsApp."""
        try:
            from modules.humanoid.comms.whatsapp_bridge import send_text
            
            text = f"[ATLAS {msg.subsystem.upper()}] {msg.content}"
            return send_text(text)
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def _send_audio(self, msg: CommsMessage) -> Dict[str, Any]:
        """Reproduce mensaje por audio (TTS)."""
        try:
            from modules.humanoid.voice.tts import speak
            
            text = msg.content[:240]
            speak(text)
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def _send_webhook(self, msg: CommsMessage) -> Dict[str, Any]:
        """Envía evento al webhook externo."""
        try:
            from modules.humanoid.comms.webhook_bridge import push_event
            
            event = {
                "id": msg.id,
                "content": msg.content,
                "level": msg.level.value,
                "subsystem": msg.subsystem,
                "timestamp": msg.timestamp,
                "data": msg.data,
            }
            pushed = push_event(event)
            return {"ok": pushed}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def _send_ops(self, msg: CommsMessage) -> Dict[str, Any]:
        """Registra en el bus de operaciones interno."""
        try:
            from modules.humanoid.comms.ops_bus import _RECENT, _append_log
            
            ev = {
                "ts": msg.timestamp,
                "subsystem": msg.subsystem,
                "level": msg.level.value,
                "message": msg.content[:1200],
                "data": msg.data,
            }
            _RECENT.append(ev)
            _append_log(f"{ev['ts']} [{ev['level']}] {ev['subsystem']}: {ev['message']}")
            return {"ok": True}
        except Exception as e:
            return {"ok": False, "error": str(e)}
    
    def _send_bitacora(self, msg: CommsMessage) -> Dict[str, Any]:
        """Registra en la bitácora ANS."""
        try:
            from modules.humanoid.ans.evolution_bitacora import append_evolution_log
            
            ok = msg.level.value not in ("HIGH", "CRITICAL")
            append_evolution_log(
                f"[{msg.subsystem}] {msg.content[:400]}",
                ok=ok,
                source=msg.subsystem or "comms",
            )
            return {"ok": True}
        except Exception:
            return {"ok": True, "skipped": "bitacora_not_available"}
    
    def _send_log(self, msg: CommsMessage) -> Dict[str, Any]:
        """Registra en el logger Python."""
        log_method = {
            MessageLevel.DEBUG: logger.debug,
            MessageLevel.INFO: logger.info,
            MessageLevel.LOW: logger.info,
            MessageLevel.MEDIUM: logger.warning,
            MessageLevel.HIGH: logger.warning,
            MessageLevel.CRITICAL: logger.error,
        }.get(msg.level, logger.info)
        
        log_method(f"[{msg.subsystem}] {msg.content}", extra={"data": msg.data})
        return {"ok": True}
    
    def _get_sender(self, channel: str) -> Optional[Callable[[CommsMessage], Dict[str, Any]]]:
        """Obtiene la función de envío para un canal."""
        senders = {
            "telegram": self._send_telegram,
            "whatsapp": self._send_whatsapp,
            "audio": self._send_audio,
            "webhook": self._send_webhook,
            "ops": self._send_ops,
            "bitacora": self._send_bitacora,
            "log": self._send_log,
        }
        return senders.get(channel)
    
    def send(
        self,
        content: str,
        level: str = "info",
        subsystem: str = "system",
        data: Optional[Dict[str, Any]] = None,
        evidence_path: str = "",
        channels: Optional[List[str]] = None,
        skip_channels: Optional[Set[str]] = None,
    ) -> CommsMessage:
        """Envía un mensaje a través de los canales apropiados.
        
        Args:
            content: Contenido del mensaje
            level: Nivel de importancia (debug, info, low, medium, high, critical)
            subsystem: Subsistema que envía el mensaje
            data: Datos adicionales
            evidence_path: Ruta a archivo de evidencia (screenshot, etc.)
            channels: Lista específica de canales (si None, usa mapeo por nivel)
            skip_channels: Canales a omitir
        
        Returns:
            CommsMessage con información del envío
        """
        # Normalizar nivel
        level_enum = {
            "debug": MessageLevel.DEBUG,
            "info": MessageLevel.INFO,
            "low": MessageLevel.LOW,
            "med": MessageLevel.MEDIUM,
            "medium": MessageLevel.MEDIUM,
            "high": MessageLevel.HIGH,
            "critical": MessageLevel.CRITICAL,
        }.get(level.lower(), MessageLevel.INFO)
        
        # Crear mensaje
        msg = CommsMessage(
            id=self._generate_message_id(),
            content=content.strip()[:2000],
            level=level_enum,
            subsystem=subsystem.strip(),
            timestamp=datetime.now(timezone.utc).isoformat(),
            data=data or {},
            evidence_path=evidence_path,
        )
        
        # Determinar canales
        if channels is None:
            channels = list(self.LEVEL_CHANNEL_MAP.get(level_enum, ["log", "ops"]))
        
        skip = skip_channels or set()
        
        # Enviar a cada canal
        for channel in channels:
            if channel in skip:
                continue
            
            # Verificar si está habilitado
            if not self._is_channel_enabled(channel):
                continue
            
            # Verificar circuit breaker
            if self._circuit_breaker.is_open(channel):
                msg.channels_failed.append(f"{channel}:circuit_open")
                continue
            
            # Verificar rate limit
            if not self._check_rate_limit(channel):
                msg.channels_failed.append(f"{channel}:rate_limited")
                continue
            
            # Obtener sender
            sender = self._get_sender(channel)
            if not sender:
                continue
            
            # Enviar con retry
            def send_fn() -> Dict[str, Any]:
                return sender(msg)
            
            success, result, exc = self._retry_policy.execute(
                send_fn,
                channel_name=channel,
            )
            
            if success and result and result.get("ok"):
                msg.channels_sent.append(channel)
                self._update_health(channel, True)
            else:
                error = str(exc) if exc else (result.get("error") if result else "unknown")
                msg.channels_failed.append(f"{channel}:{error[:50]}")
                self._update_health(channel, False, error or "")
        
        # Guardar en historial
        with self._lock:
            self._message_history.append(msg)
        
        # Notificar subscribers
        self._notify_subscribers(msg)
        
        return msg
    
    def broadcast(
        self,
        content: str,
        level: str = "info",
        subsystem: str = "system",
        data: Optional[Dict[str, Any]] = None,
        evidence_path: str = "",
    ) -> CommsMessage:
        """Alias para send() con canales automáticos por nivel."""
        return self.send(
            content=content,
            level=level,
            subsystem=subsystem,
            data=data,
            evidence_path=evidence_path,
        )
    
    def subscribe(self, topic: str, handler: Callable[[CommsMessage], None]) -> None:
        """Suscribe un handler para recibir mensajes."""
        with self._lock:
            if topic not in self._subscribers:
                self._subscribers[topic] = []
            self._subscribers[topic].append(handler)
    
    def unsubscribe(self, topic: str, handler: Callable[[CommsMessage], None]) -> None:
        """Desuscribe un handler."""
        with self._lock:
            if topic in self._subscribers:
                try:
                    self._subscribers[topic].remove(handler)
                except ValueError:
                    pass
    
    def _notify_subscribers(self, msg: CommsMessage) -> None:
        """Notifica a todos los subscribers."""
        with self._lock:
            # Notificar al topic específico del subsistema
            handlers = list(self._subscribers.get(msg.subsystem, []))
            # Notificar al topic global
            handlers.extend(self._subscribers.get("*", []))
        
        for handler in handlers:
            try:
                handler(msg)
            except Exception as e:
                logger.warning(f"Subscriber error: {e}")
    
    def get_health(self) -> Dict[str, Any]:
        """Obtiene el estado de salud de todos los canales."""
        with self._lock:
            channels = {}
            for name, health in self._channel_health.items():
                cb_state = self._circuit_breaker.get_state(name)
                channels[name] = {
                    "state": health.state.value,
                    "circuit_breaker": cb_state,
                    "enabled": self._is_channel_enabled(name),
                    "last_success": health.last_success,
                    "last_failure": health.last_failure,
                    "consecutive_failures": health.consecutive_failures,
                    "total_sent": health.total_sent,
                    "total_failed": health.total_failed,
                    "last_error": health.last_error[:100] if health.last_error else None,
                }
            
            # Estado general
            all_healthy = all(
                h.state in (ChannelState.HEALTHY, ChannelState.DISABLED)
                for h in self._channel_health.values()
            )
            any_critical_failed = any(
                h.state == ChannelState.FAILED
                and h.name in ("telegram", "ops")
                for h in self._channel_health.values()
            )
            
            return {
                "ok": all_healthy and not any_critical_failed,
                "status": "healthy" if all_healthy else ("degraded" if not any_critical_failed else "unhealthy"),
                "channels": channels,
                "message_count": len(self._message_history),
            }
    
    def get_recent_messages(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Obtiene los mensajes recientes."""
        with self._lock:
            messages = list(self._message_history)[-limit:]
            return [
                {
                    "id": m.id,
                    "content": m.content[:500],
                    "level": m.level.value,
                    "subsystem": m.subsystem,
                    "timestamp": m.timestamp,
                    "channels_sent": m.channels_sent,
                    "channels_failed": m.channels_failed,
                }
                for m in messages
            ]
    
    def reset_channel(self, channel: str) -> Dict[str, Any]:
        """Resetea el estado de un canal (útil después de arreglar un problema)."""
        with self._lock:
            if channel in self._channel_health:
                self._channel_health[channel] = ChannelHealth(name=channel)
            self._circuit_breaker.record_success(channel)
            return {"ok": True, "channel": channel, "state": "reset"}
        return {"ok": False, "error": "channel_not_found"}
    
    def disable_channel(self, channel: str) -> Dict[str, Any]:
        """Deshabilita temporalmente un canal."""
        with self._lock:
            if channel in self._channel_health:
                self._channel_health[channel].state = ChannelState.DISABLED
                return {"ok": True, "channel": channel, "state": "disabled"}
        return {"ok": False, "error": "channel_not_found"}
    
    def enable_channel(self, channel: str) -> Dict[str, Any]:
        """Habilita un canal deshabilitado."""
        with self._lock:
            if channel in self._channel_health:
                self._channel_health[channel].state = ChannelState.HEALTHY
                self._channel_health[channel].consecutive_failures = 0
                return {"ok": True, "channel": channel, "state": "enabled"}
        return {"ok": False, "error": "channel_not_found"}


# Singleton global
_hub_instance: Optional[CommsHub] = None
_hub_lock = threading.Lock()


def get_hub() -> CommsHub:
    """Obtiene la instancia singleton del CommsHub."""
    global _hub_instance
    if _hub_instance is None:
        with _hub_lock:
            if _hub_instance is None:
                _hub_instance = CommsHub()
    return _hub_instance


def emit(
    subsystem: str,
    message: str,
    level: str = "info",
    data: Optional[Dict[str, Any]] = None,
    evidence_path: str = "",
) -> Dict[str, Any]:
    """Función de conveniencia para emitir mensajes (compatible con ops_bus.emit)."""
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


def status() -> Dict[str, Any]:
    """Obtiene el estado del hub de comunicación."""
    return get_hub().get_health()


def recent(limit: int = 50) -> List[Dict[str, Any]]:
    """Obtiene los mensajes recientes."""
    return get_hub().get_recent_messages(limit=limit)
