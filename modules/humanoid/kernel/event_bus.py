"""In-memory event dispatch con async support, logging y manejo robusto de errores.

Características:
- Soporte para handlers síncronos y asíncronos
- Logging estructurado de eventos
- Manejo de errores por handler (no falla todo por uno)
- Historial de eventos recientes
- Métricas de rendimiento
- Priorización de handlers
"""
from __future__ import annotations

import asyncio
import inspect
import logging
import threading
import time
import traceback
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Callable, Coroutine, Deque, Dict, List, Optional, Set, Union

logger = logging.getLogger("atlas.kernel.event_bus")

# Tipo para handlers síncronos o asíncronos
Handler = Union[Callable[..., None], Callable[..., Coroutine[Any, Any, None]]]


@dataclass
class HandlerInfo:
    """Información de un handler registrado."""
    handler: Handler
    priority: int = 0  # Mayor = se ejecuta primero
    once: bool = False  # Si True, se desuscribe después de una ejecución
    error_count: int = 0
    call_count: int = 0
    total_time_ms: float = 0.0
    last_error: str = ""


@dataclass
class EventRecord:
    """Registro de un evento emitido."""
    topic: str
    timestamp: str
    args_preview: str
    handlers_called: int
    handlers_failed: int
    duration_ms: float


class EventBus:
    """Event bus mejorado con async support, logging y métricas.
    
    Uso básico:
        bus = EventBus()
        bus.subscribe("user.created", lambda user: print(f"New user: {user}"))
        bus.publish("user.created", {"id": 1, "name": "Alan"})
    
    Uso async:
        async def async_handler(data):
            await some_async_operation(data)
        
        bus.subscribe("data.received", async_handler)
        await bus.publish_async("data.received", data)
    """

    def __init__(
        self,
        max_history: int = 200,
        log_events: bool = True,
        max_handler_errors: int = 10,
    ) -> None:
        """Inicializa el EventBus.
        
        Args:
            max_history: Número máximo de eventos a mantener en historial
            log_events: Si True, loguea cada evento publicado
            max_handler_errors: Máximo de errores antes de desactivar un handler
        """
        self._handlers: Dict[str, List[HandlerInfo]] = {}
        self._lock = threading.RLock()
        self._history: Deque[EventRecord] = deque(maxlen=max_history)
        self._log_events = log_events
        self._max_handler_errors = max_handler_errors
        self._disabled_handlers: Set[int] = set()  # IDs de handlers desactivados
        self._event_count = 0
        self._total_publish_time_ms = 0.0

    def subscribe(
        self,
        topic: str,
        handler: Handler,
        priority: int = 0,
        once: bool = False,
    ) -> Callable[[], None]:
        """Suscribe un handler a un topic.
        
        Args:
            topic: Nombre del topic (puede usar wildcards: "user.*")
            handler: Función o coroutine a ejecutar
            priority: Prioridad (mayor = primero)
            once: Si True, se desuscribe después de la primera ejecución
        
        Returns:
            Función para desuscribirse
        """
        info = HandlerInfo(handler=handler, priority=priority, once=once)
        
        with self._lock:
            if topic not in self._handlers:
                self._handlers[topic] = []
            self._handlers[topic].append(info)
            # Ordenar por prioridad (mayor primero)
            self._handlers[topic].sort(key=lambda h: h.priority, reverse=True)
        
        if self._log_events:
            logger.debug(f"Handler subscribed to '{topic}' (priority={priority}, once={once})")
        
        # Retorna función de unsuscribe
        def unsubscribe() -> None:
            self._unsubscribe(topic, info)
        
        return unsubscribe

    def _unsubscribe(self, topic: str, info: HandlerInfo) -> None:
        """Desuscribe un handler específico."""
        with self._lock:
            if topic in self._handlers:
                try:
                    self._handlers[topic].remove(info)
                except ValueError:
                    pass

    def unsubscribe(self, topic: str, handler: Handler) -> bool:
        """Desuscribe un handler por referencia.
        
        Returns:
            True si se encontró y removió el handler
        """
        with self._lock:
            if topic not in self._handlers:
                return False
            
            for info in self._handlers[topic][:]:
                if info.handler is handler:
                    self._handlers[topic].remove(info)
                    return True
        return False

    def _get_matching_handlers(self, topic: str) -> List[HandlerInfo]:
        """Obtiene handlers que coinciden con el topic (soporta wildcards)."""
        handlers = []
        
        with self._lock:
            # Handlers exactos
            if topic in self._handlers:
                handlers.extend(self._handlers[topic])
            
            # Handlers con wildcard (ej: "user.*" matchea "user.created")
            parts = topic.split(".")
            for registered_topic, topic_handlers in self._handlers.items():
                if registered_topic == topic:
                    continue
                
                registered_parts = registered_topic.split(".")
                if len(registered_parts) != len(parts):
                    if registered_parts[-1] == "*" and len(registered_parts) <= len(parts):
                        # Wildcard al final: "user.*" matchea "user.created.admin"
                        if parts[:len(registered_parts)-1] == registered_parts[:-1]:
                            handlers.extend(topic_handlers)
                    continue
                
                match = True
                for rp, p in zip(registered_parts, parts):
                    if rp != "*" and rp != p:
                        match = False
                        break
                
                if match:
                    handlers.extend(topic_handlers)
        
        # Filtrar handlers desactivados y ordenar por prioridad
        handlers = [h for h in handlers if id(h.handler) not in self._disabled_handlers]
        handlers.sort(key=lambda h: h.priority, reverse=True)
        
        return handlers

    def _execute_handler(
        self,
        info: HandlerInfo,
        args: tuple,
        kwargs: dict,
    ) -> tuple[bool, float, Optional[str]]:
        """Ejecuta un handler síncrono.
        
        Returns:
            (success, duration_ms, error_message)
        """
        start = time.perf_counter()
        try:
            info.handler(*args, **kwargs)
            duration_ms = (time.perf_counter() - start) * 1000
            info.call_count += 1
            info.total_time_ms += duration_ms
            return (True, duration_ms, None)
        except Exception as e:
            duration_ms = (time.perf_counter() - start) * 1000
            info.error_count += 1
            info.last_error = f"{type(e).__name__}: {str(e)[:200]}"
            
            # Desactivar handler si supera el límite de errores
            if info.error_count >= self._max_handler_errors:
                self._disabled_handlers.add(id(info.handler))
                logger.warning(
                    f"Handler disabled after {info.error_count} errors: {info.last_error}"
                )
            else:
                logger.warning(f"Handler error: {info.last_error}")
            
            return (False, duration_ms, info.last_error)

    async def _execute_handler_async(
        self,
        info: HandlerInfo,
        args: tuple,
        kwargs: dict,
    ) -> tuple[bool, float, Optional[str]]:
        """Ejecuta un handler (síncrono o asíncrono).
        
        Returns:
            (success, duration_ms, error_message)
        """
        start = time.perf_counter()
        try:
            if asyncio.iscoroutinefunction(info.handler):
                await info.handler(*args, **kwargs)
            else:
                # Ejecutar síncrono en thread pool para no bloquear
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(None, lambda: info.handler(*args, **kwargs))
            
            duration_ms = (time.perf_counter() - start) * 1000
            info.call_count += 1
            info.total_time_ms += duration_ms
            return (True, duration_ms, None)
        except Exception as e:
            duration_ms = (time.perf_counter() - start) * 1000
            info.error_count += 1
            info.last_error = f"{type(e).__name__}: {str(e)[:200]}"
            
            if info.error_count >= self._max_handler_errors:
                self._disabled_handlers.add(id(info.handler))
                logger.warning(
                    f"Handler disabled after {info.error_count} errors: {info.last_error}"
                )
            else:
                logger.warning(f"Handler error: {info.last_error}")
            
            return (False, duration_ms, info.last_error)

    def publish(self, topic: str, *args: Any, **kwargs: Any) -> Dict[str, Any]:
        """Publica un evento de forma síncrona.
        
        Returns:
            Dict con información de la publicación
        """
        start = time.perf_counter()
        handlers = self._get_matching_handlers(topic)
        
        handlers_called = 0
        handlers_failed = 0
        to_remove: List[tuple[str, HandlerInfo]] = []
        
        for info in handlers:
            # Si es async, no podemos ejecutarlo síncronamente de forma simple
            if asyncio.iscoroutinefunction(info.handler):
                try:
                    # Intentar obtener loop existente
                    loop = asyncio.get_event_loop()
                    if loop.is_running():
                        # Programar la coroutine sin esperar
                        asyncio.ensure_future(info.handler(*args, **kwargs))
                        handlers_called += 1
                    else:
                        loop.run_until_complete(info.handler(*args, **kwargs))
                        handlers_called += 1
                except RuntimeError:
                    # No hay event loop, crear uno temporal
                    asyncio.run(info.handler(*args, **kwargs))
                    handlers_called += 1
                except Exception as e:
                    handlers_failed += 1
                    info.error_count += 1
                    info.last_error = str(e)[:200]
            else:
                success, _, _ = self._execute_handler(info, args, kwargs)
                handlers_called += 1
                if not success:
                    handlers_failed += 1
            
            if info.once:
                to_remove.append((topic, info))
        
        # Remover handlers "once"
        for t, info in to_remove:
            self._unsubscribe(t, info)
        
        duration_ms = (time.perf_counter() - start) * 1000
        self._event_count += 1
        self._total_publish_time_ms += duration_ms
        
        # Registrar en historial
        args_preview = str(args)[:100] if args else ""
        record = EventRecord(
            topic=topic,
            timestamp=datetime.now(timezone.utc).isoformat(),
            args_preview=args_preview,
            handlers_called=handlers_called,
            handlers_failed=handlers_failed,
            duration_ms=duration_ms,
        )
        self._history.append(record)
        
        if self._log_events:
            logger.debug(
                f"Event '{topic}': {handlers_called} handlers, "
                f"{handlers_failed} failed, {duration_ms:.2f}ms"
            )
        
        return {
            "ok": handlers_failed == 0,
            "topic": topic,
            "handlers_called": handlers_called,
            "handlers_failed": handlers_failed,
            "duration_ms": duration_ms,
        }

    async def publish_async(self, topic: str, *args: Any, **kwargs: Any) -> Dict[str, Any]:
        """Publica un evento de forma asíncrona (soporta handlers async).
        
        Returns:
            Dict con información de la publicación
        """
        start = time.perf_counter()
        handlers = self._get_matching_handlers(topic)
        
        handlers_called = 0
        handlers_failed = 0
        to_remove: List[tuple[str, HandlerInfo]] = []
        
        for info in handlers:
            success, _, _ = await self._execute_handler_async(info, args, kwargs)
            handlers_called += 1
            if not success:
                handlers_failed += 1
            
            if info.once:
                to_remove.append((topic, info))
        
        # Remover handlers "once"
        for t, info in to_remove:
            self._unsubscribe(t, info)
        
        duration_ms = (time.perf_counter() - start) * 1000
        self._event_count += 1
        self._total_publish_time_ms += duration_ms
        
        # Registrar en historial
        args_preview = str(args)[:100] if args else ""
        record = EventRecord(
            topic=topic,
            timestamp=datetime.now(timezone.utc).isoformat(),
            args_preview=args_preview,
            handlers_called=handlers_called,
            handlers_failed=handlers_failed,
            duration_ms=duration_ms,
        )
        self._history.append(record)
        
        if self._log_events:
            logger.debug(
                f"Event '{topic}': {handlers_called} handlers, "
                f"{handlers_failed} failed, {duration_ms:.2f}ms"
            )
        
        return {
            "ok": handlers_failed == 0,
            "topic": topic,
            "handlers_called": handlers_called,
            "handlers_failed": handlers_failed,
            "duration_ms": duration_ms,
        }

    def clear(self, topic: Optional[str] = None) -> None:
        """Limpia handlers de un topic o todos si topic es None."""
        with self._lock:
            if topic is None:
                self._handlers.clear()
                self._disabled_handlers.clear()
            else:
                self._handlers.pop(topic, None)

    def get_topics(self) -> List[str]:
        """Retorna lista de topics con handlers registrados."""
        with self._lock:
            return list(self._handlers.keys())

    def get_handler_count(self, topic: Optional[str] = None) -> int:
        """Retorna el número de handlers para un topic o total."""
        with self._lock:
            if topic:
                return len(self._handlers.get(topic, []))
            return sum(len(handlers) for handlers in self._handlers.values())

    def get_history(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Retorna el historial de eventos recientes."""
        with self._lock:
            records = list(self._history)[-limit:]
            return [
                {
                    "topic": r.topic,
                    "timestamp": r.timestamp,
                    "args_preview": r.args_preview,
                    "handlers_called": r.handlers_called,
                    "handlers_failed": r.handlers_failed,
                    "duration_ms": r.duration_ms,
                }
                for r in records
            ]

    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas del event bus."""
        with self._lock:
            handler_stats = []
            for topic, handlers in self._handlers.items():
                for info in handlers:
                    handler_stats.append({
                        "topic": topic,
                        "priority": info.priority,
                        "once": info.once,
                        "call_count": info.call_count,
                        "error_count": info.error_count,
                        "avg_time_ms": info.total_time_ms / max(1, info.call_count),
                        "disabled": id(info.handler) in self._disabled_handlers,
                        "last_error": info.last_error[:100] if info.last_error else None,
                    })
            
            return {
                "total_topics": len(self._handlers),
                "total_handlers": self.get_handler_count(),
                "total_events": self._event_count,
                "total_publish_time_ms": self._total_publish_time_ms,
                "avg_publish_time_ms": self._total_publish_time_ms / max(1, self._event_count),
                "disabled_handlers": len(self._disabled_handlers),
                "handlers": handler_stats,
            }

    def reset_handler_errors(self, topic: Optional[str] = None) -> int:
        """Resetea contadores de errores y rehabilita handlers desactivados.
        
        Returns:
            Número de handlers rehabilitados
        """
        rehabilitated = 0
        with self._lock:
            if topic:
                topics = [topic] if topic in self._handlers else []
            else:
                topics = list(self._handlers.keys())
            
            for t in topics:
                for info in self._handlers.get(t, []):
                    if id(info.handler) in self._disabled_handlers:
                        self._disabled_handlers.remove(id(info.handler))
                        rehabilitated += 1
                    info.error_count = 0
                    info.last_error = ""
        
        return rehabilitated
