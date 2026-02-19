"""
POT Dispatcher: Motor de Ejecución Autónoma de POTs.
=====================================================
Este módulo es el CEREBRO que hace que ATLAS ejecute POTs automáticamente.

Responsabilidades:
1. Escuchar eventos del sistema (ANS, Scheduler, Triggers)
2. Seleccionar el POT correcto para cada situación
3. Ejecutar POTs de forma autónoma
4. Reportar resultados a todos los canales

ESTE ES EL COMPONENTE QUE FALTABA PARA CONFIABILIDAD 100%
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from queue import Queue, Empty
from typing import Any, Callable, Dict, List, Optional, Set

_log = logging.getLogger("humanoid.quality.dispatcher")

# ============================================================================
# CONFIGURACIÓN
# ============================================================================

REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent


class TriggerType(str, Enum):
    """Tipos de trigger que disparan un POT."""
    INCIDENT = "incident"           # Incidente detectado por ANS
    SCHEDULED = "scheduled"         # Job programado en scheduler
    MANUAL = "manual"               # Invocación manual
    EVENT = "event"                 # Evento del sistema (git, api, etc)
    CONDITION = "condition"         # Condición monitoreada (watcher)
    STARTUP = "startup"             # Inicio de sesión ATLAS
    SHUTDOWN = "shutdown"           # Cierre de sesión ATLAS
    WEBHOOK = "webhook"             # Webhook externo


@dataclass
class DispatchRequest:
    """Solicitud de ejecución de POT."""
    trigger_type: TriggerType
    trigger_id: str                  # ID único del trigger
    pot_id: Optional[str] = None     # POT específico o None para auto-selección
    context: Dict[str, Any] = field(default_factory=dict)
    priority: int = 5               # 1-10, donde 1 es más alta
    timestamp: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    source: str = "unknown"         # Origen del request (ans, scheduler, api, etc)
    require_approval: bool = False  # Si requiere aprobación antes de ejecutar
    dry_run: bool = False           # Simular sin ejecutar


@dataclass
class DispatchResult:
    """Resultado de una ejecución de POT."""
    request_id: str
    pot_id: str
    ok: bool
    started_at: str
    ended_at: str
    elapsed_ms: int
    steps_ok: int
    steps_total: int
    error: Optional[str] = None
    report_path: Optional[str] = None
    notified_channels: List[str] = field(default_factory=list)


# ============================================================================
# POT DISPATCHER
# ============================================================================

class POTDispatcher:
    """
    Motor de ejecución autónoma de POTs.
    
    Este es el componente que conecta TODOS los sistemas con los POTs:
    - ANS → Dispatcher → POT → Ejecución → Reportes
    - Scheduler → Dispatcher → POT → Ejecución → Reportes
    - Eventos → Dispatcher → POT → Ejecución → Reportes
    """
    
    # Cooldown mínimo entre ejecuciones del mismo POT tras fallo (segundos)
    _POT_COOLDOWN_ON_FAIL: Dict[str, int] = {
        "services_repair": 300,   # 5 min: evita bucle reactor infinito
        "camera_repair":   180,
        "api_repair":      120,
        "diagnostic_full": 600,
    }
    _DEFAULT_COOLDOWN = 60  # 1 min por defecto para cualquier POT que falle

    def __init__(self):
        self._queue: Queue[DispatchRequest] = Queue()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None
        self._hooks: Dict[str, List[Callable]] = {
            "pre_dispatch": [],
            "post_dispatch": [],
            "on_error": [],
        }
        self._active_executions: Dict[str, DispatchRequest] = {}
        self._execution_history: List[DispatchResult] = []
        self._max_history = 100
        self._lock = threading.Lock()

        # Circuit breaker: {pot_id: {"last_fail_ts": float, "consecutive_fails": int}}
        self._circuit_breaker: Dict[str, Dict] = {}

        # Estadísticas
        self._stats = {
            "total_dispatched": 0,
            "successful": 0,
            "failed": 0,
            "pending": 0,
            "skipped_cooldown": 0,
            "by_trigger_type": {},
            "by_pot_id": {},
        }
    
    # ========================================================================
    # LIFECYCLE
    # ========================================================================
    
    def start(self) -> None:
        """Inicia el dispatcher en un thread separado."""
        if self._running:
            _log.warning("Dispatcher already running")
            return
        
        self._running = True
        self._worker_thread = threading.Thread(
            target=self._worker_loop,
            name="POTDispatcher-Worker",
            daemon=True,
        )
        self._worker_thread.start()
        _log.info("POT Dispatcher started")
        
        # Ejecutar POT de inicio de sesión
        self.dispatch(DispatchRequest(
            trigger_type=TriggerType.STARTUP,
            trigger_id="dispatcher_start",
            pot_id="session_startup",
            source="dispatcher",
            priority=2,
        ))
    
    def stop(self, graceful: bool = True) -> None:
        """Detiene el dispatcher."""
        if not self._running:
            return
        
        # Ejecutar POT de cierre si es graceful
        if graceful:
            self.dispatch(DispatchRequest(
                trigger_type=TriggerType.SHUTDOWN,
                trigger_id="dispatcher_stop",
                pot_id="session_shutdown",
                source="dispatcher",
                priority=1,
            ))
            # Esperar a que se procese
            time.sleep(2)
        
        self._running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=5)
        _log.info("POT Dispatcher stopped")
    
    def is_running(self) -> bool:
        """Verifica si el dispatcher está corriendo."""
        return self._running
    
    # ========================================================================
    # DISPATCH
    # ========================================================================
    
    def _circuit_open(self, pot_id: str) -> bool:
        """
        Devuelve True si el circuit breaker está abierto para este POT
        (es decir, está en cooldown tras fallos consecutivos).
        """
        if not pot_id:
            return False
        with self._lock:
            cb = self._circuit_breaker.get(pot_id)
            if not cb:
                return False
            cooldown = self._POT_COOLDOWN_ON_FAIL.get(pot_id, self._DEFAULT_COOLDOWN)
            # Escalar cooldown con fallos consecutivos (máx 10 min)
            scaled = min(cooldown * max(1, cb.get("consecutive_fails", 1)), 600)
            elapsed = time.time() - cb.get("last_fail_ts", 0)
            if elapsed < scaled:
                _log.warning(
                    "Circuit breaker OPEN for POT %s (%d consecutive fails, %.0fs cooldown, %.0fs remaining)",
                    pot_id, cb.get("consecutive_fails", 1), scaled, scaled - elapsed
                )
                return True
            return False

    def _circuit_record_result(self, pot_id: str, ok: bool) -> None:
        """Actualiza el circuit breaker tras una ejecución."""
        if not pot_id:
            return
        with self._lock:
            if ok:
                self._circuit_breaker.pop(pot_id, None)
            else:
                cb = self._circuit_breaker.setdefault(pot_id, {"consecutive_fails": 0, "last_fail_ts": 0})
                cb["consecutive_fails"] += 1
                cb["last_fail_ts"] = time.time()

    def dispatch(self, request: DispatchRequest) -> str:
        """
        Encola un request para ejecución.
        
        Args:
            request: Solicitud de ejecución
        
        Returns:
            ID del request encolado
        """
        request_id = f"{request.trigger_type.value}_{request.trigger_id}_{int(time.time()*1000)}"
        request.context["request_id"] = request_id

        # Circuit breaker: bloquear si el POT está en cooldown por fallos
        pot_id = request.pot_id or ""
        if pot_id and self._circuit_open(pot_id):
            with self._lock:
                self._stats["skipped_cooldown"] = self._stats.get("skipped_cooldown", 0) + 1
            _log.info("Skipped dispatch of POT %s (circuit breaker open)", pot_id)
            return request_id  # Devuelve ID sin encolar
        
        with self._lock:
            self._stats["pending"] += 1
            self._stats["by_trigger_type"].setdefault(request.trigger_type.value, 0)
            self._stats["by_trigger_type"][request.trigger_type.value] += 1
        
        self._queue.put(request)
        _log.info(
            "Dispatched request %s: trigger=%s, pot=%s, priority=%d",
            request_id, request.trigger_type.value, request.pot_id or "auto", request.priority
        )
        return request_id
    
    def dispatch_incident(
        self,
        check_id: str,
        message: str,
        severity: str = "medium",
        context: Optional[Dict[str, Any]] = None,
    ) -> str:
        """
        Shortcut para despachar un incidente.
        
        El POT se selecciona automáticamente basado en check_id.
        """
        return self.dispatch(DispatchRequest(
            trigger_type=TriggerType.INCIDENT,
            trigger_id=check_id,
            pot_id=None,  # Auto-selección
            context={
                "check_id": check_id,
                "message": message,
                "severity": severity,
                **(context or {}),
            },
            priority=3 if severity in ("high", "critical") else 5,
            source="ans",
            require_approval=severity == "critical",
        ))
    
    def dispatch_scheduled(
        self,
        job_id: str,
        pot_id: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> str:
        """Shortcut para despachar un job programado."""
        return self.dispatch(DispatchRequest(
            trigger_type=TriggerType.SCHEDULED,
            trigger_id=job_id,
            pot_id=pot_id,
            context=context or {},
            priority=6,
            source="scheduler",
        ))
    
    def dispatch_event(
        self,
        event_type: str,
        pot_id: Optional[str] = None,
        context: Optional[Dict[str, Any]] = None,
    ) -> str:
        """Shortcut para despachar un evento del sistema."""
        return self.dispatch(DispatchRequest(
            trigger_type=TriggerType.EVENT,
            trigger_id=event_type,
            pot_id=pot_id,
            context={"event_type": event_type, **(context or {})},
            priority=5,
            source="event_bus",
        ))
    
    # ========================================================================
    # WORKER
    # ========================================================================
    
    def _worker_loop(self) -> None:
        """Loop principal del worker que procesa la cola (TURBO: 100ms polling)."""
        while self._running:
            try:
                # TURBO: Polling cada 100ms en lugar de 1s
                request = self._queue.get(timeout=0.1)
            except Empty:
                continue
            
            try:
                self._process_request(request)
            except Exception as e:
                _log.exception("Error processing dispatch request: %s", e)
                self._call_hooks("on_error", request, e)
            finally:
                with self._lock:
                    self._stats["pending"] = max(0, self._stats["pending"] - 1)
    
    def _process_request(self, request: DispatchRequest) -> None:
        """Procesa una solicitud de dispatch."""
        request_id = request.context.get("request_id", "unknown")
        
        # Hook pre-dispatch
        self._call_hooks("pre_dispatch", request)
        
        # Seleccionar POT
        pot = self._select_pot(request)
        if not pot:
            _log.warning("No POT found for request %s", request_id)
            self._record_result(DispatchResult(
                request_id=request_id,
                pot_id="none",
                ok=False,
                started_at=datetime.now(timezone.utc).isoformat(),
                ended_at=datetime.now(timezone.utc).isoformat(),
                elapsed_ms=0,
                steps_ok=0,
                steps_total=0,
                error="No POT found for trigger",
            ))
            return
        
        # Verificar aprobación si es necesario
        if request.require_approval and not request.dry_run:
            approved = self._request_approval(request, pot)
            if not approved:
                _log.info("POT %s execution not approved for request %s", pot.id, request_id)
                self._record_result(DispatchResult(
                    request_id=request_id,
                    pot_id=pot.id,
                    ok=False,
                    started_at=datetime.now(timezone.utc).isoformat(),
                    ended_at=datetime.now(timezone.utc).isoformat(),
                    elapsed_ms=0,
                    steps_ok=0,
                    steps_total=0,
                    error="Approval denied or timeout",
                ))
                return
        
        # Marcar como activo
        with self._lock:
            self._active_executions[request_id] = request
        
        # Ejecutar POT
        t0 = time.time()
        started_at = datetime.now(timezone.utc).isoformat()
        
        try:
            from .executor import execute_pot
            
            result = execute_pot(
                pot=pot,
                context=request.context,
                dry_run=request.dry_run,
                stop_on_failure=True,
                sync_to_cerebro=True,
                notify_on_complete=True,
            )
            
            elapsed_ms = int((time.time() - t0) * 1000)
            ended_at = datetime.now(timezone.utc).isoformat()
            
            # Registrar resultado
            dispatch_result = DispatchResult(
                request_id=request_id,
                pot_id=pot.id,
                ok=result.ok,
                started_at=started_at,
                ended_at=ended_at,
                elapsed_ms=elapsed_ms,
                steps_ok=result.steps_ok,
                steps_total=result.steps_total,
                report_path=result.report_path,
                notified_channels=["cerebro", "telegram"] if not request.dry_run else [],
            )
            
            self._record_result(dispatch_result)

            # Circuit breaker: registrar resultado
            self._circuit_record_result(pot.id, result.ok)
            
            # Actualizar estadísticas
            with self._lock:
                self._stats["total_dispatched"] += 1
                if result.ok:
                    self._stats["successful"] += 1
                else:
                    self._stats["failed"] += 1
                self._stats["by_pot_id"].setdefault(pot.id, {"ok": 0, "fail": 0})
                if result.ok:
                    self._stats["by_pot_id"][pot.id]["ok"] += 1
                else:
                    self._stats["by_pot_id"][pot.id]["fail"] += 1
            
            _log.info(
                "POT %s executed: ok=%s, steps=%d/%d, elapsed=%dms",
                pot.id, result.ok, result.steps_ok, result.steps_total, elapsed_ms
            )
            
        except Exception as e:
            _log.exception("POT execution failed: %s", e)
            elapsed_ms = int((time.time() - t0) * 1000)
            self._record_result(DispatchResult(
                request_id=request_id,
                pot_id=pot.id,
                ok=False,
                started_at=started_at,
                ended_at=datetime.now(timezone.utc).isoformat(),
                elapsed_ms=elapsed_ms,
                steps_ok=0,
                steps_total=len(pot.steps),
                error=str(e),
            ))
            with self._lock:
                self._stats["total_dispatched"] += 1
                self._stats["failed"] += 1
        
        finally:
            with self._lock:
                self._active_executions.pop(request_id, None)
        
        # Hook post-dispatch
        self._call_hooks("post_dispatch", request, dispatch_result if 'dispatch_result' in locals() else None)
    
    def _select_pot(self, request: DispatchRequest):
        """Selecciona el POT apropiado para un request."""
        from .registry import get_pot, get_pot_by_incident, get_pot_for_maintenance
        
        # Si se especificó pot_id, usarlo directamente
        if request.pot_id:
            return get_pot(request.pot_id)
        
        # Auto-selección basada en tipo de trigger
        if request.trigger_type == TriggerType.INCIDENT:
            return get_pot_by_incident(
                check_id=request.context.get("check_id", request.trigger_id),
                message=request.context.get("message", ""),
                severity=request.context.get("severity"),
            )
        
        if request.trigger_type == TriggerType.SCHEDULED:
            # Intentar inferir del trigger_id
            tid = request.trigger_id.lower()
            if "daily" in tid:
                return get_pot_for_maintenance("daily")
            elif "weekly" in tid:
                return get_pot_for_maintenance("weekly")
            elif "autonomy" in tid or "cycle" in tid:
                return get_pot("autonomy_full_cycle")
            elif "update" in tid or "evolution" in tid:
                return get_pot("auto_update_full")
        
        if request.trigger_type == TriggerType.STARTUP:
            return get_pot("session_startup")
        
        if request.trigger_type == TriggerType.SHUTDOWN:
            return get_pot("session_shutdown")
        
        if request.trigger_type == TriggerType.EVENT:
            # Usar sync_engine para mapear evento a POT
            from .sync_engine import OPERATION_POT_MAP
            event_type = request.context.get("event_type", request.trigger_id)
            pot_id = OPERATION_POT_MAP.get(event_type.lower())
            if pot_id:
                return get_pot(pot_id)
        
        # Fallback: diagnostic
        return get_pot("diagnostic_full")
    
    def _request_approval(self, request: DispatchRequest, pot) -> bool:
        """Solicita aprobación para ejecutar un POT crítico.
        
        Usa el sistema de aprobaciones de ATLAS que envía botones inline
        a Telegram y espera la respuesta del usuario.
        """
        try:
            from modules.humanoid.approvals import create_approval, wait_for_resolution
            
            # Crear aprobación
            approval_result = create_approval(
                action=f"pot_execute:{pot.id}",
                risk="high" if pot.severity.value in ("high", "critical") else "medium",
                description=f"POT: {pot.id} - {pot.name}\nTrigger: {request.trigger_type.value}",
                payload={"pot_id": pot.id, "trigger": request.trigger_type.value},
            )
            
            if not approval_result.get("ok"):
                _log.warning("Could not create approval: %s", approval_result.get("error"))
                return False
            
            approval_id = approval_result.get("id")
            if not approval_id:
                return False
            
            # Esperar resolución (máximo 5 minutos)
            resolution = wait_for_resolution(approval_id, timeout_seconds=300)
            
            return resolution.get("status") == "approved"
            
        except ImportError:
            # Fallback: usar Telegram inline buttons directamente
            try:
                from modules.humanoid.comms.telegram_bridge import TelegramBridge
                from modules.humanoid.comms.ops_bus import _telegram_chat_id
                
                chat_id = _telegram_chat_id()
                if not chat_id:
                    _log.warning("No chat_id for approval request")
                    return False
                
                bridge = TelegramBridge()
                import uuid
                approval_id = str(uuid.uuid4())[:8]
                
                result = bridge.send_approval_inline(
                    chat_id=chat_id,
                    approval_id=approval_id,
                    action=f"pot_execute:{pot.id}",
                    risk=pot.severity.value,
                )
                
                if not result.get("ok"):
                    _log.warning("Could not send approval request: %s", result.get("error"))
                    return False
                
                # Nota: Este fallback no espera respuesta, asume denegado
                # Para esperar respuesta se necesita el sistema completo de aprobaciones
                _log.info("Approval request sent to Telegram. Auto-denying for safety.")
                return False
                
            except Exception as e:
                _log.warning("Fallback approval failed: %s", e)
                return False
            
        except Exception as e:
            _log.warning("Could not request approval: %s. Auto-denying.", e)
            return False
    
    # ========================================================================
    # HOOKS
    # ========================================================================
    
    def register_hook(self, event: str, callback: Callable) -> None:
        """Registra un hook para eventos del dispatcher."""
        if event in self._hooks:
            self._hooks[event].append(callback)
    
    def _call_hooks(self, event: str, *args) -> None:
        """Ejecuta todos los hooks registrados para un evento."""
        for hook in self._hooks.get(event, []):
            try:
                hook(*args)
            except Exception as e:
                _log.warning("Hook %s failed: %s", event, e)
    
    # ========================================================================
    # HISTORY & STATS
    # ========================================================================
    
    def _record_result(self, result: DispatchResult) -> None:
        """Guarda resultado en historial."""
        with self._lock:
            self._execution_history.append(result)
            if len(self._execution_history) > self._max_history:
                self._execution_history = self._execution_history[-self._max_history:]
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadísticas del dispatcher."""
        with self._lock:
            return {
                **self._stats,
                "active_executions": len(self._active_executions),
                "queue_size": self._queue.qsize(),
                "running": self._running,
            }
    
    def get_history(self, limit: int = 20) -> List[Dict[str, Any]]:
        """Obtiene historial de ejecuciones."""
        with self._lock:
            results = self._execution_history[-limit:]
            return [
                {
                    "request_id": r.request_id,
                    "pot_id": r.pot_id,
                    "ok": r.ok,
                    "started_at": r.started_at,
                    "elapsed_ms": r.elapsed_ms,
                    "steps": f"{r.steps_ok}/{r.steps_total}",
                    "error": r.error,
                }
                for r in reversed(results)
            ]


# ============================================================================
# INSTANCIA GLOBAL
# ============================================================================

_dispatcher: Optional[POTDispatcher] = None
_dispatcher_lock = threading.Lock()


def get_dispatcher() -> POTDispatcher:
    """Obtiene la instancia global del dispatcher."""
    global _dispatcher
    with _dispatcher_lock:
        if _dispatcher is None:
            _dispatcher = POTDispatcher()
        return _dispatcher


def start_dispatcher() -> POTDispatcher:
    """Inicia el dispatcher global."""
    dispatcher = get_dispatcher()
    if not dispatcher.is_running():
        dispatcher.start()
    return dispatcher


def stop_dispatcher(graceful: bool = True) -> None:
    """Detiene el dispatcher global."""
    global _dispatcher
    with _dispatcher_lock:
        if _dispatcher:
            _dispatcher.stop(graceful=graceful)


# ============================================================================
# FUNCIONES DE CONVENIENCIA
# ============================================================================

def dispatch_pot(
    pot_id: str,
    context: Optional[Dict[str, Any]] = None,
    priority: int = 5,
    dry_run: bool = False,
) -> str:
    """
    Despacha un POT específico para ejecución.
    
    Args:
        pot_id: ID del POT a ejecutar
        context: Contexto para la ejecución
        priority: Prioridad (1-10, menor es más urgente)
        dry_run: Simular sin ejecutar
    
    Returns:
        ID del request
    """
    dispatcher = get_dispatcher()
    return dispatcher.dispatch(DispatchRequest(
        trigger_type=TriggerType.MANUAL,
        trigger_id=f"manual_{pot_id}",
        pot_id=pot_id,
        context=context or {},
        priority=priority,
        source="api",
        dry_run=dry_run,
    ))


def dispatch_incident(
    check_id: str,
    message: str,
    severity: str = "medium",
    context: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Despacha un incidente para procesamiento automático.
    
    El POT se selecciona automáticamente basado en check_id.
    """
    dispatcher = get_dispatcher()
    return dispatcher.dispatch_incident(check_id, message, severity, context)


def dispatch_event(
    event_type: str,
    pot_id: Optional[str] = None,
    context: Optional[Dict[str, Any]] = None,
) -> str:
    """Despacha un evento del sistema."""
    dispatcher = get_dispatcher()
    return dispatcher.dispatch_event(event_type, pot_id, context)


def get_dispatch_stats() -> Dict[str, Any]:
    """Obtiene estadísticas del dispatcher."""
    return get_dispatcher().get_stats()


def get_dispatch_history(limit: int = 20) -> List[Dict[str, Any]]:
    """Obtiene historial de ejecuciones."""
    return get_dispatcher().get_history(limit)


# ============================================================================
# EXPORTACIONES
# ============================================================================

__all__ = [
    "TriggerType",
    "DispatchRequest",
    "DispatchResult",
    "POTDispatcher",
    "get_dispatcher",
    "start_dispatcher",
    "stop_dispatcher",
    "dispatch_pot",
    "dispatch_incident",
    "dispatch_event",
    "get_dispatch_stats",
    "get_dispatch_history",
]
