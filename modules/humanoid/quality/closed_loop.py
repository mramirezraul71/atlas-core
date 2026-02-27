"""
ATLAS Closed Loop System
=========================
Sistema de ciclo cerrado: Detectar → Ejecutar → Verificar → Reportar

Este módulo implementa el ciclo completo de autonomía:
1. DETECTAR: Identificar problemas o condiciones
2. EJECUTAR: Correr el POT apropiado
3. VERIFICAR: Confirmar que se resolvió
4. REPORTAR: Notificar resultado

Sin este ciclo, la autonomía está incompleta.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

_log = logging.getLogger("humanoid.quality.closed_loop")


# ============================================================================
# TIPOS
# ============================================================================

class LoopPhase(str, Enum):
    """Fases del ciclo cerrado."""
    DETECT = "detect"
    EXECUTE = "execute"
    VERIFY = "verify"
    REPORT = "report"
    COMPLETE = "complete"
    FAILED = "failed"


class LoopOutcome(str, Enum):
    """Resultado del ciclo."""
    SUCCESS = "success"           # Problema detectado y resuelto
    PARTIAL = "partial"           # Parcialmente resuelto
    FAILED = "failed"             # No se pudo resolver
    NO_ACTION = "no_action"       # Nada que hacer
    ESCALATED = "escalated"       # Escalado a humano


@dataclass
class LoopContext:
    """Contexto de un ciclo cerrado."""
    id: str
    trigger: str
    pot_id: str
    phase: LoopPhase = LoopPhase.DETECT
    outcome: Optional[LoopOutcome] = None
    started_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    ended_at: Optional[str] = None
    
    # Datos de cada fase
    detection_data: Dict[str, Any] = field(default_factory=dict)
    execution_data: Dict[str, Any] = field(default_factory=dict)
    verification_data: Dict[str, Any] = field(default_factory=dict)
    
    # Métricas
    attempts: int = 0
    max_attempts: int = 3


# ============================================================================
# CLOSED LOOP ENGINE
# ============================================================================

class ClosedLoopEngine:
    """
    Motor de ciclos cerrados.
    
    Gestiona la ejecución completa de ciclos:
    Detectar problema → Ejecutar POT → Verificar solución → Reportar
    """
    
    def __init__(self):
        self._active_loops: Dict[str, LoopContext] = {}
        self._completed_loops: List[LoopContext] = []
        self._max_history = 100
        self._lock = threading.Lock()
        
        # Verificadores por POT
        self._verifiers: Dict[str, Callable[[LoopContext], bool]] = {}
        self._setup_default_verifiers()
    
    def _setup_default_verifiers(self) -> None:
        """Configura verificadores por defecto."""
        
        # Verificador para reparación de servicios
        def verify_services(ctx: LoopContext) -> bool:
            try:
                import urllib.request
                services = ctx.detection_data.get("services", ["atlas_api"])
                for svc in services:
                    if svc == "atlas_api":
                        req = urllib.request.Request("http://127.0.0.1:8791/health")
                        with urllib.request.urlopen(req, timeout=5) as r:
                            if r.status != 200:
                                return False
                return True
            except:
                return False
        
        # Verificador para git operations
        def verify_git(ctx: LoopContext) -> bool:
            try:
                import subprocess
                from pathlib import Path
                repo = Path(__file__).resolve().parent.parent.parent.parent
                
                result = subprocess.run(
                    ["git", "status", "--porcelain"],
                    capture_output=True,
                    timeout=10,
                    cwd=str(repo),
                )
                
                # Si era commit, verificar que no hay cambios pendientes
                if ctx.pot_id in ("git_commit", "repo_update"):
                    return result.returncode == 0 and len(result.stdout.strip()) == 0
                
                # Para pull/push, solo verificar que git funciona
                return result.returncode == 0
            except:
                return False
        
        # Verificador para API
        def verify_api(ctx: LoopContext) -> bool:
            try:
                import urllib.request
                req = urllib.request.Request("http://127.0.0.1:8791/health")
                with urllib.request.urlopen(req, timeout=5) as r:
                    return r.status == 200
            except:
                return False
        
        # Verificador genérico (asume éxito si POT terminó OK)
        def verify_generic(ctx: LoopContext) -> bool:
            return ctx.execution_data.get("ok", False)
        
        # Registrar verificadores
        self._verifiers["services_repair"] = verify_services
        self._verifiers["api_repair"] = verify_api
        self._verifiers["git_commit"] = verify_git
        self._verifiers["git_push"] = verify_git
        self._verifiers["git_pull"] = verify_git
        self._verifiers["repo_update"] = verify_git
        self._verifiers["_default"] = verify_generic
    
    def register_verifier(self, pot_id: str, verifier: Callable[[LoopContext], bool]) -> None:
        """Registra un verificador personalizado."""
        self._verifiers[pot_id] = verifier
    
    def start_loop(
        self,
        trigger: str,
        pot_id: str,
        detection_data: Optional[Dict[str, Any]] = None,
    ) -> LoopContext:
        """
        Inicia un nuevo ciclo cerrado.
        
        Args:
            trigger: Qué disparó el ciclo
            pot_id: POT a ejecutar
            detection_data: Datos de la detección
        
        Returns:
            Contexto del ciclo
        """
        loop_id = f"loop_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{pot_id}"
        
        ctx = LoopContext(
            id=loop_id,
            trigger=trigger,
            pot_id=pot_id,
            detection_data=detection_data or {},
        )
        
        with self._lock:
            self._active_loops[loop_id] = ctx
        
        _log.info("[LOOP:%s] Started - trigger=%s, pot=%s", loop_id, trigger, pot_id)
        
        # Ejecutar el ciclo completo
        self._execute_loop(ctx)
        
        return ctx
    
    def _execute_loop(self, ctx: LoopContext) -> None:
        """Ejecuta el ciclo completo."""
        try:
            # Fase 1: DETECT (ya completada al llegar aquí)
            ctx.phase = LoopPhase.DETECT
            _log.info("[LOOP:%s] Phase: DETECT", ctx.id)
            
            # Fase 2: EXECUTE
            ctx.phase = LoopPhase.EXECUTE
            _log.info("[LOOP:%s] Phase: EXECUTE - Running POT %s", ctx.id, ctx.pot_id)
            
            success = self._execute_pot(ctx)
            
            if not success:
                # Reintentar si no excedemos intentos
                ctx.attempts += 1
                if ctx.attempts < ctx.max_attempts:
                    _log.warning("[LOOP:%s] POT failed, retrying (%d/%d)", 
                                ctx.id, ctx.attempts, ctx.max_attempts)
                    time.sleep(5)  # Esperar antes de reintentar
                    return self._execute_loop(ctx)
                else:
                    ctx.phase = LoopPhase.FAILED
                    ctx.outcome = LoopOutcome.FAILED
                    self._report(ctx)
                    return
            
            # Fase 3: VERIFY
            ctx.phase = LoopPhase.VERIFY
            _log.info("[LOOP:%s] Phase: VERIFY", ctx.id)
            
            verified = self._verify(ctx)
            
            if not verified:
                ctx.attempts += 1
                if ctx.attempts < ctx.max_attempts:
                    _log.warning("[LOOP:%s] Verification failed, retrying (%d/%d)",
                                ctx.id, ctx.attempts, ctx.max_attempts)
                    time.sleep(10)
                    ctx.phase = LoopPhase.EXECUTE
                    return self._execute_loop(ctx)
                else:
                    ctx.outcome = LoopOutcome.PARTIAL
            else:
                ctx.outcome = LoopOutcome.SUCCESS
            
            # Fase 4: REPORT
            ctx.phase = LoopPhase.REPORT
            _log.info("[LOOP:%s] Phase: REPORT", ctx.id)
            self._report(ctx)
            
            # Completar
            ctx.phase = LoopPhase.COMPLETE
            ctx.ended_at = datetime.now(timezone.utc).isoformat()
            
            _log.info("[LOOP:%s] Completed - outcome=%s", ctx.id, ctx.outcome.value)
            
        except Exception as e:
            _log.exception("[LOOP:%s] Error: %s", ctx.id, e)
            ctx.phase = LoopPhase.FAILED
            ctx.outcome = LoopOutcome.FAILED
            ctx.ended_at = datetime.now(timezone.utc).isoformat()
            self._report(ctx, error=str(e))
        
        finally:
            # Mover a completados
            with self._lock:
                if ctx.id in self._active_loops:
                    del self._active_loops[ctx.id]
                self._completed_loops.append(ctx)
                if len(self._completed_loops) > self._max_history:
                    self._completed_loops.pop(0)
    
    def _execute_pot(self, ctx: LoopContext) -> bool:
        """Ejecuta el POT."""
        try:
            from .executor import execute_pot
            
            result = execute_pot(
                pot_id=ctx.pot_id,
                context=ctx.detection_data,
            )
            
            ctx.execution_data = {
                "ok": result.ok,
                "steps_ok": result.steps_ok,
                "steps_total": result.steps_total,
                "elapsed_ms": result.elapsed_ms,
                "error": result.error,
            }
            
            return result.ok
            
        except Exception as e:
            ctx.execution_data = {"ok": False, "error": str(e)}
            return False
    
    def _verify(self, ctx: LoopContext) -> bool:
        """Verifica que el problema se resolvió."""
        verifier = self._verifiers.get(ctx.pot_id, self._verifiers["_default"])
        
        try:
            # Esperar un momento para que los cambios surtan efecto
            time.sleep(2)
            
            verified = verifier(ctx)
            ctx.verification_data = {
                "verified": verified,
                "timestamp": datetime.now(timezone.utc).isoformat(),
            }
            
            return verified
            
        except Exception as e:
            ctx.verification_data = {
                "verified": False,
                "error": str(e),
            }
            return False
    
    def _report(self, ctx: LoopContext, error: Optional[str] = None) -> None:
        """Reporta el resultado del ciclo."""
        try:
            from .cerebro_connector import get_bridge
            
            bridge = get_bridge()
            
            # Construir mensaje
            if ctx.outcome == LoopOutcome.SUCCESS:
                emoji = "✅"
                level = "success"
            elif ctx.outcome == LoopOutcome.PARTIAL:
                emoji = "⚠️"
                level = "warning"
            else:
                emoji = "❌"
                level = "error"
            
            message = f"{emoji} [CICLO CERRADO] {ctx.pot_id}\n"
            message += f"Trigger: {ctx.trigger}\n"
            message += f"Resultado: {ctx.outcome.value if ctx.outcome else 'unknown'}\n"
            message += f"Intentos: {ctx.attempts}/{ctx.max_attempts}"
            
            if error:
                message += f"\nError: {error}"
            
            # Enviar a OPS
            bridge.channels.send_ops(message, level)
            
            # Enviar a Telegram si falló
            if ctx.outcome in (LoopOutcome.FAILED, LoopOutcome.ESCALATED):
                bridge.channels.send_telegram_sync(message)
            
            # Registrar en cerebro
            bridge.cerebro._http_request("POST", "/ans/evolution-log", {
                "type": "closed_loop",
                "source": f"quality.loop.{ctx.pot_id}",
                "message": f"[LOOP] {ctx.pot_id}: {ctx.outcome.value if ctx.outcome else 'unknown'}",
                "ok": ctx.outcome == LoopOutcome.SUCCESS,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "metadata": {
                    "loop_id": ctx.id,
                    "trigger": ctx.trigger,
                    "attempts": ctx.attempts,
                    "execution_data": ctx.execution_data,
                    "verification_data": ctx.verification_data,
                },
            })
            
        except Exception as e:
            _log.warning("Failed to report loop result: %s", e)
    
    def get_active_loops(self) -> List[Dict[str, Any]]:
        """Retorna loops activos."""
        with self._lock:
            return [
                {
                    "id": ctx.id,
                    "pot_id": ctx.pot_id,
                    "phase": ctx.phase.value,
                    "trigger": ctx.trigger,
                    "attempts": ctx.attempts,
                }
                for ctx in self._active_loops.values()
            ]
    
    def get_loop_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """Retorna historial de loops."""
        with self._lock:
            return [
                {
                    "id": ctx.id,
                    "pot_id": ctx.pot_id,
                    "outcome": ctx.outcome.value if ctx.outcome else None,
                    "trigger": ctx.trigger,
                    "attempts": ctx.attempts,
                    "started_at": ctx.started_at,
                    "ended_at": ctx.ended_at,
                }
                for ctx in self._completed_loops[-limit:]
            ]
    
    def get_stats(self) -> Dict[str, Any]:
        """Retorna estadísticas."""
        with self._lock:
            total = len(self._completed_loops)
            success = sum(1 for c in self._completed_loops if c.outcome == LoopOutcome.SUCCESS)
            failed = sum(1 for c in self._completed_loops if c.outcome == LoopOutcome.FAILED)
            
            return {
                "total_loops": total,
                "successful": success,
                "failed": failed,
                "success_rate": (success / total * 100) if total > 0 else 0,
                "active": len(self._active_loops),
            }


# ============================================================================
# SINGLETON Y FUNCIONES
# ============================================================================

_engine: Optional[ClosedLoopEngine] = None


def get_closed_loop_engine() -> ClosedLoopEngine:
    """Obtiene la instancia del engine."""
    global _engine
    if _engine is None:
        _engine = ClosedLoopEngine()
    return _engine


def run_closed_loop(
    trigger: str,
    pot_id: str,
    detection_data: Optional[Dict[str, Any]] = None,
) -> LoopContext:
    """
    Ejecuta un ciclo cerrado completo.
    
    Esta es la función principal para autonomía:
    Detecta → Ejecuta → Verifica → Reporta
    """
    engine = get_closed_loop_engine()
    return engine.start_loop(trigger, pot_id, detection_data)


def get_loop_stats() -> Dict[str, Any]:
    """Obtiene estadísticas de loops."""
    engine = get_closed_loop_engine()
    return engine.get_stats()


__all__ = [
    "ClosedLoopEngine",
    "LoopContext",
    "LoopPhase",
    "LoopOutcome",
    "get_closed_loop_engine",
    "run_closed_loop",
    "get_loop_stats",
]
