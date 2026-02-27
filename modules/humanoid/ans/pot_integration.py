"""
ANS POT Integration: Conecta el Sistema Nervioso Autónomo con POTs.
====================================================================
Este módulo permite que ANS despache incidentes automáticamente a POTs.

Flujo:
1. ANS detecta incidente → create_incident()
2. Este hook intercepta → dispatch_incident_to_pot()
3. POT Dispatcher recibe → selecciona POT → ejecuta
4. Resultado → actualiza incidente con actions_taken
"""
from __future__ import annotations

import logging
import threading
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.ans.pot_integration")

# Flag para habilitar/deshabilitar integración
_integration_enabled = True
_integration_lock = threading.Lock()


def set_integration_enabled(enabled: bool) -> None:
    """Habilita o deshabilita la integración ANS-POT."""
    global _integration_enabled
    with _integration_lock:
        _integration_enabled = enabled
        _log.info("ANS-POT integration %s", "enabled" if enabled else "disabled")


def is_integration_enabled() -> bool:
    """Verifica si la integración está habilitada."""
    with _integration_lock:
        return _integration_enabled


def dispatch_incident_to_pot(
    incident_id: str,
    check_id: str,
    severity: str,
    message: str,
    evidence: Optional[Dict[str, Any]] = None,
    suggested_heals: Optional[List[str]] = None,
) -> Optional[str]:
    """
    Despacha un incidente al POT Dispatcher para procesamiento automático.
    
    Esta función es llamada automáticamente cuando se crea un incidente en ANS.
    
    Args:
        incident_id: ID del incidente creado
        check_id: ID del check que falló
        severity: Severidad (low, medium, high, critical)
        message: Mensaje del incidente
        evidence: Evidencia asociada
        suggested_heals: Heals sugeridos
    
    Returns:
        ID del request de dispatch o None si no se despachó
    """
    if not is_integration_enabled():
        _log.debug("ANS-POT integration disabled, skipping dispatch")
        return None
    
    try:
        from modules.humanoid.quality.dispatcher import (
            dispatch_incident,
            get_dispatcher,
        )
        
        # Asegurar que el dispatcher está corriendo
        dispatcher = get_dispatcher()
        if not dispatcher.is_running():
            dispatcher.start()
        
        # Construir contexto con toda la info del incidente
        context = {
            "incident_id": incident_id,
            "check_id": check_id,
            "severity": severity,
            "message": message,
            "evidence": evidence or {},
            "suggested_heals": suggested_heals or [],
            "source": "ans_auto_dispatch",
        }
        
        request_id = dispatch_incident(
            check_id=check_id,
            message=message,
            severity=severity,
            context=context,
        )
        
        _log.info(
            "Dispatched incident %s to POT: request_id=%s, check=%s, severity=%s",
            incident_id, request_id, check_id, severity
        )
        
        return request_id
        
    except ImportError as e:
        _log.debug("Quality module not available: %s", e)
        return None
    except Exception as e:
        _log.exception("Failed to dispatch incident to POT: %s", e)
        return None


def on_pot_complete(
    incident_id: str,
    pot_id: str,
    ok: bool,
    steps_ok: int,
    steps_total: int,
    error: Optional[str] = None,
) -> None:
    """
    Callback cuando un POT completa la ejecución para un incidente.
    Actualiza el incidente con las acciones tomadas.
    """
    try:
        from .incident import add_action, resolve_incident, get_incident
        
        inc = get_incident(incident_id)
        if not inc:
            _log.warning("Incident %s not found for POT completion", incident_id)
            return
        
        # Agregar acción
        add_action(
            inc_id=incident_id,
            heal_id=f"pot_{pot_id}",
            ok=ok,
            message=f"POT {pot_id}: {steps_ok}/{steps_total} steps OK" + (f" - Error: {error}" if error else ""),
        )
        
        # Si fue exitoso, resolver el incidente
        if ok:
            resolve_incident(incident_id)
            _log.info("Incident %s resolved by POT %s", incident_id, pot_id)
        else:
            _log.warning("Incident %s POT %s failed: %s", incident_id, pot_id, error)
            
    except Exception as e:
        _log.exception("Failed to update incident after POT: %s", e)


def hook_incident_creation(
    check_id: str,
    fingerprint: str,
    severity: str,
    message: str,
    evidence: Dict[str, Any] = None,
    suggested_heals: List[str] = None,
) -> Optional[str]:
    """
    Hook que se puede usar para interceptar la creación de incidentes
    y despachar automáticamente a POT.
    
    Uso:
        from modules.humanoid.ans.pot_integration import hook_incident_creation
        # Al crear incidente, también despachar
        inc_id = create_incident(...)
        hook_incident_creation(check_id, fingerprint, severity, message, evidence)
    """
    # Esta función crea el incidente Y lo despacha
    from .incident import create_incident as _create
    
    inc_id = _create(
        check_id=check_id,
        fingerprint=fingerprint,
        severity=severity,
        message=message,
        evidence=evidence,
        suggested_heals=suggested_heals,
    )
    
    # Despachar a POT
    dispatch_incident_to_pot(
        incident_id=inc_id,
        check_id=check_id,
        severity=severity,
        message=message,
        evidence=evidence,
        suggested_heals=suggested_heals,
    )
    
    return inc_id


def get_open_incidents(limit: int = 50) -> List[Dict[str, Any]]:
    """
    Obtiene incidentes abiertos (wrapper para usar desde triggers).
    """
    from .incident import get_incidents
    return get_incidents(status="open", limit=limit)


def register_dispatcher_hooks() -> None:
    """
    Registra hooks en el POT Dispatcher para actualizar incidentes
    cuando los POTs completan.
    """
    try:
        from modules.humanoid.quality.dispatcher import get_dispatcher
        
        dispatcher = get_dispatcher()
        
        def post_dispatch_hook(request, result):
            if not result:
                return
            
            incident_id = request.context.get("incident_id") if request else None
            if not incident_id:
                return
            
            on_pot_complete(
                incident_id=incident_id,
                pot_id=result.pot_id if hasattr(result, 'pot_id') else "unknown",
                ok=result.ok if hasattr(result, 'ok') else False,
                steps_ok=result.steps_ok if hasattr(result, 'steps_ok') else 0,
                steps_total=result.steps_total if hasattr(result, 'steps_total') else 0,
                error=result.error if hasattr(result, 'error') else None,
            )
        
        dispatcher.register_hook("post_dispatch", post_dispatch_hook)
        _log.info("Registered POT Dispatcher hooks for ANS")
        
    except Exception as e:
        _log.warning("Could not register dispatcher hooks: %s", e)


# ============================================================================
# AUTO-INITIALIZE
# ============================================================================

def init_ans_pot_integration() -> Dict[str, Any]:
    """
    Inicializa la integración ANS-POT.
    Llamar al inicio del sistema ATLAS.
    """
    try:
        # Habilitar integración
        set_integration_enabled(True)
        
        # Registrar hooks
        register_dispatcher_hooks()
        
        return {
            "ok": True,
            "integration_enabled": True,
            "hooks_registered": True,
        }
    except Exception as e:
        return {
            "ok": False,
            "error": str(e),
        }


# ============================================================================
# EXPORTACIONES
# ============================================================================

__all__ = [
    "set_integration_enabled",
    "is_integration_enabled",
    "dispatch_incident_to_pot",
    "on_pot_complete",
    "hook_incident_creation",
    "get_open_incidents",
    "register_dispatcher_hooks",
    "init_ans_pot_integration",
]
