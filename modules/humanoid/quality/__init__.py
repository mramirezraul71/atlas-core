"""
ATLAS Quality Module (QA/QC)
============================
Sistema de Calidad con POTs (Procedimientos Operacionales de Trabajo).

ARQUITECTURA COMPLETA DE AUTONOMÍA:
===================================

┌─────────────────────────────────────────────────────────────────────────┐
│                        ATLAS QUALITY SYSTEM                            │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │   TRIGGERS   │───▶│  DISPATCHER  │───▶│   EXECUTOR   │              │
│  │  (triggers)  │    │ (dispatcher) │    │  (executor)  │              │
│  └──────────────┘    └──────────────┘    └──────────────┘              │
│         ▲                   │                   │                       │
│         │                   ▼                   ▼                       │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐              │
│  │     ANS      │    │   REGISTRY   │    │    POTs      │              │
│  │ (incidents)  │    │  (registry)  │    │   (pots/)    │              │
│  └──────────────┘    └──────────────┘    └──────────────┘              │
│         │                                       │                       │
│         ▼                                       ▼                       │
│  ┌──────────────┐                        ┌──────────────┐              │
│  │  SCHEDULER   │                        │   REPORTS    │              │
│  │   (jobs)     │                        │  (reports/)  │              │
│  └──────────────┘                        └──────────────┘              │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘

Componentes:
- POTs: Procedimientos estandarizados para cada tipo de operación
- Dispatcher: Motor de despacho automático de POTs
- Triggers: Sistema de triggers que detectan condiciones y disparan POTs
- Executor: Motor de ejecución de POTs
- Registry: Registro y búsqueda de POTs
- Sync Engine: Sincronización automática con Git/Remote
- Cerebro Connector: Conexión con ANS/Dashboard/Canales

POTs Disponibles (18):
----------------------
Git/Deployment:
    - git_commit: Commit de cambios
    - git_push: Push al remoto
    - git_pull: Pull desde remoto
    - repo_update: Actualización completa del repo
    - deployment_full: Despliegue a producción

Repairs:
    - camera_repair: Reparación de cámara
    - services_repair: Reparación de servicios
    - api_repair: Reparación de API

Maintenance:
    - maintenance_daily: Mantenimiento diario
    - maintenance_weekly: Mantenimiento semanal

Incidents:
    - incident_triage: Clasificación de incidentes
    - incident_response: Respuesta a incidentes

Diagnostics:
    - diagnostic_full: Diagnóstico completo

Session:
    - session_startup: Inicio de sesión
    - session_shutdown: Cierre de sesión

Communication:
    - notification_broadcast: Difusión a todos los canales

Autonomy:
    - autonomy_full_cycle: Ciclo completo de autonomía

Auto-Update:
    - auto_update_full: Actualización automática completa

INICIO RÁPIDO:
==============
    from modules.humanoid.quality import start_autonomous_system
    
    # Iniciar todo el sistema de autonomía
    start_autonomous_system()
    
    # O manualmente:
    from modules.humanoid.quality import start_dispatcher, start_triggers
    start_dispatcher()  # Inicia el motor de ejecución
    start_triggers()    # Inicia el monitoreo de condiciones
"""

from .registry import get_pot, list_pots, get_pot_by_incident
from .executor import execute_pot, execute_step
from .models import POT, POTStep, POTResult, StepResult, POTCategory, POTSeverity
from .sync_engine import (
    SyncEngine, 
    get_sync_engine, 
    sync_operation, 
    OPERATION_POT_MAP,
    build_pot_context,
)
from .cerebro_connector import (
    AtlasQualityBridge,
    CerebroConnector,
    DashboardConnector,
    ChannelConnector,
    get_bridge,
)
from .dispatcher import (
    POTDispatcher,
    DispatchRequest,
    DispatchResult,
    TriggerType,
    get_dispatcher,
    start_dispatcher,
    stop_dispatcher,
    dispatch_pot,
    dispatch_incident,
    dispatch_event,
    get_dispatch_stats,
    get_dispatch_history,
)
from .triggers import (
    TriggerCondition,
    TriggerRule,
    TriggerRegistry,
    TriggerEngine,
    get_trigger_engine,
    start_triggers,
    stop_triggers,
    register_trigger,
    get_trigger_stats,
)


def start_autonomous_system() -> dict:
    """
    Inicia el sistema completo de autonomía ATLAS.
    
    Esto incluye:
    1. POT Dispatcher - Motor de ejecución de POTs
    2. Trigger Engine - Monitoreo de condiciones
    3. ANS Integration - Conexión con sistema nervioso
    
    Returns:
        Estado del sistema iniciado
    """
    results = {}
    
    # 1. Iniciar Dispatcher
    try:
        dispatcher = start_dispatcher()
        results["dispatcher"] = {
            "ok": True,
            "running": dispatcher.is_running(),
        }
    except Exception as e:
        results["dispatcher"] = {"ok": False, "error": str(e)}
    
    # 2. Iniciar Triggers
    try:
        trigger_engine = start_triggers()
        results["triggers"] = {
            "ok": True,
            "running": trigger_engine.is_running(),
            "rules_enabled": len(trigger_engine.registry.list_all(enabled_only=True)),
        }
    except Exception as e:
        results["triggers"] = {"ok": False, "error": str(e)}
    
    # 3. Iniciar ANS Integration
    try:
        from modules.humanoid.ans.pot_integration import init_ans_pot_integration
        ans_result = init_ans_pot_integration()
        results["ans_integration"] = ans_result
    except Exception as e:
        results["ans_integration"] = {"ok": False, "error": str(e)}
    
    results["all_ok"] = all(r.get("ok", False) for r in results.values())
    return results


def stop_autonomous_system() -> dict:
    """
    Detiene el sistema de autonomía.
    """
    results = {}
    
    try:
        stop_triggers()
        results["triggers"] = {"ok": True, "stopped": True}
    except Exception as e:
        results["triggers"] = {"ok": False, "error": str(e)}
    
    try:
        stop_dispatcher(graceful=True)
        results["dispatcher"] = {"ok": True, "stopped": True}
    except Exception as e:
        results["dispatcher"] = {"ok": False, "error": str(e)}
    
    return results


def get_autonomy_status() -> dict:
    """
    Obtiene el estado completo del sistema de autonomía.
    """
    status = {
        "dispatcher": get_dispatch_stats(),
        "triggers": get_trigger_stats(),
        "pots_available": len(list_pots()),
        "history": get_dispatch_history(limit=5),
    }
    return status


__all__ = [
    # Core POT functions
    "get_pot",
    "list_pots",
    "get_pot_by_incident",
    "execute_pot",
    "execute_step",
    
    # Models
    "POT",
    "POTStep",
    "POTResult",
    "StepResult",
    "POTCategory",
    "POTSeverity",
    
    # Sync Engine
    "SyncEngine",
    "get_sync_engine",
    "sync_operation",
    "OPERATION_POT_MAP",
    "build_pot_context",
    
    # Connectors
    "AtlasQualityBridge",
    "CerebroConnector",
    "DashboardConnector",
    "ChannelConnector",
    "get_bridge",
    
    # Dispatcher
    "POTDispatcher",
    "DispatchRequest",
    "DispatchResult",
    "TriggerType",
    "get_dispatcher",
    "start_dispatcher",
    "stop_dispatcher",
    "dispatch_pot",
    "dispatch_incident",
    "dispatch_event",
    "get_dispatch_stats",
    "get_dispatch_history",
    
    # Triggers
    "TriggerCondition",
    "TriggerRule",
    "TriggerRegistry",
    "TriggerEngine",
    "get_trigger_engine",
    "start_triggers",
    "stop_triggers",
    "register_trigger",
    "get_trigger_stats",
    
    # High-level autonomy
    "start_autonomous_system",
    "stop_autonomous_system",
    "get_autonomy_status",
]
