"""
ATLAS Quality Module (QA/QC)
============================
Sistema de Calidad con POTs (Procedimientos Operacionales de Trabajo).

Componentes:
- POTs: Procedimientos estandarizados para cada tipo de operación
- Snapshots: Estados de referencia del sistema (golden states)
- Reports: Reportes de ejecución y auditoría
- Engine: Motor de ejecución de procedimientos
- Sync Engine: Sincronización automática con Git/Remote
- Cerebro Connector: Conexión con ANS/Dashboard/Canales

POTs Disponibles:
-----------------
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

Uso:
    from modules.humanoid.quality import get_pot, execute_pot, list_pots
    
    # Listar POTs disponibles
    pots = list_pots(category="repair")
    
    # Obtener un POT específico
    pot = get_pot("camera_repair")
    
    # Ejecutar un POT
    result = execute_pot("camera_repair", context={"incident_id": "abc123"})
    
    # Usar sincronización automática
    from modules.humanoid.quality import sync_operation
    result = sync_operation("commit")  # Auto-detecta y ejecuta git_commit POT
    
    # Conectar con cerebro/dashboard
    from modules.humanoid.quality import get_bridge
    bridge = get_bridge()
    bridge.on_pot_complete("camera_repair", execution_id, ok=True, ...)
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
]
