"""
POTs Registry: Todos los POTs disponibles.
===============================================
Catálogo completo de Procedimientos Operacionales de Trabajo (POT).

Cada POT define cómo ATLAS debe ejecutar una tarea específica,
sirviendo como guía tutorial para operaciones internas.
"""
from typing import Dict, List, Callable

# ============================================================================
# IMPORTS DE POTs POR CATEGORÍA
# ============================================================================

# === DEPLOYMENT / GIT ===
from . import git_commit
from . import git_push
from . import git_pull
from . import repo_update
from . import deployment_full

# === REPAIRS ===
from . import camera_repair
from . import services_repair
from . import api_repair

# === MAINTENANCE ===
from . import maintenance_daily
from . import maintenance_weekly

# === INCIDENTS ===
from . import incident_triage
from . import incident_response

# === DIAGNOSTICS ===
from . import diagnostic_full

# === NOTIFICATIONS / COMMUNICATION ===
from . import notification_broadcast

# === SESSION MANAGEMENT ===
from . import session_startup
from . import session_shutdown

# === AUTONOMY ===
from . import autonomy_full_cycle

# === AUTO-UPDATE ===
from . import auto_update_full


# ============================================================================
# REGISTRY DE POTs
# ============================================================================

_POT_MODULES: Dict[str, Callable] = {
    # Git / Deployment
    "git_commit": git_commit.get_pot,
    "git_push": git_push.get_pot,
    "git_pull": git_pull.get_pot,
    "repo_update": repo_update.get_pot,
    "deployment_full": deployment_full.get_pot,
    
    # Repairs
    "camera_repair": camera_repair.get_pot,
    "services_repair": services_repair.get_pot,
    "api_repair": api_repair.get_pot,
    
    # Maintenance
    "maintenance_daily": maintenance_daily.get_pot,
    "maintenance_weekly": maintenance_weekly.get_pot,
    
    # Incidents
    "incident_triage": incident_triage.get_pot,
    "incident_response": incident_response.get_pot,
    
    # Diagnostics
    "diagnostic_full": diagnostic_full.get_pot,
    
    # Notifications
    "notification_broadcast": notification_broadcast.get_pot,
    
    # Session
    "session_startup": session_startup.get_pot,
    "session_shutdown": session_shutdown.get_pot,
    
    # Autonomy
    "autonomy_full_cycle": autonomy_full_cycle.get_pot,
    
    # Auto-Update
    "auto_update_full": auto_update_full.get_pot,
}


def get_all_pot_ids() -> List[str]:
    """Retorna lista de todos los IDs de POT disponibles."""
    return list(_POT_MODULES.keys())


def get_pot_factory(pot_id: str) -> Callable:
    """Retorna la función factory para crear un POT."""
    return _POT_MODULES.get(pot_id)


__all__ = ["get_all_pot_ids", "get_pot_factory", "_POT_MODULES"]
