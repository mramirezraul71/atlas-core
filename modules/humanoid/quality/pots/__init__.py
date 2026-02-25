"""
POTs Registry: Todos los POTs disponibles.
===============================================
Catálogo completo de Procedimientos Operacionales de Trabajo (POT).

Cada POT define cómo ATLAS debe ejecutar una tarea específica,
sirviendo como guía tutorial para operaciones internas.
"""
from typing import Callable, Dict, List

# === QUALITY / TUTORIAS ===
# === ROBOTICS ===
# === AUTO-UPDATE ===
# === AUTONOMY ===
# === SESSION MANAGEMENT ===
# === NOTIFICATIONS / COMMUNICATION ===
# === DIAGNOSTICS ===
# === INCIDENTS ===
# === MAINTENANCE ===
# === REPAIRS ===
# === DEPLOYMENT / GIT ===
from . import (api_repair, auto_update_full, autonomy_full_cycle,
               camera_repair, comms_management, deployment_full,
               diagnostic_full, git_commit, git_pull, git_push, git_safe_sync,
               hri_interaction, incident_response, incident_triage,
               maintenance_daily, maintenance_weekly, manipulation_grasp,
               navigation_slam, notification_broadcast, repo_update,
               sensor_fusion, services_repair, session_shutdown,
               session_startup, simulation_training, specialist_visit,
               vision_pipeline)

# ============================================================================
# IMPORTS DE POTs POR CATEGORÍA
# ============================================================================


# ============================================================================
# REGISTRY DE POTs
# ============================================================================

_POT_MODULES: Dict[str, Callable] = {
    # Git / Deployment
    "git_commit": git_commit.get_pot,
    "git_push": git_push.get_pot,
    "git_pull": git_pull.get_pot,
    "git_safe_sync": git_safe_sync.get_pot,
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
    # Notifications / Communications
    "notification_broadcast": notification_broadcast.get_pot,
    "comms_management": comms_management.get_pot,
    # Session
    "session_startup": session_startup.get_pot,
    "session_shutdown": session_shutdown.get_pot,
    # Autonomy
    "autonomy_full_cycle": autonomy_full_cycle.get_pot,
    # Auto-Update
    "auto_update_full": auto_update_full.get_pot,
    # Robotics
    "navigation_slam": navigation_slam.get_pot,
    "simulation_training": simulation_training.get_pot,
    "sensor_fusion": sensor_fusion.get_pot,
    "manipulation_grasp": manipulation_grasp.get_pot,
    "hri_interaction": hri_interaction.get_pot,
    "vision_pipeline": vision_pipeline.get_pot,
    # Quality / Tutorias
    "specialist_visit": specialist_visit.get_pot,
}


def get_all_pot_ids() -> List[str]:
    """Retorna lista de todos los IDs de POT disponibles."""
    return list(_POT_MODULES.keys())


def get_pot_factory(pot_id: str) -> Callable:
    """Retorna la función factory para crear un POT."""
    return _POT_MODULES.get(pot_id)


__all__ = ["get_all_pot_ids", "get_pot_factory", "_POT_MODULES"]
