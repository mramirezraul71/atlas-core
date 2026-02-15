"""ANS checks: register all checks."""
from __future__ import annotations

from . import api_health, scheduler_health, memory_health, audit_health, llm_health, deps_health, router_health
from . import ui_health, deploy_health, gateway_health, cluster_health, disk_health, logs_health, evolution_health
from . import nexus_services_health
from . import robot_camera_health
from . import nervous_health
from modules.humanoid.ans.registry import register_check


def _register_all() -> None:
    register_check("api_health", api_health.run)
    register_check("evolution_health", evolution_health.run)
    register_check("scheduler_health", scheduler_health.run)
    register_check("memory_health", memory_health.run)
    register_check("audit_health", audit_health.run)
    register_check("llm_health", llm_health.run)
    register_check("router_health", router_health.run)
    register_check("deps_health", deps_health.run)
    register_check("ui_health", ui_health.run)
    register_check("deploy_health", deploy_health.run)
    register_check("gateway_health", gateway_health.run)
    register_check("cluster_health", cluster_health.run)
    register_check("disk_health", disk_health.run)
    register_check("logs_health", logs_health.run)
    register_check("nexus_services_health", nexus_services_health.run)
    register_check("robot_camera_health", robot_camera_health.run)
    register_check("nervous_health", nervous_health.run)


_register_all()
