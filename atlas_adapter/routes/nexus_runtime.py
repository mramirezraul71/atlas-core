"""NEXUS/Robot runtime routes extracted from the main PUSH app."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from atlas_adapter.services.nexus_robot_runtime import (
    execute_feet_command,
    get_nerve_status,
    get_nexus_connection_payload,
    get_nexus_log_tail,
    get_robot_log_tail,
    get_robot_start_commands,
    get_robot_status,
    reconnect_cuerpo,
    reconnect_nexus,
    reconnect_robot,
    update_nexus_connection_state,
)
from fastapi import APIRouter
from pydantic import BaseModel

router = APIRouter(tags=["NEXUS"])


class NexusConnectionBody(BaseModel):
    connected: bool = False
    message: str = ""
    active: Optional[bool] = None


def build_router(repo_root: Path, env_path: Path) -> APIRouter:
    local_router = APIRouter()
    local_router.include_router(router)

    @local_router.get("/api/nexus/connection")
    def get_nexus_connection():
        """CEREBRO - CUERPO (NEXUS): estado de conexión consolidado en PUSH."""
        return get_nexus_connection_payload()

    @local_router.post("/api/nexus/connection")
    def post_nexus_connection(body: NexusConnectionBody):
        """Actualiza estado de conexión NEXUS desde heartbeat o pruebas locales."""
        return update_nexus_connection_state(
            connected=body.connected,
            message=body.message or "",
            active=body.active,
        )

    @local_router.post("/api/nexus/reconnect")
    def nexus_reconnect(clear_cache: bool = False):
        """Reconectar NEXUS y devolver el estado inicial sin bloquear."""
        return reconnect_nexus(clear_cache=clear_cache)

    @local_router.get("/api/robot/status")
    def robot_status():
        """Estado del Robot (cámaras, visión)."""
        return get_robot_status()

    @local_router.post("/api/robot/reconnect")
    def robot_reconnect():
        """Arranca el backend del Robot y devuelve enseguida."""
        return reconnect_robot(repo_root=repo_root, env_path=env_path)

    @local_router.post("/api/cuerpo/reconnect")
    def cuerpo_reconnect():
        """Arranca NEXUS y Robot en paralelo y responde rápido."""
        return reconnect_cuerpo(repo_root=repo_root, env_path=env_path)

    @local_router.get("/api/robot/start-commands")
    def robot_start_commands():
        """Devuelve comandos manuales para arrancar NEXUS y Robot."""
        return get_robot_start_commands(repo_root=repo_root, env_path=env_path)

    @local_router.get("/api/robot/log/tail")
    def robot_log_tail(lines: int = 200):
        """Ultimas lineas del log del backend Robot."""
        return get_robot_log_tail(base_dir=repo_root, lines=lines)

    @local_router.get("/api/nexus/log/tail")
    def nexus_log_tail(lines: int = 200):
        """Ultimas lineas del log de NEXUS."""
        return get_nexus_log_tail(base_dir=repo_root, lines=lines)

    @local_router.get("/nervous/services")
    def nervous_services():
        """Alias compacto del estado del sistema nervioso para compatibilidad."""
        return get_nerve_status()

    @local_router.get("/api/nerve/status")
    def nerve_status():
        """Estado consolidado de ojos, manos y pies."""
        return get_nerve_status()

    @local_router.post("/api/feet/execute")
    def api_feet_execute(body: dict):
        """Ejecuta pies internos o digitales con el payload solicitado."""
        payload = body if isinstance(body, dict) else {}
        return execute_feet_command(
            command=payload.get("command") or "",
            payload=payload.get("payload") or {},
        )

    return local_router
