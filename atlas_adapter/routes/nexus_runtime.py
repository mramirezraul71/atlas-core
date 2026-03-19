"""NEXUS/Robot runtime routes extracted from the main PUSH app."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

from atlas_adapter.services.nexus_robot_runtime import (
    get_nexus_connection_payload,
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

    return local_router
