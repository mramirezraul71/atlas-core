"""Directives API — endpoints REST para gestion de directivas (migrado de NEXUS)."""
from __future__ import annotations

from fastapi import APIRouter, Query
from pydantic import BaseModel
from typing import Optional
import logging

log = logging.getLogger("atlas.directives")

router = APIRouter(prefix="/directives", tags=["Directivas"])


def _mgr():
    from modules.humanoid.directives.manager import get_directives_manager
    return get_directives_manager()


class DirectiveContent(BaseModel):
    content: str

class ProjectDirective(BaseModel):
    project_name: str
    content: str

class ToggleRequest(BaseModel):
    enabled: bool


@router.get("/global")
def get_global():
    mgr = _mgr()
    content = mgr.get_global_directives()
    meta = mgr._load_metadata()
    return {"ok": True, "content": content, "enabled": meta.get("global_enabled", True),
            "size": len(content), "last_updated": meta.get("last_updated")}


@router.post("/global")
def set_global(body: DirectiveContent):
    ok = _mgr().set_global_directives(body.content)
    return {"ok": ok, "message": "Directivas globales actualizadas" if ok else "Error"}


@router.post("/global/append")
def append_global(body: DirectiveContent):
    ok = _mgr().append_global_directives(body.content)
    return {"ok": ok}


@router.post("/global/toggle")
def toggle_global(body: ToggleRequest):
    ok = _mgr().toggle_global(body.enabled)
    return {"ok": ok, "enabled": body.enabled}


@router.get("/projects")
def list_projects():
    projects = _mgr().list_projects()
    return {"ok": True, "projects": projects, "total": len(projects)}


@router.get("/projects/{name}")
def get_project(name: str):
    content = _mgr().get_project_directives(name)
    return {"ok": True, "project_name": name, "content": content, "size": len(content)}


@router.post("/projects")
def set_project(body: ProjectDirective):
    ok = _mgr().set_project_directives(body.project_name, body.content)
    return {"ok": ok, "project_name": body.project_name}


@router.delete("/projects/{name}")
def delete_project(name: str):
    ok = _mgr().delete_project(name)
    return {"ok": ok}


@router.post("/projects/{name}/toggle")
def toggle_project(name: str, body: ToggleRequest):
    ok = _mgr().toggle_project(name, body.enabled)
    return {"ok": ok, "enabled": body.enabled}


@router.get("/active")
def get_active(project_name: Optional[str] = Query(None)):
    content = _mgr().get_active_directives(project_name)
    return {"ok": True, "content": content, "project_name": project_name, "size": len(content)}


@router.get("/summary")
def get_summary():
    return {"ok": True, **_mgr().get_summary()}


@router.get("/health")
def health():
    mgr = _mgr()
    meta = mgr._load_metadata()
    return {"ok": True, "status": "healthy", "projects_count": len(mgr.list_projects()),
            "global_exists": len(mgr.get_global_directives()) > 0}
