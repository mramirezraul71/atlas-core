"""Tools API — endpoint para listar herramientas disponibles (migrado de NEXUS)."""
from __future__ import annotations

from fastapi import APIRouter
import logging

log = logging.getLogger("atlas.tools")

router = APIRouter(tags=["Herramientas"])

_registry = None

def _get_registry():
    global _registry
    if _registry is None:
        try:
            from modules.humanoid.tools.registry import ToolsRegistry
            _registry = ToolsRegistry()
        except Exception as e:
            log.warning("ToolsRegistry not loaded: %s", e)
            return None
    return _registry


@router.get("/tools")
def list_tools():
    reg = _get_registry()
    if not reg:
        return {"ok": True, "tools": [], "count": 0}
    tools = reg.list_tools()
    data = [{"name": t.name, "description": t.description,
             "category": t.category.value, "requires_approval": t.requires_approval}
            for t in tools]
    return {"ok": True, "tools": data, "count": len(data)}


@router.get("/tools/{name}")
def get_tool(name: str):
    reg = _get_registry()
    if not reg:
        return {"ok": False, "error": "Registry not available"}
    tool = reg.get_tool(name)
    if not tool:
        return {"ok": False, "error": f"Tool '{name}' not found"}
    return {"ok": True, "name": tool.metadata.name, "description": tool.metadata.description,
            "category": tool.metadata.category.value, "parameters": tool.metadata.parameters}
