#!/usr/bin/env python3
"""
ATLAS NEXUS - Directives API
Endpoints REST para gestión de directivas
"""

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel
from typing import Dict, List, Optional
import logging

from directives_manager import directives_manager

logger = logging.getLogger(__name__)

# Crear router de FastAPI
router = APIRouter(prefix="/directives", tags=["directives"])

# Modelos Pydantic
class DirectiveContent(BaseModel):
    content: str

class ProjectDirective(BaseModel):
    project_name: str
    content: str

class ToggleRequest(BaseModel):
    enabled: bool

class ProjectTemplateRequest(BaseModel):
    description: str = ""

# ENDPOINTS PARA DIRECTIVAS GLOBALES

@router.get("/global")
async def get_global_directives():
    """Obtener directivas globales"""
    try:
        content = directives_manager.get_global_directives()
        return {
            "content": content,
            "enabled": True,  # TODO: Implementar desde metadata
            "size": len(content),
            "last_updated": directives_manager._load_metadata().get("last_updated")
        }
    except Exception as e:
        logger.error(f"Error obteniendo directivas globales: {e}")
        raise HTTPException(status_code=500, detail="Error obteniendo directivas globales")

@router.post("/global")
async def update_global_directives(request: DirectiveContent):
    """Actualizar directivas globales"""
    try:
        success = directives_manager.set_global_directives(request.content)
        if success:
            return {"message": "Directivas globales actualizadas exitosamente"}
        else:
            raise HTTPException(status_code=500, detail="Error actualizando directivas globales")
    except Exception as e:
        logger.error(f"Error actualizando directivas globales: {e}")
        raise HTTPException(status_code=500, detail="Error actualizando directivas globales")

@router.post("/global/append")
async def append_global_directives(request: DirectiveContent):
    """Agregar contenido a directivas globales"""
    try:
        success = directives_manager.append_global_directives(request.content)
        if success:
            return {"message": "Contenido agregado a directivas globales"}
        else:
            raise HTTPException(status_code=500, detail="Error agregando a directivas globales")
    except Exception as e:
        logger.error(f"Error agregando a directivas globales: {e}")
        raise HTTPException(status_code=500, detail="Error agregando a directivas globales")

@router.post("/global/toggle")
async def toggle_global_directives(request: ToggleRequest):
    """Activar/desactivar directivas globales"""
    try:
        success = directives_manager.toggle_global(request.enabled)
        if success:
            status = "activadas" if request.enabled else "desactivadas"
            return {"message": f"Directivas globales {status}"}
        else:
            raise HTTPException(status_code=500, detail="Error cambiando estado de directivas globales")
    except Exception as e:
        logger.error(f"Error cambiando estado de directivas globales: {e}")
        raise HTTPException(status_code=500, detail="Error cambiando estado de directivas globales")

# ENDPOINTS PARA DIRECTIVAS POR PROYECTO

@router.get("/projects")
async def list_projects():
    """Listar todos los proyectos"""
    try:
        projects = directives_manager.list_projects()
        return {
            "projects": projects,
            "total": len(projects)
        }
    except Exception as e:
        logger.error(f"Error listando proyectos: {e}")
        raise HTTPException(status_code=500, detail="Error listando proyectos")

@router.get("/projects/{project_name}")
async def get_project_directives(project_name: str):
    """Obtener directivas de un proyecto específico"""
    try:
        content = directives_manager.get_project_directives(project_name)
        if not content:
            raise HTTPException(status_code=404, detail=f"Proyecto '{project_name}' no encontrado")
            
        metadata = directives_manager._load_metadata()
        project_info = metadata.get("projects", {}).get(project_name, {})
        
        return {
            "project_name": project_name,
            "content": content,
            "enabled": project_info.get("enabled", True),
            "size": len(content),
            "created": project_info.get("created"),
            "last_updated": project_info.get("last_updated")
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error obteniendo directivas del proyecto {project_name}: {e}")
        raise HTTPException(status_code=500, detail="Error obteniendo directivas del proyecto")

@router.post("/projects")
async def create_or_update_project(request: ProjectDirective):
    """Crear o actualizar directivas de un proyecto"""
    try:
        success = directives_manager.set_project_directives(request.project_name, request.content)
        if success:
            return {
                "message": f"Directivas del proyecto '{request.project_name}' actualizadas",
                "project_name": request.project_name
            }
        else:
            raise HTTPException(status_code=500, detail="Error actualizando directivas del proyecto")
    except Exception as e:
        logger.error(f"Error actualizando directivas del proyecto {request.project_name}: {e}")
        raise HTTPException(status_code=500, detail="Error actualizando directivas del proyecto")

@router.post("/projects/{project_name}/template")
async def create_project_from_template(project_name: str, request: ProjectTemplateRequest):
    """Crear un proyecto desde template"""
    try:
        success = directives_manager.create_project_template(project_name, request.description)
        if success:
            return {
                "message": f"Proyecto '{project_name}' creado desde template",
                "project_name": project_name,
                "description": request.description
            }
        else:
            raise HTTPException(status_code=500, detail="Error creando proyecto desde template")
    except Exception as e:
        logger.error(f"Error creando template para {project_name}: {e}")
        raise HTTPException(status_code=500, detail="Error creando proyecto desde template")

@router.delete("/projects/{project_name}")
async def delete_project(project_name: str):
    """Eliminar un proyecto"""
    try:
        success = directives_manager.delete_project(project_name)
        if success:
            return {"message": f"Proyecto '{project_name}' eliminado"}
        else:
            raise HTTPException(status_code=404, detail=f"Proyecto '{project_name}' no encontrado")
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error eliminando proyecto {project_name}: {e}")
        raise HTTPException(status_code=500, detail="Error eliminando proyecto")

@router.post("/projects/{project_name}/toggle")
async def toggle_project_directives(project_name: str, request: ToggleRequest):
    """Activar/desactivar directivas de un proyecto"""
    try:
        success = directives_manager.toggle_project(project_name, request.enabled)
        if success:
            status = "activadas" if request.enabled else "desactivadas"
            return {"message": f"Directivas del proyecto '{project_name}' {status}"}
        else:
            raise HTTPException(status_code=500, detail="Error cambiando estado del proyecto")
    except Exception as e:
        logger.error(f"Error cambiando estado del proyecto {project_name}: {e}")
        raise HTTPException(status_code=500, detail="Error cambiando estado del proyecto")

# ENDPOINTS DE UTILIDADES

@router.get("/active")
async def get_active_directives(project_name: Optional[str] = Query(None)):
    """Obtener directivas activas (globales + de proyecto si se especifica)"""
    try:
        content = directives_manager.get_active_directives(project_name)
        metadata = directives_manager._load_metadata()
        
        return {
            "content": content,
            "project_name": project_name,
            "global_enabled": metadata.get("global_enabled", True),
            "project_enabled": True if project_name else None,  # TODO: Implementar desde metadata
            "size": len(content),
            "directives_count": len([d for d in [directives_manager.get_global_directives(), 
                                                directives_manager.get_project_directives(project_name) if project_name else ""] if d])
        }
    except Exception as e:
        logger.error(f"Error obteniendo directivas activas: {e}")
        raise HTTPException(status_code=500, detail="Error obteniendo directivas activas")

@router.get("/summary")
async def get_directives_summary():
    """Obtener resumen completo del sistema de directivas"""
    try:
        summary = directives_manager.get_summary()
        return summary
    except Exception as e:
        logger.error(f"Error obteniendo resumen: {e}")
        raise HTTPException(status_code=500, detail="Error obteniendo resumen")

@router.post("/quick/create-default")
async def create_default_directives():
    """Crear directivas por defecto"""
    try:
        success = directives_manager.create_default_directives()
        if success:
            return {"message": "Directivas por defecto creadas exitosamente"}
        else:
            raise HTTPException(status_code=500, detail="Error creando directivas por defecto")
    except Exception as e:
        logger.error(f"Error creando directivas por defecto: {e}")
        raise HTTPException(status_code=500, detail="Error creando directivas por defecto")

# ENDPOINT DE HEALTH CHECK

@router.get("/health")
async def health_check():
    """Health check del sistema de directivas"""
    try:
        metadata = directives_manager._load_metadata()
        projects = directives_manager.list_projects()
        global_content = directives_manager.get_global_directives()
        
        return {
            "status": "healthy",
            "version": metadata.get("version", "1.0.0"),
            "last_updated": metadata.get("last_updated"),
            "global_directives": len(global_content) > 0,
            "projects_count": len(projects),
            "base_path": str(directives_manager.base_path)
        }
    except Exception as e:
        logger.error(f"Error en health check: {e}")
        return {
            "status": "unhealthy",
            "error": str(e)
        }

# Función para registrar el router en la app principal
def register_directives_api(app):
    """Registrar endpoints de directivas en la app FastAPI"""
    app.include_router(router)
    logger.info("API de Directivas registrada exitosamente")
