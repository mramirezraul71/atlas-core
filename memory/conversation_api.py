# -*- coding: utf-8 -*-
"""
ATLAS Conversation API
API REST para gestionar hilos de conversación y autoprogramación
"""

import logging
from datetime import datetime
from typing import Any, Dict, List, Optional

from autoprogrammer_daemon import get_daemon, start_daemon, stop_daemon
from conversation_thread_manager import get_autoprogrammer, get_thread_manager
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# Configurar logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Modelos Pydantic
class MessageRequest(BaseModel):
    role: str  # user, assistant, system
    content: str
    metadata: Optional[Dict[str, Any]] = None


class ThreadCreateRequest(BaseModel):
    title: str
    tags: Optional[List[str]] = None


class ThreadUpdateRequest(BaseModel):
    title: Optional[str] = None
    status: Optional[str] = None
    priority: Optional[str] = None
    tags: Optional[List[str]] = None


class ActionRequest(BaseModel):
    action_type: str
    thread_id: str
    description: Optional[str] = None


# Crear app
app = FastAPI(
    title="ATLAS Conversation API",
    version="1.0.0",
    description="API para gestionar hilos de conversación y autoprogramación",
)


# Inicializar daemon
@app.on_event("startup")
async def startup_event():
    logger.info("Iniciando Conversation API...")
    start_daemon()


@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Deteniendo Conversation API...")
    stop_daemon()


# ==================== ENDPOINTS DE HILOS ====================


@app.post("/threads/create")
def create_thread(req: ThreadCreateRequest):
    """Crea un nuevo hilo de conversación"""
    try:
        manager = get_thread_manager()
        thread_id = manager.create_thread(req.title, req.tags)
        thread = manager.get_thread(thread_id)
        return {"ok": True, "thread_id": thread_id, "thread": thread.to_dict()}
    except Exception as e:
        logger.error(f"Error creando hilo: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/threads/{thread_id}")
def get_thread(thread_id: str):
    """Obtiene un hilo específico"""
    try:
        manager = get_thread_manager()
        thread = manager.get_thread(thread_id)
        if not thread:
            raise HTTPException(status_code=404, detail="Thread not found")
        return {"ok": True, "thread": thread.to_dict()}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error obteniendo hilo: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/threads")
def list_threads(status: Optional[str] = None, priority: Optional[str] = None):
    """Lista todos los hilos"""
    try:
        manager = get_thread_manager()
        threads = list(manager.threads.values())

        if status:
            threads = [t for t in threads if t.status == status]
        if priority:
            threads = [t for t in threads if t.priority == priority]

        return {
            "ok": True,
            "total": len(threads),
            "threads": [t.to_dict() for t in threads],
        }
    except Exception as e:
        logger.error(f"Error listando hilos: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/threads/{thread_id}/message")
def add_message(thread_id: str, req: MessageRequest):
    """Añade un mensaje a un hilo"""
    try:
        manager = get_thread_manager()
        manager.add_message(thread_id, req.role, req.content, req.metadata)
        thread = manager.get_thread(thread_id)
        return {"ok": True, "thread": thread.to_dict()}
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except Exception as e:
        logger.error(f"Error añadiendo mensaje: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.put("/threads/{thread_id}")
def update_thread(thread_id: str, req: ThreadUpdateRequest):
    """Actualiza un hilo"""
    try:
        manager = get_thread_manager()
        thread = manager.get_thread(thread_id)
        if not thread:
            raise HTTPException(status_code=404, detail="Thread not found")

        if req.title:
            thread.title = req.title
        if req.status:
            thread.status = req.status
        if req.priority:
            thread.priority = req.priority
        if req.tags:
            thread.tags = req.tags

        thread.updated_at = datetime.now().isoformat()
        manager.save_thread(thread_id)

        return {"ok": True, "thread": thread.to_dict()}
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error actualizando hilo: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== ENDPOINTS DE AUTOPROGRAMACIÓN ====================


@app.post("/actions/schedule")
def schedule_action(req: ActionRequest):
    """Programa una acción"""
    try:
        autoprogrammer = get_autoprogrammer()
        action = {
            "thread_id": req.thread_id,
            "action_type": req.action_type,
            "trigger_message": req.description or "",
            "timestamp": datetime.now().isoformat(),
            "status": "pending",
        }
        autoprogrammer.schedule_action(action)
        return {"ok": True, "action_id": action["id"], "action": action}
    except Exception as e:
        logger.error(f"Error programando acción: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/actions/pending")
def get_pending_actions():
    """Obtiene acciones pendientes"""
    try:
        autoprogrammer = get_autoprogrammer()
        pending = autoprogrammer.get_pending_actions()
        return {"ok": True, "total": len(pending), "actions": pending}
    except Exception as e:
        logger.error(f"Error obteniendo acciones: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/actions/history")
def get_action_history(limit: int = 50):
    """Obtiene historial de acciones"""
    try:
        daemon = get_daemon()
        history = daemon.get_execution_history(limit)
        return {"ok": True, "total": len(history), "history": history}
    except Exception as e:
        logger.error(f"Error obteniendo historial: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


# ==================== ENDPOINTS DE ESTADO ====================


@app.get("/status")
def get_status():
    """Obtiene el estado del sistema"""
    try:
        daemon = get_daemon()
        manager = get_thread_manager()
        return {
            "ok": True,
            "daemon": daemon.get_status(),
            "context": manager.get_context_snapshot(),
        }
    except Exception as e:
        logger.error(f"Error obteniendo estado: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/health")
def health_check():
    """Health check"""
    return {
        "ok": True,
        "timestamp": datetime.now().isoformat(),
        "service": "ATLAS Conversation API",
    }


# ==================== ENDPOINTS DE CONTEXTO ====================


@app.get("/context/snapshot")
def get_context_snapshot():
    """Obtiene snapshot del contexto actual"""
    try:
        manager = get_thread_manager()
        return {"ok": True, "snapshot": manager.get_context_snapshot()}
    except Exception as e:
        logger.error(f"Error obteniendo snapshot: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/context/threads/by-tag/{tag}")
def get_threads_by_tag(tag: str):
    """Obtiene hilos por etiqueta"""
    try:
        manager = get_thread_manager()
        threads = manager.get_threads_by_tag(tag)
        return {
            "ok": True,
            "tag": tag,
            "total": len(threads),
            "threads": [t.to_dict() for t in threads],
        }
    except Exception as e:
        logger.error(f"Error obteniendo hilos por tag: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/context/threads/by-priority/{priority}")
def get_threads_by_priority(priority: str):
    """Obtiene hilos por prioridad"""
    try:
        manager = get_thread_manager()
        threads = manager.get_threads_by_priority(priority)
        return {
            "ok": True,
            "priority": priority,
            "total": len(threads),
            "threads": [t.to_dict() for t in threads],
        }
    except Exception as e:
        logger.error(f"Error obteniendo hilos por prioridad: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8792)
