#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ATLAS Chat API Router
Endpoints para gestionar hilos de conversación
"""

import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

# Importar el manager
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from chat_thread_manager import ChatThreadManager

# Inicializar manager
chat_manager = ChatThreadManager()

# Router
router = APIRouter(prefix="/api/chat", tags=["chat"])

# ═══════════════════════════════════════════════════════════════
# Modelos Pydantic
# ═══════════════════════════════════════════════════════════════


class CreateThreadRequest(BaseModel):
    title: str
    user_id: str
    description: Optional[str] = ""
    context: Optional[Dict] = None


class AddMessageRequest(BaseModel):
    thread_id: str
    sender: str
    role: str  # "user", "assistant", "system"
    content: str
    metadata: Optional[Dict] = None


class SetContextRequest(BaseModel):
    thread_id: str
    key: str
    value: Any


# ═══════════════════════════════════════════════════════════════
# Endpoints
# ═══════════════════════════════════════════════════════════════


@router.post("/threads/create")
def create_thread(req: CreateThreadRequest):
    """Crear nuevo hilo de conversación"""
    try:
        thread_id = chat_manager.create_thread(
            title=req.title,
            user_id=req.user_id,
            description=req.description,
            context=req.context,
        )
        return {
            "ok": True,
            "thread_id": thread_id,
            "message": "Hilo creado exitosamente",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/messages/add")
def add_message(req: AddMessageRequest):
    """Agregar mensaje a un hilo"""
    try:
        message_id = chat_manager.add_message(
            thread_id=req.thread_id,
            sender=req.sender,
            role=req.role,
            content=req.content,
            metadata=req.metadata,
        )
        return {"ok": True, "message_id": message_id, "message": "Mensaje agregado"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/threads/{thread_id}/history")
def get_thread_history(thread_id: str, limit: int = Query(100, ge=1, le=1000)):
    """Obtener historial de un hilo"""
    try:
        history = chat_manager.get_thread_history(thread_id, limit)
        return {
            "ok": True,
            "thread_id": thread_id,
            "messages": history,
            "count": len(history),
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/threads")
def get_all_threads(user_id: Optional[str] = Query(None)):
    """Obtener todos los hilos (opcionalmente filtrados por usuario)"""
    try:
        threads = chat_manager.get_all_threads(user_id)
        return {"ok": True, "threads": threads, "count": len(threads)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/threads/search")
def search_threads(q: str = Query(..., min_length=1)):
    """Buscar hilos por título o descripción"""
    try:
        results = chat_manager.search_threads(q)
        return {"ok": True, "query": q, "results": results, "count": len(results)}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/threads/{thread_id}/context")
def get_thread_context(thread_id: str, key: Optional[str] = Query(None)):
    """Obtener contexto de un hilo"""
    try:
        context = chat_manager.get_context(thread_id, key)
        return {"ok": True, "thread_id": thread_id, "context": context}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/threads/{thread_id}/context")
def set_thread_context(thread_id: str, req: SetContextRequest):
    """Guardar contexto en un hilo"""
    try:
        chat_manager.set_context(thread_id, req.key, req.value)
        return {
            "ok": True,
            "thread_id": thread_id,
            "key": req.key,
            "message": "Contexto guardado",
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/stats")
def get_chat_stats():
    """Obtener estadísticas del sistema de chats"""
    try:
        stats = chat_manager.get_stats()
        return {"ok": True, "stats": stats}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
def health_check():
    """Verificar salud del sistema de chats"""
    try:
        stats = chat_manager.get_stats()
        return {"ok": True, "status": "healthy", "stats": stats}
    except Exception as e:
        return {"ok": False, "status": "unhealthy", "error": str(e)}
