"""
Workspace Memory Connector - ConexiÃ³n entre Atlas Nexus y Unified Memory Cortex
Permite que el workspace acceda y persista informaciÃ³n en los sistemas de memoria de Atlas
"""
from __future__ import annotations

import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

# Router para endpoints de memoria
memory_router = APIRouter(prefix="/api/memory", tags=["memory"])


class MemoryQuery(BaseModel):
    query: str
    memory_types: Optional[List[str]] = None
    limit: int = 10
    min_relevance: float = 0.0


class MemoryStore(BaseModel):
    content: str
    memory_type: str
    metadata: Optional[Dict[str, Any]] = None


class ChatMessage(BaseModel):
    message: str
    role: str  # user, assistant, system
    session_id: Optional[str] = None
    context: Optional[Dict[str, Any]] = None


class WorkspaceMemoryConnector:
    """
    Conector entre Workspace y Unified Memory Cortex.

    Provee:
    - BÃºsqueda unificada en memoria
    - Almacenamiento de conversaciones
    - Contexto para acciones
    - Persistencia de resultados
    """

    def __init__(self):
        self.memory_enabled = True
        self.chat_sessions: Dict[str, List[Dict]] = {}
        try:
            print("Workspace Memory Connector iniciado")
        except Exception:
            pass

    def search_memory(
        self,
        query: str,
        memory_types: List[str] = None,
        limit: int = 10,
        min_relevance: float = 0.0,
    ) -> Dict[str, Any]:
        """
        Buscar en Unified Memory Cortex.

        Args:
            query: Query de bÃºsqueda
            memory_types: Tipos de memoria a consultar
            limit: LÃ­mite de resultados
            min_relevance: Relevancia mÃ­nima

        Returns:
            Resultados de bÃºsqueda
        """
        try:
            from modules.humanoid.cortex.unified_memory import \
                get_unified_memory

            memory = get_unified_memory()
            results = memory.recall(
                query=query,
                memory_types=memory_types,
                limit=limit,
                min_relevance=min_relevance,
            )

            return {
                "success": True,
                "query": query,
                "results": [r.to_dict() for r in results],
                "total_found": len(results),
                "sources": list(set(r.source for r in results)),
            }

        except Exception as e:
            return {"success": False, "error": str(e), "results": [], "total_found": 0}

    def store_memory(
        self, content: str, memory_type: str, metadata: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Almacenar en ChromaDB.

        Args:
            content: Contenido a almacenar
            memory_type: Tipo de memoria
            metadata: Metadatos adicionales

        Returns:
            Resultado del almacenamiento
        """
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            memory_id = chroma.add_memory(
                content=content, memory_type=memory_type, metadata=metadata or {}
            )

            return {
                "success": True,
                "memory_id": memory_id,
                "memory_type": memory_type,
                "timestamp": datetime.now().isoformat(),
            }

        except Exception as e:
            return {"success": False, "error": str(e), "memory_id": None}

    def store_chat_message(
        self,
        message: str,
        role: str,
        session_id: str = None,
        context: Dict[str, Any] = None,
    ) -> Dict[str, Any]:
        """
        Almacenar mensaje de chat en memoria.

        Args:
            message: Mensaje
            role: Rol (user/assistant/system)
            session_id: ID de sesiÃ³n
            context: Contexto adicional

        Returns:
            Resultado del almacenamiento
        """
        if session_id is None:
            session_id = str(uuid.uuid4())

        # Almacenar en ChromaDB
        memory_result = self.store_memory(
            content=message,
            memory_type="chat_message",
            metadata={
                "role": role,
                "session_id": session_id,
                "timestamp": datetime.now().isoformat(),
                **(context or {}),
            },
        )

        # Mantener en memoria local para sesiÃ³n activa
        if session_id not in self.chat_sessions:
            self.chat_sessions[session_id] = []

        self.chat_sessions[session_id].append(
            {
                "message": message,
                "role": role,
                "timestamp": datetime.now().isoformat(),
                "memory_id": memory_result.get("memory_id"),
            }
        )

        return {
            "success": memory_result["success"],
            "session_id": session_id,
            "memory_id": memory_result.get("memory_id"),
            "role": role,
            "timestamp": datetime.now().isoformat(),
        }

    def get_chat_history(self, session_id: str, limit: int = 50) -> Dict[str, Any]:
        """
        Obtener historial de chat.

        Args:
            session_id: ID de sesiÃ³n
            limit: LÃ­mite de mensajes

        Returns:
            Historial de chat
        """
        try:
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory

            chroma = get_chroma_memory()
            results = chroma.search_memories(
                query=session_id, memory_types=["chat_message"], limit=limit
            )

            # Filtrar por session_id y ordenar
            messages = []
            for result in results:
                metadata = result["metadata"]
                if metadata.get("session_id") == session_id:
                    messages.append(
                        {
                            "message": result["content"],
                            "role": metadata["role"],
                            "timestamp": metadata.get("timestamp"),
                            "memory_id": result["id"],
                        }
                    )

            # Ordenar por timestamp
            messages.sort(key=lambda x: x.get("timestamp", ""))

            return {
                "success": True,
                "session_id": session_id,
                "messages": messages,
                "total_messages": len(messages),
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "messages": [],
                "total_messages": 0,
            }

    def get_context_for_action(self, action: str, goal: str = "") -> Dict[str, Any]:
        """
        Obtener contexto relevante para una acciÃ³n.

        Args:
            action: AcciÃ³n a realizar
            goal: Objetivo principal

        Returns:
            Contexto relevante
        """
        try:
            from modules.humanoid.cortex.unified_memory import \
                get_unified_memory

            memory = get_unified_memory()
            context = memory.get_context_for_action(action, goal)

            return {"success": True, "action": action, "goal": goal, "context": context}

        except Exception as e:
            return {"success": False, "error": str(e), "context": {}}

    def create_task_chain(
        self, goal: str, context: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Crear cadena de tareas usando LangFlow.

        Args:
            goal: Objetivo principal
            context: Contexto adicional

        Returns:
            InformaciÃ³n de la cadena creada
        """
        try:
            from modules.humanoid.orchestrator.langflow_orchestrator import \
                get_langflow_orchestrator

            orchestrator = get_langflow_orchestrator()
            chain_info = orchestrator.create_task_chain(goal, context)

            return {
                "success": True,
                "task_id": chain_info["task_id"],
                "goal": goal,
                "flow_name": chain_info["flow_name"],
                "status": chain_info["status"],
            }

        except Exception as e:
            return {"success": False, "error": str(e), "task_id": None}

    def execute_task_chain(
        self, task_id: str, inputs: Dict[str, Any] = None
    ) -> Dict[str, Any]:
        """
        Ejecutar cadena de tareas.

        Args:
            task_id: ID de la tarea
            inputs: Inputs adicionales

        Returns:
            Resultados de ejecuciÃ³n
        """
        try:
            from modules.humanoid.orchestrator.langflow_orchestrator import \
                get_langflow_orchestrator

            orchestrator = get_langflow_orchestrator()
            result = orchestrator.execute_task_chain(task_id, inputs)

            return {
                "success": result["status"] != "error",
                "task_id": task_id,
                "status": result["status"],
                "results": result.get("results"),
                "executed_at": result.get("executed_at"),
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "task_id": task_id,
                "status": "error",
            }

    def get_memory_statistics(self) -> Dict[str, Any]:
        """Obtener estadÃ­sticas de todos los sistemas de memoria."""
        try:
            from modules.humanoid.cortex.unified_memory import \
                get_unified_memory
            from modules.humanoid.memory_engine.chroma_memory import \
                get_chroma_memory
            from modules.humanoid.orchestrator.langflow_orchestrator import \
                get_langflow_orchestrator

            # EstadÃ­sticas Unified Memory
            unified_memory = get_unified_memory()
            unified_stats = unified_memory.get_all_stats()

            # EstadÃ­sticas ChromaDB
            chroma = get_chroma_memory()
            chroma_stats = chroma.get_statistics()

            # EstadÃ­sticas LangFlow
            orchestrator = get_langflow_orchestrator()
            langflow_stats = orchestrator.get_statistics()

            return {
                "success": True,
                "unified_memory": unified_stats,
                "chromadb": chroma_stats,
                "langflow": langflow_stats,
                "chat_sessions": len(self.chat_sessions),
                "connector_status": "active",
            }

        except Exception as e:
            return {"success": False, "error": str(e), "connector_status": "error"}


# Instancia global del conector
memory_connector = WorkspaceMemoryConnector()


# Endpoints de la API
@memory_router.post("/search")
async def search_memory(query: MemoryQuery):
    """Buscar en memoria."""
    result = memory_connector.search_memory(
        query=query.query,
        memory_types=query.memory_types,
        limit=query.limit,
        min_relevance=query.min_relevance,
    )

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


@memory_router.post("/store")
async def store_memory(memory: MemoryStore):
    """Almacenar en memoria."""
    result = memory_connector.store_memory(
        content=memory.content, memory_type=memory.memory_type, metadata=memory.metadata
    )

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


@memory_router.post("/chat")
async def store_chat_message(message: ChatMessage):
    """Almacenar mensaje de chat."""
    result = memory_connector.store_chat_message(
        message=message.message,
        role=message.role,
        session_id=message.session_id,
        context=message.context,
    )

    if not result["success"]:
        raise HTTPException(status_code=500, detail="Error storing chat message")

    return result


@memory_router.get("/chat/{session_id}")
async def get_chat_history(session_id: str, limit: int = 50):
    """Obtener historial de chat."""
    result = memory_connector.get_chat_history(session_id, limit)

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


@memory_router.post("/context")
async def get_context_for_action(action: str, goal: str = ""):
    """Obtener contexto para acciÃ³n."""
    result = memory_connector.get_context_for_action(action, goal)

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


@memory_router.post("/task/create")
async def create_task_chain(goal: str, context: Dict[str, Any] = None):
    """Crear cadena de tareas."""
    result = memory_connector.create_task_chain(goal, context)

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


@memory_router.post("/task/{task_id}/execute")
async def execute_task_chain(task_id: str, inputs: Dict[str, Any] = None):
    """Ejecutar cadena de tareas."""
    result = memory_connector.execute_task_chain(task_id, inputs)

    if not result["success"]:
        raise HTTPException(
            status_code=500, detail=result.get("error", "Execution failed")
        )

    return result


@memory_router.get("/statistics")
async def get_memory_statistics():
    """Obtener estadÃ­sticas de memoria."""
    result = memory_connector.get_memory_statistics()

    if not result["success"]:
        raise HTTPException(status_code=500, detail=result["error"])

    return result


# Exportar para uso en main.py
__all__ = ["memory_router", "memory_connector"]


