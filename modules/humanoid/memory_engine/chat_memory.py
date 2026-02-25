"""
Chat Memory Engine - Persistencia inteligente de conversaciones
Integrado con ChromaDB y Unified Memory Cortex
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from modules.humanoid.memory_engine.chroma_memory import get_chroma_memory


class ChatMemoryEngine:
    """
    Motor de persistencia de conversaciones con capacidades avanzadas.

    Características:
    - Sesiones persistentes con ID único
    - Resumen automático de conversaciones largas
    - Búsqueda semántica en historial
    - Contexto relevante para nuevas consultas
    - Integración con Unified Memory Cortex
    """

    def __init__(self):
        self.chroma = get_chroma_memory()
        self.active_sessions: Dict[str, Dict] = {}
        self.max_messages_per_session = 100
        self.summary_threshold = 20  # Mensajes antes de resumir

        print("✅ Chat Memory Engine iniciado")

    def create_session(
        self, user_id: str = None, context: Dict[str, Any] = None
    ) -> str:
        """
        Crear nueva sesión de chat.

        Args:
            user_id: ID del usuario (opcional)
            context: Contexto inicial de la sesión

        Returns:
            ID de sesión creado
        """
        session_id = str(uuid.uuid4())

        session_info = {
            "session_id": session_id,
            "user_id": user_id,
            "created_at": datetime.now().isoformat(),
            "last_activity": datetime.now().isoformat(),
            "message_count": 0,
            "context": context or {},
            "status": "active",
        }

        # Guardar en ChromaDB
        self.chroma.add_memory(
            content=json.dumps(session_info),
            memory_type="chat_session",
            metadata={
                "session_id": session_id,
                "user_id": user_id,
                "created_at": session_info["created_at"],
                "type": "session_info",
            },
        )

        # Mantener en memoria activa
        self.active_sessions[session_id] = session_info

        return session_id

    def add_message(
        self,
        session_id: str,
        message: str,
        role: str,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Añadir mensaje a sesión.

        Args:
            session_id: ID de sesión
            message: Contenido del mensaje
            role: Rol (user/assistant/system)
            metadata: Metadatos adicionales

        Returns:
            Resultado de la operación
        """
        # Verificar si existe sesión activa
        if session_id not in self.active_sessions:
            self._load_session(session_id)

        if session_id not in self.active_sessions:
            return {
                "success": False,
                "error": "Session not found",
                "session_id": session_id,
            }

        # Crear mensaje
        message_data = {
            "session_id": session_id,
            "message": message,
            "role": role,
            "timestamp": datetime.now().isoformat(),
            "message_number": self.active_sessions[session_id]["message_count"] + 1,
            **(metadata or {}),
        }

        # Guardar en ChromaDB
        memory_id = self.chroma.add_memory(
            content=message, memory_type="chat_message", metadata=message_data
        )

        # Actualizar sesión
        self.active_sessions[session_id]["message_count"] += 1
        self.active_sessions[session_id]["last_activity"] = message_data["timestamp"]

        # Verificar si necesita resumen
        if self.active_sessions[session_id]["message_count"] >= self.summary_threshold:
            self._create_summary(session_id)

        return {
            "success": True,
            "session_id": session_id,
            "message_id": memory_id,
            "message_number": message_data["message_number"],
            "timestamp": message_data["timestamp"],
        }

    def get_session_history(
        self, session_id: str, limit: int = 50, include_summaries: bool = True
    ) -> Dict[str, Any]:
        """
        Obtener historial de sesión.

        Args:
            session_id: ID de sesión
            limit: Límite de mensajes
            include_summaries: Incluir resúmenes

        Returns:
            Historial de la sesión
        """
        try:
            # Buscar mensajes de la sesión
            results = self.chroma.search_memories(
                query=session_id,
                memory_types=["chat_message"],
                limit=limit * 2,  # Buscar más para filtrar
            )

            # Filtrar y ordenar mensajes
            messages = []
            for result in results:
                metadata = result["metadata"]
                if metadata.get("session_id") == session_id:
                    messages.append(
                        {
                            "message": result["content"],
                            "role": metadata["role"],
                            "timestamp": metadata.get("timestamp"),
                            "message_number": metadata.get("message_number", 0),
                            "memory_id": result["id"],
                        }
                    )

            # Ordenar por número de mensaje
            messages.sort(key=lambda x: x["message_number"])

            # Limitar resultados
            messages = messages[-limit:]

            # Obtener resúmenes si se solicitan
            summaries = []
            if include_summaries:
                summary_results = self.chroma.search_memories(
                    query=f"{session_id} summary",
                    memory_types=["chat_summary"],
                    limit=10,
                )

                for result in summary_results:
                    metadata = result["metadata"]
                    if metadata.get("session_id") == session_id:
                        summaries.append(
                            {
                                "summary": result["content"],
                                "created_at": metadata.get("created_at"),
                                "message_range": metadata.get("message_range", []),
                            }
                        )

            # Obtener info de sesión
            session_info = self.active_sessions.get(session_id, {})
            if not session_info:
                session_info = self._get_session_info(session_id)

            return {
                "success": True,
                "session_id": session_id,
                "session_info": session_info,
                "messages": messages,
                "summaries": summaries,
                "total_messages": len(messages),
                "total_summaries": len(summaries),
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "session_id": session_id,
                "messages": [],
                "summaries": [],
            }

    def search_conversations(
        self,
        query: str,
        user_id: str = None,
        limit: int = 20,
        date_range: Tuple[str, str] = None,
    ) -> Dict[str, Any]:
        """
        Buscar en conversaciones anteriores.

        Args:
            query: Query de búsqueda
            user_id: Filtrar por usuario
            limit: Límite de resultados
            date_range: Rango de fechas (start, end)

        Returns:
            Resultados de búsqueda
        """
        try:
            # Construir metadata filters
            where_clause = {"memory_type": "chat_message"}

            if user_id:
                where_clause["user_id"] = user_id

            # Búsqueda en ChromaDB
            results = self.chroma.search_memories(query, limit=limit)

            # Filtrar y procesar resultados
            conversations = []
            for result in results:
                metadata = result["metadata"]

                # Filtrar por usuario si se especifica
                if user_id and metadata.get("user_id") != user_id:
                    continue

                # Filtrar por rango de fechas si se especifica
                if date_range:
                    timestamp = metadata.get("timestamp")
                    if timestamp:
                        msg_date = datetime.fromisoformat(
                            timestamp.replace("Z", "+00:00")
                        )
                        start_date = datetime.fromisoformat(
                            date_range[0].replace("Z", "+00:00")
                        )
                        end_date = datetime.fromisoformat(
                            date_range[1].replace("Z", "+00:00")
                        )

                        if not (start_date <= msg_date <= end_date):
                            continue

                conversations.append(
                    {
                        "message": result["content"],
                        "role": metadata.get("role", "unknown"),
                        "session_id": metadata.get("session_id", "unknown"),
                        "timestamp": metadata.get("timestamp"),
                        "message_number": metadata.get("message_number", 0),
                        "relevance_score": result["score"],
                        "memory_id": result["id"],
                    }
                )

            # Agrupar por sesión
            by_session = {}
            for conv in conversations:
                session_id = conv["session_id"]
                if session_id not in by_session:
                    by_session[session_id] = {
                        "session_id": session_id,
                        "messages": [],
                        "max_relevance": 0,
                    }

                by_session[session_id]["messages"].append(conv)
                by_session[session_id]["max_relevance"] = max(
                    by_session[session_id]["max_relevance"], conv["relevance_score"]
                )

            # Ordenar sesiones por relevancia
            sorted_sessions = sorted(
                by_session.values(), key=lambda x: x["max_relevance"], reverse=True
            )

            return {
                "success": True,
                "query": query,
                "results": sorted_sessions[:limit],
                "total_sessions": len(sorted_sessions),
                "total_messages": len(conversations),
            }

        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "results": [],
                "total_sessions": 0,
                "total_messages": 0,
            }

    def get_context_for_new_message(
        self, session_id: str, new_message: str, context_limit: int = 10
    ) -> Dict[str, Any]:
        """
        Obtener contexto relevante para un nuevo mensaje.

        Args:
            session_id: ID de sesión
            new_message: Nuevo mensaje
            context_limit: Límite de mensajes de contexto

        Returns:
            Contexto relevante
        """
        try:
            # Obtener historial reciente
            history_result = self.get_session_history(
                session_id, limit=context_limit, include_summaries=False
            )

            if not history_result["success"]:
                return {
                    "success": False,
                    "error": "Could not retrieve session history",
                    "context": [],
                }

            recent_messages = history_result["messages"]

            # Buscar mensajes relevantes en todo el historial
            search_result = self.search_conversations(query=new_message, limit=5)

            relevant_context = []
            if search_result["success"]:
                for session in search_result["results"]:
                    if session["session_id"] == session_id:
                        relevant_context.extend(
                            session["messages"][:2]
                        )  # Top 2 por sesión

            # Combinar contexto
            context = {
                "recent_messages": recent_messages,
                "relevant_messages": relevant_context,
                "session_info": history_result["session_info"],
                "total_context_messages": len(recent_messages) + len(relevant_context),
            }

            return {
                "success": True,
                "session_id": session_id,
                "new_message": new_message,
                "context": context,
            }

        except Exception as e:
            return {"success": False, "error": str(e), "context": []}

    def _load_session(self, session_id: str):
        """Cargar sesión desde ChromaDB."""
        try:
            results = self.chroma.search_memories(
                query=session_id, memory_types=["chat_session"], limit=1
            )

            if results:
                for result in results:
                    metadata = result["metadata"]
                    if (
                        metadata.get("session_id") == session_id
                        and metadata.get("type") == "session_info"
                    ):
                        session_info = json.loads(result["content"])
                        self.active_sessions[session_id] = session_info
                        break
        except Exception:
            pass

    def _get_session_info(self, session_id: str) -> Dict[str, Any]:
        """Obtener información de sesión."""
        try:
            results = self.chroma.search_memories(
                query=session_id, memory_types=["chat_session"], limit=1
            )

            if results:
                for result in results:
                    metadata = result["metadata"]
                    if (
                        metadata.get("session_id") == session_id
                        and metadata.get("type") == "session_info"
                    ):
                        return json.loads(result["content"])
        except Exception:
            pass

        return {}

    def _create_summary(self, session_id: str):
        """Crear resumen de conversación larga."""
        try:
            # Obtener mensajes para resumir
            history = self.get_session_history(
                session_id, limit=self.summary_threshold, include_summaries=False
            )

            if (
                not history["success"]
                or len(history["messages"]) < self.summary_threshold
            ):
                return

            messages = history["messages"]

            # Crear resumen simple (esto podría usar LLM en el futuro)
            summary_content = self._generate_simple_summary(messages)

            # Guardar resumen
            message_range = [
                messages[0]["message_number"],
                messages[-1]["message_number"],
            ]

            self.chroma.add_memory(
                content=summary_content,
                memory_type="chat_summary",
                metadata={
                    "session_id": session_id,
                    "created_at": datetime.now().isoformat(),
                    "message_range": message_range,
                    "message_count": len(messages),
                },
            )

            print(
                f"✅ Resumen creado para sesión {session_id} ({len(messages)} mensajes)"
            )

        except Exception as e:
            print(f"❌ Error creando resumen: {e}")

    def _generate_simple_summary(self, messages: List[Dict]) -> str:
        """Generar resumen simple de conversación."""
        user_messages = [m for m in messages if m["role"] == "user"]
        assistant_messages = [m for m in messages if m["role"] == "assistant"]

        summary_parts = [
            f"Conversación con {len(user_messages)} mensajes del usuario y {len(assistant_messages)} respuestas del asistente.",
            f"Rango de mensajes: {messages[0]['message_number']} a {messages[-1]['message_number']}.",
        ]

        # Extraer temas principales (simple heuristic)
        all_text = " ".join([m["message"] for m in messages])
        topics = []

        # Palabras clave simples
        keywords = [
            "implementar",
            "desarrollar",
            "crear",
            "solucionar",
            "analizar",
            "diseñar",
        ]
        for keyword in keywords:
            if keyword in all_text.lower():
                topics.append(keyword)

        if topics:
            summary_parts.append(f"Temas principales: {', '.join(topics)}")

        return " ".join(summary_parts)

    def get_statistics(self) -> Dict[str, Any]:
        """Obtener estadísticas del motor de chat."""
        try:
            # Estadísticas de ChromaDB
            chroma_stats = self.chroma.get_statistics()

            # Estadísticas específicas de chat
            chat_memories = self.chroma.search_memories(
                query="chat_message", memory_types=["chat_message"], limit=1000
            )

            session_memories = self.chroma.search_memories(
                query="chat_session", memory_types=["chat_session"], limit=1000
            )

            summary_memories = self.chroma.search_memories(
                query="chat_summary", memory_types=["chat_summary"], limit=1000
            )

            # Contar sesiones activas
            active_sessions = len(self.active_sessions)

            # Analizar distribución por rol
            role_counts = {}
            for memory in chat_memories:
                role = memory["metadata"].get("role", "unknown")
                role_counts[role] = role_counts.get(role, 0) + 1

            return {
                "total_chat_messages": len(chat_memories),
                "total_sessions": len(session_memories),
                "total_summaries": len(summary_memories),
                "active_sessions": active_sessions,
                "role_distribution": role_counts,
                "chromadb_stats": chroma_stats,
                "status": "active",
            }

        except Exception as e:
            return {
                "status": "error",
                "error": str(e),
                "total_chat_messages": 0,
                "total_sessions": 0,
                "active_sessions": 0,
            }


# Instancia global
_instance: Optional[ChatMemoryEngine] = None


def get_chat_memory() -> ChatMemoryEngine:
    """Obtener instancia global del motor de chat."""
    global _instance
    if _instance is None:
        _instance = ChatMemoryEngine()
    return _instance


if __name__ == "__main__":
    # Test básico
    chat_engine = ChatMemoryEngine()

    # Crear sesión
    session_id = chat_engine.create_session(user_id="test_user")
    print(f"Sesión creada: {session_id}")

    # Añadir mensajes
    chat_engine.add_message(session_id, "Hola, necesito ayuda con ChromaDB", "user")
    chat_engine.add_message(
        session_id, "Claro, ¿qué necesitas saber sobre ChromaDB?", "assistant"
    )

    # Obtener historial
    history = chat_engine.get_session_history(session_id)
    print(f"Historial: {history['total_messages']} mensajes")

    # Estadísticas
    stats = chat_engine.get_statistics()
    print(f"Estadísticas: {stats}")
