"""
ATLAS NEXUS - Memory Integration
Conecta el agente principal con el sistema de memoria de Atlas
"""
from __future__ import annotations

import json
import logging
from typing import Any, Dict, List, Optional
from datetime import datetime

logger = logging.getLogger("atlas.brain.memory")


class NexusMemoryIntegration:
    """
    Integración entre ATLAS NEXUS y el sistema de memoria de Atlas.
    
    Provee:
    - Búsqueda en memoria antes de responder
    - Almacenamiento de conversaciones
    - Contexto relevante para tareas
    - Persistencia de estado
    """
    
    def __init__(self):
        self.memory_enabled = True
        self.session_id: Optional[str] = None
        self.user_id: Optional[str] = None
        
        # Intentar cargar sistemas de memoria
        self._load_memory_systems()
        
        logger.info("✅ Nexus Memory Integration initialized")
    
    def _load_memory_systems(self):
        """Cargar sistemas de memoria con manejo de errores."""
        try:
            # Importar Chat Memory
            from modules.humanoid.memory_engine.chat_memory import get_chat_memory
            self.chat_memory = get_chat_memory()
            logger.info("  ✅ Chat Memory loaded")
        except Exception as e:
            logger.warning(f"  ⚠️ Chat Memory not available: {e}")
            self.chat_memory = None
        
        try:
            # Importar Unified Memory Cortex
            from modules.humanoid.cortex.unified_memory import get_unified_memory
            self.unified_memory = get_unified_memory()
            logger.info("  ✅ Unified Memory Cortex loaded")
        except Exception as e:
            logger.warning(f"  ⚠️ Unified Memory Cortex not available: {e}")
            self.unified_memory = None
        
        try:
            # Importar Memory Connector
            from nexus.atlas_nexus_robot.backend.brain.memory_connector import memory_connector
            self.memory_connector = memory_connector
            logger.info("  ✅ Memory Connector loaded")
        except Exception as e:
            logger.warning(f"  ⚠️ Memory Connector not available: {e}")
            self.memory_connector = None
    
    def set_session(self, user_id: str = None, session_id: str = None):
        """Establecer sesión actual."""
        self.user_id = user_id
        if session_id:
            self.session_id = session_id
        elif self.chat_memory:
            self.session_id = self.chat_memory.create_session(user_id=user_id)
        
        logger.info(f"Session set: user={user_id}, session={self.session_id}")
    
    def search_memory(self, query: str, context_type: str = "general") -> Dict[str, Any]:
        """
        Buscar en memoria antes de procesar una consulta.
        
        Args:
            query: Query del usuario
            context_type: Tipo de contexto (conversation, task, analysis)
        
        Returns:
            Contexto encontrado en memoria
        """
        if not self.memory_enabled:
            return {"found": False, "reason": "Memory disabled"}
        
        context = {
            "found": False,
            "query": query,
            "context_type": context_type,
            "relevant_memories": [],
            "chat_history": [],
            "task_context": {}
        }
        
        # 1. Buscar en Unified Memory Cortex
        if self.unified_memory:
            try:
                memory_types = ["episodic", "semantic", "procedural"]
                if context_type == "conversation":
                    memory_types.append("chat_message")
                
                results = self.unified_memory.recall(
                    query=query,
                    memory_types=memory_types,
                    limit=10,
                    min_relevance=0.3
                )
                
                if results:
                    context["found"] = True
                    context["relevant_memories"] = [r.to_dict() for r in results[:5]]
                    logger.info(f"Found {len(results)} memories in Unified Memory")
            except Exception as e:
                logger.warning(f"Unified Memory search failed: {e}")
        
        # 2. Buscar en Chat Memory para conversaciones
        if self.chat_memory and context_type == "conversation":
            try:
                # Buscar conversaciones previas
                search_results = self.chat_memory.search_conversations(
                    query=query,
                    user_id=self.user_id,
                    limit=5
                )
                
                if search_results["success"] and search_results["total_sessions"] > 0:
                    context["found"] = True
                    context["chat_history"] = search_results["results"][:3]
                    logger.info(f"Found {search_results['total_sessions']} relevant chat sessions")
                
                # Obtener contexto de sesión actual
                if self.session_id:
                    session_context = self.chat_memory.get_context_for_new_message(
                        session_id=self.session_id,
                        new_message=query,
                        context_limit=5
                    )
                    
                    if session_context["success"]:
                        context["current_session"] = session_context["context"]
            except Exception as e:
                logger.warning(f"Chat Memory search failed: {e}")
        
        # 3. Buscar contexto para tareas
        if context_type in ["task", "analysis"] and self.memory_connector:
            try:
                task_context = self.memory_connector.get_context_for_action(
                    action=query,
                    goal=""
                )
                
                if task_context["success"]:
                    context["found"] = True
                    context["task_context"] = task_context["context"]
            except Exception as e:
                logger.warning(f"Task context search failed: {e}")
        
        return context
    
    def store_interaction(self, user_message: str, assistant_response: str, 
                         metadata: Dict[str, Any] = None):
        """
        Almacenar interacción en memoria.
        
        Args:
            user_message: Mensaje del usuario
            assistant_response: Respuesta del asistente
            metadata: Metadatos adicionales
        """
        if not self.memory_enabled or not self.chat_memory:
            return
        
        try:
            # Almacenar mensaje del usuario
            self.chat_memory.add_message(
                session_id=self.session_id,
                message=user_message,
                role="user",
                metadata=metadata
            )
            
            # Almacenar respuesta del asistente
            self.chat_memory.add_message(
                session_id=self.session_id,
                message=assistant_response,
                role="assistant",
                metadata=metadata
            )
            
            logger.debug(f"Stored interaction in session {self.session_id}")
        except Exception as e:
            logger.warning(f"Failed to store interaction: {e}")
    
    def store_task_result(self, task: str, result: Any, status: str = "completed"):
        """Almacenar resultado de tarea en memoria."""
        if not self.memory_enabled or not self.memory_connector:
            return
        
        try:
            # Almacenar en ChromaDB
            self.memory_connector.store_memory(
                content=f"Task: {task}\nResult: {str(result)[:500]}",
                memory_type="task_result",
                metadata={
                    "task": task,
                    "status": status,
                    "timestamp": datetime.now().isoformat(),
                    "user_id": self.user_id
                }
            )
            
            logger.info(f"Stored task result: {task}")
        except Exception as e:
            logger.warning(f"Failed to store task result: {e}")
    
    def get_memory_summary(self) -> Dict[str, Any]:
        """Obtener resumen del estado de la memoria."""
        summary = {
            "memory_enabled": self.memory_enabled,
            "session_id": self.session_id,
            "user_id": self.user_id,
            "systems": {}
        }
        
        # Estadísticas de Chat Memory
        if self.chat_memory:
            try:
                summary["systems"]["chat_memory"] = self.chat_memory.get_statistics()
            except Exception as e:
                summary["systems"]["chat_memory"] = {"error": str(e)}
        
        # Estadísticas de Unified Memory
        if self.unified_memory:
            try:
                summary["systems"]["unified_memory"] = self.unified_memory.get_all_stats()
            except Exception as e:
                summary["systems"]["unified_memory"] = {"error": str(e)}
        
        # Estadísticas de Memory Connector
        if self.memory_connector:
            try:
                summary["systems"]["memory_connector"] = self.memory_connector.get_memory_statistics()
            except Exception as e:
                summary["systems"]["memory_connector"] = {"error": str(e)}
        
        return summary
    
    def enhance_prompt_with_memory(self, prompt: str, context_type: str = "general") -> str:
        """
        Enhance prompt con contexto de memoria.
        
        Args:
            prompt: Prompt original
            context_type: Tipo de contexto
        
        Returns:
            Prompt enhanced con memoria
        """
        if not self.memory_enabled:
            return prompt
        
        # Buscar contexto relevante
        memory_context = self.search_memory(prompt, context_type)
        
        if not memory_context["found"]:
            return prompt
        
        enhanced_prompt = prompt
        
        # Añadir contexto de memoria relevante
        if memory_context["relevant_memories"]:
            enhanced_prompt += "\n\n--- MEMORIA RELEVANTE ---\n"
            for i, memory in enumerate(memory_context["relevant_memories"][:3]):
                enhanced_prompt += f"{i+1}. [{memory['memory_type']}] {memory['content'][:200]}...\n"
        
        # Añadir historial de chat si aplica
        if memory_context.get("chat_history"):
            enhanced_prompt += "\n--- CONVERSACIONES ANTERIORES ---\n"
            for session in memory_context["chat_history"][:2]:
                enhanced_prompt += f"Sesión {session['session_id'][-8:]}:\n"
                for msg in session["messages"][:3]:
                    role = "👤" if msg["role"] == "user" else "🤖"
                    enhanced_prompt += f"  {role} {msg['message'][:100]}...\n"
        
        # Añadir contexto de tareas
        if memory_context.get("task_context"):
            task_ctx = memory_context["task_context"]
            if task_ctx.get("relevant_memories"):
                enhanced_prompt += "\n--- CONTEXTO DE TAREAS ---\n"
                for memory in task_ctx["relevant_memories"][:2]:
                    enhanced_prompt += f"• {memory['content'][:150]}...\n"
        
        return enhanced_prompt


# Instancia global para uso en todo NEXUS
_memory_integration: Optional[NexusMemoryIntegration] = None


def get_memory_integration() -> NexusMemoryIntegration:
    """Obtener instancia global de integración de memoria."""
    global _memory_integration
    if _memory_integration is None:
        _memory_integration = NexusMemoryIntegration()
    return _memory_integration


def enhance_agent_response_with_memory(query: str, response: str, 
                                     context_type: str = "conversation") -> str:
    """
    Enhance respuesta del agente con información de memoria.
    
    Args:
        query: Query original del usuario
        response: Respuesta generada
        context_type: Tipo de contexto
    
    Returns:
        Respuesta enhanced con memoria
    """
    memory_integration = get_memory_integration()
    
    # Si la respuesta es "no recuerdo", buscar en memoria
    if "no tengo acceso" in response.lower() or "no recuerdo" in response.lower():
        memory_context = memory_integration.search_memory(query, context_type)
        
        if memory_context["found"]:
            # Construir respuesta basada en memoria
            memory_response = f"Basándome en mi memoria:\n\n"
            
            # Añadir memorias relevantes
            if memory_context["relevant_memories"]:
                memory_response += "**Información relevante:**\n"
                for memory in memory_context["relevant_memories"][:3]:
                    memory_response += f"• {memory['content'][:200]}...\n"
            
            # Añadir conversaciones anteriores
            if memory_context.get("chat_history"):
                memory_response += "\n**Conversaciones anteriores:**\n"
                for session in memory_context["chat_history"][:2]:
                    memory_response += f"En sesión anterior: {session['messages'][0]['message'][:100]}...\n"
            
            # Almacenar esta interacción
            memory_integration.store_interaction(query, memory_response)
            
            return memory_response
    
    # Almacenar interacción normal
    memory_integration.store_interaction(query, response)
    
    return response


if __name__ == "__main__":
    # Test de integración
    integration = NexusMemoryIntegration()
    integration.set_session(user_id="test_user")
    
    # Buscar memoria
    context = integration.search_memory("tarea que estabas haciendo", "conversation")
    print(f"Memory search: {context['found']}")
    
    # Enhance prompt
    enhanced = integration.enhance_prompt_with_memory("¿Qué estabas haciendo?")
    print(f"Enhanced prompt length: {len(enhanced)}")
    
    # Estadísticas
    summary = integration.get_memory_summary()
    print(f"Memory systems: {list(summary['systems'].keys())}")
