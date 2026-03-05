# -*- coding: utf-8 -*-
"""
ATLAS Conversation Thread Manager
Sistema de autoprogramación para llevar el hilo de todas las conversaciones
"""

import hashlib
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional


class ConversationThread:
    """Representa un hilo de conversación único"""

    def __init__(self, thread_id: str, title: str = ""):
        self.thread_id = thread_id
        self.title = title
        self.created_at = datetime.now().isoformat()
        self.updated_at = datetime.now().isoformat()
        self.messages: List[Dict[str, Any]] = []
        self.context: Dict[str, Any] = {}
        self.tags: List[str] = []
        self.status = "active"  # active, paused, archived
        self.priority = "normal"  # low, normal, high, critical
        self.related_threads: List[str] = []
        self.action_items: List[Dict[str, Any]] = []

    def add_message(self, role: str, content: str, metadata: Optional[Dict] = None):
        """Añade un mensaje al hilo"""
        msg = {
            "timestamp": datetime.now().isoformat(),
            "role": role,  # user, assistant, system
            "content": content,
            "metadata": metadata or {},
        }
        self.messages.append(msg)
        self.updated_at = datetime.now().isoformat()
        return msg

    def get_context_summary(self) -> str:
        """Retorna un resumen del contexto actual"""
        summary = f"Thread: {self.title}\n"
        summary += f"Status: {self.status} | Priority: {self.priority}\n"
        summary += f"Messages: {len(self.messages)}\n"
        summary += f"Tags: {', '.join(self.tags)}\n"
        if self.action_items:
            summary += f"Pending Actions: {len(self.action_items)}\n"
        return summary

    def to_dict(self) -> Dict:
        """Convierte el hilo a diccionario"""
        return {
            "thread_id": self.thread_id,
            "title": self.title,
            "created_at": self.created_at,
            "updated_at": self.updated_at,
            "messages": self.messages,
            "context": self.context,
            "tags": self.tags,
            "status": self.status,
            "priority": self.priority,
            "related_threads": self.related_threads,
            "action_items": self.action_items,
        }


class ConversationThreadManager:
    """Gestor central de hilos de conversación"""

    def __init__(self, storage_dir: str = r"C:\ATLAS_PUSH\memory\threads"):
        self.storage_dir = Path(storage_dir)
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        self.threads: Dict[str, ConversationThread] = {}
        self.index_file = self.storage_dir / "index.json"
        self.load_all_threads()

    def create_thread(self, title: str = "", tags: List[str] = None) -> str:
        """Crea un nuevo hilo de conversación"""
        thread_id = self._generate_thread_id(title)
        thread = ConversationThread(thread_id, title)
        if tags:
            thread.tags = tags
        self.threads[thread_id] = thread
        self.save_thread(thread_id)
        return thread_id

    def _generate_thread_id(self, title: str) -> str:
        """Genera un ID único para el hilo"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        title_hash = hashlib.md5(title.encode()).hexdigest()[:8]
        return f"thread_{timestamp}_{title_hash}"

    def get_thread(self, thread_id: str) -> Optional[ConversationThread]:
        """Obtiene un hilo por ID"""
        if thread_id in self.threads:
            return self.threads[thread_id]
        return self.load_thread(thread_id)

    def add_message(
        self, thread_id: str, role: str, content: str, metadata: Optional[Dict] = None
    ):
        """Añade un mensaje a un hilo"""
        thread = self.get_thread(thread_id)
        if not thread:
            raise ValueError(f"Thread {thread_id} not found")
        thread.add_message(role, content, metadata)
        self.save_thread(thread_id)

    def save_thread(self, thread_id: str):
        """Guarda un hilo a disco"""
        thread = self.threads.get(thread_id)
        if not thread:
            return

        thread_file = self.storage_dir / f"{thread_id}.json"
        with open(thread_file, "w", encoding="utf-8") as f:
            json.dump(thread.to_dict(), f, indent=2, ensure_ascii=False)

        self.update_index()

    def load_thread(self, thread_id: str) -> Optional[ConversationThread]:
        """Carga un hilo desde disco"""
        thread_file = self.storage_dir / f"{thread_id}.json"
        if not thread_file.exists():
            return None

        with open(thread_file, "r", encoding="utf-8") as f:
            data = json.load(f)

        thread = ConversationThread(data["thread_id"], data["title"])
        thread.created_at = data["created_at"]
        thread.updated_at = data["updated_at"]
        thread.messages = data["messages"]
        thread.context = data["context"]
        thread.tags = data["tags"]
        thread.status = data["status"]
        thread.priority = data["priority"]
        thread.related_threads = data["related_threads"]
        thread.action_items = data["action_items"]

        self.threads[thread_id] = thread
        return thread

    def load_all_threads(self):
        """Carga todos los hilos desde disco"""
        for thread_file in self.storage_dir.glob("thread_*.json"):
            thread_id = thread_file.stem
            self.load_thread(thread_id)

    def update_index(self):
        """Actualiza el índice de hilos"""
        index = {
            "total_threads": len(self.threads),
            "updated_at": datetime.now().isoformat(),
            "threads": {},
        }

        for thread_id, thread in self.threads.items():
            index["threads"][thread_id] = {
                "title": thread.title,
                "status": thread.status,
                "priority": thread.priority,
                "tags": thread.tags,
                "message_count": len(thread.messages),
                "updated_at": thread.updated_at,
            }

        with open(self.index_file, "w", encoding="utf-8") as f:
            json.dump(index, f, indent=2, ensure_ascii=False)

    def get_active_threads(self) -> List[ConversationThread]:
        """Retorna todos los hilos activos"""
        return [t for t in self.threads.values() if t.status == "active"]

    def get_threads_by_tag(self, tag: str) -> List[ConversationThread]:
        """Retorna hilos con una etiqueta específica"""
        return [t for t in self.threads.values() if tag in t.tags]

    def get_threads_by_priority(self, priority: str) -> List[ConversationThread]:
        """Retorna hilos con una prioridad específica"""
        return [t for t in self.threads.values() if t.priority == priority]

    def get_context_snapshot(self) -> Dict[str, Any]:
        """Retorna un snapshot del contexto actual de todos los hilos"""
        return {
            "timestamp": datetime.now().isoformat(),
            "total_threads": len(self.threads),
            "active_threads": len(self.get_active_threads()),
            "threads_by_priority": {
                "critical": len(self.get_threads_by_priority("critical")),
                "high": len(self.get_threads_by_priority("high")),
                "normal": len(self.get_threads_by_priority("normal")),
                "low": len(self.get_threads_by_priority("low")),
            },
            "recent_threads": [
                {
                    "id": t.thread_id,
                    "title": t.title,
                    "updated_at": t.updated_at,
                    "status": t.status,
                }
                for t in sorted(
                    self.threads.values(), key=lambda x: x.updated_at, reverse=True
                )[:5]
            ],
        }


class AutoProgrammer:
    """Sistema de autoprogramación basado en contexto de conversaciones"""

    def __init__(self, thread_manager: ConversationThreadManager):
        self.thread_manager = thread_manager
        self.scheduled_tasks: List[Dict[str, Any]] = []
        self.completed_tasks: List[Dict[str, Any]] = []

    def analyze_thread_for_actions(self, thread_id: str) -> List[Dict[str, Any]]:
        """Analiza un hilo para identificar acciones a ejecutar"""
        thread = self.thread_manager.get_thread(thread_id)
        if not thread:
            return []

        actions = []

        # Buscar palabras clave que indiquen acciones
        keywords = {
            "ejecutar": "execute",
            "correr": "execute",
            "hacer": "execute",
            "crear": "create",
            "generar": "generate",
            "guardar": "save",
            "enviar": "send",
            "actualizar": "update",
            "eliminar": "delete",
            "revisar": "review",
            "analizar": "analyze",
        }

        for msg in thread.messages:
            content = msg["content"].lower()
            for keyword, action_type in keywords.items():
                if keyword in content:
                    actions.append(
                        {
                            "thread_id": thread_id,
                            "action_type": action_type,
                            "trigger_message": msg["content"][:100],
                            "timestamp": msg["timestamp"],
                            "status": "pending",
                        }
                    )

        return actions

    def schedule_action(self, action: Dict[str, Any]):
        """Programa una acción para ejecución"""
        action["scheduled_at"] = datetime.now().isoformat()
        action["id"] = hashlib.md5(
            f"{action['thread_id']}{action['action_type']}{action['scheduled_at']}".encode()
        ).hexdigest()[:12]
        self.scheduled_tasks.append(action)

    def get_pending_actions(self) -> List[Dict[str, Any]]:
        """Retorna todas las acciones pendientes"""
        return [t for t in self.scheduled_tasks if t["status"] == "pending"]

    def mark_action_complete(self, action_id: str, result: Dict[str, Any] = None):
        """Marca una acción como completada"""
        for task in self.scheduled_tasks:
            if task["id"] == action_id:
                task["status"] = "completed"
                task["completed_at"] = datetime.now().isoformat()
                task["result"] = result or {}
                self.completed_tasks.append(task)
                self.scheduled_tasks.remove(task)
                break


# Instancia global
_manager = None


def get_thread_manager() -> ConversationThreadManager:
    """Obtiene la instancia global del gestor de hilos"""
    global _manager
    if _manager is None:
        _manager = ConversationThreadManager()
    return _manager


def get_autoprogrammer() -> AutoProgrammer:
    """Obtiene la instancia global del autoprogramador"""
    return AutoProgrammer(get_thread_manager())
