#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ATLAS Chat Thread Manager
Sistema para llevar el hilo de todos los chats y conversaciones
"""

import sqlite3
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any
import uuid

class ChatThreadManager:
    """Gestor centralizado de hilos de conversación"""
    
    def __init__(self, db_path: str = "C:\\ATLAS_PUSH\\data\\chat_threads.db"):
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._init_db()
    
    def _init_db(self):
        """Inicializar base de datos con esquema de chats"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        # Tabla de conversaciones (threads)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_threads (
                thread_id TEXT PRIMARY KEY,
                title TEXT NOT NULL,
                description TEXT,
                user_id TEXT,
                created_at TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                status TEXT DEFAULT 'active',
                context TEXT,
                metadata TEXT
            )
        """)
        
        # Tabla de mensajes
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_messages (
                message_id TEXT PRIMARY KEY,
                thread_id TEXT NOT NULL,
                sender TEXT NOT NULL,
                role TEXT NOT NULL,
                content TEXT NOT NULL,
                timestamp TEXT NOT NULL,
                metadata TEXT,
                FOREIGN KEY (thread_id) REFERENCES chat_threads(thread_id)
            )
        """)
        
        # Tabla de contexto (para mantener el hilo)
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_context (
                context_id TEXT PRIMARY KEY,
                thread_id TEXT NOT NULL,
                key TEXT NOT NULL,
                value TEXT NOT NULL,
                updated_at TEXT NOT NULL,
                FOREIGN KEY (thread_id) REFERENCES chat_threads(thread_id)
            )
        """)
        
        # Tabla de participantes
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS chat_participants (
                participant_id TEXT PRIMARY KEY,
                thread_id TEXT NOT NULL,
                user_id TEXT NOT NULL,
                role TEXT,
                joined_at TEXT NOT NULL,
                FOREIGN KEY (thread_id) REFERENCES chat_threads(thread_id)
            )
        """)
        
        # Indices para búsquedas rápidas
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_thread_user ON chat_threads(user_id)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_message_thread ON chat_messages(thread_id)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_message_timestamp ON chat_messages(timestamp)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_context_thread ON chat_context(thread_id)")
        
        conn.commit()
        conn.close()
    
    def create_thread(self, title: str, user_id: str, description: str = "", 
                     context: Dict = None) -> str:
        """Crear nuevo hilo de conversación"""
        thread_id = str(uuid.uuid4())
        now = datetime.utcnow().isoformat()
        
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        cursor.execute("""
            INSERT INTO chat_threads 
            (thread_id, title, description, user_id, created_at, updated_at, context, metadata)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (thread_id, title, description, user_id, now, now, 
              json.dumps(context or {}), json.dumps({})))
        
        conn.commit()
        conn.close()
        
        return thread_id
    
    def add_message(self, thread_id: str, sender: str, role: str, 
                   content: str, metadata: Dict = None) -> str:
        """Agregar mensaje a un hilo"""
        message_id = str(uuid.uuid4())
        now = datetime.utcnow().isoformat()
        
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        # Agregar mensaje
        cursor.execute("""
            INSERT INTO chat_messages 
            (message_id, thread_id, sender, role, content, timestamp, metadata)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        """, (message_id, thread_id, sender, role, content, now, 
              json.dumps(metadata or {})))
        
        # Actualizar timestamp del thread
        cursor.execute("""
            UPDATE chat_threads SET updated_at = ? WHERE thread_id = ?
        """, (now, thread_id))
        
        conn.commit()
        conn.close()
        
        return message_id
    
    def get_thread_history(self, thread_id: str, limit: int = 100) -> List[Dict]:
        """Obtener historial completo de un hilo"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        cursor.execute("""
            SELECT message_id, sender, role, content, timestamp, metadata
            FROM chat_messages
            WHERE thread_id = ?
            ORDER BY timestamp ASC
            LIMIT ?
        """, (thread_id, limit))
        
        messages = []
        for row in cursor.fetchall():
            messages.append({
                "message_id": row[0],
                "sender": row[1],
                "role": row[2],
                "content": row[3],
                "timestamp": row[4],
                "metadata": json.loads(row[5] or "{}")
            })
        
        conn.close()
        return messages

    def thread_exists(self, thread_id: str) -> bool:
        """Verifica si existe un hilo."""
        if not thread_id:
            return False
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        cursor.execute("SELECT 1 FROM chat_threads WHERE thread_id = ? LIMIT 1", (thread_id,))
        row = cursor.fetchone()
        conn.close()
        return bool(row)

    def get_recent_messages(self, thread_id: str, limit: int = 30) -> List[Dict]:
        """Obtener últimos mensajes de un hilo (orden cronológico ascendente)."""
        limit = max(1, min(int(limit or 30), 500))
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()

        cursor.execute(
            """
            SELECT message_id, sender, role, content, timestamp, metadata
            FROM chat_messages
            WHERE thread_id = ?
            ORDER BY timestamp DESC
            LIMIT ?
            """,
            (thread_id, limit),
        )
        rows = cursor.fetchall()
        conn.close()

        messages: List[Dict] = []
        for row in reversed(rows):
            messages.append({
                "message_id": row[0],
                "sender": row[1],
                "role": row[2],
                "content": row[3],
                "timestamp": row[4],
                "metadata": json.loads(row[5] or "{}")
            })
        return messages
    
    def get_all_threads(self, user_id: str = None) -> List[Dict]:
        """Obtener todos los hilos (opcionalmente filtrados por usuario)"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        if user_id:
            cursor.execute("""
                SELECT thread_id, title, description, user_id, created_at, updated_at, status
                FROM chat_threads
                WHERE user_id = ?
                ORDER BY updated_at DESC
            """, (user_id,))
        else:
            cursor.execute("""
                SELECT thread_id, title, description, user_id, created_at, updated_at, status
                FROM chat_threads
                ORDER BY updated_at DESC
            """)
        
        threads = []
        for row in cursor.fetchall():
            # Contar mensajes
            cursor.execute("SELECT COUNT(*) FROM chat_messages WHERE thread_id = ?", (row[0],))
            msg_count = cursor.fetchone()[0]
            
            threads.append({
                "thread_id": row[0],
                "title": row[1],
                "description": row[2],
                "user_id": row[3],
                "created_at": row[4],
                "updated_at": row[5],
                "status": row[6],
                "message_count": msg_count
            })
        
        conn.close()
        return threads
    
    def set_context(self, thread_id: str, key: str, value: Any) -> None:
        """Guardar contexto del hilo (variables, estado, etc)"""
        context_id = str(uuid.uuid4())
        now = datetime.utcnow().isoformat()
        
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        # Verificar si ya existe
        cursor.execute("""
            SELECT context_id FROM chat_context 
            WHERE thread_id = ? AND key = ?
        """, (thread_id, key))
        
        existing = cursor.fetchone()
        
        if existing:
            cursor.execute("""
                UPDATE chat_context 
                SET value = ?, updated_at = ?
                WHERE thread_id = ? AND key = ?
            """, (json.dumps(value), now, thread_id, key))
        else:
            cursor.execute("""
                INSERT INTO chat_context 
                (context_id, thread_id, key, value, updated_at)
                VALUES (?, ?, ?, ?, ?)
            """, (context_id, thread_id, key, json.dumps(value), now))
        
        conn.commit()
        conn.close()
    
    def get_context(self, thread_id: str, key: str = None) -> Dict:
        """Obtener contexto del hilo"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        if key:
            cursor.execute("""
                SELECT value FROM chat_context 
                WHERE thread_id = ? AND key = ?
            """, (thread_id, key))
            row = cursor.fetchone()
            conn.close()
            return json.loads(row[0]) if row else None
        else:
            cursor.execute("""
                SELECT key, value FROM chat_context 
                WHERE thread_id = ?
            """, (thread_id,))
            
            context = {}
            for row in cursor.fetchall():
                context[row[0]] = json.loads(row[1])
            
            conn.close()
            return context
    
    def search_threads(self, query: str) -> List[Dict]:
        """Buscar hilos por título o descripción"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        search_term = f"%{query}%"
        cursor.execute("""
            SELECT thread_id, title, description, user_id, created_at, updated_at, status
            FROM chat_threads
            WHERE title LIKE ? OR description LIKE ?
            ORDER BY updated_at DESC
        """, (search_term, search_term))
        
        threads = []
        for row in cursor.fetchall():
            threads.append({
                "thread_id": row[0],
                "title": row[1],
                "description": row[2],
                "user_id": row[3],
                "created_at": row[4],
                "updated_at": row[5],
                "status": row[6]
            })
        
        conn.close()
        return threads
    
    def get_stats(self) -> Dict:
        """Obtener estadísticas del sistema de chats"""
        conn = sqlite3.connect(str(self.db_path))
        cursor = conn.cursor()
        
        cursor.execute("SELECT COUNT(*) FROM chat_threads")
        total_threads = cursor.fetchone()[0]
        
        cursor.execute("SELECT COUNT(*) FROM chat_messages")
        total_messages = cursor.fetchone()[0]
        
        cursor.execute("SELECT COUNT(DISTINCT user_id) FROM chat_threads")
        total_users = cursor.fetchone()[0]
        
        cursor.execute("""
            SELECT COUNT(*) FROM chat_threads WHERE status = 'active'
        """)
        active_threads = cursor.fetchone()[0]
        
        conn.close()
        
        return {
            "total_threads": total_threads,
            "total_messages": total_messages,
            "total_users": total_users,
            "active_threads": active_threads
        }


# Ejemplo de uso
if __name__ == "__main__":
    manager = ChatThreadManager()
    
    # Crear un hilo
    thread_id = manager.create_thread(
        title="Consulta sobre ATLAS",
        user_id="user_001",
        description="Preguntas sobre configuracion del sistema"
    )
    print(f"Thread creado: {thread_id}")
    
    # Agregar mensajes
    manager.add_message(thread_id, "user_001", "user", "Hola, necesito ayuda con ATLAS")
    manager.add_message(thread_id, "atlas", "assistant", "Claro, estoy aqui para ayudarte. Que necesitas?")
    manager.add_message(thread_id, "user_001", "user", "Como configuro el sistema?")
    
    # Guardar contexto
    manager.set_context(thread_id, "topic", "configuration")
    manager.set_context(thread_id, "resolved", False)
    
    # Obtener historial
    history = manager.get_thread_history(thread_id)
    print(f"\nHistorial ({len(history)} mensajes):")
    for msg in history:
        print(f"  [{msg['role']}] {msg['sender']}: {msg['content'][:50]}...")
    
    # Estadísticas
    stats = manager.get_stats()
    print(f"\nEstadísticas: {stats}")
