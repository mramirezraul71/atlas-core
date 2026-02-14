"""
ATLAS NEXUS - Short Term Memory
Memoria de corto plazo (conversaciones recientes)
"""

import logging
from typing import List, Dict, Optional
from collections import deque
from datetime import datetime

logger = logging.getLogger(__name__)


class ShortTermMemory:
    """Memoria de corto plazo para conversaciones"""
    
    def __init__(self, max_size: int = 20):
        """
        Args:
            max_size: Número máximo de mensajes a recordar
        """
        self.max_size = max_size
        self.messages: deque = deque(maxlen=max_size)
    
    def add_message(self, role: str, content: str, metadata: Optional[Dict] = None):
        """
        Agrega un mensaje a la memoria
        
        Args:
            role: 'user' o 'assistant'
            content: Contenido del mensaje
            metadata: Información adicional
        """
        message = {
            'role': role,
            'content': content,
            'timestamp': datetime.now().isoformat(),
            'metadata': metadata or {}
        }
        
        self.messages.append(message)
        logger.debug(f"Mensaje agregado a memoria: {role}")
    
    def get_history(self, last_n: Optional[int] = None) -> List[Dict]:
        """
        Obtiene historia de conversación
        
        Args:
            last_n: Número de mensajes recientes (None = todos)
            
        Returns:
            Lista de mensajes
        """
        if last_n is None:
            return list(self.messages)
        return list(self.messages)[-last_n:]
    
    def clear(self):
        """Limpia la memoria"""
        self.messages.clear()
        logger.info("Memoria de corto plazo limpiada")
    
    def get_context_summary(self) -> str:
        """Resume el contexto de la conversación"""
        if not self.messages:
            return "Sin historial de conversación"
        
        total = len(self.messages)
        user_msgs = sum(1 for m in self.messages if m['role'] == 'user')
        
        return (f"Conversación: {total} mensajes "
                f"({user_msgs} del usuario)")


# Singleton
_short_term_memory: Optional[ShortTermMemory] = None

def get_short_term_memory() -> ShortTermMemory:
    """Obtiene instancia de memoria de corto plazo"""
    global _short_term_memory
    if _short_term_memory is None:
        _short_term_memory = ShortTermMemory()
    return _short_term_memory
