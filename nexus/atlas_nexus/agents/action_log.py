"""
ATLAS NEXUS - Action Log
Panel tipo Cursor: registro en vivo de pip install, comandos, acciones autónomas
"""

from datetime import datetime
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass
import json
import logging

logger = logging.getLogger("atlas.actions")


@dataclass
class ActionEntry:
    """Una entrada del log de acciones"""
    id: str
    timestamp: str
    action_type: str  # pip, cmd, file, heal, etc.
    description: str
    command: Optional[str] = None
    output: str = ""
    status: str = "running"  # running | ok | error
    details: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "timestamp": self.timestamp,
            "action_type": self.action_type,
            "description": self.description,
            "command": self.command,
            "output": self.output,
            "status": self.status,
            "details": self.details or {},
        }


class ActionLog:
    """
    Log central de acciones autónomas.
    Almacena y emite vía callback (WebSocket) todo lo que hace el sistema.
    """

    _instance: Optional["ActionLog"] = None

    def __new__(cls) -> "ActionLog":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._entries = []
            cls._instance._max_entries = 200
            cls._instance._broadcast_cb = None
            cls._instance._counter = 0
        return cls._instance

    def set_broadcast(self, callback: Callable[[Dict], None]) -> None:
        """Registra callback para enviar por WebSocket"""
        self._broadcast_cb = callback

    def _next_id(self) -> str:
        self._counter += 1
        return f"act_{self._counter}_{datetime.now().strftime('%H%M%S')}"

    def start(self, action_type: str, description: str, command: Optional[str] = None) -> str:
        """Inicia una acción. Devuelve id para actualizar luego."""
        entry = ActionEntry(
            id=self._next_id(),
            timestamp=datetime.now().isoformat(),
            action_type=action_type,
            description=description,
            command=command,
            output="",
            status="running",
        )
        self._entries.append(entry)
        while len(self._entries) > self._max_entries:
            self._entries.pop(0)
        self._emit(entry, "start")
        return entry.id

    def append_output(self, action_id: str, text: str) -> None:
        """Añade salida (stdout/stderr) a una acción en curso"""
        for e in reversed(self._entries):
            if e.id == action_id:
                e.output = (e.output or "") + text
                self._emit(e, "output")
                return

    def finish(self, action_id: str, status: str = "ok", output: str = "", details: Optional[Dict] = None) -> None:
        """Marca una acción como terminada"""
        for e in reversed(self._entries):
            if e.id == action_id:
                e.status = status
                if output:
                    e.output = (e.output or "") + output
                if details:
                    e.details = details
                self._emit(e, "finish")
                return

    def log(self, action_type: str, description: str, command: Optional[str] = None,
            status: str = "ok", output: str = "", details: Optional[Dict] = None) -> str:
        """Registro rápido de acción ya completada"""
        action_id = self.start(action_type, description, command)
        self.finish(action_id, status=status, output=output, details=details)
        return action_id

    def _emit(self, entry: ActionEntry, event: str) -> None:
        payload = {
            "type": "action",
            "event": event,
            "data": entry.to_dict(),
        }
        logger.info(f"Action {event}: {entry.action_type} - {entry.description[:50]}...")
        if self._broadcast_cb:
            try:
                self._broadcast_cb(payload)
            except Exception as ex:
                logger.warning(f"Broadcast failed: {ex}")

    def get_entries(self, limit: int = 50) -> List[Dict[str, Any]]:
        """Últimas entradas para GET /actions/log"""
        return [e.to_dict() for e in self._entries[-limit:]]

    def clear(self) -> None:
        """Limpia el log (mantiene último para referencia)"""
        if self._entries:
            self._entries = self._entries[-10:]
        logger.info("Action log cleared")


# Singleton global
action_log = ActionLog()
