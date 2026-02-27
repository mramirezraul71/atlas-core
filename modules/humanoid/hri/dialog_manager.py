"""
Dialog Manager: Gestión de diálogos.
=====================================
Mantiene el estado de conversaciones.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

from .intent_parser import Intent, IntentType

_log = logging.getLogger("humanoid.hri.dialog")


class DialogState(str, Enum):
    """Estados del diálogo."""
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    CONFIRMING = "confirming"
    EXECUTING = "executing"
    ERROR = "error"


@dataclass
class DialogTurn:
    """Un turno de diálogo."""
    speaker: str  # user, robot
    text: str
    intent: Optional[Intent] = None
    timestamp: float = 0.0


@dataclass
class DialogContext:
    """Contexto de la conversación."""
    state: DialogState = DialogState.IDLE
    turns: List[DialogTurn] = field(default_factory=list)
    pending_action: Optional[Dict[str, Any]] = None
    slots: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "state": self.state.value,
            "turns_count": len(self.turns),
            "pending_action": self.pending_action,
            "slots": self.slots,
        }


class DialogManager:
    """
    Gestor de diálogos para ATLAS.
    
    Funcionalidades:
    - Mantenimiento de contexto
    - Gestión de slots
    - Respuestas según intención
    - Confirmación de acciones
    """
    
    # Respuestas predefinidas
    RESPONSES = {
        IntentType.GREETING: [
            "¡Hola! ¿En qué puedo ayudarte?",
            "Buenos días, estoy listo para ayudar.",
            "¡Hey! ¿Qué necesitas?",
        ],
        IntentType.FAREWELL: [
            "¡Hasta luego!",
            "Nos vemos pronto.",
            "Adiós, que tengas un buen día.",
        ],
        IntentType.HELP: [
            "Puedo navegar a lugares, traer objetos, y seguirte. ¿Qué necesitas?",
            "Estoy aquí para ayudar. Puedes pedirme que vaya a algún lugar o te traiga algo.",
        ],
        IntentType.STATUS: [
            "Estoy funcionando correctamente. Todos mis sistemas están operativos.",
            "Todo bien por aquí. ¿Necesitas algo?",
        ],
        IntentType.CONFIRM: [
            "Entendido, procediendo.",
            "De acuerdo, ejecutando.",
        ],
        IntentType.DENY: [
            "Entendido, cancelando.",
            "Ok, no haré nada.",
        ],
        IntentType.UNKNOWN: [
            "No entendí bien. ¿Puedes repetirlo?",
            "Lo siento, no comprendí. ¿Podrías decirlo de otra forma?",
        ],
    }
    
    def __init__(self):
        self._context = DialogContext()
        self._response_index: Dict[IntentType, int] = {}
        
        _log.info("Dialog manager initialized")
    
    def process_intent(self, intent: Intent) -> str:
        """
        Procesa una intención y genera respuesta.
        
        Args:
            intent: Intención del usuario
            
        Returns:
            Respuesta del robot
        """
        import time
        
        # Añadir turno del usuario
        self._context.turns.append(DialogTurn(
            speaker="user",
            text=intent.text,
            intent=intent,
            timestamp=time.time(),
        ))
        
        # Procesar según tipo de intención
        response = self._generate_response(intent)
        
        # Añadir turno del robot
        self._context.turns.append(DialogTurn(
            speaker="robot",
            text=response,
            timestamp=time.time(),
        ))
        
        # Actualizar estado
        self._update_state(intent)
        
        return response
    
    def _generate_response(self, intent: Intent) -> str:
        """Genera respuesta según la intención."""
        
        # Intenciones de acción
        if intent.type == IntentType.NAVIGATE:
            destination = intent.slots.get("destination")
            if destination:
                self._context.pending_action = {
                    "type": "navigate",
                    "destination": destination,
                }
                self._context.state = DialogState.CONFIRMING
                return f"¿Quieres que vaya a {destination}?"
            else:
                return "¿A dónde quieres que vaya?"
        
        elif intent.type == IntentType.FETCH:
            obj = intent.slots.get("object")
            location = intent.slots.get("location")
            
            if obj:
                self._context.pending_action = {
                    "type": "fetch",
                    "object": obj,
                    "location": location,
                }
                if location:
                    self._context.state = DialogState.CONFIRMING
                    return f"¿Quieres que traiga {obj} de {location}?"
                else:
                    return f"¿Dónde está {obj}?"
            else:
                return "¿Qué quieres que te traiga?"
        
        elif intent.type == IntentType.FOLLOW:
            self._context.pending_action = {"type": "follow"}
            self._context.state = DialogState.CONFIRMING
            return "¿Quieres que te siga?"
        
        elif intent.type == IntentType.STOP:
            self._context.state = DialogState.IDLE
            self._context.pending_action = None
            return "Deteniéndome."
        
        elif intent.type == IntentType.CONFIRM:
            if self._context.pending_action:
                self._context.state = DialogState.EXECUTING
                action = self._context.pending_action
                self._context.pending_action = None
                return f"Ejecutando: {action['type']}."
            else:
                return "¿Qué quieres confirmar?"
        
        elif intent.type == IntentType.DENY:
            self._context.pending_action = None
            self._context.state = DialogState.IDLE
            return self._get_response(IntentType.DENY)
        
        # Respuestas genéricas
        return self._get_response(intent.type)
    
    def _get_response(self, intent_type: IntentType) -> str:
        """Obtiene una respuesta variada."""
        import random
        
        responses = self.RESPONSES.get(intent_type, self.RESPONSES[IntentType.UNKNOWN])
        
        # Rotar respuestas para variedad
        idx = self._response_index.get(intent_type, 0)
        response = responses[idx % len(responses)]
        self._response_index[intent_type] = idx + 1
        
        return response
    
    def _update_state(self, intent: Intent) -> None:
        """Actualiza el estado del diálogo."""
        if intent.type in (IntentType.GREETING, IntentType.HELP, IntentType.STATUS):
            self._context.state = DialogState.LISTENING
        
        # Limpiar historial antiguo
        if len(self._context.turns) > 100:
            self._context.turns = self._context.turns[-50:]
    
    def get_context(self) -> DialogContext:
        """Retorna el contexto actual."""
        return self._context
    
    def get_pending_action(self) -> Optional[Dict[str, Any]]:
        """Retorna acción pendiente de ejecución."""
        if self._context.state == DialogState.EXECUTING:
            return self._context.pending_action
        return None
    
    def clear_action(self) -> None:
        """Limpia la acción pendiente."""
        self._context.pending_action = None
        self._context.state = DialogState.IDLE
    
    def reset(self) -> None:
        """Resetea el gestor de diálogo."""
        self._context = DialogContext()
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "context": self._context.to_dict(),
            "recent_turns": [
                {"speaker": t.speaker, "text": t.text}
                for t in self._context.turns[-5:]
            ],
        }
