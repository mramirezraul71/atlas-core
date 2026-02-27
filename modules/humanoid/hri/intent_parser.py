"""
Intent Parser: Interpretación de intenciones.
==============================================
Analiza texto para extraer intenciones y entidades.
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional

_log = logging.getLogger("humanoid.hri.intent")


class IntentType(str, Enum):
    """Tipos de intención."""
    UNKNOWN = "unknown"
    GREETING = "greeting"
    FAREWELL = "farewell"
    NAVIGATE = "navigate"
    FETCH = "fetch"
    FOLLOW = "follow"
    STOP = "stop"
    STATUS = "status"
    HELP = "help"
    CONFIRM = "confirm"
    DENY = "deny"
    REPEAT = "repeat"


@dataclass
class Intent:
    """Intención parseada."""
    type: IntentType
    confidence: float
    text: str
    entities: Dict[str, Any] = field(default_factory=dict)
    slots: Dict[str, str] = field(default_factory=dict)
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type.value,
            "confidence": self.confidence,
            "text": self.text,
            "entities": self.entities,
            "slots": self.slots,
        }


class IntentParser:
    """
    Parser de intenciones para ATLAS.
    
    Métodos:
    - Reglas basadas en patrones
    - Keywords matching
    - Integración con LLM (opcional)
    """
    
    # Patrones por intención
    INTENT_PATTERNS = {
        IntentType.GREETING: [
            r"\b(hola|hey|buenos?\s+d[ií]as?|buenas?\s+(tardes?|noches?))\b",
            r"\b(hi|hello|hey)\b",
        ],
        IntentType.FAREWELL: [
            r"\b(adi[oó]s|hasta\s+luego|chao|bye|nos\s+vemos)\b",
            r"\b(goodbye|see\s+you)\b",
        ],
        IntentType.NAVIGATE: [
            r"\b(ve\s+a|ir\s+a|navega|muévete|llévame)\b",
            r"\b(go\s+to|navigate|move\s+to)\b",
        ],
        IntentType.FETCH: [
            r"\b(trae|tr[áa]eme|busca|recoge|dame)\b",
            r"\b(fetch|bring|get\s+me|pick\s+up)\b",
        ],
        IntentType.FOLLOW: [
            r"\b(s[ií]gueme|ven\s+conmigo|acomp[aá][ñn]ame)\b",
            r"\b(follow\s+me|come\s+with)\b",
        ],
        IntentType.STOP: [
            r"\b(para|det[eé]nte|alto|espera|stop)\b",
            r"\b(stop|halt|wait|freeze)\b",
        ],
        IntentType.STATUS: [
            r"\b(c[oó]mo\s+est[aá]s|estado|qu[eé]\s+tal)\b",
            r"\b(status|how\s+are\s+you|state)\b",
        ],
        IntentType.HELP: [
            r"\b(ayuda|qu[eé]\s+puedes\s+hacer|help)\b",
            r"\b(help|what\s+can\s+you\s+do)\b",
        ],
        IntentType.CONFIRM: [
            r"\b(s[ií]|claro|por\s+supuesto|ok|vale|de\s+acuerdo)\b",
            r"\b(yes|sure|okay|alright|correct)\b",
        ],
        IntentType.DENY: [
            r"\b(no|nope|negativo|para\s+nada)\b",
            r"\b(no|nope|negative|wrong)\b",
        ],
        IntentType.REPEAT: [
            r"\b(repite|otra\s+vez|no\s+entend[ií]|qu[eé]\s+dijiste)\b",
            r"\b(repeat|again|what\s+did\s+you\s+say|pardon)\b",
        ],
    }
    
    # Patrones de entidades
    ENTITY_PATTERNS = {
        "location": r"\b(cocina|sala|dormitorio|ba[ñn]o|oficina|garaje|jard[ií]n)\b",
        "object": r"\b(vaso|taza|libro|control|tel[eé]fono|llaves|bot[eé]lla)\b",
        "person": r"\b(yo|t[uú]|[eé]l|ella|nosotros|ellos)\b",
        "number": r"\b(\d+)\b",
        "direction": r"\b(izquierda|derecha|adelante|atr[aá]s|arriba|abajo)\b",
    }
    
    def __init__(self):
        self._context: Dict[str, Any] = {}
        self._last_intent: Optional[Intent] = None
    
    def parse(self, text: str) -> Intent:
        """
        Parsea texto para extraer intención.
        
        Args:
            text: Texto de entrada
            
        Returns:
            Intención parseada
        """
        text_lower = text.lower().strip()
        
        # Buscar intención por patrones
        best_intent = IntentType.UNKNOWN
        best_confidence = 0.0
        
        for intent_type, patterns in self.INTENT_PATTERNS.items():
            for pattern in patterns:
                if re.search(pattern, text_lower, re.IGNORECASE):
                    confidence = 0.8  # Base confidence
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_intent = intent_type
        
        # Extraer entidades
        entities = self._extract_entities(text_lower)
        
        # Extraer slots específicos
        slots = self._extract_slots(text_lower, best_intent)
        
        intent = Intent(
            type=best_intent,
            confidence=best_confidence,
            text=text,
            entities=entities,
            slots=slots,
        )
        
        self._last_intent = intent
        _log.debug("Parsed intent: %s (%.2f)", intent.type.value, intent.confidence)
        
        return intent
    
    def _extract_entities(self, text: str) -> Dict[str, Any]:
        """Extrae entidades del texto."""
        entities = {}
        
        for entity_type, pattern in self.ENTITY_PATTERNS.items():
            matches = re.findall(pattern, text, re.IGNORECASE)
            if matches:
                entities[entity_type] = matches if len(matches) > 1 else matches[0]
        
        return entities
    
    def _extract_slots(self, text: str, intent: IntentType) -> Dict[str, str]:
        """Extrae slots específicos de la intención."""
        slots = {}
        
        if intent == IntentType.NAVIGATE:
            # Buscar destino
            location_match = re.search(self.ENTITY_PATTERNS["location"], text)
            if location_match:
                slots["destination"] = location_match.group(0)
            
            direction_match = re.search(self.ENTITY_PATTERNS["direction"], text)
            if direction_match:
                slots["direction"] = direction_match.group(0)
        
        elif intent == IntentType.FETCH:
            # Buscar objeto
            object_match = re.search(self.ENTITY_PATTERNS["object"], text)
            if object_match:
                slots["object"] = object_match.group(0)
            
            location_match = re.search(self.ENTITY_PATTERNS["location"], text)
            if location_match:
                slots["location"] = location_match.group(0)
        
        return slots
    
    def set_context(self, key: str, value: Any) -> None:
        """Establece contexto para futuras interpretaciones."""
        self._context[key] = value
    
    def get_context(self, key: str) -> Any:
        """Obtiene valor del contexto."""
        return self._context.get(key)
    
    def clear_context(self) -> None:
        """Limpia el contexto."""
        self._context.clear()
    
    def get_last_intent(self) -> Optional[Intent]:
        return self._last_intent
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "last_intent": self._last_intent.to_dict() if self._last_intent else None,
            "context_keys": list(self._context.keys()),
        }
