"""
NaturalLanguageFeedback: Integracion de feedback humano en lenguaje natural.

Permite mejorar comportamientos mediante instrucciones verbales.
"""
from __future__ import annotations

import logging
import re
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


class FeedbackType:
    """Tipos de feedback reconocidos."""
    CORRECTION = "correction"      # "No, hazlo asi"
    PREFERENCE = "preference"      # "Prefiero que sea mas lento"
    PROHIBITION = "prohibition"    # "No hagas eso"
    CONFIRMATION = "confirmation"  # "Si, exacto"
    REWARD = "reward"              # "Muy bien!"
    PUNISHMENT = "punishment"      # "Mal hecho"
    INSTRUCTION = "instruction"    # "Ahora haz X"


@dataclass
class FeedbackIntent:
    """Intencion extraida del feedback."""
    feedback_type: str
    target: Optional[str] = None  # Que se esta corrigiendo/prefiriendo
    instruction: Optional[str] = None  # Instruccion especifica
    parameters: Dict[str, Any] = field(default_factory=dict)
    sentiment: float = 0.0  # -1 (negativo) a 1 (positivo)
    confidence: float = 1.0
    raw_text: str = ""
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "feedback_type": self.feedback_type,
            "target": self.target,
            "instruction": self.instruction,
            "parameters": self.parameters,
            "sentiment": self.sentiment,
            "confidence": self.confidence,
        }


@dataclass
class FeedbackRecord:
    """Registro de feedback recibido."""
    id: str
    timestamp_ns: int
    intent: FeedbackIntent
    context: Dict[str, Any]
    applied: bool = False
    result: Optional[str] = None


class NaturalLanguageFeedback:
    """
    Sistema de feedback en lenguaje natural.
    
    Permite:
    - Parsear feedback humano
    - Aplicar correcciones
    - Actualizar preferencias
    - Agregar prohibiciones
    """
    
    # Patrones de feedback
    PATTERNS = {
        FeedbackType.CORRECTION: [
            r"(?:no,?\s*)?(?:hazlo|haerlo)\s+(.+)",
            r"(?:mejor|prefiero)\s+(?:que\s+)?(.+)",
            r"(?:asi\s+no|no\s+asi),?\s*(.+)?",
            r"(?:corrige|corrije)\s+(.+)",
        ],
        FeedbackType.PROHIBITION: [
            r"(?:no\s+(?:hagas|toques|muevas))\s+(.+)",
            r"(?:nunca|jamas)\s+(.+)",
            r"(?:prohibido|no\s+permitido)\s+(.+)",
            r"(?:deja\s+de|para\s+de)\s+(.+)",
        ],
        FeedbackType.PREFERENCE: [
            r"(?:prefiero|me\s+gusta\s+mas)\s+(?:que\s+)?(.+)",
            r"(?:mas\s+(?:lento|rapido|suave|fuerte))",
            r"(?:hazlo\s+(?:mas|menos))\s+(.+)",
        ],
        FeedbackType.REWARD: [
            r"(?:muy\s+bien|excelente|perfecto|genial|bravo)",
            r"(?:asi\s+(?:es|esta\s+bien)|correcto)",
            r"(?:buen\s+trabajo|bien\s+hecho)",
        ],
        FeedbackType.PUNISHMENT: [
            r"(?:mal|incorrecto|error|fallo)",
            r"(?:no\s+esta\s+bien|asi\s+no)",
            r"(?:eso\s+no|no\s+es\s+asi)",
        ],
        FeedbackType.CONFIRMATION: [
            r"(?:si|correcto|exacto|afirmativo)",
            r"(?:esta\s+bien|ok|vale|de\s+acuerdo)",
        ],
        FeedbackType.INSTRUCTION: [
            r"(?:ahora\s+)?(?:haz|realiza|ejecuta)\s+(.+)",
            r"(?:mueve|gira|agarra|suelta)\s+(.+)",
        ],
    }
    
    def __init__(self, llm_client: Any = None):
        """
        Inicializa el sistema de feedback.
        
        Args:
            llm_client: Cliente LLM opcional para parsing avanzado
        """
        self.llm_client = llm_client
        
        # Historial de feedback
        self._history: List[FeedbackRecord] = []
        
        # Preferencias aprendidas
        self._preferences: Dict[str, Any] = {}
        
        # Prohibiciones activas
        self._prohibitions: List[Dict[str, Any]] = []
        
        # Callbacks para aplicar feedback
        self._correction_handlers: List[Callable[[FeedbackIntent, Dict], None]] = []
        self._preference_handlers: List[Callable[[str, Any], None]] = []
        self._prohibition_handlers: List[Callable[[str], None]] = []
        
        # Contador de IDs
        self._feedback_count = 0
    
    def parse_feedback(self, text: str, context: Dict[str, Any] = None) -> FeedbackIntent:
        """
        Parsea feedback en lenguaje natural.
        
        Args:
            text: Texto del feedback
            context: Contexto actual (accion reciente, estado, etc.)
        
        Returns:
            FeedbackIntent con tipo e instruccion
        """
        text_lower = text.lower().strip()
        context = context or {}
        
        # Intentar matching con patrones
        for feedback_type, patterns in self.PATTERNS.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    groups = match.groups()
                    instruction = groups[0] if groups else None
                    
                    # Determinar sentimiento
                    sentiment = self._analyze_sentiment(feedback_type)
                    
                    return FeedbackIntent(
                        feedback_type=feedback_type,
                        target=context.get("last_action"),
                        instruction=instruction,
                        sentiment=sentiment,
                        confidence=0.8,
                        raw_text=text,
                    )
        
        # Fallback: analizar sentimiento basico
        sentiment = self._basic_sentiment(text_lower)
        
        if sentiment > 0.3:
            feedback_type = FeedbackType.REWARD
        elif sentiment < -0.3:
            feedback_type = FeedbackType.PUNISHMENT
        else:
            feedback_type = FeedbackType.INSTRUCTION
        
        return FeedbackIntent(
            feedback_type=feedback_type,
            instruction=text,
            sentiment=sentiment,
            confidence=0.5,
            raw_text=text,
        )
    
    def _analyze_sentiment(self, feedback_type: str) -> float:
        """Determina sentimiento basado en tipo de feedback."""
        sentiment_map = {
            FeedbackType.REWARD: 0.8,
            FeedbackType.CONFIRMATION: 0.5,
            FeedbackType.PREFERENCE: 0.2,
            FeedbackType.CORRECTION: -0.2,
            FeedbackType.INSTRUCTION: 0.0,
            FeedbackType.PUNISHMENT: -0.7,
            FeedbackType.PROHIBITION: -0.5,
        }
        return sentiment_map.get(feedback_type, 0.0)
    
    def _basic_sentiment(self, text: str) -> float:
        """Analisis de sentimiento basico."""
        positive_words = {"bien", "bueno", "perfecto", "excelente", "genial", 
                         "correcto", "si", "gracias", "bravo"}
        negative_words = {"mal", "no", "error", "incorrecto", "fallo", 
                         "nunca", "prohibido", "para"}
        
        words = set(text.split())
        positive = len(words & positive_words)
        negative = len(words & negative_words)
        
        total = positive + negative
        if total == 0:
            return 0.0
        
        return (positive - negative) / total
    
    async def process_feedback(self, text: str, 
                              context: Dict[str, Any] = None) -> FeedbackRecord:
        """
        Procesa feedback y aplica acciones correspondientes.
        
        Args:
            text: Texto del feedback
            context: Contexto actual
        
        Returns:
            Registro del feedback procesado
        """
        context = context or {}
        
        # Parsear
        intent = self.parse_feedback(text, context)
        
        # Crear registro
        self._feedback_count += 1
        record = FeedbackRecord(
            id=f"fb_{self._feedback_count}",
            timestamp_ns=time.time_ns(),
            intent=intent,
            context=context,
        )
        
        # Aplicar segun tipo
        try:
            if intent.feedback_type == FeedbackType.CORRECTION:
                await self._apply_correction(intent, context)
                record.applied = True
                record.result = "Correction applied"
                
            elif intent.feedback_type == FeedbackType.PREFERENCE:
                self._update_preference(intent)
                record.applied = True
                record.result = "Preference updated"
                
            elif intent.feedback_type == FeedbackType.PROHIBITION:
                self._add_prohibition(intent)
                record.applied = True
                record.result = "Prohibition added"
                
            elif intent.feedback_type in (FeedbackType.REWARD, FeedbackType.PUNISHMENT):
                # Estos se usan para actualizar reward model
                record.applied = True
                record.result = f"Feedback recorded (sentiment: {intent.sentiment:.2f})"
                
            elif intent.feedback_type == FeedbackType.CONFIRMATION:
                record.applied = True
                record.result = "Confirmation acknowledged"
                
            else:
                record.result = "Feedback recorded"
                
        except Exception as e:
            logger.error(f"Error applying feedback: {e}")
            record.result = f"Error: {e}"
        
        # Guardar en historial
        self._history.append(record)
        
        return record
    
    async def _apply_correction(self, intent: FeedbackIntent, 
                               context: Dict[str, Any]) -> None:
        """Aplica correccion."""
        logger.info(f"Applying correction: {intent.instruction}")
        
        for handler in self._correction_handlers:
            try:
                if hasattr(handler, "__call__"):
                    handler(intent, context)
            except Exception as e:
                logger.error(f"Error in correction handler: {e}")
    
    def _update_preference(self, intent: FeedbackIntent) -> None:
        """Actualiza preferencia."""
        if not intent.instruction:
            return
        
        # Extraer preferencia
        pref_text = intent.instruction.lower()
        
        # Detectar preferencias comunes
        speed_match = re.search(r"(mas|menos)\s+(lento|rapido)", pref_text)
        if speed_match:
            direction = speed_match.group(1)
            quality = speed_match.group(2)
            
            current = self._preferences.get("speed_factor", 1.0)
            if "lento" in quality:
                delta = 0.2 if direction == "mas" else -0.2
            else:
                delta = 0.2 if direction == "mas" else -0.2
                delta = -delta  # Invertir para rapido
            
            self._preferences["speed_factor"] = max(0.2, min(2.0, current + delta))
        
        # Notificar handlers
        for handler in self._preference_handlers:
            try:
                handler(intent.instruction, intent.parameters)
            except Exception as e:
                logger.error(f"Error in preference handler: {e}")
        
        logger.info(f"Updated preference: {intent.instruction}")
    
    def _add_prohibition(self, intent: FeedbackIntent) -> None:
        """Agrega prohibicion."""
        if not intent.instruction and not intent.target:
            return
        
        prohibition = {
            "target": intent.target or intent.instruction,
            "instruction": intent.instruction,
            "timestamp_ns": time.time_ns(),
        }
        
        self._prohibitions.append(prohibition)
        
        # Notificar handlers
        for handler in self._prohibition_handlers:
            try:
                handler(intent.instruction or intent.target)
            except Exception as e:
                logger.error(f"Error in prohibition handler: {e}")
        
        logger.info(f"Added prohibition: {prohibition['target']}")
    
    def is_prohibited(self, action: str) -> bool:
        """Verifica si una accion esta prohibida."""
        action_lower = action.lower()
        
        for prohibition in self._prohibitions:
            target = prohibition.get("target", "").lower()
            instruction = prohibition.get("instruction", "").lower()
            
            if target in action_lower or action_lower in target:
                return True
            if instruction and (instruction in action_lower or action_lower in instruction):
                return True
        
        return False
    
    def get_preference(self, key: str, default: Any = None) -> Any:
        """Obtiene preferencia aprendida."""
        return self._preferences.get(key, default)
    
    def clear_prohibitions(self) -> None:
        """Limpia todas las prohibiciones."""
        self._prohibitions.clear()
    
    def on_correction(self, handler: Callable[[FeedbackIntent, Dict], None]) -> None:
        """Registra handler para correcciones."""
        self._correction_handlers.append(handler)
    
    def on_preference(self, handler: Callable[[str, Any], None]) -> None:
        """Registra handler para preferencias."""
        self._preference_handlers.append(handler)
    
    def on_prohibition(self, handler: Callable[[str], None]) -> None:
        """Registra handler para prohibiciones."""
        self._prohibition_handlers.append(handler)
    
    def get_recent_feedback(self, limit: int = 10) -> List[Dict[str, Any]]:
        """Obtiene feedback reciente."""
        return [
            {
                "id": r.id,
                "type": r.intent.feedback_type,
                "sentiment": r.intent.sentiment,
                "applied": r.applied,
                "result": r.result,
            }
            for r in self._history[-limit:]
        ]
    
    def get_stats(self) -> Dict[str, Any]:
        """Obtiene estadisticas."""
        by_type = {}
        for record in self._history:
            ft = record.intent.feedback_type
            by_type[ft] = by_type.get(ft, 0) + 1
        
        return {
            "total_feedback": len(self._history),
            "by_type": by_type,
            "preferences_count": len(self._preferences),
            "prohibitions_count": len(self._prohibitions),
            "avg_sentiment": sum(r.intent.sentiment for r in self._history) / len(self._history) if self._history else 0,
        }
