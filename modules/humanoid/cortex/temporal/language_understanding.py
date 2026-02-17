"""
LanguageUnderstanding: Comprensión del lenguaje del lóbulo temporal.

Análogo biológico: Área de Wernicke + corteza temporal anterior
- NLU (Natural Language Understanding)
- Extracción de intenciones y entidades
- Comprensión contextual
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class Entity:
    """Entidad extraída del texto."""
    entity_type: str  # "object", "location", "person", "number", "time"
    value: str
    original_text: str
    confidence: float = 1.0
    start_pos: int = 0
    end_pos: int = 0


@dataclass
class Intent:
    """Intención extraída del texto."""
    intent_type: str  # "command", "query", "greeting", "confirmation", "negation"
    action: str  # "fetch", "move", "speak", "observe", "stop", etc.
    entities: List[Entity] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    raw_text: str = ""
    
    def get_entity(self, entity_type: str) -> Optional[Entity]:
        """Obtiene la primera entidad de un tipo."""
        for entity in self.entities:
            if entity.entity_type == entity_type:
                return entity
        return None
    
    def get_entities(self, entity_type: str) -> List[Entity]:
        """Obtiene todas las entidades de un tipo."""
        return [e for e in self.entities if e.entity_type == entity_type]
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "intent_type": self.intent_type,
            "action": self.action,
            "entities": [
                {"type": e.entity_type, "value": e.value, "confidence": e.confidence}
                for e in self.entities
            ],
            "parameters": self.parameters,
            "confidence": self.confidence,
        }


class PatternMatcher:
    """
    Extractor de patrones basado en reglas.
    
    Usa expresiones regulares para extraer intenciones y entidades.
    """
    
    # Patrones de acciones
    ACTION_PATTERNS = {
        "fetch": [
            r"(?:trae|traeme|dame|busca|recoge|agarra|coge|fetch|get|bring|grab)\s+(?:la |el |un |una )?(.+?)(?:\s+de\s+|\s+from\s+)?(.+)?$",
        ],
        "place": [
            r"(?:pon|coloca|deja|guarda|place|put|store)\s+(?:la |el )?(.+?)\s+(?:en|on|at|in)\s+(.+)",
        ],
        "move": [
            r"(?:ve|muevete|camina|navega|go|move|walk|navigate)\s+(?:a|hacia|to|towards)\s+(.+)",
            r"(?:acercate|alejate|approach|retreat)\s+(?:a|de|to|from)\s+(.+)",
        ],
        "look": [
            r"(?:mira|observa|busca|encuentra|look|watch|find|locate)\s+(?:la |el |a )?(.+)",
        ],
        "speak": [
            r"(?:di|dile|habla|repite|say|tell|speak|repeat)\s+(.+)",
        ],
        "stop": [
            r"(?:para|detente|alto|stop|halt|freeze)",
        ],
        "wait": [
            r"(?:espera|aguarda|wait|hold)\s*(?:(\d+)\s*(?:segundos?|seconds?|minutos?|minutes?))?",
        ],
        "greet": [
            r"(?:hola|buenos|buenas|hello|hi|hey)\s*(.+)?",
        ],
        "query": [
            r"(?:que es|donde esta|cuanto|como|quien|what is|where is|how|who)\s+(.+)\??",
        ],
    }
    
    # Patrones de entidades
    ENTITY_PATTERNS = {
        "location": [
            r"(?:la |el )?(?:mesa|silla|cocina|sala|puerta|ventana|cama|escritorio|table|chair|kitchen|door|window|desk)",
            r"(?:aqui|alli|alla|aca|here|there)",
        ],
        "object": [
            r"(?:la |el |un |una )?(?:taza|vaso|botella|libro|control|telefono|llave|cup|glass|bottle|book|remote|phone|key)",
        ],
        "person": [
            r"(?:el |la )?(?:usuario|persona|humano|user|person|human)",
            r"(?:yo|mi|me|I|my)",
        ],
        "color": [
            r"(?:rojo|azul|verde|amarillo|negro|blanco|red|blue|green|yellow|black|white)",
        ],
        "number": [
            r"(\d+(?:\.\d+)?)",
        ],
    }
    
    def match_action(self, text: str) -> Optional[tuple]:
        """
        Busca acción en el texto.
        
        Returns:
            (action, groups) o None
        """
        text_lower = text.lower().strip()
        
        for action, patterns in self.ACTION_PATTERNS.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    return (action, match.groups())
        
        return None
    
    def extract_entities(self, text: str) -> List[Entity]:
        """Extrae entidades del texto."""
        entities = []
        text_lower = text.lower()
        
        for entity_type, patterns in self.ENTITY_PATTERNS.items():
            for pattern in patterns:
                for match in re.finditer(pattern, text_lower):
                    entities.append(Entity(
                        entity_type=entity_type,
                        value=match.group(0).strip(),
                        original_text=match.group(0),
                        start_pos=match.start(),
                        end_pos=match.end(),
                    ))
        
        return entities


class LanguageUnderstanding:
    """
    Módulo de comprensión del lenguaje.
    
    Combina:
    - Extracción basada en reglas (rápida, determinista)
    - LLM para casos complejos (flexible, contextual)
    """
    
    def __init__(self, llm_client: Any = None, use_llm_fallback: bool = True):
        """
        Inicializa el módulo de NLU.
        
        Args:
            llm_client: Cliente de LLM opcional (ej: Ollama)
            use_llm_fallback: Usar LLM como fallback si patrones fallan
        """
        self.llm_client = llm_client
        self.use_llm_fallback = use_llm_fallback
        self.pattern_matcher = PatternMatcher()
        
        # Historial de contexto
        self._context_history: List[Dict[str, Any]] = []
        self._max_context = 5
    
    async def understand(self, text: str, 
                        context: Dict[str, Any] = None) -> Intent:
        """
        Comprende texto y extrae intención.
        
        Args:
            text: Texto a comprender
            context: Contexto adicional (estado del mundo, conversación previa)
        
        Returns:
            Intent con acción y entidades
        """
        context = context or {}
        
        # 1. Intentar extracción basada en reglas
        rule_result = self._understand_with_rules(text)
        
        if rule_result.confidence > 0.7:
            self._add_to_context(text, rule_result)
            return rule_result
        
        # 2. Fallback a LLM si está disponible
        if self.use_llm_fallback and self.llm_client:
            try:
                llm_result = await self._understand_with_llm(text, context)
                if llm_result.confidence > rule_result.confidence:
                    self._add_to_context(text, llm_result)
                    return llm_result
            except Exception as e:
                logger.error(f"LLM understanding failed: {e}")
        
        # 3. Retornar mejor resultado disponible
        self._add_to_context(text, rule_result)
        return rule_result
    
    def _understand_with_rules(self, text: str) -> Intent:
        """Comprende usando reglas y patrones."""
        # Extraer acción
        action_match = self.pattern_matcher.match_action(text)
        
        if action_match:
            action, groups = action_match
            entities = self.pattern_matcher.extract_entities(text)
            
            # Construir parámetros según la acción
            parameters = self._build_parameters(action, groups, entities)
            
            # Determinar tipo de intent
            if action in ("greet",):
                intent_type = "greeting"
            elif action in ("query",):
                intent_type = "query"
            elif action in ("stop", "wait"):
                intent_type = "command"
            else:
                intent_type = "command"
            
            return Intent(
                intent_type=intent_type,
                action=action,
                entities=entities,
                parameters=parameters,
                confidence=0.8,
                raw_text=text,
            )
        
        # No se encontró acción clara
        entities = self.pattern_matcher.extract_entities(text)
        
        # Intentar inferir tipo básico
        if any(text.lower().startswith(w) for w in ["que", "donde", "como", "quien", "what", "where", "how", "who"]):
            return Intent(
                intent_type="query",
                action="query",
                entities=entities,
                parameters={"query": text},
                confidence=0.5,
                raw_text=text,
            )
        
        return Intent(
            intent_type="unknown",
            action="unknown",
            entities=entities,
            parameters={"raw": text},
            confidence=0.3,
            raw_text=text,
        )
    
    def _build_parameters(self, action: str, groups: tuple, 
                         entities: List[Entity]) -> Dict[str, Any]:
        """Construye parámetros según acción y grupos capturados."""
        params = {}
        
        if action == "fetch" and groups:
            params["object"] = groups[0].strip() if groups[0] else None
            params["location"] = groups[1].strip() if len(groups) > 1 and groups[1] else None
        
        elif action == "place" and groups:
            params["object"] = groups[0].strip() if groups[0] else None
            params["target"] = groups[1].strip() if len(groups) > 1 else None
        
        elif action == "move" and groups:
            params["target"] = groups[0].strip() if groups[0] else None
        
        elif action == "look" and groups:
            params["target"] = groups[0].strip() if groups[0] else None
        
        elif action == "speak" and groups:
            params["message"] = groups[0].strip() if groups[0] else None
        
        elif action == "wait" and groups:
            if groups[0]:
                params["duration_s"] = int(groups[0])
        
        # Agregar entidades a parámetros
        for entity in entities:
            if entity.entity_type not in params:
                params[entity.entity_type] = entity.value
        
        return params
    
    async def _understand_with_llm(self, text: str, 
                                  context: Dict[str, Any]) -> Intent:
        """Comprende usando LLM."""
        # Construir contexto de conversación
        conversation_context = self._get_conversation_context()
        
        prompt = f"""Eres un asistente de comprensión del lenguaje para un robot humanoide.

Contexto de conversación:
{conversation_context}

Estado actual: {context.get('world_state', 'desconocido')}

Entrada del usuario: "{text}"

Extrae la intención del usuario y responde en JSON con este formato:
{{
    "intent_type": "command|query|greeting|confirmation|negation",
    "action": "fetch|place|move|look|speak|stop|wait|greet|query",
    "entities": [
        {{"type": "object|location|person|color|number", "value": "..."}}
    ],
    "parameters": {{"object": "...", "target": "...", "location": "..."}},
    "confidence": 0.0-1.0
}}

Solo responde con el JSON, sin explicaciones."""
        
        response = await self.llm_client.generate(prompt)
        
        # Parsear respuesta
        try:
            import json
            # Limpiar respuesta
            response = response.strip()
            if response.startswith("```"):
                response = response.split("```")[1]
                if response.startswith("json"):
                    response = response[4:]
            
            data = json.loads(response)
            
            entities = [
                Entity(
                    entity_type=e.get("type", "unknown"),
                    value=e.get("value", ""),
                    original_text=e.get("value", ""),
                )
                for e in data.get("entities", [])
            ]
            
            return Intent(
                intent_type=data.get("intent_type", "unknown"),
                action=data.get("action", "unknown"),
                entities=entities,
                parameters=data.get("parameters", {}),
                confidence=data.get("confidence", 0.5),
                raw_text=text,
            )
        except Exception as e:
            logger.error(f"Error parsing LLM response: {e}")
            raise
    
    def _add_to_context(self, text: str, intent: Intent) -> None:
        """Agrega interacción al contexto."""
        self._context_history.append({
            "text": text,
            "intent": intent.to_dict(),
        })
        
        # Limitar tamaño
        if len(self._context_history) > self._max_context:
            self._context_history = self._context_history[-self._max_context:]
    
    def _get_conversation_context(self) -> str:
        """Obtiene contexto de conversación como texto."""
        if not self._context_history:
            return "(sin conversación previa)"
        
        lines = []
        for item in self._context_history[-3:]:
            lines.append(f"- Usuario: {item['text']}")
            lines.append(f"  Intención: {item['intent']['action']}")
        
        return "\n".join(lines)
    
    def clear_context(self) -> None:
        """Limpia el contexto de conversación."""
        self._context_history.clear()
