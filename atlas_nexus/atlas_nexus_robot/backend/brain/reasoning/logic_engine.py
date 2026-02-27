"""
ATLAS NEXUS - Logic Engine
Motor de razonamiento con LangChain
"""

import logging
from typing import List, Dict, Optional
from langchain.chat_models import ChatOpenAI
from langchain.schema import HumanMessage, AIMessage, SystemMessage
from langchain.prompts import ChatPromptTemplate
import os

logger = logging.getLogger(__name__)


class LogicEngine:
    """Motor de razonamiento basado en LangChain"""
    
    def __init__(
        self,
        model_name: str = "gpt-3.5-turbo",
        temperature: float = 0.7,
        api_key: Optional[str] = None
    ):
        """
        Args:
            model_name: Modelo de OpenAI a usar
            temperature: Creatividad (0.0 - 1.0)
            api_key: API key de OpenAI
        """
        self.model_name = model_name
        self.temperature = temperature
        
        # API Key desde env o parámetro
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        
        self.llm: Optional[ChatOpenAI] = None
        self.is_initialized = False
        
        # Forzar modo fallback inmediatamente
        if not self.api_key:
            logger.warning("⚠️ OpenAI API key no configurada")
            logger.info("Modo fallback: respuestas predefinidas")
            self.is_initialized = False
        else:
            # Intentar inicializar solo si hay API key
            self.initialize()
        
        # System prompt del robot
        self.system_prompt = """Eres ATLAS, un robot autónomo inteligente.

Características:
- Tienes cámara y puedes ver el entorno
- Puedes detectar objetos, personas y analizar escenas
- Eres profesional, eficiente y servicial
- Respondes de forma concisa y clara
- Puedes razonar sobre lo que ves

Cuando el usuario te pregunte algo:
1. Analiza el contexto y tu memoria
2. Si es sobre visión, usa la información de detecciones
3. Responde de forma útil y directa
4. Si no sabes algo, dilo honestamente

Responde siempre en español."""
    
    def initialize(self) -> bool:
        """Inicializa el motor de IA"""
        try:
            if not self.api_key:
                logger.warning("⚠️ OpenAI API key no configurada")
                logger.info("Modo fallback: respuestas predefinidas")
                return False
            
            logger.info(f"Inicializando LangChain con {self.model_name}...")
            
            self.llm = ChatOpenAI(
                model_name=self.model_name,
                temperature=self.temperature,
                openai_api_key=self.api_key
            )
            
            self.is_initialized = True
            logger.info("✓ Motor de razonamiento inicializado")
            return True
            
        except Exception as e:
            logger.error(f"Error inicializando IA: {e}")
            self.is_initialized = False
            return False
    
    def reason(
        self,
        query: str,
        context: Optional[Dict] = None,
        conversation_history: Optional[List[Dict]] = None
    ) -> str:
        """
        Razona sobre una consulta
        
        Args:
            query: Pregunta o consulta del usuario
            context: Contexto adicional (detecciones, estado, etc.)
            conversation_history: Historia de conversación
            
        Returns:
            Respuesta razonada
        """
        if not self.is_initialized:
            return self._fallback_response(query, context)
        
        try:
            # Construir mensajes
            messages = [SystemMessage(content=self.system_prompt)]
            
            # Agregar contexto si existe
            if context:
                context_str = self._format_context(context)
                messages.append(SystemMessage(content=f"Contexto actual:\n{context_str}"))
            
            # Agregar historia de conversación
            if conversation_history:
                for msg in conversation_history[-5:]:  # Últimos 5 mensajes
                    if msg['role'] == 'user':
                        messages.append(HumanMessage(content=msg['content']))
                    elif msg['role'] == 'assistant':
                        messages.append(AIMessage(content=msg['content']))
            
            # Agregar query actual
            messages.append(HumanMessage(content=query))
            
            # Ejecutar LLM
            response = self.llm(messages)
            
            return response.content
            
        except Exception as e:
            logger.error(f"Error en razonamiento: {e}")
            return self._fallback_response(query, context)
    
    def _format_context(self, context: Dict) -> str:
        """Formatea el contexto para el LLM"""
        parts = []
        
        # Estado del robot
        if 'robot_status' in context:
            status = context['robot_status']
            parts.append(f"Estado: {status.get('status', 'unknown')}")
            parts.append(f"Modo: {status.get('mode', 'unknown')}")
        
        # Detecciones de visión
        if 'detections' in context:
            detections = context['detections']
            if detections:
                summary = detections.get('summary', {})
                total = summary.get('total', 0)
                by_class = summary.get('by_class', {})
                
                if total > 0:
                    parts.append(f"\nVisión - Objetos detectados ({total}):")
                    for obj_class, count in by_class.items():
                        parts.append(f"  - {obj_class}: {count}")
                else:
                    parts.append("\nVisión: No se detectan objetos")
            else:
                parts.append("\nVisión: No hay datos")
        
        # Sistemas activos
        if 'systems' in context:
            systems = context['systems']
            active = [name for name, status in systems.items() if status]
            parts.append(f"\nSistemas activos: {', '.join(active)}")
        
        return "\n".join(parts)
    
    def _fallback_response(self, query: str, context: Optional[Dict] = None) -> str:
        """Respuesta fallback sin IA"""
        query_lower = query.lower()
        
        # Respuestas predefinidas
        if any(word in query_lower for word in ['hola', 'hey', 'hi']):
            return "👋 Hola! Soy ATLAS, tu robot autónomo. ¿En qué puedo ayudarte?"
        
        if any(word in query_lower for word in ['cómo estás', 'como estas', 'qué tal']):
            return "🤖 Estoy operativo y listo para ayudarte. Todos mis sistemas funcionan correctamente."
        
        if any(word in query_lower for word in ['qué ves', 'que ves', 'qué hay', 'detectas']):
            if context and 'detections' in context:
                detections = context['detections']
                summary = detections.get('summary', {})
                total = summary.get('total', 0)
                
                if total > 0:
                    by_class = summary.get('by_class', {})
                    objects = ', '.join([f"{count} {obj}" for obj, count in by_class.items()])
                    return f"👁️ Detecto {total} objeto(s): {objects}"
                else:
                    return "👁️ No detecto ningún objeto en este momento."
            return "👁️ Mi sistema de visión está activo pero no tengo datos de detección."
        
        if any(word in query_lower for word in ['ayuda', 'help', 'qué puedes', 'que puedes']):
            return """🤖 Puedo ayudarte con:
- 👁️ Ver y detectar objetos con mi cámara
- 📊 Monitorear el estado de mis sistemas
- 💬 Responder preguntas sobre mi operación
- 🎯 Analizar el entorno

Pregúntame lo que necesites!"""
        
        # Respuesta genérica
        return "🤖 Entiendo tu consulta. Actualmente estoy en modo básico. Para habilitar razonamiento avanzado, configura OPENAI_API_KEY en el backend."
    
    def analyze_scene(self, detections: Dict) -> str:
        """Analiza una escena basado en detecciones"""
        if not detections or not detections.get('summary'):
            return "No hay objetos detectados en la escena."
        
        summary = detections['summary']
        total = summary.get('total', 0)
        by_class = summary.get('by_class', {})
        highest = summary.get('highest_confidence')
        
        if total == 0:
            return "La escena está vacía, no detecto objetos."
        
        # Análisis simple
        analysis_parts = [f"Detecto {total} objeto(s) en la escena:"]
        
        for obj_class, count in by_class.items():
            analysis_parts.append(f"- {count} {obj_class}(s)")
        
        if highest:
            analysis_parts.append(
                f"\nObjeto principal: {highest['class']} "
                f"(confianza: {highest['confidence']:.0%})"
            )
        
        # Inferencias simples
        if 'persona' in by_class:
            count = by_class['persona']
            if count == 1:
                analysis_parts.append("\n👤 Hay una persona presente.")
            else:
                analysis_parts.append(f"\n👥 Hay {count} personas presentes.")
        
        if 'laptop' in by_class or 'teclado' in by_class:
            analysis_parts.append("💻 Parece un espacio de trabajo.")
        
        if any(obj in by_class for obj in ['sofa', 'cama', 'tv']):
            analysis_parts.append("🏠 Parece un espacio residencial.")
        
        return "\n".join(analysis_parts)


# Singleton
_logic_engine: Optional[LogicEngine] = None

def get_logic_engine() -> LogicEngine:
    """Obtiene instancia del motor de lógica"""
    global _logic_engine
    if _logic_engine is None:
        _logic_engine = LogicEngine()
        _logic_engine.initialize()
    return _logic_engine
