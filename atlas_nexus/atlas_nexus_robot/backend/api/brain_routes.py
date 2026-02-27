"""
Brain API Routes - IA y Razonamiento
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Dict, Optional
import logging
from datetime import datetime

from brain.reasoning.logic_engine import get_logic_engine
from brain.memory.short_term import get_short_term_memory
from vision.object_detection import get_detector
from identity.face_recognition_system import get_face_system
from identity.voice_recognition_system import get_voice_system
from user.profile_manager import get_profile_manager
from brain.reasoning.personality_engine import get_personality_engine
from database.models import create_tables
from brain.routing.ai_router import get_ai_router
from brain.routing.query_classifier import get_query_classifier

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/api/brain", tags=["brain"])


class ChatMessage(BaseModel):
    message: str
    include_vision: bool = True
    include_status: bool = True
    emotion: Optional[str] = None
    personality_mode: Optional[str] = None


class PersonalChatMessage(BaseModel):
    message: str
    emotion: Optional[str] = None
    personality_mode: Optional[str] = None
    include_context: bool = True


class MultiAIChatMessage(BaseModel):
    message: str
    force_provider: Optional[str] = None
    force_model: Optional[str] = None
    include_routing_info: bool = True


class FaceRegistrationRequest(BaseModel):
    image_data: str  # base64 encoded image


class VoiceRegistrationRequest(BaseModel):
    audio_data: str  # base64 encoded audio


class UserProfileRequest(BaseModel):
    name: str
    nickname: Optional[str] = None
    birthdate: Optional[str] = None
    profession: Optional[str] = None
    preferred_temperature: Optional[float] = 22.0
    preferred_formality: Optional[str] = "casual"
    wake_up_time: Optional[str] = None
    sleep_time: Optional[str] = None
    work_hours: Optional[Dict] = None


class ChatResponse(BaseModel):
    response: str
    context_used: Dict
    timestamp: str


class MultiAIChatResponse(BaseModel):
    response: str
    provider: str
    model: str
    cost: str
    response_time: float
    query_type: str
    routing_info: Optional[Dict] = None
    metadata: Optional[Dict] = None
    timestamp: str


@router.post("/chat", response_model=ChatResponse)
async def chat(msg: ChatMessage):
    """
    Chat con el robot (incluye razonamiento)
    """
    try:
        logic_engine = get_logic_engine()
        memory = get_short_term_memory()
        
        # Construir contexto
        context = {}
        
        # Estado del robot
        if msg.include_status:
            context['robot_status'] = {
                'status': 'online',
                'mode': 'autonomous'
            }
            context['systems'] = {
                'camera': True,
                'vision': True,
                'ai': logic_engine.is_initialized,
                'yolo': True
            }
        
        # Información de visión
        if msg.include_vision:
            try:
                import cv2
                detector = get_detector()
                
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    ret, frame = cap.read()
                    cap.release()
                    
                    if ret:
                        detections = detector.detect(frame)
                        summary = detector.get_summary(detections)
                        context['detections'] = {
                            'detections': detections,
                            'summary': summary
                        }
            except Exception as e:
                logger.warning(f"No se pudo obtener visión: {e}")
        
        # Historia de conversación
        history = memory.get_history(last_n=10)
        
        # Razonar
        response = logic_engine.reason(
            query=msg.message,
            context=context,
            conversation_history=history
        )
        
        # Guardar en memoria
        memory.add_message('user', msg.message)
        memory.add_message('assistant', response)
        
        from datetime import datetime
        
        return {
            'response': response,
            'context_used': {
                'has_vision': 'detections' in context,
                'has_status': 'robot_status' in context,
                'history_size': len(history)
            },
            'timestamp': datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error en chat: {e}")
        raise HTTPException(500, f"Error en chat: {str(e)}")


@router.get("/analyze-scene")
async def analyze_scene():
    """Analiza la escena actual con IA"""
    try:
        import cv2
        detector = get_detector()
        logic_engine = get_logic_engine()
        
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            raise HTTPException(500, "Camera not available")
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            raise HTTPException(500, "Cannot read frame")
        
        # Detectar objetos
        detections = detector.detect(frame)
        summary = detector.get_summary(detections)
        
        # Analizar con IA
        analysis = logic_engine.analyze_scene({
            'detections': detections,
            'summary': summary
        })
        
        return {
            'analysis': analysis,
            'detections': detections,
            'summary': summary
        }
        
    except Exception as e:
        logger.error(f"Error en análisis: {e}")
        raise HTTPException(500, str(e))


@router.post("/initialize")
async def initialize_brain():
    """Inicializa el motor de IA"""
    try:
        logic_engine = get_logic_engine()
        success = logic_engine.initialize()
        
        return {
            'success': success,
            'initialized': logic_engine.is_initialized,
            'model': logic_engine.model_name
        }
    except Exception as e:
        logger.error(f"Error inicializando brain: {e}")
        raise HTTPException(500, str(e))


@router.get("/memory/history")
async def get_memory_history():
    """Obtiene historia de conversación"""
    memory = get_short_term_memory()
    history = memory.get_history()
    
    return {
        'messages': history,
        'total': len(history),
        'summary': memory.get_context_summary()
    }


@router.post("/memory/clear")
async def clear_memory():
    """Limpia la memoria de conversación"""
    memory = get_short_term_memory()
    memory.clear()
    
    return {
        'success': True,
        'message': 'Memoria limpiada'
    }


# IDENTITY AND PERSONALIZATION ENDPOINTS

@router.post("/identity/register-face")
async def register_face(request: FaceRegistrationRequest):
    """Register user face from image data"""
    try:
        face_system = get_face_system()
        result = face_system.register_face_from_image(request.image_data)
        
        if result["success"]:
            # Update user profile
            profile_manager = get_profile_manager()
            profile_manager.update_user_profile({"face_registered": True})
        
        return result
        
    except Exception as e:
        logger.error(f"Error registering face: {e}")
        raise HTTPException(500, f"Error registering face: {str(e)}")


@router.post("/identity/check-owner")
async def check_owner_presence(request: FaceRegistrationRequest):
    """Check if owner is present in image"""
    try:
        face_system = get_face_system()
        result = face_system.is_owner_present(request.image_data)
        
        return result
        
    except Exception as e:
        logger.error(f"Error checking owner: {e}")
        raise HTTPException(500, f"Error checking owner: {str(e)}")


@router.post("/identity/register-voice")
async def register_voice(request: VoiceRegistrationRequest):
    """Register user voice from audio data"""
    try:
        voice_system = get_voice_system()
        result = voice_system.register_voice_from_audio(request.audio_data)
        
        if result["success"]:
            # Update user profile
            profile_manager = get_profile_manager()
            profile_manager.update_user_profile({"voice_registered": True})
        
        return result
        
    except Exception as e:
        logger.error(f"Error registering voice: {e}")
        raise HTTPException(500, f"Error registering voice: {str(e)}")


@router.post("/identity/verify-voice")
async def verify_speaker(request: VoiceRegistrationRequest):
    """Verify if speaker is the registered owner"""
    try:
        voice_system = get_voice_system()
        result = voice_system.verify_speaker(request.audio_data)
        
        return result
        
    except Exception as e:
        logger.error(f"Error verifying voice: {e}")
        raise HTTPException(500, f"Error verifying voice: {str(e)}")


@router.get("/identity/status")
async def get_identity_status():
    """Get identity registration status"""
    try:
        face_system = get_face_system()
        voice_system = get_voice_system()
        
        face_status = face_system.get_registration_status()
        voice_status = voice_system.get_registration_status()
        
        return {
            "face": face_status,
            "voice": voice_status,
            "ready_for_identification": (
                face_status["ready_for_identification"] and 
                voice_status["ready_for_identification"]
            )
        }
        
    except Exception as e:
        logger.error(f"Error getting identity status: {e}")
        raise HTTPException(500, f"Error getting identity status: {str(e)}")


@router.post("/profile/create")
async def create_user_profile(request: UserProfileRequest):
    """Create user profile"""
    try:
        profile_manager = get_profile_manager()
        
        profile_data = request.dict()
        if request.birthdate:
            from datetime import datetime
            profile_data["birthdate"] = datetime.fromisoformat(request.birthdate.replace("Z", "+00:00"))
        
        result = profile_manager.create_user_profile(profile_data)
        
        return result
        
    except Exception as e:
        logger.error(f"Error creating profile: {e}")
        raise HTTPException(500, f"Error creating profile: {str(e)}")


@router.get("/profile")
async def get_user_profile():
    """Get user profile"""
    try:
        profile_manager = get_profile_manager()
        result = profile_manager.get_user_profile()
        
        return result
        
    except Exception as e:
        logger.error(f"Error getting profile: {e}")
        raise HTTPException(500, f"Error getting profile: {str(e)}")


@router.put("/profile")
async def update_user_profile(request: UserProfileRequest):
    """Update user profile"""
    try:
        profile_manager = get_profile_manager()
        
        updates = request.dict(exclude_unset=True)
        if "birthdate" in updates and updates["birthdate"]:
            from datetime import datetime
            updates["birthdate"] = datetime.fromisoformat(updates["birthdate"].replace("Z", "+00:00"))
        
        result = profile_manager.update_user_profile(updates)
        
        return result
        
    except Exception as e:
        logger.error(f"Error updating profile: {e}")
        raise HTTPException(500, f"Error updating profile: {str(e)}")


@router.post("/chat-personal", response_model=ChatResponse)
async def personal_chat(msg: PersonalChatMessage):
    """Personalized chat with personality engine"""
    try:
        # Get systems
        logic_engine = get_logic_engine()
        memory = get_short_term_memory()
        profile_manager = get_profile_manager()
        personality_engine = get_personality_engine()
        
        # Set personality mode if provided
        if msg.personality_mode:
            from brain.reasoning.personality_engine import PersonalityMode
            try:
                personality_engine.set_mode(PersonalityMode(msg.personality_mode))
            except ValueError:
                logger.warning(f"Invalid personality mode: {msg.personality_mode}")
        
        # Set user emotion if provided
        if msg.emotion:
            from brain.reasoning.personality_engine import Emotion
            try:
                personality_engine.set_user_emotion(Emotion(msg.emotion))
            except ValueError:
                logger.warning(f"Invalid emotion: {msg.emotion}")
        
        # Build context
        context = {}
        if msg.include_context:
            user_context_result = profile_manager.get_user_context()
            if user_context_result["success"]:
                context = user_context_result["context"]
        
        # Get base response from logic engine
        history = memory.get_history(last_n=10)
        base_response = logic_engine.reason(
            query=msg.message,
            context=context,
            conversation_history=history
        )
        
        # Adapt response with personality
        user_name = context.get("user", {}).get("name", "amigo")
        adapted_response = personality_engine.adapt_response(
            base_response, 
            user_name=user_name, 
            context=context
        )
        
        # Record interaction
        profile_manager.record_interaction({
            "interaction_type": "chat",
            "user_input": msg.message,
            "robot_response": adapted_response,
            "user_emotion": msg.emotion,
            "context": {
                "personality_mode": personality_engine.current_mode.value,
                "detected_emotion": msg.emotion
            }
        })
        
        # Save to memory
        memory.add_message('user', msg.message)
        memory.add_message('assistant', adapted_response)
        
        from datetime import datetime
        
        return {
            'response': adapted_response,
            'context_used': {
                'has_user_context': 'user' in context,
                'personality_mode': personality_engine.current_mode.value,
                'user_emotion': msg.emotion,
                'history_size': len(history)
            },
            'timestamp': datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error in personal chat: {e}")
        raise HTTPException(500, f"Error in personal chat: {str(e)}")


@router.get("/personality/mode")
async def get_personality_mode():
    """Get current personality mode"""
    try:
        personality_engine = get_personality_engine()
        
        return {
            "current_mode": personality_engine.current_mode.value,
            "description": personality_engine.get_mode_description(),
            "user_emotion": personality_engine.user_emotion.value
        }
        
    except Exception as e:
        logger.error(f"Error getting personality mode: {e}")
        raise HTTPException(500, f"Error getting personality mode: {str(e)}")


@router.post("/personality/mode")
async def set_personality_mode(mode: str):
    """Set personality mode"""
    try:
        personality_engine = get_personality_engine()
        
        from brain.reasoning.personality_engine import PersonalityMode
        try:
            personality_engine.set_mode(PersonalityMode(mode))
            
            return {
                "success": True,
                "message": f"Personality mode set to {mode}",
                "current_mode": personality_engine.current_mode.value
            }
        except ValueError:
            raise HTTPException(400, f"Invalid personality mode: {mode}")
        
    except Exception as e:
        logger.error(f"Error setting personality mode: {e}")
        raise HTTPException(500, f"Error setting personality mode: {str(e)}")


@router.get("/personality/stats")
async def get_personality_stats():
    """Get personality engine statistics"""
    try:
        personality_engine = get_personality_engine()
        
        return personality_engine.get_response_stats()
        
    except Exception as e:
        logger.error(f"Error getting personality stats: {e}")
        raise HTTPException(500, f"Error getting personality stats: {str(e)}")


@router.get("/preferences")
async def get_user_preferences(category: Optional[str] = None):
    """Get user preferences"""
    try:
        profile_manager = get_profile_manager()
        result = profile_manager.get_preferences(category)
        
        return result
        
    except Exception as e:
        logger.error(f"Error getting preferences: {e}")
        raise HTTPException(500, f"Error getting preferences: {str(e)}")


@router.post("/preferences")
async def update_preference(category: str, item: str, preference_score: float, context: Optional[Dict] = None):
    """Update user preference"""
    try:
        profile_manager = get_profile_manager()
        result = profile_manager.update_preference(category, item, preference_score, context)
        
        return result
        
    except Exception as e:
        logger.error(f"Error updating preference: {e}")
        raise HTTPException(500, f"Error updating preference: {str(e)}")


@router.get("/interactions")
async def get_interaction_history(limit: int = 50, interaction_type: Optional[str] = None):
    """Get interaction history"""
    try:
        profile_manager = get_profile_manager()
        result = profile_manager.get_interaction_history(limit, interaction_type)
        
        return result
        
    except Exception as e:
        logger.error(f"Error getting interaction history: {e}")
        raise HTTPException(500, f"Error getting interaction history: {str(e)}")


@router.post("/initialize-database")
async def initialize_database():
    """Initialize database tables"""
    try:
        create_tables()
        
        return {
            "success": True,
            "message": "Database initialized successfully"
        }
        
    except Exception as e:
        logger.error(f"Error initializing database: {e}")
        raise HTTPException(500, f"Error initializing database: {str(e)}")


# MULTI-A SYSTEM ENDPOINTS

@router.post("/chat-multi-ai", response_model=MultiAIChatResponse)
async def chat_multi_ai(msg: MultiAIChatMessage):
    """
    Chat con sistema Multi-IA y routing automático inteligente
    """
    try:
        ai_router = get_ai_router()
        query_classifier = get_query_classifier()
        
        # Generar respuesta usando el sistema de routing
        result = ai_router.generate_response(msg.message)
        
        if not result["success"]:
            raise HTTPException(500, result["error"])
        
        # Obtener información de routing si se solicita
        routing_info = None
        if msg.include_routing_info:
            routing_info = ai_router.get_routing_info(msg.message)
        
        return MultiAIChatResponse(
            response=result["response"],
            provider=result["provider"],
            model=result["model"],
            cost=result["cost"],
            response_time=result["response_time"],
            query_type=result["query_type"],
            routing_info=routing_info,
            metadata=result.get("metadata"),
            timestamp=datetime.now().isoformat()
        )
        
    except Exception as e:
        logger.error(f"Error in multi-ai chat: {e}")
        raise HTTPException(500, f"Error in multi-ai chat: {str(e)}")


@router.get("/available-ais")
async def get_available_ais():
    """
    Obtiene el estado de todos los proveedores de IA disponibles
    """
    try:
        ai_router = get_ai_router()
        
        providers_info = ai_router.get_available_providers()
        
        return {
            "success": True,
            "providers": providers_info,
            "total_providers": len(providers_info),
            "available_count": sum(1 for p in providers_info.values() if p["status"] == "available"),
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error getting available AIs: {e}")
        raise HTTPException(500, f"Error getting available AIs: {str(e)}")


@router.post("/classify-query")
async def classify_query_endpoint(msg: MultiAIChatMessage):
    """
    Clasifica una query para mostrar información de routing
    """
    try:
        query_classifier = get_query_classifier()
        ai_router = get_ai_router()
        
        classification = query_classifier.get_classification_details(msg.message)
        routing_info = ai_router.get_routing_info(msg.message)
        
        return {
            "success": True,
            "classification": classification,
            "routing": routing_info.get("routing", {}),
            "timestamp": datetime.now().isoformat()
        }
        
    except Exception as e:
        logger.error(f"Error classifying query: {e}")
        raise HTTPException(500, f"Error classifying query: {str(e)}")
