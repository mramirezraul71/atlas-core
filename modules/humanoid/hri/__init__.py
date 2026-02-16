"""
ATLAS Human-Robot Interaction (HRI) Module
============================================
Sistema de interacción humano-robot multimodal.

Componentes:
- VoiceInterface: Reconocimiento y síntesis de voz
- GestureRecognition: Detección de gestos
- EmotionRecognition: Detección de emociones
- IntentParser: Interpretación de intenciones
- DialogManager: Gestión de diálogos
- SafetyMonitor: Seguridad en interacción física

Uso:
    from modules.humanoid.hri import HRISystem
    
    hri = HRISystem()
    hri.start()
    intent = hri.process_input("Hola, tráeme un vaso")
"""
from .voice_interface import VoiceInterface, VoiceConfig
from .gesture_recognition import GestureRecognizer, Gesture
from .emotion_recognition import EmotionRecognizer, Emotion
from .intent_parser import IntentParser, Intent
from .dialog_manager import DialogManager, DialogState
from .safety_monitor import SafetyMonitor, SafetyZone
from .hri_system import HRISystem, HRIConfig

__all__ = [
    "VoiceInterface",
    "VoiceConfig",
    "GestureRecognizer",
    "Gesture",
    "EmotionRecognizer",
    "Emotion",
    "IntentParser",
    "Intent",
    "DialogManager",
    "DialogState",
    "SafetyMonitor",
    "SafetyZone",
    "HRISystem",
    "HRIConfig",
]
